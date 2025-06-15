// ========== IMPROVED ESP8266 BINARY PROTOCOL ==========

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <WiFiClientSecure.h>

// ==== Cấu hình WiFi và Server ====
const char* ssid = "LV_2.0";           
const char* password = "12341234";   
const char* serverURL = "https://iot.theman.vn/api/gas-sensor"; 
const char* deviceID = "GAS_SENSOR_01";

// ==== IMPROVED Binary Protocol ====
#define PACKET_START_MARKER 0x55
#define PACKET_HEADER       0xAA  
#define PACKET_END_MARKER   0x33
#define PACKET_SIZE         10    // Updated size with markers
#define STM32_TIMEOUT       90000

// Improved packet structure (must match STM32)
typedef struct {
    uint8_t start_marker;   // 0x55
    uint8_t header;         // 0xAA
    uint16_t gas_value;     // ADC value 0-4095
    uint8_t flags;          // bit0-1: gas_level, bit2: system_state, bit3: relay_state
    uint8_t sequence;       // Message counter
    uint8_t reserved;       // For future use
    uint16_t checksum;      // CRC16
    uint8_t end_marker;     // 0x33
} __attribute__((packed)) sensor_packet_t;

// ==== Hardware ====
SoftwareSerial stm32Serial(D2, -1); // RX=D2, TX=không dùng

// ==== Biến toàn cục ====
// WiFiClient wifiClient;
WiFiClientSecure wifiClient;
HTTPClient http;
String textBuffer = "";
uint8_t binaryBuffer[32];
int binaryBufferIndex = 0;
bool inBinaryPacket = false;

// Sensor data
struct {
    int gasValue;
    String gasStatus;
    String systemStatus;
    String relayStatus;
    unsigned long lastUpdateTime;
    bool hasData;
    bool forceNextSend;
    uint8_t lastSequence;
} sensor = {0, "UNKNOWN", "UNKNOWN", "UNKNOWN", 0, false, false, 255};

// Enhanced stats
struct {
    unsigned long bootTime;
    unsigned long lastSTM32Contact;
    unsigned long lastHeartbeat;
    unsigned long lastServerSend;
    bool stm32Connected;
    bool stm32SystemActive;
    bool wifiConnected;
    int totalMessages;
    int totalBinaryPackets;
    int heartbeatCount;
    int buttonEvents;
    int binaryErrors;
    int textMessages;
    int validPackets;
    int invalidHeaders;
    int crcErrors;
    int duplicatePackets;
} stats = {0, 0, 0, 0, false, false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// ==== Function prototypes ====
uint16_t calculate_crc16(uint8_t* data, uint8_t len);
bool parse_binary_packet(uint8_t* buffer);
String getGasStatusString(uint8_t level);
void processTextMessage(String msg);

void setup() {
    Serial.begin(115200);
    stm32Serial.begin(9600);
    delay(1000);
    
    stats.bootTime = millis();
    
    Serial.println("=== ESP8266 Gas Gateway v4.1 Binary Improved ===");
    Serial.println("Device: " + String(deviceID));
    Serial.println("STM32: 9600 baud, Improved Binary Protocol");
    
    connectWiFi();
    
    Serial.println("Ready. Waiting for STM32...");
    Serial.println("Expected connection:");
    Serial.println("  STM32 PA9(TX) → ESP8266 D2(GPIO4)");
    Serial.println("  STM32 GND → ESP8266 GND");
    Serial.println("Protocol: Binary v1.1 + Text messages");
    Serial.println("Packet format: [0x55][0xAA][DATA...][CRC][0x33]");
    Serial.println("========================================");
}

void loop() {
    readSTM32();
    checkSTM32Health();
    sendToServer();
    printStatus();
    
    delay(20); // Further reduced delay
}

void connectWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("WiFi connecting");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" OK");
        Serial.println("IP: " + WiFi.localIP().toString());
        stats.wifiConnected = true;
    } else {
        Serial.println(" FAILED");
        stats.wifiConnected = false;
    }
}

// ==== CRC16 Calculation (must match STM32) ====
uint16_t calculate_crc16(uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

// ==== Gas Status String ====
String getGasStatusString(uint8_t level) {
    switch(level) {
        case 0: return "SAFE";
        case 1: return "WARNING";
        case 2: return "DANGER";
        case 3: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

// ==== IMPROVED Binary Packet Parser ====
bool parse_binary_packet(uint8_t* buffer) {
    sensor_packet_t* packet = (sensor_packet_t*)buffer;
    
    // Verify start and end markers
    if (packet->start_marker != PACKET_START_MARKER) {
        Serial.println("❌ Invalid start marker: 0x" + String(packet->start_marker, HEX));
        stats.invalidHeaders++;
        return false;
    }
    
    if (packet->end_marker != PACKET_END_MARKER) {
        Serial.println("❌ Invalid end marker: 0x" + String(packet->end_marker, HEX));
        stats.invalidHeaders++;
        return false;
    }
    
    // Verify header
    if (packet->header != PACKET_HEADER) {
        Serial.println("❌ Invalid header: 0x" + String(packet->header, HEX));
        stats.invalidHeaders++;
        return false;
    }
    
    // Verify checksum (exclude markers and checksum itself)
    uint8_t* crc_data = buffer + 1; // Skip start marker
    uint16_t calculated_crc = calculate_crc16(crc_data, PACKET_SIZE - 4); // Exclude markers and checksum
    
    if (calculated_crc != packet->checksum) {
        Serial.println("❌ CRC error - Expected: 0x" + String(packet->checksum, HEX) + 
                      ", Calculated: 0x" + String(calculated_crc, HEX));
        stats.crcErrors++;
        return false;
    }
    
    // Check for duplicate packets
    if (packet->sequence == sensor.lastSequence) {
        Serial.println("⚠️ Duplicate packet #" + String(packet->sequence) + " ignored");
        stats.duplicatePackets++;
        return false;
    }
    
    // Extract data from packet
    uint16_t newGas = packet->gas_value;
    uint8_t gasLevel = packet->flags & 0x03;
    uint8_t systemState = (packet->flags >> 2) & 0x01;
    uint8_t relayState = (packet->flags >> 3) & 0x01;
    
    // Validate gas value
    if (newGas > 4095) {
        Serial.println("❌ Invalid gas value: " + String(newGas));
        stats.binaryErrors++;
        return false;
    }
    
    // Check for IMMEDIATE send conditions
    bool gasChanged = abs((int)newGas - sensor.gasValue) >= 30; // Lowered threshold
    bool statusChanged = (getGasStatusString(gasLevel) != sensor.gasStatus);
    bool systemChanged = (sensor.systemStatus != String(systemState ? "ACTIVE" : "STOPPED"));
    bool relayChanged = (sensor.relayStatus != String(relayState ? "ON" : "OFF"));
    
    // IMMEDIATE SEND logic
    bool hasGasDanger = (gasLevel >= 1); // WARNING or higher
    bool wasNoGas = (sensor.gasStatus == "SAFE" || sensor.gasStatus == "UNKNOWN");
    bool newGasDanger = hasGasDanger && wasNoGas;
    
    if (newGasDanger || gasChanged || statusChanged || systemChanged || relayChanged) {
        Serial.println("🚨 IMMEDIATE SEND triggered by binary packet!");
        Serial.println("    Gas: " + String(sensor.gasValue) + " → " + String(newGas) + 
                      " (change: " + String(abs((int)newGas - sensor.gasValue)) + ")");
        Serial.println("    Status: " + sensor.gasStatus + " → " + getGasStatusString(gasLevel));
        sensor.forceNextSend = true;
    }
    
    // Update sensor data
    sensor.gasValue = newGas;
    sensor.gasStatus = getGasStatusString(gasLevel);
    sensor.systemStatus = systemState ? "ACTIVE" : "STOPPED";
    sensor.relayStatus = relayState ? "ON" : "OFF";
    sensor.lastUpdateTime = millis();
    sensor.hasData = true;
    sensor.lastSequence = packet->sequence;
    
    // Update system state
    stats.stm32SystemActive = systemState;
    stats.validPackets++;
    
    Serial.println("✅ Binary packet #" + String(packet->sequence) + 
                   ": Gas=" + String(newGas) + " (" + sensor.gasStatus + 
                   "), Sys=" + sensor.systemStatus + ", Relay=" + sensor.relayStatus +
                   " [CRC:OK]");
    
    return true;
}

// ==== IMPROVED STM32 Data Reader ====
void readSTM32() {
    while (stm32Serial.available()) {
        uint8_t incomingByte = stm32Serial.read();
        
        // State machine for binary packet parsing
        if (!inBinaryPacket) {
            // Look for start marker
            if (incomingByte == PACKET_START_MARKER) {
                binaryBuffer[0] = incomingByte;
                binaryBufferIndex = 1;
                inBinaryPacket = true;
                continue;
            }
            
            // Handle text messages when not in binary packet
            char c = (char)incomingByte;
            if (c == '\n') {
                if (textBuffer.length() > 0) {
                    textBuffer.trim();
                    if (textBuffer.startsWith("TXT:")) {
                        textBuffer = textBuffer.substring(4); // Remove "TXT:" prefix
                        processTextMessage(textBuffer);
                    }
                    textBuffer = "";
                }
            } else if (c != '\r' && c >= 10 && c <= 127) {
                textBuffer += c;
                if (textBuffer.length() > 200) {
                    textBuffer = ""; // Overflow protection
                }
            }
        } else {
            // Building binary packet
            binaryBuffer[binaryBufferIndex++] = incomingByte;
            
            // Check for complete packet
            if (binaryBufferIndex >= PACKET_SIZE) {
                stats.totalBinaryPackets++;
                stats.lastSTM32Contact = millis();
                stats.stm32Connected = true;
                
                // Verify end marker before parsing
                if (binaryBuffer[PACKET_SIZE - 1] == PACKET_END_MARKER) {
                    if (parse_binary_packet(binaryBuffer)) {
                        // Successfully parsed binary packet
                    } else {
                        Serial.println("❌ Failed to parse binary packet");
                        stats.binaryErrors++;
                    }
                } else {
                    Serial.println("❌ Missing end marker in packet");
                    stats.binaryErrors++;
                }
                
                // Reset for next packet
                binaryBufferIndex = 0;
                inBinaryPacket = false;
                continue;
            }
        }
        
        // Timeout for incomplete binary packets
        static unsigned long lastByteTime = 0;
        if (inBinaryPacket && (millis() - lastByteTime) > 50) { // Reduced timeout
            Serial.println("⚠️ Binary packet timeout, resetting buffer (had " + 
                          String(binaryBufferIndex) + " bytes)");
            binaryBufferIndex = 0;
            inBinaryPacket = false;
            stats.binaryErrors++;
        }
        lastByteTime = millis();
    }
}

// ==== Text Message Processor (unchanged) ====
void processTextMessage(String msg) {
    if (msg.length() == 0) return;
    
    stats.textMessages++;
    stats.lastSTM32Contact = millis();
    stats.stm32Connected = true;
    
    Serial.println("[TXT:" + String(stats.textMessages) + "] " + msg);
    
    if (msg.startsWith("HEARTBEAT")) {
        stats.heartbeatCount++;
        parseHeartbeat(msg);
    } else if (msg.startsWith("BTN_")) {
        stats.buttonEvents++;
        Serial.println("  Button event detected");
        sensor.forceNextSend = true;
    } else if (msg.indexOf("START") != -1 || msg.indexOf("READY") != -1) {
        Serial.println("  STM32 startup detected");
        sensor.forceNextSend = true;
    } else if (msg.indexOf("IMMEDIATE_SEND") != -1) {
        Serial.println("  Immediate send notification");
    } else if (msg.indexOf("SENT_BINARY") != -1) {
        Serial.println("  Binary packet send confirmation");
    }
}

void parseHeartbeat(String msg) {
    stats.lastHeartbeat = millis();
    
    bool wasActive = stats.stm32SystemActive;
    
    if (msg.indexOf("ACTIVE") != -1) {
        stats.stm32SystemActive = true;
        sensor.systemStatus = "ACTIVE";
    } else if (msg.indexOf("STOPPED") != -1) {
        stats.stm32SystemActive = false;
        sensor.systemStatus = "STOPPED";
    }
    
    if (wasActive != stats.stm32SystemActive) {
        Serial.println("  System state changed via heartbeat");
        sensor.forceNextSend = true;
    }
    
    Serial.println("  Heartbeat: " + sensor.systemStatus);
}

void checkSTM32Health() {
    unsigned long now = millis();
    
    if (stats.stm32Connected && (now - stats.lastSTM32Contact) > STM32_TIMEOUT) {
        stats.stm32Connected = false;
        stats.stm32SystemActive = false;
        sensor.systemStatus = "OFFLINE";
        sensor.forceNextSend = true;
        
        Serial.println("STM32 OFFLINE! (" + String((now - stats.lastSTM32Contact)/1000) + "s)");
    }
}

void sendToServer() {
    if (!stats.wifiConnected || WiFi.status() != WL_CONNECTED) {
        return;
    }

    unsigned long now = millis();
    bool shouldSend = false;

    // Điều kiện gửi ngay lập tức
    if (sensor.forceNextSend) {
        shouldSend = true;
        Serial.println("🚨 IMMEDIATE SEND - Gas detected or major change!");
    } else if (sensor.hasData || !stats.stm32Connected) {
        unsigned long interval;

        if (!stats.stm32Connected) {
            interval = 60000;
        } else if (!stats.stm32SystemActive) {
            interval = 30000;
        } else {
            if (sensor.gasStatus == "CRITICAL") {
                interval = 2000;
            } else if (sensor.gasStatus == "DANGER") {
                interval = 3000;
            } else if (sensor.gasStatus == "WARNING") {
                interval = 5000;
            } else {
                interval = 10000;
            }
        }

        if (now - stats.lastServerSend >= interval) {
            shouldSend = true;
            Serial.println("⏰ Interval send (" + String(interval / 1000) + "s) - " + sensor.gasStatus);
        }
    }

    if (!shouldSend) return;

    Serial.println("📤 Sending to server...");

    DynamicJsonDocument doc(1024);

    doc["device_id"] = deviceID;
    doc["timestamp"] = now;
    doc["uptime"] = (now - stats.bootTime) / 1000;
    doc["stm32_connected"] = stats.stm32Connected;
    doc["protocol"] = "binary_v1.1_improved";

    if (stats.stm32Connected) {
        if (stats.stm32SystemActive && sensor.hasData) {
            doc["gas_value"] = sensor.gasValue;
            doc["gas_status"] = sensor.gasStatus;
            doc["system_status"] = "ACTIVE";
            doc["relay_status"] = sensor.relayStatus;

            if (sensor.gasStatus != "SAFE") {
                doc["urgent"] = true;
                doc["alert_level"] = sensor.gasStatus;
            }
        } else if (!stats.stm32SystemActive) {
            doc["gas_value"] = -1;
            doc["gas_status"] = "SYSTEM_STOPPED";
            doc["system_status"] = "STOPPED";
            doc["relay_status"] = "OFF";
            doc["message"] = "Gas sensor module turned OFF";
        } else {
            doc["gas_value"] = -1;
            doc["gas_status"] = "INITIALIZING";
            doc["system_status"] = "INITIALIZING";
            doc["relay_status"] = "UNKNOWN";
        }
    } else {
        doc["gas_value"] = -1;
        doc["gas_status"] = "OFFLINE";
        doc["system_status"] = "OFFLINE";
        doc["relay_status"] = "UNKNOWN";
        doc["error"] = "STM32_NOT_RESPONDING";
        doc["message"] = "Gas sensor module offline";
        doc["urgent"] = true;
    }

    // Các thống kê mở rộng
    doc["wifi_rssi"] = WiFi.RSSI();
    doc["total_binary_packets"] = stats.totalBinaryPackets;
    doc["valid_packets"] = stats.validPackets;
    doc["total_text_messages"] = stats.textMessages;
    doc["heartbeat_count"] = stats.heartbeatCount;
    doc["button_events"] = stats.buttonEvents;
    doc["binary_errors"] = stats.binaryErrors;
    doc["crc_errors"] = stats.crcErrors;
    doc["invalid_headers"] = stats.invalidHeaders;
    doc["duplicate_packets"] = stats.duplicatePackets;
    doc["last_sequence"] = sensor.lastSequence;
    doc["packet_success_rate"] = stats.totalBinaryPackets > 0 ?
                                  (float)stats.validPackets / stats.totalBinaryPackets * 100.0 : 0.0;

    if (sensor.forceNextSend) {
        doc["immediate_send"] = true;
    }

    wifiClient.setInsecure();

    // Gửi JSON
    String jsonString;
    serializeJson(doc, jsonString);
    Serial.println("JSON:\n" + jsonString);


    http.begin(wifiClient, serverURL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Accept", "application/json");
    http.setTimeout(8000);

    int responseCode = http.POST(jsonString);

    if (responseCode > 0) {
        Serial.println("✅ Server OK: " + String(responseCode));
        stats.lastServerSend = now;
    } else {
        Serial.println("❌ Server ERROR: " + String(responseCode));
    }

    http.end();
    sensor.forceNextSend = false;
}


void printStatus() {
    static unsigned long lastReport = 0;
    
    if (millis() - lastReport >= 30000) { // Every 30s (faster)
        lastReport = millis();
        
        Serial.println("\n--- IMPROVED BINARY PROTOCOL STATUS ---");
        Serial.println("Uptime: " + String((millis() - stats.bootTime)/1000) + "s");
        Serial.println("WiFi: " + String(stats.wifiConnected ? "OK" : "FAIL"));
        Serial.println("STM32: " + String(stats.stm32Connected ? "ONLINE" : "OFFLINE"));
        
        if (stats.stm32Connected) {
            Serial.println("System: " + String(stats.stm32SystemActive ? "ACTIVE" : "STOPPED"));
        }
        
        // Enhanced binary statistics
        float successRate = stats.totalBinaryPackets > 0 ? 
                           (float)stats.validPackets / stats.totalBinaryPackets * 100 : 0;
        
        Serial.println("Binary packets: " + String(stats.totalBinaryPackets) + 
                      " (valid: " + String(stats.validPackets) + 
                      ", success: " + String(successRate, 1) + "%)");
                      
        if (stats.binaryErrors > 0) {
            Serial.println("Errors breakdown:");
            Serial.println("  CRC errors: " + String(stats.crcErrors));
            Serial.println("  Invalid headers: " + String(stats.invalidHeaders));
            Serial.println("  Duplicates: " + String(stats.duplicatePackets));
            Serial.println("  Other errors: " + String(stats.binaryErrors - stats.crcErrors - stats.invalidHeaders));
        }
        
        Serial.println("Text messages: " + String(stats.textMessages) + 
                      " (♥️:" + String(stats.heartbeatCount) + 
                      ", BTN:" + String(stats.buttonEvents) + ")");
        
        if (sensor.hasData && stats.stm32SystemActive) {
            Serial.println("Gas: " + String(sensor.gasValue) + " (" + sensor.gasStatus + ")");
            Serial.println("Relay: " + sensor.relayStatus + ", Last seq: " + String(sensor.lastSequence));
        }
        
        // Detailed diagnostics
        if (stats.binaryErrors > 0) {
            Serial.println("⚠️ Binary errors detected:");
            if (stats.crcErrors > 0) Serial.println("  - CRC errors suggest transmission noise");
            if (stats.invalidHeaders > 0) Serial.println("  - Header errors suggest sync issues");
            if (stats.duplicatePackets > 0) Serial.println("  - Duplicate packets (normal, filtered out)");
        }
        
        if (!stats.stm32Connected && stats.totalBinaryPackets == 0 && stats.textMessages == 0) {
            Serial.println("❌ No communication detected. Check:");
            Serial.println("  STM32 PA9(TX) → ESP8266 D2");
            Serial.println("  STM32 GND → ESP8266 GND");
            Serial.println("  STM32 powered and running");
            Serial.println("  Baudrate: 9600");
            Serial.println("  Binary protocol v1.1");
        }
        
        Serial.println("Free heap: " + String(ESP.getFreeHeap()));
        Serial.println("---------------------------------------\n");
    }
}