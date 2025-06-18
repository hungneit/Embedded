// ========== HIGHLY OPTIMIZED ESP8266 WITH ROBUST BINARY PROTOCOL ==========

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

// ==== ROBUST Binary Protocol với timeout handling ====
#define PACKET_START_MARKER 0x55
#define PACKET_HEADER       0xAA  
#define PACKET_END_MARKER   0x33
#define PACKET_SIZE         10
#define STM32_TIMEOUT       120000
#define PACKET_TIMEOUT      100    // Timeout cho incomplete packets (ms)
#define SYNC_BUFFER_SIZE    20     // Buffer để tìm sync pattern

// Improved packet structure
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

// ==== Advanced Binary Parser State Machine ====
typedef enum {
    PARSER_IDLE,
    PARSER_SYNC_FOUND,
    PARSER_BUILDING_PACKET,
    PARSER_PACKET_COMPLETE
} parser_state_t;

typedef struct {
    parser_state_t state;
    uint8_t buffer[PACKET_SIZE];
    uint8_t index;
    uint8_t sync_buffer[SYNC_BUFFER_SIZE];
    uint8_t sync_index;
    unsigned long packet_start_time;
    unsigned long last_byte_time;
} binary_parser_t;

binary_parser_t parser = {PARSER_IDLE, {0}, 0, {0}, 0, 0, 0};

// ==== Sensor Data với Power Save Mode ====
struct {
    int gasValue;
    String gasStatus;
    String systemStatus;
    String relayStatus;
    unsigned long lastUpdateTime;
    bool hasData;
    bool forceNextSend;
    uint8_t lastSequence;
    bool systemActive;       // NEW: Track STM32 system state
    bool powerSaveMode;      // NEW: Track if STM32 in power save
} sensor = {0, "UNKNOWN", "UNKNOWN", "UNKNOWN", 0, false, false, 255, false, false};

// ==== Enhanced stats với error tracking ====
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
    int incompletePackets;    // NEW
    int syncLostCount;        // NEW
    int timeoutCount;         // NEW
    int markerMismatchCount;  // NEW
} stats = {0, 0, 0, 0, false, false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// ==== Function prototypes ====
uint16_t calculate_crc16(uint8_t* data, uint8_t len);
bool parse_binary_packet_robust(uint8_t* buffer);
String getGasStatusString(uint8_t level);
void processTextMessage(String msg);
void reset_binary_parser(void);
bool find_sync_pattern(uint8_t byte);
void handle_incomplete_packet(void);

void setup() {
    Serial.begin(115200);
    stm32Serial.begin(9600);
    delay(1000);
    
    stats.bootTime = millis();
    reset_binary_parser();
    
    Serial.println("=== ESP8266 Optimized Gateway v5.0 ===");
    Serial.println("Device: " + String(deviceID));
    Serial.println("Features: Power Save + Robust Binary Protocol");
    Serial.println("STM32: 9600 baud, Sync Detection + Timeout Handling");
    
    connectWiFi();
    
    Serial.println("Ready. Optimized for power efficiency...");
    Serial.println("Expected connection:");
    Serial.println("  STM32 PA9(TX) → ESP8266 D2(GPIO4)");
    Serial.println("  STM32 GND → ESP8266 GND");
    Serial.println("Binary Protocol: v2.0 with robust sync detection");
    Serial.println("========================================");
}

void loop() {
    readSTM32_Robust();
    checkSTM32Health();
    sendToServer_Optimized();
    printStatus_Enhanced();
    
    delay(10); // Optimized delay
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

// ==== CRC16 Calculation ====
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

// ==== Binary Parser Reset ====
void reset_binary_parser(void) {
    parser.state = PARSER_IDLE;
    parser.index = 0;
    parser.sync_index = 0;
    parser.packet_start_time = 0;
    parser.last_byte_time = 0;
    memset(parser.buffer, 0, PACKET_SIZE);
    memset(parser.sync_buffer, 0, SYNC_BUFFER_SIZE);
}

// ==== Sync Pattern Detection ====
bool find_sync_pattern(uint8_t byte) {
    // Shift sync buffer and add new byte
    for (int i = 0; i < SYNC_BUFFER_SIZE - 1; i++) {
        parser.sync_buffer[i] = parser.sync_buffer[i + 1];
    }
    parser.sync_buffer[SYNC_BUFFER_SIZE - 1] = byte;
    
    // Look for start marker + header pattern
    for (int i = 0; i < SYNC_BUFFER_SIZE - 1; i++) {
        if (parser.sync_buffer[i] == PACKET_START_MARKER && 
            parser.sync_buffer[i + 1] == PACKET_HEADER) {
            
            // Found sync pattern, start building packet
            parser.buffer[0] = PACKET_START_MARKER;
            parser.buffer[1] = PACKET_HEADER;
            parser.index = 2;
            parser.state = PARSER_BUILDING_PACKET;
            parser.packet_start_time = millis();
            
            Serial.println("🔄 Sync pattern found, starting packet build");
            return true;
        }
    }
    return false;
}

// ==== Handle Incomplete Packet ====
void handle_incomplete_packet(void) {
    Serial.println("⚠️ Incomplete packet timeout (had " + String(parser.index) + 
                   " bytes), searching for new sync...");
    
    stats.incompletePackets++;
    stats.timeoutCount++;
    
    // Try to recover by looking for sync in current buffer
    bool found_sync = false;
    for (int i = 1; i < parser.index - 1; i++) {
        if (parser.buffer[i] == PACKET_START_MARKER && 
            parser.buffer[i + 1] == PACKET_HEADER) {
            
            // Found potential new packet start
            Serial.println("🔄 Recovery: Found sync at position " + String(i));
            
            // Move data to beginning
            for (int j = 0; j < parser.index - i; j++) {
                parser.buffer[j] = parser.buffer[i + j];
            }
            parser.index = parser.index - i;
            parser.packet_start_time = millis();
            found_sync = true;
            break;
        }
    }
    
    if (!found_sync) {
        reset_binary_parser();
        stats.syncLostCount++;
    }
}

// ==== ROBUST Binary Packet Parser ====
bool parse_binary_packet_robust(uint8_t* buffer) {
    sensor_packet_t* packet = (sensor_packet_t*)buffer;
    
    // Comprehensive validation
    if (packet->start_marker != PACKET_START_MARKER) {
        Serial.println("❌ Invalid start marker: 0x" + String(packet->start_marker, HEX));
        stats.markerMismatchCount++;
        return false;
    }
    
    if (packet->end_marker != PACKET_END_MARKER) {
        Serial.println("❌ Invalid end marker: 0x" + String(packet->end_marker, HEX) + 
                      " (expected 0x" + String(PACKET_END_MARKER, HEX) + ")");
        stats.markerMismatchCount++;
        return false;
    }
    
    if (packet->header != PACKET_HEADER) {
        Serial.println("❌ Invalid header: 0x" + String(packet->header, HEX));
        stats.invalidHeaders++;
        return false;
    }
    
    // CRC verification
    uint8_t* crc_data = buffer + 1;
    uint16_t calculated_crc = calculate_crc16(crc_data, PACKET_SIZE - 4);
    
    if (calculated_crc != packet->checksum) {
        Serial.println("❌ CRC error - Expected: 0x" + String(packet->checksum, HEX) + 
                      ", Calculated: 0x" + String(calculated_crc, HEX));
        stats.crcErrors++;
        return false;
    }
    
    // Duplicate check
    if (packet->sequence == sensor.lastSequence) {
        Serial.println("⚠️ Duplicate packet #" + String(packet->sequence) + " ignored");
        stats.duplicatePackets++;
        return false;
    }
    
    // Data extraction and validation
    uint16_t newGas = packet->gas_value;
    uint8_t gasLevel = packet->flags & 0x03;
    uint8_t systemState = (packet->flags >> 2) & 0x01;
    uint8_t relayState = (packet->flags >> 3) & 0x01;
    
    if (newGas > 4095) {
        Serial.println("❌ Invalid gas value: " + String(newGas));
        stats.binaryErrors++;
        return false;
    }
    
    // ==== POWER SAVE MODE DETECTION - Improved ====
    bool wasActive = sensor.systemActive;
    bool wasPowerSave = sensor.powerSaveMode;
    
    sensor.systemActive = systemState;
    
    // Chỉ update power save mode khi có thay đổi thực sự
    if (wasActive != systemState) {
        if (wasActive && !systemState) {
            Serial.println("🔋 STM32 entered POWER SAVE mode");
            sensor.powerSaveMode = true;
            sensor.forceNextSend = true;
        } else if (!wasActive && systemState) {
            Serial.println("⚡ STM32 exited POWER SAVE mode");
            sensor.powerSaveMode = false;
            sensor.forceNextSend = true;
        }
    }
    
    // Đảm bảo power save mode đúng với system state
    sensor.powerSaveMode = !systemState;
    
    // Change detection for immediate send
    bool gasChanged = abs((int)newGas - sensor.gasValue) >= 30;
    bool statusChanged = (getGasStatusString(gasLevel) != sensor.gasStatus);
    bool systemChanged = (sensor.systemStatus != String(systemState ? "ACTIVE" : "STOPPED"));
    bool relayChanged = (sensor.relayStatus != String(relayState ? "ON" : "OFF"));
    
    if (systemState) { // Only check gas changes when system is active
        bool hasGasDanger = (gasLevel >= 1);
        bool wasNoGas = (sensor.gasStatus == "SAFE" || sensor.gasStatus == "UNKNOWN");
        bool newGasDanger = hasGasDanger && wasNoGas;
        
        if (newGasDanger || gasChanged || statusChanged || systemChanged || relayChanged) {
            Serial.println("🚨 IMMEDIATE SEND triggered!");
            sensor.forceNextSend = true;
        }
    }
    
    // Update sensor data
    sensor.gasValue = newGas;
    sensor.gasStatus = getGasStatusString(gasLevel);
    sensor.systemStatus = systemState ? "ACTIVE" : "STOPPED";
    sensor.relayStatus = relayState ? "ON" : "OFF";
    sensor.lastUpdateTime = millis();
    sensor.hasData = true;
    sensor.lastSequence = packet->sequence;
    
    stats.stm32SystemActive = systemState;
    stats.validPackets++;
    
    Serial.println("✅ Binary packet #" + String(packet->sequence) + 
                   ": Gas=" + String(newGas) + " (" + sensor.gasStatus + 
                   "), Sys=" + sensor.systemStatus + ", Relay=" + sensor.relayStatus +
                   " [CRC:OK]");
    
    return true;
}

// ==== ROBUST STM32 Data Reader với State Machine ====
void readSTM32_Robust() {
    unsigned long now = millis();
    
    while (stm32Serial.available()) {
        uint8_t incomingByte = stm32Serial.read();
        parser.last_byte_time = now;
        
        switch (parser.state) {
            case PARSER_IDLE:
                // Look for sync pattern
                if (find_sync_pattern(incomingByte)) {
                    // Sync found, state already changed to BUILDING
                } else {
                    // Check if it's a text message
                    char c = (char)incomingByte;
                    static String textBuffer = "";
                    
                    if (c == '\n') {
                        if (textBuffer.length() > 0) {
                            textBuffer.trim();
                            if (textBuffer.startsWith("TXT:")) {
                                textBuffer = textBuffer.substring(4);
                                processTextMessage(textBuffer);
                            }
                            textBuffer = "";
                        }
                    } else if (c != '\r' && c >= 10 && c <= 127) {
                        textBuffer += c;
                        if (textBuffer.length() > 200) {
                            textBuffer = "";
                        }
                    }
                }
                break;
                
            case PARSER_BUILDING_PACKET:
                parser.buffer[parser.index++] = incomingByte;
                
                if (parser.index >= PACKET_SIZE) {
                    // Complete packet received
                    parser.state = PARSER_PACKET_COMPLETE;
                    stats.totalBinaryPackets++;
                    stats.lastSTM32Contact = now;
                    stats.stm32Connected = true;
                    
                    if (parse_binary_packet_robust(parser.buffer)) {
                        // Successfully parsed
                    } else {
                        stats.binaryErrors++;
                    }
                    
                    reset_binary_parser();
                }
                break;
                
            default:
                reset_binary_parser();
                break;
        }
    }
    
    // Timeout handling for incomplete packets
    if (parser.state == PARSER_BUILDING_PACKET && 
        (now - parser.packet_start_time) > PACKET_TIMEOUT) {
        handle_incomplete_packet();
    }
}

// ==== Text Message Processor ====
void processTextMessage(String msg) {
    if (msg.length() == 0) return;
    
    stats.textMessages++;
    stats.lastSTM32Contact = millis();
    stats.stm32Connected = true;
    
    Serial.println("[TXT:" + String(stats.textMessages) + "] " + msg);
    
    if (msg.startsWith("HEARTBEAT")) {
        stats.heartbeatCount++;
        parseHeartbeat(msg);
        
        // POWER SAVE detection từ heartbeat - chỉ khi có POWER_SAVE keyword
        if (msg.indexOf("POWER_SAVE") != -1 && !sensor.powerSaveMode) {
            sensor.powerSaveMode = true;
            Serial.println("🔋 Power save mode detected from heartbeat");
            sensor.forceNextSend = true;
        }
    } else if (msg.startsWith("BTN_")) {
        stats.buttonEvents++;
        Serial.println("  Button event detected");
        sensor.forceNextSend = true;
    } else if (msg.indexOf("START") != -1 || msg.indexOf("READY") != -1) {
        Serial.println("  STM32 startup detected");
        sensor.forceNextSend = true;
        sensor.powerSaveMode = false;
    } else if (msg.indexOf("STOPPED") != -1) {
        Serial.println("  STM32 stopped detected");
        sensor.powerSaveMode = true;
        sensor.forceNextSend = true;
    }
}

void parseHeartbeat(String msg) {
    stats.lastHeartbeat = millis();
    
    bool wasActive = stats.stm32SystemActive;
    bool wasPowerSave = sensor.powerSaveMode;
    
    if (msg.indexOf("ACTIVE") != -1) {
        stats.stm32SystemActive = true;
        sensor.systemStatus = "ACTIVE";
        if (wasPowerSave) {
            sensor.powerSaveMode = false;
            Serial.println("⚡ System activated via heartbeat");
            sensor.forceNextSend = true;
        }
    } else if (msg.indexOf("STOPPED") != -1) {
        stats.stm32SystemActive = false;
        sensor.systemStatus = "STOPPED";
        if (!wasPowerSave) {
            sensor.powerSaveMode = true;
            Serial.println("🔋 System stopped via heartbeat");
            sensor.forceNextSend = true;
        }
    }
    
    // Chỉ force send khi có thay đổi thực sự
    if (wasActive != stats.stm32SystemActive) {
        sensor.forceNextSend = true;
    }
    
    Serial.println("  Heartbeat: " + sensor.systemStatus + 
                   (sensor.powerSaveMode ? " (Power Save)" : " (Active)"));
}

void checkSTM32Health() {
    unsigned long now = millis();
    
    if (stats.stm32Connected && (now - stats.lastSTM32Contact) > STM32_TIMEOUT) {
        stats.stm32Connected = false;
        stats.stm32SystemActive = false;
        sensor.systemStatus = "OFFLINE";
        sensor.powerSaveMode = false;
        sensor.forceNextSend = true;
        
        Serial.println("STM32 OFFLINE! (" + String((now - stats.lastSTM32Contact)/1000) + "s)");
    }
}

// ==== OPTIMIZED Server Sending với Power Save ====
void sendToServer_Optimized() {
    if (!stats.wifiConnected || WiFi.status() != WL_CONNECTED) {
        return;
    }

    unsigned long now = millis();
    bool shouldSend = false;
    static bool powerSaveMessageShown = false;  // NEW: Prevent spam

    // Force send conditions
    if (sensor.forceNextSend) {
        shouldSend = true;
        Serial.println("🚨 FORCE SEND - State change or critical event!");
        powerSaveMessageShown = false;  // Reset flag on force send
    } else {
        unsigned long interval;

        if (!stats.stm32Connected) {
            interval = 60000; // STM32 offline
            powerSaveMessageShown = false;  // Reset when offline
        } else if (sensor.powerSaveMode || !stats.stm32SystemActive) {
            // ==== POWER SAVE MODE: Gửi ít hơn ====
            interval = 180000; // 3 phút khi power save
            
            // Chỉ hiển thị thông báo power save 1 lần
            if (!powerSaveMessageShown) {
                Serial.println("🔋 Entering power save mode - reduced sending frequency (3min intervals)");
                powerSaveMessageShown = true;
            }
        } else {
            // Active mode - gửi theo gas level
            if (sensor.gasStatus == "CRITICAL") {
                interval = 2000;
            } else if (sensor.gasStatus == "DANGER") {
                interval = 3000;
            } else if (sensor.gasStatus == "WARNING") {
                interval = 5000;
            } else {
                interval = 10000;
            }
            
            // Reset power save message when returning to active
            if (powerSaveMessageShown) {
                Serial.println("⚡ Exiting power save mode - resuming normal frequency");
                powerSaveMessageShown = false;
            }
        }

        if (now - stats.lastServerSend >= interval) {
            shouldSend = true;
            
            // Chỉ hiển thị loại interval send khi KHÔNG phải power save
            if (!sensor.powerSaveMode && stats.stm32SystemActive) {
                Serial.println("⏰ Interval send (" + String(interval / 1000) + "s) - " + sensor.gasStatus);
            } else if (sensor.powerSaveMode || !stats.stm32SystemActive) {
                Serial.println("⏰ Power save interval send (" + String(interval / 1000) + "s)");
            }
        }
    }

    if (!shouldSend) return;

    Serial.println("📤 Sending to server...");

    DynamicJsonDocument doc(1024);

    doc["device_id"] = deviceID;
    doc["timestamp"] = now;
    doc["uptime"] = (now - stats.bootTime) / 1000;
    doc["stm32_connected"] = stats.stm32Connected;
    doc["protocol"] = "optimized_v2.0";
    doc["power_save_mode"] = sensor.powerSaveMode;  // NEW

    if (stats.stm32Connected) {
        if (sensor.powerSaveMode || !stats.stm32SystemActive) {
            // Power save mode
            doc["gas_value"] = -1;
            doc["gas_status"] = "POWER_SAVE";
            doc["system_status"] = "POWER_SAVE";
            doc["relay_status"] = "OFF";
            doc["message"] = "Gas sensor in power save mode";
            doc["mode"] = "ENERGY_EFFICIENT";
        } else if (stats.stm32SystemActive && sensor.hasData) {
            // Active mode with gas data
            doc["gas_value"] = sensor.gasValue;
            doc["gas_status"] = sensor.gasStatus;
            doc["system_status"] = "ACTIVE";
            doc["relay_status"] = sensor.relayStatus;
            doc["mode"] = "MONITORING";

            if (sensor.gasStatus != "SAFE") {
                doc["urgent"] = true;
                doc["alert_level"] = sensor.gasStatus;
            }
        } else {
            // Initializing
            doc["gas_value"] = -1;
            doc["gas_status"] = "INITIALIZING";
            doc["system_status"] = "INITIALIZING";
            doc["relay_status"] = "UNKNOWN";
            doc["mode"] = "STARTUP";
        }
    } else {
        // STM32 offline
        doc["gas_value"] = -1;
        doc["gas_status"] = "OFFLINE";
        doc["system_status"] = "OFFLINE";
        doc["relay_status"] = "UNKNOWN";
        doc["error"] = "STM32_NOT_RESPONDING";
        doc["message"] = "Gas sensor module offline";
        doc["urgent"] = true;
        doc["mode"] = "ERROR";
    }

    // Enhanced statistics
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
    doc["incomplete_packets"] = stats.incompletePackets;
    doc["sync_lost_count"] = stats.syncLostCount;
    doc["timeout_count"] = stats.timeoutCount;
    doc["marker_mismatch_count"] = stats.markerMismatchCount;
    doc["last_sequence"] = sensor.lastSequence;
    
    float successRate = stats.totalBinaryPackets > 0 ?
                        (float)stats.validPackets / stats.totalBinaryPackets * 100.0 : 0.0;
    doc["packet_success_rate"] = successRate;

    if (sensor.forceNextSend) {
        doc["immediate_send"] = true;
    }

    WiFiClientSecure wifiClient;
    wifiClient.setInsecure();

    String jsonString;
    serializeJson(doc, jsonString);
    Serial.println("JSON:\n" + jsonString);

    HTTPClient http;
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

// ==== Enhanced Status Reporting ====
void printStatus_Enhanced() {
    static unsigned long lastReport = 0;
    
    if (millis() - lastReport >= 30000) {
        lastReport = millis();
        
        Serial.println("\n--- OPTIMIZED GATEWAY STATUS v5.0 ---");
        Serial.println("Uptime: " + String((millis() - stats.bootTime)/1000) + "s");
        Serial.println("WiFi: " + String(stats.wifiConnected ? "OK" : "FAIL"));
        Serial.println("STM32: " + String(stats.stm32Connected ? "ONLINE" : "OFFLINE"));
        
        if (stats.stm32Connected) {
            String mode = sensor.powerSaveMode ? "POWER_SAVE" : 
                         (stats.stm32SystemActive ? "ACTIVE" : "STOPPED");
            Serial.println("Mode: " + mode);
        }
        
        // Binary protocol statistics
        float successRate = stats.totalBinaryPackets > 0 ? 
                           (float)stats.validPackets / stats.totalBinaryPackets * 100 : 0;
        
        Serial.println("Binary packets: " + String(stats.totalBinaryPackets) + 
                      " (valid: " + String(stats.validPackets) + 
                      ", success: " + String(successRate, 1) + "%)");
        
        // Enhanced error breakdown
        if (stats.binaryErrors > 0) {
            Serial.println("Error analysis:");
            Serial.println("  CRC errors: " + String(stats.crcErrors));
            Serial.println("  Invalid headers: " + String(stats.invalidHeaders));
            Serial.println("  Incomplete packets: " + String(stats.incompletePackets));
            Serial.println("  Marker mismatches: " + String(stats.markerMismatchCount));
            Serial.println("  Sync lost: " + String(stats.syncLostCount));
            Serial.println("  Timeouts: " + String(stats.timeoutCount));
            Serial.println("  Duplicates: " + String(stats.duplicatePackets) + " (filtered)");
        }
        
        Serial.println("Text messages: " + String(stats.textMessages) + 
                      " (♥️:" + String(stats.heartbeatCount) + 
                      ", BTN:" + String(stats.buttonEvents) + ")");
        
        if (sensor.hasData && stats.stm32SystemActive && !sensor.powerSaveMode) {
            Serial.println("Gas: " + String(sensor.gasValue) + " (" + sensor.gasStatus + ")");
            Serial.println("Relay: " + sensor.relayStatus + ", Last seq: " + String(sensor.lastSequence));
        } else if (sensor.powerSaveMode) {
            Serial.println("🔋 Power save mode active - minimal monitoring");
        }
        
        // Parser state info
        Serial.println("Parser state: " + String(parser.state) + 
                      ", Buffer: " + String(parser.index) + "/" + String(PACKET_SIZE));
        
        // Diagnostic recommendations
        if (stats.binaryErrors > 0) {
            Serial.println("🔧 DIAGNOSTICS:");
            
            if (stats.incompletePackets > stats.validPackets * 0.1) {
                Serial.println("  ⚠️ High incomplete packet rate - check connections");
            }
            
            if (stats.crcErrors > stats.validPackets * 0.05) {
                Serial.println("  ⚠️ High CRC error rate - possible interference");
            }
            
            if (stats.syncLostCount > 5) {
                Serial.println("  ⚠️ Frequent sync loss - check STM32 timing");
            }
            
            if (stats.markerMismatchCount > stats.validPackets * 0.1) {
                Serial.println("  ⚠️ Marker mismatches - possible protocol mismatch");
            }
        }
        
        if (!stats.stm32Connected && stats.totalBinaryPackets == 0 && stats.textMessages == 0) {
            Serial.println("❌ No communication detected. Check:");
            Serial.println("  STM32 PA9(TX) → ESP8266 D2");
            Serial.println("  STM32 GND → ESP8266 GND");
            Serial.println("  STM32 powered and running");
            Serial.println("  Baudrate: 9600");
            Serial.println("  Binary protocol v2.0");
        }
        
        // Power efficiency status
        if (sensor.powerSaveMode) {
            Serial.println("🔋 POWER EFFICIENCY MODE:");
            Serial.println("  STM32: Minimal gas readings");
            Serial.println("  ESP8266: Reduced server updates");
            Serial.println("  Next update: " + String((180000 - (millis() - stats.lastServerSend))/1000) + "s");
        }
        
        Serial.println("Free heap: " + String(ESP.getFreeHeap()));
        Serial.println("--------------------------------------\n");
    }
}