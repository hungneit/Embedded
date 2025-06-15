#include "stm32f1xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// ==== Cấu hình ngưỡng gas ====
#define GAS_SAFE_THRESHOLD     400
#define GAS_WARNING_THRESHOLD  1000
#define GAS_DANGER_THRESHOLD   2000

// ==== Cấu hình relay thông minh ====
#define RELAY_ON_THRESHOLD     1200
#define RELAY_OFF_THRESHOLD    800
#define RELAY_MIN_ON_TIME      5000  // 5s
#define RELAY_MIN_OFF_TIME     3000  // 3s

// ==== Binary Protocol ====
#define PACKET_START_MARKER 0x55
#define PACKET_HEADER       0xAA
#define PACKET_END_MARKER   0x33
#define PACKET_SIZE         10

// Packet structure (10 bytes với markers)
typedef struct {
    uint8_t start_marker;   // 0x55
    uint8_t header;         // 0xAA - sync byte
    uint16_t gas_value;     // ADC value 0-4095
    uint8_t flags;          // bit0-1: gas_level, bit2: system_state, bit3: relay_state
    uint8_t sequence;       // Message counter 0-255
    uint8_t reserved;       // For future use
    uint16_t checksum;      // CRC16 of packet (excluding markers)
    uint8_t end_marker;     // 0x33
} __attribute__((packed)) sensor_packet_t;

// ==== Function prototypes ====
void SystemInit72MHz(void);
void GPIO_Config(void);
void ADC_Config(void);
void uart_init(void);
void timer_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint16_t Read_MQ2_Fast(void);
uint8_t Read_Button_Fast(uint8_t pin);
void uart_send_text(char* str);
void uart_send_bytes_safe(uint8_t* data, uint8_t len);
void Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue);
uint8_t Get_Gas_Level(uint16_t gas_value);
uint16_t calculate_crc16(uint8_t* data, uint8_t len);
void send_binary_packet_safe(uint16_t gas, uint8_t gas_level, uint8_t system_state, uint8_t relay_state);
void send_text_message(char* msg);
void Update_Relay_Smart(uint16_t gas_value);

// ==== Biến toàn cục ====
volatile uint8_t system_state = 0; // 0: STOPPED, 1: ACTIVE
volatile uint32_t loop_counter = 0;
volatile uint32_t blink_counter = 0;
volatile uint32_t fast_blink_counter = 0;

// Relay control
uint8_t relay_state = 0;
uint32_t relay_last_change_time = 0;

// Data tracking
uint16_t last_sent_gas = 0;
uint8_t last_sent_state = 255;
uint8_t packet_sequence = 0;

// ==== System Clock 72MHz ====
void SystemInit72MHz(void) {
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

// ==== Timer delay ====
void timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 72 - 1;    // 1MHz timer
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void delay_us(uint32_t us) {
    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

// ==== UART Config - 9600 baud ====
void uart_init(void) {
    // Bật clock cho GPIOA, USART1, và AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;

    // Cấu hình PA9 (TX) - Alternate function push-pull
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= (0x0B << 4);  // 1011 = Alt func push-pull, 50MHz

    // Cấu hình UART: 9600 baud
    USART1->BRR = 72000000 / 9600;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;  // TX enable + UART enable
}

// ==== UART Send Functions ====
void uart_send_text(char* str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *str++;
    }
    // Wait for transmission complete
    while (!(USART1->SR & USART_SR_TC));
}

void uart_send_bytes_safe(uint8_t* data, uint8_t len) {
    // Wait for UART to be completely ready
    while (!(USART1->SR & USART_SR_TC));
    delay_us(50); // Small pre-delay

    for (uint8_t i = 0; i < len; i++) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = data[i];

        // Small delay between bytes for stability at 9600 baud
        delay_us(10);
    }

    // Wait for final transmission complete
    while (!(USART1->SR & USART_SR_TC));
    delay_us(100); // Post-transmission delay
}

// ==== Text Message Sending ====
void send_text_message(char* msg) {
    uart_send_text("TXT:");
    uart_send_text(msg);
    uart_send_text("\r\n");
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

// ==== Binary Packet Sending ====
void send_binary_packet_safe(uint16_t gas, uint8_t gas_level, uint8_t system_state, uint8_t relay_state) {
    sensor_packet_t packet;

    // Fill packet data
    packet.start_marker = PACKET_START_MARKER;
    packet.header = PACKET_HEADER;
    packet.gas_value = gas;
    packet.flags = (gas_level & 0x03) | ((system_state & 0x01) << 2) | ((relay_state & 0x01) << 3);
    packet.sequence = packet_sequence++;
    packet.reserved = 0x00;
    packet.end_marker = PACKET_END_MARKER;

    // Calculate CRC16 (exclude markers and checksum)
    uint8_t* crc_data = ((uint8_t*)&packet) + 1; // Skip start marker
    packet.checksum = calculate_crc16(crc_data, sizeof(packet) - 4); // Exclude markers and checksum

    // Send packet with improved timing
    uart_send_bytes_safe((uint8_t*)&packet, sizeof(packet));
}

// ==== Cấu hình GPIO ====
void GPIO_Config(void) {
    // Bật clock GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // PA0 - Input analog cho MQ2
    GPIOA->CRL &= ~(0xF << (0 * 4));
    GPIOA->CRL |= (0x0 << (0 * 4));  // Analog input

    // PA1=RED, PA2=GREEN, PA3=BLUE, PA4=BUZZER, PA5=RELAY - Output push-pull
    GPIOA->CRL &= ~((0xF << (1 * 4)) | (0xF << (2 * 4)) | (0xF << (3 * 4)) |
                    (0xF << (4 * 4)) | (0xF << (5 * 4)));
    GPIOA->CRL |=  ((0x2 << (1 * 4)) | (0x2 << (2 * 4)) | (0x2 << (3 * 4)) |
                    (0x2 << (4 * 4)) | (0x2 << (5 * 4)));

    // PA6, PA7 - Input pull-up (SW1, SW2)
    GPIOA->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOA->CRL |=  ((0x8 << (6 * 4)) | (0x8 << (7 * 4)));
    GPIOA->ODR |= (1 << 6) | (1 << 7); // Enable pull-up

    // Tắt tất cả output ban đầu
    GPIOA->ODR &= ~((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5));
}

// ==== Cấu hình ADC1 channel 0 (PA0) ====
void ADC_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    // Bật clock ADC1
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;         // Clear ADC prescaler
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;     // PCLK2 / 6 = 12MHz

    // Cấu hình ADC
    ADC1->CR2 = 0;
    ADC1->CR1 = 0;
    ADC1->SQR1 = 0;        // 1 conversion
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0;        // Channel 0
    ADC1->SMPR2 = 0x7;     // 239.5 cycles sampling time for channel 0

    ADC1->CR2 |= ADC_CR2_ADON;             // Bật ADC
    delay_ms(2);                           // Đợi ADC ổn định

    ADC1->CR2 |= ADC_CR2_CAL;              // Bắt đầu hiệu chuẩn
    while (ADC1->CR2 & ADC_CR2_CAL);       // Đợi hiệu chuẩn xong
}

// ==== Faster Gas Reading with Averaging ====
uint16_t Read_MQ2_Fast(void) {
    uint32_t sum = 0;

    // Take 3 quick readings and average
    for(int i = 0; i < 3; i++) {
        ADC1->SQR3 = 0;                        // Channel 0 = PA0
        ADC1->CR2 |= ADC_CR2_ADON;             // Start conversion
        while (!(ADC1->SR & ADC_SR_EOC));      // Wait for conversion
        sum += ADC1->DR;
        delay_us(100); // Small delay between readings
    }

    return sum / 3; // Return average
}

// ==== Faster Button Reading ====
uint8_t Read_Button_Fast(uint8_t pin) {
    if ((GPIOA->IDR & (1 << pin)) == 0) { // Button pressed
        delay_ms(10); // Reduced debounce time
        if ((GPIOA->IDR & (1 << pin)) == 0) { // Confirm press
            while ((GPIOA->IDR & (1 << pin)) == 0); // Wait for release
            delay_ms(10); // Reduced release debounce
            return 1; // Valid button press
        }
    }
    return 0; // No button press
}

// ==== Phân loại mức độ gas ====
uint8_t Get_Gas_Level(uint16_t gas_value) {
    if (gas_value < GAS_SAFE_THRESHOLD) return 0;      // SAFE
    if (gas_value < GAS_WARNING_THRESHOLD) return 1;   // WARNING
    if (gas_value < GAS_DANGER_THRESHOLD) return 2;    // DANGER
    return 3;                                          // CRITICAL
}

// ==== Điều khiển LED RGB ====
void Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue) {
    // Reset tất cả màu
    GPIOA->ODR &= ~((1 << 1) | (1 << 2) | (1 << 3));

    // Set màu theo yêu cầu (active high)
    if (red)   GPIOA->ODR |= (1 << 1);   // PA1 = RED
    if (green) GPIOA->ODR |= (1 << 2);   // PA2 = GREEN
    if (blue)  GPIOA->ODR |= (1 << 3);   // PA3 = BLUE
}

// ==== Relay thông minh ====
void Update_Relay_Smart(uint16_t gas_value) {
    uint32_t time_since_change = loop_counter - relay_last_change_time;

    if (relay_state == 0) {
        // Relay OFF - chỉ bật khi gas cao và đã đủ thời gian OFF
        if (gas_value > RELAY_ON_THRESHOLD && time_since_change > RELAY_MIN_OFF_TIME) {
            relay_state = 1;
            GPIOA->ODR |= (1 << 5);  // Bật relay
            relay_last_change_time = loop_counter;
        }
    } else {
        // Relay ON - chỉ tắt khi gas thấp và đã đủ thời gian ON
        if (gas_value < RELAY_OFF_THRESHOLD && time_since_change > RELAY_MIN_ON_TIME) {
            relay_state = 0;
            GPIOA->ODR &= ~(1 << 5); // Tắt relay
            relay_last_change_time = loop_counter;
        }
    }
}

// ==== Main chính ====
int main(void) {
    // Khởi tạo system
    SystemInit72MHz();
    timer_init();

    // Khởi tạo các module
    GPIO_Config();
    ADC_Config();
    uart_init();

    // Khởi động
    delay_ms(1000);  // Reduced startup delay

    // Gửi thông báo khởi động
    send_text_message("STM32_START_BINARY_V1.1_IMPROVED");

    // Test LED - faster blink
    for(int i = 0; i < 3; i++) {
        Set_RGB_LED(0, 1, 0); // GREEN
        delay_ms(200);
        Set_RGB_LED(0, 0, 0); // OFF
        delay_ms(200);
    }

    send_text_message("READY_BINARY_PROTOCOL_IMPROVED");

    while (1) {
        loop_counter++;

        // ✅ FASTER GAS READING
        uint16_t gas = Read_MQ2_Fast();
        uint8_t gas_level = Get_Gas_Level(gas);

        // ✅ FASTER BUTTON HANDLING
        if (Read_Button_Fast(6)) {
            system_state = !system_state;

            // IMMEDIATE send binary packet for button press
            send_binary_packet_safe(gas, gas_level, system_state, relay_state);

            char btn_msg[40];
            sprintf(btn_msg, "BTN_SW1_STATE_%s_IMMEDIATE", system_state ? "ACTIVE" : "STOPPED");
            send_text_message(btn_msg);
        }

        if (Read_Button_Fast(7)) {
            system_state = 0;

            // IMMEDIATE send for reset
            send_binary_packet_safe(gas, gas_level, system_state, relay_state);
            send_text_message("BTN_SW2_RESET_IMMEDIATE");
        }

        if (system_state) {
            // ==== HỆ THỐNG HOẠT ĐỘNG ====
            Update_Relay_Smart(gas);

            // LED and buzzer control
            if (gas < GAS_SAFE_THRESHOLD) {
                Set_RGB_LED(0, 0, 1);
                GPIOA->ODR &= ~(1 << 4);
            } else if (gas < GAS_WARNING_THRESHOLD) {
                Set_RGB_LED(1, 1, 0);
                GPIOA->ODR &= ~(1 << 4);
            } else if (gas < GAS_DANGER_THRESHOLD) {
                blink_counter++;
                if (blink_counter >= 1000) {
                    blink_counter = 0;
                    static uint8_t blink_state = 0;
                    if (blink_state) {
                        Set_RGB_LED(1, 0, 0);
                    } else {
                        Set_RGB_LED(0, 0, 0);
                    }
                    blink_state = !blink_state;
                }
                GPIOA->ODR |= (1 << 4);
            } else {
                fast_blink_counter++;
                if (fast_blink_counter >= 200) {
                    fast_blink_counter = 0;
                    static uint8_t fast_state = 0;
                    if (fast_state) {
                        Set_RGB_LED(1, 0, 0);
                    } else {
                        Set_RGB_LED(0, 0, 0);
                    }
                    fast_state = !fast_state;
                }
                GPIOA->ODR |= (1 << 4);
            }

        } else {
            // ==== HỆ THỐNG DỪNG ====
            Set_RGB_LED(0, 1, 0);
            GPIOA->ODR &= ~(1 << 4);
            GPIOA->ODR &= ~(1 << 5);
            relay_state = 0;
            blink_counter = 0;
            fast_blink_counter = 0;
        }

        // ✅ IMPROVED SEND LOGIC - MORE AGGRESSIVE
        uint8_t should_send = 0;

        // 1. IMMEDIATE SEND conditions
        if (system_state != last_sent_state) {
            should_send = 1;
            send_text_message("IMMEDIATE_SEND_SYSTEM_STATE_CHANGED");
        } else {
            uint8_t current_level = Get_Gas_Level(gas);
            uint8_t last_level = Get_Gas_Level(last_sent_gas);

            // IMMEDIATE send for gas changes
            if (last_level == 0 && current_level > 0) {
                should_send = 1;
                send_text_message("IMMEDIATE_SEND_GAS_DETECTED");
            }
            // Lower threshold for immediate send (30 instead of 50)
            else if (abs((int)gas - (int)last_sent_gas) > 30) {
                should_send = 1;
                send_text_message("IMMEDIATE_SEND_GAS_CHANGED_SIGNIFICANT");
            }
            else if (current_level != last_level) {
                should_send = 1;
                send_text_message("IMMEDIATE_SEND_GAS_LEVEL_CHANGED");
            }
        }

        // 2. FASTER interval sending
        if (!should_send) {
            uint32_t send_interval;
            if (system_state == 0) {
                send_interval = 20000; // Reduced from 30s to 20s
            } else {
                switch(gas_level) {
                    case 0: send_interval = 10000; break; // Reduced from 20s to 10s - SAFE
                    case 1: send_interval = 5000;  break; // Reduced from 8s to 5s - WARNING
                    case 2: send_interval = 3000;  break; // Reduced from 5s to 3s - DANGER
                    case 3: send_interval = 2000;  break; // Reduced from 3s to 2s - CRITICAL
                    default: send_interval = 10000; break;
                }
            }

            static uint32_t last_send_time = 0;
            if (loop_counter - last_send_time >= send_interval) {
                should_send = 1;
                last_send_time = loop_counter;
            }
        }

        // ✅ SEND BINARY PACKET WITH IMPROVED FUNCTION
        if (should_send) {
            send_binary_packet_safe(gas, gas_level, system_state, relay_state);

            last_sent_gas = gas;
            last_sent_state = system_state;
        }

        // ✅ FASTER HEARTBEAT
        uint32_t heartbeat_interval = system_state ? 30000 : 60000; // Faster heartbeat
        if (loop_counter % heartbeat_interval == 0) {
            char hb_msg[60];
            sprintf(hb_msg, "HEARTBEAT_STATUS_%s_UPTIME_%lu_GAS_%d",
                    system_state ? "ACTIVE" : "STOPPED", loop_counter/1000, gas);
            send_text_message(hb_msg);
        }

        delay_ms(1);  // Keep 1ms loop delay
    }
}
