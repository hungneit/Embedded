#include "stm32f1xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// ==== LCD I2C Address ====
#define LCD_I2C_ADDR 0x4E  // 0x27 << 1

// ==== Cấu hình ngưỡng gas ====
#define GAS_SAFE_THRESHOLD     400
#define GAS_WARNING_THRESHOLD  1000
#define GAS_DANGER_THRESHOLD   2000

// ==== Cấu hình relay thông minh ====
#define RELAY_ON_THRESHOLD     1200
#define RELAY_OFF_THRESHOLD    800
#define RELAY_MIN_ON_TIME      5000  // 5s
#define RELAY_MIN_OFF_TIME     3000  // 3s

// ==== IMPROVED Binary Protocol với sync ====
#define PACKET_START_MARKER 0x55
#define PACKET_HEADER       0xAA
#define PACKET_END_MARKER   0x33
#define PACKET_SIZE         10
#define PACKET_SYNC_DELAY   50  // Delay giữa các byte để ESP sync

// Packet structure với improved sync
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
void uart_send_bytes_sync(uint8_t* data, uint8_t len);  // Improved sync version
void Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue);
uint8_t Get_Gas_Level(uint16_t gas_value);
uint16_t calculate_crc16(uint8_t* data, uint8_t len);
void send_binary_packet_sync(uint16_t gas, uint8_t gas_level, uint8_t system_state, uint8_t relay_state);
void send_text_message(char* msg);
void send_heartbeat_only(void);  // NEW: Heartbeat chỉ gửi trạng thái
void Update_Relay_Smart(uint16_t gas_value);

// ==== LCD Function prototypes ====
void I2C1_Init(void);
void I2C1_SendBytes(uint8_t addr, uint8_t *data, uint8_t len);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendString(char *str);
void LCD_Init(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Clear(void);
void LCD_Update_Display(uint16_t gas_value, uint8_t system_state, uint8_t relay_state);
void LCD_Show_Stopped_Screen(void);  // NEW: Màn hình tắt
uint8_t LCD_Should_Update(uint16_t current_gas, uint8_t current_system_state, uint8_t current_relay_state);  // NEW: Smart update logic

// ==== Biến toàn cục ====
volatile uint8_t system_state = 0; // 0: STOPPED, 1: ACTIVE
volatile uint32_t loop_counter = 0;
volatile uint32_t blink_counter = 0;
volatile uint32_t fast_blink_counter = 0;

// Relay control
uint8_t relay_state = 0;
uint32_t relay_last_change_time = 0;

// Data tracking - OPTIMIZED
uint16_t last_sent_gas = 0;
uint8_t last_sent_state = 255;
uint8_t packet_sequence = 0;

// LCD update tracking - OPTIMIZED
uint32_t lcd_update_counter = 0;
uint8_t lcd_needs_update = 0;
uint8_t lcd_stopped_screen_shown = 0;  // NEW: Tracking stopped screen
uint16_t last_lcd_gas_value = 0;       // NEW: Track last gas value shown on LCD
uint32_t last_lcd_update_time = 0;     // NEW: Track last LCD update time

// ==== OPTIMIZED: Chỉ đọc gas khi cần ====
typedef struct {
    uint16_t current_gas;
    uint8_t current_level;
    uint32_t last_read_time;
    uint8_t valid_data;
} gas_data_t;

gas_data_t gas_sensor = {0, 0, 0, 0};

// ==== Delay function ====
void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        for(uint32_t j = 0; j < 8000; j++) {
            __NOP();
        }
    }
}

// ==== I2C1 Init for PB6(SCL), PB7(SDA) ====
void I2C1_Init(void) {
    RCC->APB1ENR |= (1 << 21);  // I2C1 enable
    RCC->APB2ENR |= (1 << 3);   // GPIOB enable

    GPIOB->CRL &= ~(0xF << 24);
    GPIOB->CRL |= (0xF << 24);   // 1111: AF open-drain 50MHz

    GPIOB->CRL &= ~(0xF << 28);
    GPIOB->CRL |= (0xF << 28);   // 1111: AF open-drain 50MHz

    I2C1->CR1 = 0;               // Reset I2C1
    I2C1->CR2 = 36;              // APB1 = 36MHz
    I2C1->CCR = 180;             // 100kHz: 36MHz/(2*100kHz) = 180
    I2C1->TRISE = 37;            // Rise time
    I2C1->CR1 |= (1 << 0);       // Enable I2C1
}

// ==== I2C Send Bytes ====
void I2C1_SendBytes(uint8_t addr, uint8_t *data, uint8_t len) {
    while(I2C1->SR2 & (1 << 1));

    I2C1->CR1 |= (1 << 8);
    while(!(I2C1->SR1 & (1 << 0)));

    I2C1->DR = addr;
    while(!(I2C1->SR1 & (1 << 1)));
    (void)I2C1->SR2;

    for(uint8_t i = 0; i < len; i++) {
        while(!(I2C1->SR1 & (1 << 7)));
        I2C1->DR = data[i];
    }

    while(!(I2C1->SR1 & (1 << 2)));
    I2C1->CR1 |= (1 << 9);
}

// ==== LCD Functions ====
void LCD_SendCommand(uint8_t cmd) {
    uint8_t data[4];
    data[0] = (cmd & 0xF0) | 0x0C;
    data[1] = (cmd & 0xF0) | 0x08;
    data[2] = ((cmd << 4) & 0xF0) | 0x0C;
    data[3] = ((cmd << 4) & 0xF0) | 0x08;
    I2C1_SendBytes(LCD_I2C_ADDR, data, 4);
    delay_ms(2);
}

void LCD_SendData(uint8_t data) {
    uint8_t buf[4];
    buf[0] = (data & 0xF0) | 0x0D;
    buf[1] = (data & 0xF0) | 0x09;
    buf[2] = ((data << 4) & 0xF0) | 0x0D;
    buf[3] = ((data << 4) & 0xF0) | 0x09;
    I2C1_SendBytes(LCD_I2C_ADDR, buf, 4);
    delay_ms(2);
}

void LCD_SendString(char *str) {
    while(*str) {
        LCD_SendData(*str++);
    }
}

void LCD_Init(void) {
    delay_ms(50);
    LCD_SendCommand(0x30);
    delay_ms(5);
    LCD_SendCommand(0x30);
    delay_ms(1);
    LCD_SendCommand(0x30);
    delay_ms(1);
    LCD_SendCommand(0x02);
    delay_ms(1);
    LCD_SendCommand(0x28);
    delay_ms(1);
    LCD_SendCommand(0x08);
    delay_ms(1);
    LCD_SendCommand(0x01);
    delay_ms(2);
    LCD_SendCommand(0x06);
    delay_ms(1);
    LCD_SendCommand(0x0C);
    delay_ms(1);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_SendCommand(pos);
    delay_ms(2);
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    delay_ms(2);
}

// ==== SMART LCD Update Function ====
uint8_t LCD_Should_Update(uint16_t current_gas, uint8_t current_system_state, uint8_t current_relay_state) {
    // Không cập nhật nếu system tắt
    if (current_system_state == 0) {
        return 0;
    }

    uint8_t should_update = 0;
    uint32_t time_since_last_update = loop_counter - last_lcd_update_time;

    // 1. Gas level thay đổi (ưu tiên cao nhất)
    uint8_t current_level = Get_Gas_Level(current_gas);
    uint8_t last_level = Get_Gas_Level(last_lcd_gas_value);
    if (current_level != last_level) {
        should_update = 1;
        send_text_message("LCD_UPDATE_GAS_LEVEL_CHANGED");
    }

    // 2. Gas value thay đổi đáng kể (>= 10 đơn vị)
    else if (abs((int)current_gas - (int)last_lcd_gas_value) >= 20) {
        should_update = 1;
        char msg[60];
        sprintf(msg, "LCD_UPDATE_GAS_CHANGED_%d_TO_%d", last_lcd_gas_value, current_gas);
        send_text_message(msg);
    }

    // 3. Relay state thay đổi
    else if (relay_state != (current_relay_state)) {
        should_update = 1;
        send_text_message("LCD_UPDATE_RELAY_STATE_CHANGED");
    }

    // 4. Cập nhật định kỳ dựa trên gas level (tần suất khác nhau)
    else if (!should_update) {
        uint32_t update_interval;

        switch(current_level) {
            case 0: update_interval = 10000; break;  // SAFE: 10s
            case 1: update_interval = 5000;  break;  // WARNING: 5s
            case 2: update_interval = 3000;  break;  // DANGER: 3s
            case 3: update_interval = 2000;  break;  // CRITICAL: 2s
            default: update_interval = 10000; break;
        }

        if (time_since_last_update >= update_interval) {
            should_update = 1;
            char msg[50];
            sprintf(msg, "LCD_UPDATE_PERIODIC_%s",
                    (current_level == 0) ? "SAFE" :
                    (current_level == 1) ? "WARN" :
                    (current_level == 2) ? "DANG" : "CRIT");
            send_text_message(msg);
        }
    }

    return should_update;
}
void LCD_Show_Stopped_Screen(void) {
    if (lcd_stopped_screen_shown) return;  // Chỉ hiển thị 1 lần

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_SendString("SYSTEM STOPPED");
    LCD_SetCursor(1, 0);
    LCD_SendString("Press SW1 START");

    lcd_stopped_screen_shown = 1;
    send_text_message("LCD_STOPPED_SCREEN_DISPLAYED");
}

// ==== OPTIMIZED: LCD Update chỉ khi cần ====
void LCD_Update_Display(uint16_t gas_value, uint8_t system_state, uint8_t relay_state) {
    char line1[17], line2[17];
    uint8_t gas_level = Get_Gas_Level(gas_value);

    const char* level_names[] = {"SAFE", "WARN", "DANG", "CRIT"};
    sprintf(line1, "GAS:%4d %s", gas_value, level_names[gas_level]);

    const char* sys_status = system_state ? "ON " : "OFF";
    const char* relay_status = relay_state ? "R:ON " : "R:OFF";
    sprintf(line2, "SYS:%s %s", sys_status, relay_status);

    LCD_SetCursor(0, 0);
    LCD_SendString("                ");
    LCD_SetCursor(0, 0);
    LCD_SendString(line1);

    LCD_SetCursor(1, 0);
    LCD_SendString("                ");
    LCD_SetCursor(1, 0);
    LCD_SendString(line2);

    // Update tracking variables
    lcd_stopped_screen_shown = 0;  // Reset stopped screen flag
    last_lcd_gas_value = gas_value;
    last_lcd_update_time = loop_counter;

    // Debug message với gas change info
    char debug_msg[80];
    sprintf(debug_msg, "LCD_UPDATED_GAS_%d_LEVEL_%s_RELAY_%s",
            gas_value, level_names[gas_level], relay_state ? "ON" : "OFF");
    send_text_message(debug_msg);
}

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
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void delay_us(uint32_t us) {
    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}

// ==== UART Config ====
void uart_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;

    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= (0x0B << 4);

    USART1->BRR = 72000000 / 9600;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

// ==== Text Message Sending ====
void uart_send_text(char* str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *str++;
    }
    while (!(USART1->SR & USART_SR_TC));
}

// ==== IMPROVED: Synchronized UART Transmission ====
void uart_send_bytes_sync(uint8_t* data, uint8_t len) {
    // Longer pre-transmission delay for ESP sync
    delay_us(500);

    // Send start marker with extra delay
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = data[0];  // Start marker
    while (!(USART1->SR & USART_SR_TC));
    delay_us(PACKET_SYNC_DELAY * 2);  // Extra delay after start marker

    // Send remaining bytes with consistent timing
    for (uint8_t i = 1; i < len; i++) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = data[i];
        while (!(USART1->SR & USART_SR_TC));
        delay_us(PACKET_SYNC_DELAY);  // Consistent delay between bytes
    }

    // Post-transmission delay
    delay_us(500);
}

void send_text_message(char* msg) {
    uart_send_text("TXT:");
    uart_send_text(msg);
    uart_send_text("\r\n");
}

// ==== NEW: Heartbeat chỉ gửi trạng thái ====
void send_heartbeat_only(void) {
    char hb_msg[80];
    sprintf(hb_msg, "HEARTBEAT_STATUS_%s_UPTIME_%lu_MODE_POWER_SAVE",
            system_state ? "ACTIVE" : "STOPPED", loop_counter/1000);
    send_text_message(hb_msg);
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

// ==== IMPROVED: Synchronized Binary Packet Sending ====
void send_binary_packet_sync(uint16_t gas, uint8_t gas_level, uint8_t system_state, uint8_t relay_state) {
    sensor_packet_t packet;

    packet.start_marker = PACKET_START_MARKER;
    packet.header = PACKET_HEADER;
    packet.gas_value = gas;
    packet.flags = (gas_level & 0x03) | ((system_state & 0x01) << 2) | ((relay_state & 0x01) << 3);
    packet.sequence = packet_sequence++;
    packet.reserved = 0x00;
    packet.end_marker = PACKET_END_MARKER;

    uint8_t* crc_data = ((uint8_t*)&packet) + 1;
    packet.checksum = calculate_crc16(crc_data, sizeof(packet) - 4);

    // Use improved synchronized transmission
    uart_send_bytes_sync((uint8_t*)&packet, sizeof(packet));

    // Confirmation message
    char confirm_msg[50];
    sprintf(confirm_msg, "BINARY_PACKET_SENT_SEQ_%d_GAS_%d", packet_sequence-1, gas);
    send_text_message(confirm_msg);
}

// ==== GPIO Config ====
void GPIO_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // PA0 - Input analog cho MQ2
    GPIOA->CRL &= ~(0xF << (0 * 4));
    GPIOA->CRL |= (0x0 << (0 * 4));

    // PA1=RED, PA2=GREEN, PA3=BLUE, PA4=BUZZER, PA5=RELAY - Output push-pull
    GPIOA->CRL &= ~((0xF << (1 * 4)) | (0xF << (2 * 4)) | (0xF << (3 * 4)) |
                    (0xF << (4 * 4)) | (0xF << (5 * 4)));
    GPIOA->CRL |=  ((0x2 << (1 * 4)) | (0x2 << (2 * 4)) | (0x2 << (3 * 4)) |
                    (0x2 << (4 * 4)) | (0x2 << (5 * 4)));

    // PA6, PA7 - Input pull-up (SW1, SW2)
    GPIOA->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOA->CRL |=  ((0x8 << (6 * 4)) | (0x8 << (7 * 4)));
    GPIOA->ODR |= (1 << 6) | (1 << 7);

    GPIOA->ODR &= ~((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5));
}

// ==== ADC Config ====
void ADC_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

    ADC1->CR2 = 0;
    ADC1->CR1 = 0;
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->SMPR2 = 0x7;

    ADC1->CR2 |= ADC_CR2_ADON;
    delay_ms(2);

    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);
}

// ==== OPTIMIZED: Chỉ đọc gas khi system ACTIVE ====
uint16_t Read_MQ2_Fast(void) {
    uint32_t sum = 0;

    for(int i = 0; i < 3; i++) {
        ADC1->SQR3 = 0;
        ADC1->CR2 |= ADC_CR2_ADON;
        while (!(ADC1->SR & ADC_SR_EOC));
        sum += ADC1->DR;
        delay_us(100);
    }

    return sum / 3;
}

// ==== OPTIMIZED: Smart Gas Reading ====
void Update_Gas_Reading(void) {
    if (system_state == 1) {
        // Chỉ đọc gas khi system ACTIVE
        gas_sensor.current_gas = Read_MQ2_Fast();
        gas_sensor.current_level = Get_Gas_Level(gas_sensor.current_gas);
        gas_sensor.last_read_time = loop_counter;
        gas_sensor.valid_data = 1;
    } else {
        // Khi STOPPED, giữ giá trị cũ hoặc set = 0
        gas_sensor.current_gas = 0;
        gas_sensor.current_level = 0;
        gas_sensor.valid_data = 0;
    }
}

uint8_t Read_Button_Fast(uint8_t pin) {
    if ((GPIOA->IDR & (1 << pin)) == 0) {
        delay_ms(10);
        if ((GPIOA->IDR & (1 << pin)) == 0) {
            while ((GPIOA->IDR & (1 << pin)) == 0);
            delay_ms(10);
            return 1;
        }
    }
    return 0;
}

uint8_t Get_Gas_Level(uint16_t gas_value) {
    if (gas_value < GAS_SAFE_THRESHOLD) return 0;
    if (gas_value < GAS_WARNING_THRESHOLD) return 1;
    if (gas_value < GAS_DANGER_THRESHOLD) return 2;
    return 3;
}

void Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue) {
    GPIOA->ODR &= ~((1 << 1) | (1 << 2) | (1 << 3));
    if (red)   GPIOA->ODR |= (1 << 1);
    if (green) GPIOA->ODR |= (1 << 2);
    if (blue)  GPIOA->ODR |= (1 << 3);
}

void Update_Relay_Smart(uint16_t gas_value) {
    uint32_t time_since_change = loop_counter - relay_last_change_time;

    if (relay_state == 0) {
        if (gas_value > RELAY_ON_THRESHOLD && time_since_change > RELAY_MIN_OFF_TIME) {
            relay_state = 1;
            GPIOA->ODR |= (1 << 5);
            relay_last_change_time = loop_counter;
        }
    } else {
        if (gas_value < RELAY_OFF_THRESHOLD && time_since_change > RELAY_MIN_ON_TIME) {
            relay_state = 0;
            GPIOA->ODR &= ~(1 << 5);
            relay_last_change_time = loop_counter;
        }
    }
}

// ==== OPTIMIZED Main Loop ====
int main(void) {
    // Khởi tạo system
    SystemInit72MHz();
    timer_init();
    GPIO_Config();
    ADC_Config();
    uart_init();

    // Khởi tạo LCD
    I2C1_Init();
    delay_ms(100);
    LCD_Init();

    delay_ms(1000);

    // Hiển thị khởi động
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_SendString("STM32F103C8T6");
    LCD_SetCursor(1, 0);
    LCD_SendString("OPTIMIZED v2.0");

    send_text_message("STM32_START_OPTIMIZED_V2.0");

    // Test LED
    for(int i = 0; i < 3; i++) {
        Set_RGB_LED(0, 1, 0);
        delay_ms(200);
        Set_RGB_LED(0, 0, 0);
        delay_ms(200);
    }

    send_text_message("READY_OPTIMIZED_POWER_SAVE_MODE");

    // Hiển thị stopped screen ban đầu
    LCD_Show_Stopped_Screen();

    while (1) {
        loop_counter++;

        // ==== OPTIMIZED: Chỉ đọc gas khi cần ====
        Update_Gas_Reading();

        // ==== Button Handling với LCD update ngay lập tức ====
        if (Read_Button_Fast(6)) {
            system_state = !system_state;

            if (system_state) {
                // Bật: Đọc gas ngay và gửi binary packet
                Update_Gas_Reading();
                send_binary_packet_sync(gas_sensor.current_gas, gas_sensor.current_level,
                                       system_state, relay_state);

                // Cập nhật LCD ngay
                LCD_Update_Display(gas_sensor.current_gas, system_state, relay_state);

                send_text_message("BTN_SW1_SYSTEM_ACTIVATED");
            } else {
                // Tắt: Chỉ gửi heartbeat
                send_heartbeat_only();

                // Hiển thị stopped screen
                LCD_Show_Stopped_Screen();

                send_text_message("BTN_SW1_SYSTEM_STOPPED");
            }
        }

        if (Read_Button_Fast(7)) {
            system_state = 0;

            send_heartbeat_only();
            LCD_Show_Stopped_Screen();

            send_text_message("BTN_SW2_RESET_TO_STOPPED");
        }

        if (system_state) {
            // ==== HỆ THỐNG HOẠT ĐỘNG - ĐẦY ĐỦ CHỨC NĂNG ====
            Update_Relay_Smart(gas_sensor.current_gas);

            // LED và buzzer control
            if (gas_sensor.current_gas < GAS_SAFE_THRESHOLD) {
                Set_RGB_LED(0, 0, 1);
                GPIOA->ODR &= ~(1 << 4);
            } else if (gas_sensor.current_gas < GAS_WARNING_THRESHOLD) {
                Set_RGB_LED(1, 1, 0);
                GPIOA->ODR &= ~(1 << 4);
            } else if (gas_sensor.current_gas < GAS_DANGER_THRESHOLD) {
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

            // ==== OPTIMIZED LCD UPDATE - Smart update based on gas changes ====
            if (LCD_Should_Update(gas_sensor.current_gas, system_state, relay_state)) {
                LCD_Update_Display(gas_sensor.current_gas, system_state, relay_state);
            }

            // ==== OPTIMIZED SEND LOGIC ====
            uint8_t should_send = 0;

            if (system_state != last_sent_state) {
                should_send = 1;
                send_text_message("SEND_SYSTEM_STATE_CHANGED");
            } else {
                uint8_t current_level = gas_sensor.current_level;
                uint8_t last_level = Get_Gas_Level(last_sent_gas);

                if (last_level == 0 && current_level > 0) {
                    should_send = 1;
                    send_text_message("SEND_GAS_DETECTED");
                }
                else if (abs((int)gas_sensor.current_gas - (int)last_sent_gas) > 30) {
                    should_send = 1;
                    send_text_message("SEND_GAS_SIGNIFICANT_CHANGE");
                }
                else if (current_level != last_level) {
                    should_send = 1;
                    send_text_message("SEND_GAS_LEVEL_CHANGED");
                }
            }

            // Interval sending khi ACTIVE
            if (!should_send) {
                uint32_t send_interval;
                switch(gas_sensor.current_level) {
                    case 0: send_interval = 10000; break; // SAFE
                    case 1: send_interval = 5000;  break; // WARNING
                    case 2: send_interval = 3000;  break; // DANGER
                    case 3: send_interval = 2000;  break; // CRITICAL
                    default: send_interval = 10000; break;
                }

                static uint32_t last_send_time = 0;
                if (loop_counter - last_send_time >= send_interval) {
                    should_send = 1;
                    last_send_time = loop_counter;
                }
            }

            // Gửi binary packet khi ACTIVE
            if (should_send) {
                send_binary_packet_sync(gas_sensor.current_gas, gas_sensor.current_level,
                                       system_state, relay_state);
                last_sent_gas = gas_sensor.current_gas;
                last_sent_state = system_state;
            }

        } else {
            // ==== HỆ THỐNG DỪNG - POWER SAVE MODE ====
            Set_RGB_LED(0, 1, 0);  // LED xanh lá cây nhẹ
            GPIOA->ODR &= ~(1 << 4);  // Tắt buzzer
            GPIOA->ODR &= ~(1 << 5);  // Tắt relay
            relay_state = 0;
            blink_counter = 0;
            fast_blink_counter = 0;

            // ==== POWER SAVE: Chỉ gửi heartbeat định kỳ ====
            static uint32_t last_heartbeat_time = 0;
            if (loop_counter - last_heartbeat_time >= 45000) { // Mỗi 60s
                send_heartbeat_only();
                last_heartbeat_time = loop_counter;
            }
        }

        delay_ms(1);  // Giữ nguyên 1ms loop
    }
}
