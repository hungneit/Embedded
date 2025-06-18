#include "stm32f1xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// ==== LCD I2C Address ====
#define LCD_I2C_ADDR 0x4E  // 0x27 << 1

// ==== Cấu hình ngưỡng PPM theo chuẩn thực tế ====
#define PPM_SAFE_THRESHOLD     50    // An toàn: < 50 ppm (không khí sạch)
#define PPM_WARNING_THRESHOLD  200   // Cảnh báo: 50-200 ppm (mức chấp nhận được)
#define PPM_DANGER_THRESHOLD   300   // Nguy hiểm: 200-300 ppm (theo chuẩn)
#define PPM_CRITICAL_THRESHOLD 500   // Cực nguy hiểm: > 300 ppm

// ==== Cấu hình relay theo PPM ====
#define PPM_RELAY_ON_THRESHOLD  250  // Bật relay tại cuối mức warning (250 ppm)
#define PPM_RELAY_OFF_THRESHOLD 150  // Tắt relay tại 150 ppm
#define RELAY_MIN_ON_TIME      5000  // 5s
#define RELAY_MIN_OFF_TIME     3000  // 3s

// ==== MQ2 Configuration được hiệu chỉnh cho ngưỡng thấp ====
#define MODULE_RL_VALUE 10000.0      // Điện trở tải 10kΩ
#define VCC_VOLTAGE 3.3              // Điện áp cung cấp STM32
#define ADC_RESOLUTION 4095.0        // 12-bit ADC
#define R0_CLEAN_AIR 9500.0          // R0 hiệu chỉnh cho sensitivity cao hơn

// ==== Math constants cho MQ2 - hiệu chỉnh cho dải thấp ====
#define LN_10 2.302585093
// Hằng số được điều chỉnh để có sensitivity cao ở dải 50-500 ppm
#define CURVE_A 116.6020682          // Giảm để tăng sensitivity ở dải thấp
#define CURVE_B -0.24                // Điều chỉnh slope cho phù hợp

// ==== Binary Protocol ====
#define PACKET_START_MARKER 0x55
#define PACKET_HEADER       0xAA
#define PACKET_END_MARKER   0x33
#define PACKET_SIZE         10
#define PACKET_SYNC_DELAY   50

typedef struct {
    uint8_t start_marker;
    uint8_t header;
    uint16_t gas_value;     // Chứa PPM
    uint8_t flags;
    uint8_t sequence;
    uint8_t reserved;
    uint16_t checksum;
    uint8_t end_marker;
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
void uart_send_bytes_sync(uint8_t* data, uint8_t len);
void Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue);
uint8_t Get_Gas_Level(uint16_t gas_value);
uint16_t calculate_crc16(uint8_t* data, uint8_t len);
void send_binary_packet_sync(uint16_t gas, uint8_t gas_level, uint8_t system_state, uint8_t relay_state);
void send_text_message(char* msg);
void send_heartbeat_only(void);
void Update_Relay_Smart(uint16_t gas_value);

// ==== PPM Functions đã sửa ====
float my_log(float x);
float my_pow(float base, float exp);
int ADC_to_PPM_Fixed(uint16_t adc_value);
uint8_t Get_PPM_Level(int ppm);
void Update_LED_Smart(int ppm);

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
void LCD_Show_Stopped_Screen(void);
uint8_t LCD_Should_Update(uint16_t current_gas, uint8_t current_system_state, uint8_t current_relay_state);

// ==== Biến toàn cục ====
volatile uint8_t system_state = 0;
volatile uint32_t loop_counter = 0;
volatile uint32_t led_blink_counter = 0;

uint8_t relay_state = 0;
uint32_t relay_last_change_time = 0;

uint16_t last_sent_gas = 0;
uint8_t last_sent_state = 255;
uint8_t packet_sequence = 0;

uint32_t lcd_update_counter = 0;
uint8_t lcd_needs_update = 0;
uint8_t lcd_stopped_screen_shown = 0;
uint16_t last_lcd_gas_value = 0;
uint32_t last_lcd_update_time = 0;

// Biến cho LED blink
uint8_t led_blink_state = 0;
uint32_t last_led_update = 0;

typedef struct {
    uint16_t current_gas;
    uint8_t current_level;
    uint32_t last_read_time;
    uint8_t valid_data;
} gas_data_t;

gas_data_t gas_sensor = {0, 0, 0, 0};

// ==== TOÁN HỌC đã sửa ====

// Hàm tính logarithm tự nhiên
float my_ln(float x) {
    if (x <= 0) return -999999;
    if (x == 1.0) return 0;

    float result = 0;
    while (x > 2.0) {
        x = x / 2.0;
        result += 0.693147;
    }
    while (x < 0.5) {
        x = x * 2.0;
        result -= 0.693147;
    }

    float u = x - 1.0;
    float u_pow = u;
    float ln_val = 0;

    for (int i = 1; i <= 10; i++) {
        if (i % 2 == 1) {
            ln_val += u_pow / i;
        } else {
            ln_val -= u_pow / i;
        }
        u_pow *= u;
    }

    return result + ln_val;
}

float my_log10(float x) {
    return my_ln(x) / LN_10;
}

float my_pow(float base, float exp) {
    if (base <= 0) return 0;
    if (exp == 0) return 1;
    if (exp == 1) return base;

    float ln_base = my_ln(base);
    float result_ln = exp * ln_base;

    float ex = 1.0;
    float term = 1.0;

    for (int i = 1; i <= 15; i++) {
        term *= result_ln / i;
        ex += term;
        if (term < 0.000001) break;
    }

    return ex;
}

// ==== PPM calculation được hiệu chỉnh cho dải thấp ====
int ADC_to_PPM_Fixed(uint16_t adc_value) {
    // Kiểm tra giá trị ADC hợp lệ
    if (adc_value == 0 || adc_value >= 5000) {
        return 700; // Giá trị không hợp lệ
    }

    // Bước 1: ADC → Voltage
    float voltage_out = (adc_value * VCC_VOLTAGE) / ADC_RESOLUTION;

    float Rs = MODULE_RL_VALUE * (VCC_VOLTAGE - voltage_out) / voltage_out;

    // Bước 4: Tính tỷ lệ Rs/R0
    float rs_r0_ratio = Rs / R0_CLEAN_AIR;

    if (rs_r0_ratio >= 10.0) {
        return 10 + (int)((20.0 - rs_r0_ratio) * 2.0); // 10-30 ppm
    } else if (rs_r0_ratio >= 5.0) {
        return 30 + (int)((10.0 - rs_r0_ratio) * 8.0); // 30-70 ppm
    } else if (rs_r0_ratio >= 2.0) {
        return 70 + (int)((5.0 - rs_r0_ratio) * 25.0); // 70-145 ppm
    } else if (rs_r0_ratio >= 1.0) {
        return 145 + (int)((2.0 - rs_r0_ratio) * 55.0); // 145-200 ppm
    } else if (rs_r0_ratio >= 0.5) {
        return 200 + (int)((1.0 - rs_r0_ratio) * 200.0); // 200-300 ppm
    } else if (rs_r0_ratio >= 0.3) {
        return 300 + (int)((0.5 - rs_r0_ratio) * 500.0); // 300-400 ppm
    } else {
        return 400 + (int)((0.3 - rs_r0_ratio) * 1000.0); // 400-700 ppm
    }
}

uint8_t Get_PPM_Level(int ppm) {
    if (ppm < PPM_SAFE_THRESHOLD) return 0;        // SAFE - LED xanh dương (< 50 ppm)
    if (ppm < PPM_WARNING_THRESHOLD) return 1;     // WARNING - LED vàng (50-200 ppm)
    if (ppm < PPM_DANGER_THRESHOLD) return 2;      // DANGER - LED đỏ nháy 1Hz (200-300 ppm)
    return 3;                                      // CRITICAL - LED đỏ nháy 2-10Hz (> 300 ppm)
}

// ==== LED Control đã sửa ====
void Update_LED_Smart(int ppm) {
    uint8_t level = Get_PPM_Level(ppm);
    uint32_t current_time = loop_counter;

    switch(level) {
        case 0: // SAFE - LED xanh dương
            Set_RGB_LED(0, 0, 1);
            break;

        case 1: // WARNING - LED vàng
            Set_RGB_LED(1, 1, 0);
            break;

        case 2: // DANGER - LED đỏ nháy 1Hz (500ms on/off)
            if (current_time - last_led_update >= 500) {
                led_blink_state = !led_blink_state;
                if (led_blink_state) {
                    Set_RGB_LED(1, 0, 0);
                } else {
                    Set_RGB_LED(0, 0, 0);
                }
                last_led_update = current_time;
            }
            break;

        case 3: // CRITICAL - LED đỏ nháy 2-10Hz (tăng tần số theo PPM > 300)
            {
                // Tính tần số nháy dựa trên mức độ nguy hiểm từ 300 ppm trở lên
                uint32_t blink_period;
                if (ppm >= 800) {
                    blink_period = 50;   // 10Hz (50ms on/off) - rất nguy hiểm
                } else if (ppm >= 600) {
                    blink_period = 67;   // 7.5Hz
                } else if (ppm >= 500) {
                    blink_period = 100;  // 5Hz
                } else if (ppm >= 400) {
                    blink_period = 167;  // 3Hz
                } else {
                    blink_period = 250;  // 2Hz (250ms on/off) - từ 300-400 ppm
                }

                if (current_time - last_led_update >= blink_period) {
                    led_blink_state = !led_blink_state;
                    if (led_blink_state) {
                        Set_RGB_LED(1, 0, 0);
                    } else {
                        Set_RGB_LED(0, 0, 0);
                    }
                    last_led_update = current_time;
                }
            }
            break;
    }
}

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
    RCC->APB1ENR |= (1 << 21);
    RCC->APB2ENR |= (1 << 3);

    GPIOB->CRL &= ~(0xF << 24);
    GPIOB->CRL |= (0xF << 24);

    GPIOB->CRL &= ~(0xF << 28);
    GPIOB->CRL |= (0xF << 28);

    I2C1->CR1 = 0;
    I2C1->CR2 = 36;
    I2C1->CCR = 180;
    I2C1->TRISE = 37;
    I2C1->CR1 |= (1 << 0);
}

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

uint8_t LCD_Should_Update(uint16_t current_gas, uint8_t current_system_state, uint8_t current_relay_state) {
    if (current_system_state == 0) {
        return 0;
    }

    uint8_t should_update = 0;
    uint32_t time_since_last_update = loop_counter - last_lcd_update_time;

    uint8_t current_level = Get_Gas_Level(current_gas);
    uint8_t last_level = Get_Gas_Level(last_lcd_gas_value);
    if (current_level != last_level) {
        should_update = 1;
        send_text_message("LCD_UPDATE_GAS_LEVEL_CHANGED");
    }

    else if (abs((int)current_gas - (int)last_lcd_gas_value) >= 15) {
        should_update = 1;
        char msg[60];
        sprintf(msg, "LCD_UPDATE_GAS_CHANGED_%d_TO_%d", last_lcd_gas_value, current_gas);
        send_text_message(msg);
    }

    else if (relay_state != (current_relay_state)) {
        should_update = 1;
        send_text_message("LCD_UPDATE_RELAY_STATE_CHANGED");
    }

    else if (!should_update) {
        uint32_t update_interval;

        switch(current_level) {
            case 0: update_interval = 10000; break;
            case 1: update_interval = 5000;  break;
            case 2: update_interval = 3000;  break;
            case 3: update_interval = 2000;  break;
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
    if (lcd_stopped_screen_shown) return;

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_SendString("SYSTEM STOPPED");
    LCD_SetCursor(1, 0);
    LCD_SendString("Press SW1 START");

    lcd_stopped_screen_shown = 1;
    send_text_message("LCD_STOPPED_SCREEN_DISPLAYED");
}

// ==== LCD hiển thị PPM đã sửa ====
void LCD_Update_Display(uint16_t gas_value, uint8_t system_state, uint8_t relay_state) {
    char line1[17], line2[17];

    // Tính PPM chính xác từ ADC
    int ppm = ADC_to_PPM_Fixed(gas_value);
    uint8_t ppm_level = Get_PPM_Level(ppm);

    const char* level_names[] = {"SAFE", "WARN", "DANG", "CRIT"};

    // Dòng 1: Hiển thị PPM
    sprintf(line1, "PPM: %d", ppm);

    // Dòng 2: System status và level
    const char* sys_status = system_state ? "ON " : "OFF";
    const char* relay_status = relay_state ? "R:ON" : "R:OFF";
    sprintf(line2, "%s %s %s", sys_status, relay_status, level_names[ppm_level]);

    LCD_SetCursor(0, 0);
    LCD_SendString("                ");
    LCD_SetCursor(0, 0);
    LCD_SendString(line1);

    LCD_SetCursor(1, 0);
    LCD_SendString("                ");
    LCD_SetCursor(1, 0);
    LCD_SendString(line2);

    lcd_stopped_screen_shown = 0;
    last_lcd_gas_value = gas_value;
    last_lcd_update_time = loop_counter;

    // Debug message
    char debug_msg[100];
    sprintf(debug_msg, "LCD_ADC_%d_PPM_%d_LEVEL_%s",
            gas_value, ppm, level_names[ppm_level]);
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

void uart_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;

    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= (0x0B << 4);

    USART1->BRR = 72000000 / 9600;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void uart_send_text(char* str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *str++;
    }
    while (!(USART1->SR & USART_SR_TC));
}

void uart_send_bytes_sync(uint8_t* data, uint8_t len) {
    delay_us(500);

    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = data[0];
    while (!(USART1->SR & USART_SR_TC));
    delay_us(PACKET_SYNC_DELAY * 2);

    for (uint8_t i = 1; i < len; i++) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = data[i];
        while (!(USART1->SR & USART_SR_TC));
        delay_us(PACKET_SYNC_DELAY);
    }

    delay_us(500);
}

void send_text_message(char* msg) {
    uart_send_text("TXT:");
    uart_send_text(msg);
    uart_send_text("\r\n");
}

void send_heartbeat_only(void) {
    char hb_msg[80];
    sprintf(hb_msg, "HEARTBEAT_STATUS_%s_UPTIME_%lu_STANDARD_PPM_MODE",
            system_state ? "ACTIVE" : "STOPPED", loop_counter/1000);
    send_text_message(hb_msg);
}

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

// ==== Gửi PPM đã sửa ====
void send_binary_packet_sync(uint16_t gas, uint8_t gas_level, uint8_t system_state, uint8_t relay_state) {
    sensor_packet_t packet;

    // Chuyển ADC sang PPM chính xác
    int ppm = ADC_to_PPM_Fixed(gas);
    uint16_t ppm_value = (uint16_t)ppm;

    packet.start_marker = PACKET_START_MARKER;
    packet.header = PACKET_HEADER;
    packet.gas_value = ppm_value;  // GỬI PPM đã sửa
    packet.flags = (gas_level & 0x03) | ((system_state & 0x01) << 2) | ((relay_state & 0x01) << 3);
    packet.sequence = packet_sequence++;
    packet.reserved = 0x00;
    packet.end_marker = PACKET_END_MARKER;

    uint8_t* crc_data = ((uint8_t*)&packet) + 1;
    packet.checksum = calculate_crc16(crc_data, sizeof(packet) - 4);

    uart_send_bytes_sync((uint8_t*)&packet, sizeof(packet));

    char confirm_msg[60];
    sprintf(confirm_msg, "BINARY_PACKET_PPM_%d_ADC_%d_SEQ_%d", ppm_value, gas, packet_sequence-1);
    send_text_message(confirm_msg);
}

void GPIO_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIOA->CRL &= ~(0xF << (0 * 4));
    GPIOA->CRL |= (0x0 << (0 * 4));

    GPIOA->CRL &= ~((0xF << (1 * 4)) | (0xF << (2 * 4)) | (0xF << (3 * 4)) |
                    (0xF << (4 * 4)) | (0xF << (5 * 4)));
    GPIOA->CRL |=  ((0x2 << (1 * 4)) | (0x2 << (2 * 4)) | (0x2 << (3 * 4)) |
                    (0x2 << (4 * 4)) | (0x2 << (5 * 4)));

    GPIOA->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOA->CRL |=  ((0x8 << (6 * 4)) | (0x8 << (7 * 4)));
    GPIOA->ODR |= (1 << 6) | (1 << 7);

    GPIOA->ODR &= ~((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5));
}

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

uint16_t Read_MQ2_Fast(void) {
    uint32_t sum = 0;

    for(int i = 0; i < 5; i++) {  // Tăng số lần đọc để ổn định hơn
        ADC1->SQR3 = 0;
        ADC1->CR2 |= ADC_CR2_ADON;
        while (!(ADC1->SR & ADC_SR_EOC));
        sum += ADC1->DR;
        delay_us(200);  // Tăng delay giữa các lần đọc
    }

    return sum / 5;
}

void Update_Gas_Reading(void) {
    if (system_state == 1) {
        gas_sensor.current_gas = Read_MQ2_Fast();

        // Tính gas level dựa trên PPM chính xác
        int ppm = ADC_to_PPM_Fixed(gas_sensor.current_gas);
        gas_sensor.current_level = Get_PPM_Level(ppm);

        gas_sensor.last_read_time = loop_counter;
        gas_sensor.valid_data = 1;
    } else {
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
    // Chuyển sang dùng PPM để xác định level
    int ppm = ADC_to_PPM_Fixed(gas_value);
    return Get_PPM_Level(ppm);
}

void Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue) {
    GPIOA->ODR &= ~((1 << 1) | (1 << 2) | (1 << 3));
    if (red)   GPIOA->ODR |= (1 << 1);
    if (green) GPIOA->ODR |= (1 << 2);
    if (blue)  GPIOA->ODR |= (1 << 3);
}

// ==== Relay dựa trên PPM đã sửa ====
void Update_Relay_Smart(uint16_t gas_value) {
    uint32_t time_since_change = loop_counter - relay_last_change_time;

    // Chuyển ADC sang PPM chính xác
    int ppm = ADC_to_PPM_Fixed(gas_value);

    if (relay_state == 0) {
        if (ppm > PPM_RELAY_ON_THRESHOLD && time_since_change > RELAY_MIN_OFF_TIME) {
            relay_state = 1;
            GPIOA->ODR |= (1 << 5);
            relay_last_change_time = loop_counter;

            char msg[60];
            sprintf(msg, "RELAY_ON_PPM_%d", ppm);
            send_text_message(msg);
        }
    } else {
        if (ppm < PPM_RELAY_OFF_THRESHOLD && time_since_change > RELAY_MIN_ON_TIME) {
            relay_state = 0;
            GPIOA->ODR &= ~(1 << 5);
            relay_last_change_time = loop_counter;

            char msg[60];
            sprintf(msg, "RELAY_OFF_PPM_%d", ppm);
            send_text_message(msg);
        }
    }
}

// ==== MAIN ====
int main(void) {
    SystemInit72MHz();
    timer_init();
    GPIO_Config();
    ADC_Config();
    uart_init();

    I2C1_Init();
    delay_ms(100);
    LCD_Init();

    delay_ms(1000);

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_SendString("STM32F103C8T6");
    LCD_SetCursor(1, 0);
    LCD_SendString("STANDARD PPM v3.0");

    send_text_message("STM32_START_STANDARD_PPM_THRESHOLDS_MODE");

    // Khởi động với LED xanh lá cây (hệ thống dừng)
    for(int i = 0; i < 3; i++) {
        Set_RGB_LED(0, 1, 0);
        delay_ms(200);
        Set_RGB_LED(0, 0, 0);
        delay_ms(200);
    }

    send_text_message("READY_STANDARD_PPM_50_200_300_500_THRESHOLDS");
    LCD_Show_Stopped_Screen();

    while (1) {
        loop_counter++;

        Update_Gas_Reading();

        if (Read_Button_Fast(6)) {
            system_state = !system_state;

            if (system_state) {
                Update_Gas_Reading();
                send_binary_packet_sync(gas_sensor.current_gas, gas_sensor.current_level,
                                       system_state, relay_state);

                LCD_Update_Display(gas_sensor.current_gas, system_state, relay_state);

                send_text_message("BTN_SW1_SYSTEM_ACTIVATED_STANDARD_PPM");
            } else {
                send_heartbeat_only();
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
            Update_Relay_Smart(gas_sensor.current_gas);

            // LED control dựa trên PPM chính xác - SỬA CHÍNH
            int current_ppm = ADC_to_PPM_Fixed(gas_sensor.current_gas);
            Update_LED_Smart(current_ppm);

            // Buzzer control
            uint8_t ppm_level = Get_PPM_Level(current_ppm);
            if (ppm_level >= 2) {  // DANGER hoặc CRITICAL
                GPIOA->ODR |= (1 << 4);  // Bật buzzer
            } else {
                GPIOA->ODR &= ~(1 << 4); // Tắt buzzer
            }

            if (LCD_Should_Update(gas_sensor.current_gas, system_state, relay_state)) {
                LCD_Update_Display(gas_sensor.current_gas, system_state, relay_state);
            }

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

            if (!should_send) {
                uint32_t send_interval;
                switch(gas_sensor.current_level) {
                    case 0: send_interval = 15000; break; // SAFE
                    case 1: send_interval = 8000;  break; // WARNING
                    case 2: send_interval = 5000;  break; // DANGER
                    case 3: send_interval = 3000;  break; // CRITICAL
                    default: send_interval = 15000; break;
                }

                static uint32_t last_send_time = 0;
                if (loop_counter - last_send_time >= send_interval) {
                    should_send = 1;
                    last_send_time = loop_counter;
                }
            }

            if (should_send) {
                send_binary_packet_sync(gas_sensor.current_gas, gas_sensor.current_level,
                                       system_state, relay_state);

                // Gửi thêm PPM data chính xác
                char ppm_msg[80];
                int ppm = ADC_to_PPM_Fixed(gas_sensor.current_gas);
                sprintf(ppm_msg, "PPM_DATA,PPM:%d,LEVEL:%d,ADC:%d",
                        ppm, gas_sensor.current_level, gas_sensor.current_gas);
                send_text_message(ppm_msg);

                last_sent_gas = gas_sensor.current_gas;
                last_sent_state = system_state;
            }

        } else {
            // Hệ thống dừng - LED xanh lá cây
            Set_RGB_LED(0, 1, 0);
            GPIOA->ODR &= ~(1 << 4);  // Tắt buzzer
            GPIOA->ODR &= ~(1 << 5);  // Tắt relay
            relay_state = 0;
            led_blink_counter = 0;

            static uint32_t last_heartbeat_time = 0;
            if (loop_counter - last_heartbeat_time >= 45000) {
                send_heartbeat_only();
                last_heartbeat_time = loop_counter;
            }
        }

        delay_ms(1);
    }
}
