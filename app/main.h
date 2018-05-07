#ifndef __MAIN_H
#define __MAIN_H

#include "crc.h"

#define VERSION 84
// Задержка перед стимуляцией
#define DELAY 1500000 //50...200 с шагом 50. 1500000 соответствует 150 мс.

// Начальная мощность
#define INIT_PWM 2 //1...10, с шагом. В дальнейшем можно менять с пульта

// Биполярный или униполярный стимуляционный сигнал
#define BIPOLAR 1

// Уровень низкого заряда, значение АЦП
#define LO_BATTERY 133

// Модуляция ВЧ сигнала (1 кГц) НЧ сигналом (120 Гц)
//#define MODULATION_ON 1

// Настройки генератора высокого напряжения
#define V_MIN 15
#define V_MAX 95
#define V_STEP 5

// Defines
// IS_MASTER == True - правый стимулятор, на нём ставим Bluetooth
#define IS_MASTER (!(FIO1PIN2 & 0x01)) // R16 installed, R17 uninstalled = master
// R16 uninstalled, R17 installed = slave
#define UART_PC_BUF_SIZE 7 // Длина пакета при связи с ПК
#define UART_BUF_SIZE 250 // Новый размер буфера для работы с HC-05

#define LED1_ON FIO1SET2 = 0x80 // Красный светодиод
#define LED1_OFF FIO1CLR2 = 0x80

#define LED2_ON FIO0SET1 = 0x10 // Зеленый светодиод
#define LED2_OFF FIO0CLR1 = 0x10

#define LED3_ON FIO0SET1 = 0x20 // Синий светодиод. На правом - подключен к блютуз-модулю.
#define LED3_OFF FIO0CLR1 = 0x20

// Пины формирователя выходного напряжения для стимуляции (оба сразу не включать!)
#define S1_ON FIO0SET2 = 0x08 // Выдать положительный сигнал
#define S1_OFF FIO0CLR2 = 0x08

#define S2_ON FIO0SET2 = 0x04 // Выдать отрицательный сигнал
#define S2_OFF FIO0CLR2 = 0x04

// Отключать PLL при самопрограммировании (записи настроек во встроенную флеш)
#define BYPASS_IAP 1 // 1 = отключать, т.к. мы работаем от PLL, а писать можно только без PLL

// Для модбас - протокола, формирует HEX-строку CRC
#define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)

// Clock Frequency

#define XTAL 12000000 // Oscillator Frequency

#ifdef BYPASS_IAP
#define CPUCLK XTAL // CPU Clock without PLL
#else
#define CPUCLK (XTAL * 4) // CPU Clock with PLL
#endif

#define CCLK (XTAL / 1000) // CPU Clock without PLL in kHz

#include <stdint.h>
//#include "cc1101_bsp.h"
#define RF_BUF_SIZE 64

typedef enum {
    RF_RECEIVE = 0,
    RF_TRANSMIT
} rf_mode_t;

typedef struct {
    rf_mode_t mode;
    uint8_t tx_buf[RF_BUF_SIZE];
    uint8_t rx_buf[RF_BUF_SIZE];
    bool rx_rq;
    bool tx_rq;
    bool tx_hw_started;
    bool inited;
    bool master_rq;
    bool slave_rq;
} rf_t;

typedef struct {
    bool rx_flag;
    bool rx_started;
    uint8_t rx_cmd;
    uint8_t rx_buf_cnt;
    uint8_t prev_rx_buf_cnt;

    uint8_t rx_buf[UART_BUF_SIZE];
    uint8_t tx_buf[UART_BUF_SIZE];
} uart_t;

typedef struct {
    int8_t ax;
    int8_t ay;
    int8_t az;
    int8_t hx;
    int8_t hy;
    int8_t hz;
    bool drdy;
} sensors_t;

typedef struct {
    uint8_t period;
    uint8_t end_time;
    uint8_t time;
    uint8_t v_set;
    bool enabled;
    uint16_t time_ms;
} stim_t;

typedef struct {
    uint32_t rpm_tmr;
    uint8_t tr_started;
    uint32_t stim_time_ms;
    uint32_t amp_avg;
    uint32_t train_time;
    uint32_t total_time;
    uint32_t pause_time;
    uint32_t way;
    uint32_t rpm;
    uint32_t way_w_stim;
    uint32_t amp_sum;
    uint32_t amp_cnt;
} stat_t;

void delay(unsigned long i);
void system_init(void);
void peripheral_init(void);
int8_t sd_prepare(uint8_t req);
void set_pwm(uint8_t dtc);
void int2uart(unsigned int k);
void str2uart(char* Buf);
void send2uart(uint8_t cmd, uint8_t* Buf);

void Timer0Init(void);
void Timer1Init(void);
void shdn();
void resume();
void wake_for_read();
void sleep_from_read();
void sleep_from_reset();
int8_t extract_rf_packet(uint8_t* pack, uint8_t* cmd_no, uint8_t* value);

void start_pll();
void stop_pll();
unsigned int erase();
unsigned int program(void* data, unsigned int size);

//uint8_t transmit_cca(uint8_t adr, uint8_t* dat, uint8_t qty);
#endif
