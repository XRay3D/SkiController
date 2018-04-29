/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright ELTA 2013-2014
 *
 *    File name      : main.c
 *    Description    : Define main module
 *
 *    History :
 *    1. Data        : June 15, 2013
 *       Author      : Onishchenko Denis
 *       Description : Create
 *
 *
 *    $Revision: 8.4 $ 26.01.2018
 *
**************************************************************************/
#include "includes.h"
#include "main.h"
#include <math.h>
#include "uart_service.h"

//#define DBG_MSG 1  // вывод сообщений в терминал
//#define PW_CONTROL 1
//#define OLD_ALG 1

// ! TODO:
// - средн€€ частота педалировани€/шагов в минуту
// - сбор статистики с 2х устройств


stat_t Stat;

rf_t rf;
uart_t uart;
sensors_t sns;
stim_t stim;
uint8_t pwm, pwm_lvl;
bool turned_off;
bool rec_en;
bool rtc_wake;

uint8_t alg_start;
bool shdn_cmd;

uint8_t accepted, declined;

struct iap_in {
    unsigned int cmd;
    unsigned int par[4];
};

typedef void (*IAP)(struct iap_in* in, unsigned int* result);
#define iap_entry ((IAP)0x7FFFFFF1) // IAP entry point

unsigned char vals[512];

typedef struct {
    bool inited;
    bool ok;
    bool mount;
} sd_t;

unsigned char x, online;
uint8_t dtc;
uint16_t adcres;
uint32_t adc_cnt, dly = 1;
sd_t sd;
int8_t x1, y1, z1, x2, y2, z2;

uint8_t volt, cur, v_bat, v_bat_sens = 0, v_curr = 0, v_set = 40;
bool paus_en, stim_on;
uint8_t paus_time;
uint8_t stim_duration, sensitivity;
uint16_t executed;
bool resume_rq, waked;
bool rec_start;

bool rf_snif;

uint8_t turn_off_tmr;
bool turn_off_tmr_en;

char *ans_str, *ans1, *ans2;

bool bt_connected = false;
uint8_t bt_tmr = 0;

//---------------------------Int Subroutines------------------------------------
// IRQ subroutine
#pragma vector = 0x18
__irq __arm void IRQ_ISR_Handler(void)
{
    void (*interrupt_function)();
    unsigned int vector;

    vector = VICVectAddr; // Get interrupt vector.
    interrupt_function = (void (*)())vector;
    (*interrupt_function)(); // Call vectored interrupt function.
}

// FIQ subroutine
#pragma vector = 0x1c
__fiq __arm void FIQ_ISR_Handler(void)
{
    void (*interrupt_function)();
    unsigned int vector;

    vector = VICVectAddr; // Get interrupt vector.
    interrupt_function = (void (*)())vector;
    (*interrupt_function)(); // Call vectored interrupt function.
}

// non vectored callback subroutine
void NonVectISR(void)
{
}

// clear arg
void ClearFlag(void* arg)
{
    int* pFlag = (int*)arg;
    *pFlag = 0;
}

//--------------------------- ISR routine --------------------------------------
void EXT0_ISR() // sd card inserted
{
    EXTINT = 1; // clear interrupt flag

    VICVectAddr = 0; // Clear interrupt in VIC.
}

void EXT1_ISR() // rf IRQ
{
    EXTINT = 2; // clear interrupt flag

    if (rf.inited) {
        if (rf.tx_hw_started) {
            rf.tx_hw_started = false;
            rf.tx_rq = true;
            LED2_OFF;
        }
        else {
            rf.rx_rq = true;
            LED3_ON;
        }
    }
    VICVectAddr = 0; // Clear interrupt in VIC.
}

void EXT2_ISR() // STEP1 shorted (gerkon1 shorted)
{
    EXTINT = 4; // clear interrupt flag

    VICVectAddr = 0; // Clear interrupt in VIC.
}

void EXT3_ISR() // STEP2 shorted (gerkon2 shorted)
{
    EXTINT = 8; // clear interrupt flag

    VICVectAddr = 0; // Clear interrupt in VIC.
}

void RTC_ISR() // Real-time clock interrupts every second
{ // Works only if IS_MASTER
    static char k;
    ILR = 0x1; // clear interrupt flag

    if (turn_off_tmr_en)
        turn_off_tmr++;

    if (turned_off) {
        rtc_wake = true;
        //if ((k%2)==0)
        //  LED1_ON;
        //else
        //  LED1_OFF;
        k++;
    }
    if (bt_connected) {
        bt_tmr++;
        Stat.total_time++;
        if (stim.enabled)
            Stat.train_time++;
    }

    VICVectAddr = 0; // Clear interrupt in VIC.
}

void TIMER0_ISR(void) // Timer 0 - Master RF control
{
    if (T0IR_bit.MR0INT == 1) {
        //if ((!paus_en) && (stim_on)){
        alg_start = 1;
        /* }
    else{
      alg_start = 0;
      if (paus_en){
        if (paus_time < 30)
          paus_time++;
        else
          paus_en = false;
      }
    }*/
        if (stim.time == 0) {
            AD0CR &= ~0x000000FF;
            AD0CR |= 0x00000002;
            AD0CR |= 0x01000000;
            while (!(AD0GDR & 0x80000000))
                ;
            delay(50);
            adcres = AD0GDR;
            adcres = (adcres >> 8) & 0xFF;
            volt = adcres;

            AD0CR &= ~0x000000FF;
            AD0CR |= 0x00000010;
            AD0CR |= 0x01000000;
            while (!(AD0GDR & 0x80000000))
                ;
            delay(50);
            adcres = AD0GDR;
            adcres = (adcres >> 8) & 0xFF;
            v_bat = adcres;
        }
#ifdef PW_CONTROL
        if ((volt < 140) && (cur < 15)) {
            if (pwm < 8)
                pwm++;
        }
        else {
            if (pwm > 0)
                pwm--;
        }
        set_pwm(pwm);
#endif

        T0IR_bit.MR0INT = 1;
    }
    if (T0IR_bit.MR1INT == 1) {
        if ((uart.rx_started) && (!uart.rx_flag)) {
            if (uart.prev_rx_buf_cnt < uart.rx_buf_cnt) {
                uart.prev_rx_buf_cnt = uart.rx_buf_cnt;
            }
            else {
                uart.rx_started = 0;
                uart.rx_flag = 1;
            }
        }
        T0IR_bit.MR1INT = 1;
    }
    if (T0IR_bit.MR2INT == 1) {

        T0IR_bit.MR2INT = 1;
    }
    if (T0IR_bit.MR3INT == 1) {

        T0IR_bit.MR3INT = 1;
    }
    VICVectAddr = 0;
}

//--------------------------- STIMULATOR routine --------------------------------------
void TIMER1_ISR(void) // Timer 1 - Stimulator generator
{
#ifdef BIPOLAR
    if (T1IR_bit.MR0INT == 1) {

#ifdef MODULATION_ON
        if (stim.period < 10) {
#endif
            if (stim.period == ((stim.period >> 1) << 1)) {
                S1_ON;
            }
            else {
                S2_ON;
            }
#ifdef MODULATION_ON
        }
#endif
        T1IR_bit.MR0INT = 1;
    }
    if (T1IR_bit.MR1INT == 1) {
        if (stim.period == 8) {
            AD0CR &= ~0x000000FF;
            AD0CR |= 0x00000002;
            AD0CR |= 0x01000000;
            while (!(AD0GDR & 0x80000000))
                ;
            delay(50);
            adcres = AD0GDR;
            adcres = (adcres >> 8) & 0xFF;
            volt = adcres;
#ifdef DBG_MSG
            U0THR = volt;
#endif
            AD0CR &= ~0x000000FF;
            AD0CR |= 0x00000004;
            AD0CR |= 0x01000000;
            while (!(AD0GDR & 0x80000000))
                ;
            delay(50);
            adcres = AD0GDR;
            adcres = (adcres >> 8) & 0xFF;
            cur = adcres;
#ifdef PW_CONTROL
            if ((volt < 140) && (cur < 15)) {
                if (pwm < 8)
                    pwm++;
            }
            else {
                if (pwm > 0)
                    pwm--;
            }
            set_pwm(pwm);
#endif
        }
        T1IR_bit.MR1INT = 1;
    }
    if (T1IR_bit.MR2INT == 1) {
        S1_OFF;
        S2_OFF;
        stim.period++;
        if (stim.period >= 20) {
            stim.period = 0;
            stim.time++;
        }
        if (stim.time >= stim.end_time) {
            S1_OFF;
            S2_OFF;
            stim.time = 0;
            LED2_ON;
            T1IR_bit.MR2INT = 1;
            T1TCR = 2;
        }
        T1IR_bit.MR2INT = 1;
    }
    if (T1IR_bit.MR3INT == 1) {

        T1IR_bit.MR3INT = 1;
    }
    VICVectAddr = 0;
#endif

#ifdef UNIPOLAR
    if (T1IR_bit.MR0INT == 1) {
        if (stim.period == 10) {
            if (stim.time == ((stim.time >> 1) << 1)) {
                S1_ON;
            }
            else {
                S2_ON;
            }
        }
        T1IR_bit.MR0INT = 1;
    }
    if (T1IR_bit.MR1INT == 1) {
        if (stim.period == 8) {
            AD0CR &= ~0x000000FF;
            AD0CR |= 0x00000002;
            AD0CR |= 0x01000000;
            while (!(AD0GDR & 0x80000000))
                ;
            delay(50);
            adcres = AD0GDR;
            adcres = (adcres >> 8) & 0xFF;
            volt = adcres;
#ifdef DBG_MSG
            U0THR = volt;
#endif
            AD0CR &= ~0x000000FF;
            AD0CR |= 0x00000004;
            AD0CR |= 0x01000000;
            while (!(AD0GDR & 0x80000000))
                ;
            delay(50);
            adcres = AD0GDR;
            adcres = (adcres >> 8) & 0xFF;
            cur = adcres;
#ifdef PW_CONTROL
            if ((volt < 140) && (cur < 15)) {
                if (pwm < 8)
                    pwm++;
            }
            else {
                if (pwm > 0)
                    pwm--;
            }
            set_pwm(pwm);
#endif
        }
        T1IR_bit.MR1INT = 1;
    }
    if (T1IR_bit.MR2INT == 1) {
        S1_OFF;
        S2_OFF;
        stim.period++;
        if (stim.period >= 20) {
            stim.period = 0;
            stim.time++;
        }
        if (stim.time >= stim.end_time) {
            S1_OFF;
            S2_OFF;
            stim.time = 0;
            LED2_ON;
            T1IR_bit.MR2INT = 1;
            T1TCR = 2;
        }
        T1IR_bit.MR2INT = 1;
    }
    if (T1IR_bit.MR3INT == 1) {

        T1IR_bit.MR3INT = 1;
    }
    VICVectAddr = 0;
#endif
}

//--------------------------- UART routine --------------------------------------
void UART0_ISR(void) // Connection to PC
{
    if (U0LSR & 0x01) {
        ReceiveIsr();
        //        uint8_t x = U0RBR; // Reading clears interrupt flag
        //        if ((x == 10) && (uart.rx_buf_cnt == 0)) {
        //            U0THR = 12;
        //            turn_off_tmr_en = false;
        //            turn_off_tmr = 0;
        //        }
        //        else {
        //            if (uart.rx_buf_cnt == 0) {
        //                uart.rx_started = true;
        //                uart.rx_cmd = x;
        //                uart.rx_buf_cnt++;
        //            }
        //            else {
        //                uart.rx_buf[uart.rx_buf_cnt - 1] = x;
        //                if (uart.rx_buf_cnt < UART_BUF_SIZE)
        //                    uart.rx_buf_cnt++;
        //                else {
        //                    uart.rx_started = false;
        //                    uart.rx_flag = true;
        //                }
        //            }
        //        }
    }
    VICVectAddr = 0;
}

void set_pwm(uint8_t dtc)
{
    PWMTCR = 0x02;
    PWMMR1 = 250 + dtc + 1;
    PWMMR2 = 250 - dtc - 1;
    PWMMR3 = 0 + dtc + 1;
    PWMMR4 = 500 - dtc - 1;
    PWMTCR = 0x09;
}

void stimulate(char x, char t)
{
    if (stim.enabled) {
        LED2_OFF;
        Stat.amp_sum += v_set;
        Stat.amp_cnt++;
        stim.end_time = t;
        stim.time = 0;
        stim.period = 0;
        Stat.stim_time_ms += t * 8 * 2;
        executed++;
        T1TCR = 2;
        T1TC = 0;
        T1TCR = 1;
    }
}

char* utoa_divmod(uint32_t value, char* buffer)
{
    buffer += 11;
    *--buffer = 0;
    do {
        ldiv_t res = ldiv(value, 10);
        *--buffer = res.rem + '0';
        value = res.quot;
    } while (value != 0);
    return buffer;
}

void add_string(char* dest, char* new_str_bez_crc)
{
    //strcat((char*)ans1, new_str_bez_crc);
    uint8_t crclen = 0, len = 0;
    uint16_t crc_int;

    crclen = strlen(new_str_bez_crc);
    crc_int = CalcCRC16Table(crclen, (uint8_t*)new_str_bez_crc);
    strcat(dest, new_str_bez_crc);
    len = strlen(dest);
    dest[len] = TO_HEX(((crc_int & 0xF000) >> 12));
    dest[len + 1] = TO_HEX(((crc_int & 0x0F00) >> 8));
    dest[len + 2] = TO_HEX(((crc_int & 0x00F0) >> 4));
    dest[len + 3] = TO_HEX((crc_int & 0x000F));
    dest[len + 4] = 13;
}

void bt_send_statistics()
{
    uint8_t bat, len;
    uint8_t v_bat_r = 0;
    memset((void*)ans_str, 0, 250);
    memset((void*)ans1, 0, 25);
    memset((void*)ans2, 0, 25);
    v_bat_r = ((float)v_bat) * 0.26;
    if (v_bat_r >= 41)
        bat = 100;
    if (v_bat_r == 40)
        bat = 90;
    if (v_bat_r == 39)
        bat = 75;
    if (v_bat_r == 38)
        bat = 50;
    if (v_bat_r == 37)
        bat = 25;
    if (v_bat_r == 36)
        bat = 10;
    if (v_bat_r <= 35)
        bat = 0;
    strcat(ans1, ":5;1;");
    if (bat == 100) {
        ans1[5] = 49;
        ans1[6] = 48;
        ans1[7] = 48;
    }
    else {
        ans1[5] = (bat / 10) + 48;
        ans1[6] = (bat % 10) + 48;
    }
    strcat(ans1, ";");
    if (v_bat_sens >= 41)
        bat = 100;
    if (v_bat_sens == 40)
        bat = 90;
    if (v_bat_sens == 39)
        bat = 75;
    if (v_bat_sens == 38)
        bat = 50;
    if (v_bat_sens == 37)
        bat = 25;
    if (v_bat_sens <= 36)
        bat = 0;
    //bat = 90;
    len = strlen(ans1);
    if (bat == 100) {
        ans1[len] = 49;
        ans1[len + 1] = 48;
        ans1[len + 2] = 48;
    }
    else {
        ans1[len] = (bat / 10) + 48;
        ans1[len + 1] = (bat % 10) + 48;
    }
    strcat(ans1, ";");
    add_string(ans_str, ans1);

    memset((void*)ans1, 0, 25);
    strcat(ans1, ":6;2;");
    ans1[5] = VERSION / 10 + 48;
    ans1[6] = '.';
    ans1[7] = (VERSION % 10) + 48;
    ans1[8] = '0';
    ans1[9] = ';';
    add_string(ans_str, ans1);

    memset((void*)ans1, 0, 25);
    memset((void*)ans2, 0, 25);
    strcat(ans1, ":7;1;");
    strcat(ans1, utoa_divmod(Stat.stim_time_ms / 1000, ans2));
    strcat(ans1, ";");
    add_string(ans_str, ans1);

    memset((void*)ans1, 0, 25);
    memset((void*)ans2, 0, 25);
    strcat(ans1, ":8;1;");
    if (Stat.amp_cnt != 0) {
        Stat.amp_avg = Stat.amp_sum / Stat.amp_cnt;
    }
    else
        Stat.amp_avg = 0;
    strcat(ans1, utoa_divmod(Stat.amp_avg, ans2));
    strcat(ans1, ";");
    add_string(ans_str, ans1);

    memset((void*)ans1, 0, 25);
    memset((void*)ans2, 0, 25);
    strcat(ans1, ":9;1;");
    strcat(ans1, utoa_divmod(Stat.train_time, ans2));
    strcat(ans1, ";");
    add_string(ans_str, ans1);

    memset((void*)ans1, 0, 25);
    memset((void*)ans2, 0, 25);
    strcat(ans1, ":10;1;");
    strcat(ans1, utoa_divmod(Stat.total_time, ans2));
    strcat(ans1, ";");
    add_string(ans_str, ans1);

    memset((void*)ans1, 0, 25);
    memset((void*)ans2, 0, 25);
    strcat(ans1, ":11;1;");
    Stat.pause_time = Stat.total_time - Stat.train_time;
    strcat(ans1, utoa_divmod(Stat.pause_time, ans2));
    strcat(ans1, ";");
    add_string(ans_str, ans1);

    memset((void*)ans1, 0, 25);
    memset((void*)ans2, 0, 25);
    strcat(ans1, ":12;1;");
    strcat(ans1, utoa_divmod(Stat.way, ans2));
    strcat(ans1, ";");
    add_string(ans_str, ans1);

    /*memset((void*)ans1, 0, 25);
	memset((void*)ans2, 0, 25);
	strcat(ans1, ":13;1;");
	strcat(ans1, utoa_divmod(Stat.rpm, ans2));
	strcat(ans1, ";");
	add_string(ans_str, ans1);
	
	memset((void*)ans1, 0, 25);
	memset((void*)ans2, 0, 25);
	strcat(ans1, ":14;1;");
	strcat(ans1, utoa_divmod(Stat.way_w_stim, ans2));
	strcat(ans1, ";");
	add_string(ans_str, ans1);*/

    str2uart(ans_str);
    //HAL_UART_Transmit(&huart3, (uint8_t*) ans_str, strlen(ans_str), 250);
}

//-------------------------------main function----------------------------------
int main(void)
{

    uint8_t i, length;
    uint32_t cnt;
    uint8_t ADR;

    int16_t crc_int, crc_len;

    uint8_t stim_test_cnt = 0, tr_stim_cmd = 0;

    system_init();

    delay(500000);
    peripheral_init();
    rf.tx_hw_started = false;

    alg_start = 0;
    accepted = 0;
    declined = 0;

    ans_str = (char*)malloc(250);
    ans1 = (char*)malloc(25);
    ans2 = (char*)malloc(25);

    uart.rx_flag = false;
    uart.rx_started = false;
    uart.rx_cmd = 0;
    uart.rx_buf_cnt = 0;
    uart.prev_rx_buf_cnt = 0;

    stim.enabled = true; // debug, false in release

    pwm = INIT_PWM;
    pwm_lvl = pwm;
    set_pwm(pwm);
    /*rf_wr_strobe(CC1100_SIDLE);
  rf_wr_strobe(CC1100_SFRX);
  rf_wr_strobe(CC1100_SRX);*/
    paus_en = false;
    stim_on = true;
    paus_time = 0;
    stim_duration = 25;
    sensitivity = 1;
    rec_en = false;

    rf.rx_rq = false;
    rf.tx_rq = false;
    rf_snif = false;
    resume_rq = false;
    waked = false;
    rtc_wake = false;

    LED2_ON;
    executed = 0;
    shdn_cmd = false;
    rec_start = false;

    //turn_off_tmr_en = true;
    turn_off_tmr_en = false;
    turn_off_tmr = 0;

    while (1) {
        if (!(FIO1PIN2 & 0x80)) // if Charger present
            LED2_OFF; // Turn on Red LED
        else
            LED2_ON;

        if (rf.rx_rq) {
            rf.rx_rq = false;
            rf_rd_reg(CC1100_RXBYTES | 0x40, &length);
            if (length <= RF_BUF_SIZE)
                rf_rd_burst(CC1100_RXFIFO, rf.rx_buf, length); //rf.rx_buf(length & 0x3F)
            else
                rf_wr_strobe(CC1100_SFRX);
            str2uart("rx_len = ");
            int2uart(length);
            str2uart("\r\n");
            for (i = 0; i < length; i++) {
                int2uart(rf.rx_buf[i]);
                str2uart(" ");
            }
            str2uart("\r\n");
            rf_wr_strobe(CC1100_SIDLE);
            rf_wr_strobe(CC1100_SFRX);
            LED3_OFF;
            if ((rf.rx_buf[length - 1] & 0x80)) {
                LED2_ON;
                if ((rf.rx_buf[0] == 1) && (IS_MASTER))
                    stimulate(1, stim_duration);
                if ((rf.rx_buf[0] == 2) && (!IS_MASTER))
                    stimulate(1, stim_duration);
            }
            go_srx_state();
        }

        if (rf.tx_rq) {

            rf.rx_rq = false;
            rf.tx_hw_started = false;

            if (go_srx_state() != RF_OK) {
            }
        }

        UartService();

        //        if (uart.rx_flag) {
        //            switch (uart.rx_cmd) {
        //            case 1 | 0x80:
        //                uart.tx_buf[0] = acc_init();
        //                uart.tx_buf[1] = hyro_init();
        //                uart.tx_buf[2] = rf_init();
        //                uart.tx_buf[3] = v_bat * 0.26;
        //                uart.tx_buf[4] = volt / 2.48;
        //                uart.tx_buf[5] = IS_MASTER;
        //                uart.tx_buf[6] = VERSION;
        //                send2uart(uart.rx_cmd, uart.tx_buf);
        //                break;
        //            case 2 | 0x80:
        //                uart.tx_buf[0] = z2;
        //                uart.tx_buf[1] = y2;
        //                uart.tx_buf[2] = x2;
        //                uart.tx_buf[3] = z1;
        //                uart.tx_buf[4] = y1;
        //                uart.tx_buf[5] = x1;
        //                uart.tx_buf[6] = 0;
        //                send2uart(uart.rx_cmd, uart.tx_buf);
        //                break;

        //            case 3:
        //                LED3_ON;
        //                for (i = 0; i < 6; i++)
        //                    vals[i] = uart.rx_buf[i];
        //                erase();
        //                program(vals, sizeof(vals));
        //                rf_init();
        //                ADR = *(reinterpret_cast<unsigned char*>(0x08005)) | ((!IS_MASTER) << 7);
        //                send2uart(uart.rx_cmd, uart.rx_buf);
        //                LED3_OFF;
        //                break;

        //            case 3 | 0x80:
        //                for (i = 0; i < 6; i++)
        //                    uart.tx_buf[i] = *(reinterpret_cast<unsigned char*>(0x08000 + i));
        //                send2uart(uart.rx_cmd, uart.tx_buf);
        //                break;

        //            case 4:
        //                CCR = 0x2; // Reset the clock
        //                ILR = 0x3; // Clear the Interrupt Location Register
        //                YEAR = uart.rx_buf[0] + 2000;
        //                MONTH = uart.rx_buf[1];
        //                DOM = uart.rx_buf[2];
        //                HOUR = uart.rx_buf[3];
        //                MIN = uart.rx_buf[4];
        //                SEC = uart.rx_buf[5];
        //                CIIR = 0x01;
        //                CCR = 0x11;
        //                PCONP &= ~0x0200;
        //                send2uart(uart.rx_cmd, uart.rx_buf);
        //                break;
        //            case 4 | 0x80:
        //                uart.tx_buf[0] = YEAR - 2000;
        //                uart.tx_buf[1] = MONTH;
        //                uart.tx_buf[2] = DOM;
        //                uart.tx_buf[3] = HOUR;
        //                uart.tx_buf[4] = MIN;
        //                uart.tx_buf[5] = SEC;
        //                uart.tx_buf[6] = 0;
        //                send2uart(uart.rx_cmd, uart.tx_buf);
        //                break;

        //            case 5:
        //                stimulate(1, stim_duration);
        //                send2uart(uart.rx_cmd, uart.rx_buf);
        //                break;

        //            case 6:
        //                pwm = uart.rx_buf[0];
        //                v_set = V_STEP * pwm + V_MIN;
        //                if (v_set > V_MAX)
        //                    v_set = V_MAX;

        //                // set_pwm(pwm);
        //                stim_duration = 12;
        //                if (uart.rx_buf[1] == 0)
        //                    stim_duration = 12;
        //                if (uart.rx_buf[1] == 1)
        //                    stim_duration = 18;
        //                if (uart.rx_buf[1] == 2)
        //                    stim_duration = 25;
        //                if (uart.rx_buf[1] == 3)
        //                    stim_duration = 31;
        //                if (uart.rx_buf[1] == 4)
        //                    stim_duration = 37;
        //                break;

        //            case 6 | 0x80:
        //                uart.tx_buf[0] = pwm;
        //                uart.tx_buf[1] = 0;
        //                if (stim_duration <= 12)
        //                    uart.tx_buf[1] = 0;
        //                if ((stim_duration > 12) && (stim_duration <= 18))
        //                    uart.tx_buf[1] = 1;
        //                if ((stim_duration > 18) && (stim_duration <= 25))
        //                    uart.tx_buf[1] = 2;
        //                if ((stim_duration > 25) && (stim_duration <= 31))
        //                    uart.tx_buf[1] = 3;
        //                if ((stim_duration > 31) && (stim_duration <= 37))
        //                    uart.tx_buf[1] = 4;

        //                for (i = 2; i < UART_PC_BUF_SIZE; i++)
        //                    uart.tx_buf[i] = 0;
        //                send2uart(uart.rx_cmd, uart.tx_buf);
        //                break;

        //            case 7:
        //                if (uart.rx_buf[0] == 1)
        //                    turned_off = true;
        //                //LED1_OFF;
        //                break;

        //            case 8:
        //                if (uart.rx_buf[0] == 1) {
        //                    accepted = 0;
        //                    declined = 0;
        //                    rf_snif = true;
        //                }
        //                else
        //                    rf_snif = false;
        //                break;

        //            case ':':

        //                bt_connected = true;

        //                if ((uart.rx_buf[0] == '1') && (uart.rx_buf[2] == '1') && (uart.rx_buf[1] == ';') && (uart.rx_buf[3] == ';')) {
        //                    if (uart.rx_buf[4] == '1') {
        //                        stim.enabled = true;
        //                    }
        //                    else {
        //                        stim.enabled = false;
        //                    }
        //                }
        //                if ((uart.rx_buf[0] == '2') && (uart.rx_buf[2] == '1') && (uart.rx_buf[1] == ';') && (uart.rx_buf[3] == ';')) {
        //                    if ((uart.rx_buf[5] == ';') || (uart.rx_buf[6] == ';') || (uart.rx_buf[7] == ';')) {
        //                        if (uart.rx_buf[7] == ';')
        //                            stim.v_set = (uart.rx_buf[4] - 48) * 100 + (uart.rx_buf[5] - 48) * 10 + (uart.rx_buf[6] - 48);
        //                        else {
        //                            if (uart.rx_buf[5] == ';')
        //                                stim.v_set = uart.rx_buf[4] - 48;
        //                            else
        //                                stim.v_set = (uart.rx_buf[4] - 48) * 10 + (uart.rx_buf[5] - 48);
        //                        }
        //                    }
        //                    if (stim.v_set < V_MIN)
        //                        stim.v_set = V_MIN;
        //                    if (stim.v_set > V_MAX)
        //                        stim.v_set = V_MAX;
        //                    pwm = (stim.v_set - V_MIN) / V_STEP;
        //                    //pwm = uart.rx_buf[0];
        //                    v_set = V_STEP * pwm + V_MIN;
        //                }

        //                if ((uart.rx_buf[0] == '3') && (uart.rx_buf[2] == '1') && (uart.rx_buf[1] == ';') && (uart.rx_buf[3] == ';')) {
        //                    if ((uart.rx_buf[5] == ';') || (uart.rx_buf[6] == ';') || (uart.rx_buf[7] == ';')) {
        //                        if (uart.rx_buf[7] == ';')
        //                            stim.time_ms = (uart.rx_buf[4] - 48) * 100 + (uart.rx_buf[5] - 48) * 10 + (uart.rx_buf[6] - 48);
        //                        else {
        //                            if (uart.rx_buf[5] == ';')
        //                                stim.time_ms = uart.rx_buf[4] - 48;
        //                            else
        //                                stim.time_ms = (uart.rx_buf[4] - 48) * 10 + (uart.rx_buf[5] - 48);
        //                        }
        //                    }
        //                    stim_duration = stim.time_ms / 8;

        //                    // set voltage here!!!
        //                }

        //                if ((uart.rx_buf[0] == '5') && (uart.rx_buf[2] == '1') && (uart.rx_buf[1] == ';') && (uart.rx_buf[3] == ';')) {
        //                    if (stim_test_cnt == 0) {
        //                        stimulate(1, stim_duration);
        //                        stim_test_cnt = 1;
        //                    }
        //                    else {
        //                        //stim_right(stim_time_set);
        //                        tr_stim_cmd = 1;
        //                        stim_test_cnt = 0;
        //                    }
        //                }

        //                for (i = uart.rx_buf_cnt; i > 0; i--)
        //                    if (uart.rx_buf[i] == ';')
        //                        break;
        //                crc_len = i + 2;
        //                uart.tx_buf[0] = '!';
        //                for (i = 1; i < crc_len; i++)
        //                    uart.tx_buf[i] = uart.rx_buf[i - 1];
        //                crc_int = CalcCRC16Table(crc_len, uart.tx_buf);
        //                uart.tx_buf[crc_len] = TO_HEX(((crc_int & 0xF000) >> 12));
        //                uart.tx_buf[crc_len + 1] = TO_HEX(((crc_int & 0x0F00) >> 8));
        //                uart.tx_buf[crc_len + 2] = TO_HEX(((crc_int & 0x00F0) >> 4));
        //                uart.tx_buf[crc_len + 3] = TO_HEX((crc_int & 0x000F));
        //                uart.tx_buf[crc_len + 4] = 13;
        //                //			HAL_UART_Transmit(&huart3, (uint8_t*) uart.tx_buf, strlen(uart.tx_buf), 50);
        //                for (i = 0; i < crc_len + 5; i++) {
        //                    while (!U0LSR_bit.THRE)
        //                        ;
        //                    U0THR = uart.tx_buf[i];
        //                }

        //                rf.tx_buf[0] = ADR | 0x80;
        //                rf.tx_buf[1] = 2;
        //                rf.tx_buf[2] = pwm;
        //                rf.tx_buf[3] = 0;

        //                if (stim_duration <= 12) // 100 ms
        //                    rf.tx_buf[3] = 0;
        //                if ((stim_duration > 12) && (stim_duration <= 18)) // 150 ms
        //                    rf.tx_buf[3] = 1;
        //                if ((stim_duration > 18) && (stim_duration <= 25)) // 200 ms
        //                    rf.tx_buf[3] = 2;
        //                if ((stim_duration > 25) && (stim_duration <= 31)) // 250 ms
        //                    rf.tx_buf[3] = 3;
        //                if ((stim_duration > 31) && (stim_duration <= 37)) // 300 ms
        //                    rf.tx_buf[3] = 4;
        //                rf.tx_buf[4] = stim.enabled;
        //                rf.tx_buf[5] = sensitivity;
        //                rf.tx_buf[6] = v_bat * 0.26;
        //                rf.tx_buf[7] = volt / 2.48;

        //                if (tr_stim_cmd == 1) {
        //                    tr_stim_cmd = 0;
        //                    rf.tx_buf[1] = 4;
        //                }

        //                VIC_DisableInt(1 << VIC_EINT1);

        //                transmit_cca(CC1100_TXFIFO, rf.tx_buf, USER_RF_BUF);
        //                // LED1_OFF;
        //                cnt = 0;
        //                if (!RF_IRQ_ACTIVE) {
        //                    while ((!RF_IRQ_ACTIVE) && (cnt < 50000)) {
        //                        cnt++;
        //                    };
        //                }
        //                cnt = 0;
        //                while ((RF_IRQ_ACTIVE) && (cnt < 50000)) {
        //                    cnt++;
        //                };
        //                delay(1000);
        //                EXTINT = 2;
        //                VIC_EnableInt(1 << VIC_EINT1);
        //                go_srx_state();

        //                break; // case ':'

        //            } // switch

        //            uart.rx_buf_cnt = 0;
        //            uart.rx_cmd = 0;
        //            uart.rx_flag = false;
        //        }

        if (bt_tmr >= 5) {
            bt_send_statistics();
            bt_tmr = 0;
        }

        if (!turned_off) {

            if (alg_start == 1) {
                if (stim.time == 0) {
                    v_curr = volt / 2.46;
                    if (v_curr < v_set) {
                        if (pwm_lvl < (pwm + 2))
                            pwm_lvl++;
                    }
                    else {
                        if (pwm_lvl > 0)
                            pwm_lvl = 1;
                    }
                    set_pwm(pwm_lvl);
                }
                else {
                    //  if (stim.time < 2){
                    //    pwm_lvl = pwm+1;
                    //    set_pwm(pwm_lvl);
                    //  }
                    //  else{
                    if (stim.time < 2)
                        pwm_lvl = pwm;
                    v_curr = volt / 2.46;
                    if (v_curr < (v_set / pwm)) {
                        if (pwm_lvl < (pwm + 2))
                            pwm_lvl++;
                    }
                    else {
                        //if (pwm_lvl > 0)
                        //  pwm_lvl--;
                    }
                    set_pwm(pwm);
                    //}
                    //set_pwm(2+pwm/5);
                }
                if (v_bat < LO_BATTERY)
                    turned_off = true;
                //   __disable_interrupt();
                //LED2_OFF;
                acc_get(&x1, &y1, &z1);

                hyro_get(&x2, &y2, &z2);
                //LED2_ON;

                /*
    if (IS_MASTER){
      DataRight->AddData(x2,y2,z2,x1,y1,z1);
      DataRight->Process();
      if (DataRight->turn_ON==1){
        if (stim.time == 0)
          stimulate(1,stim_duration);
      }
    }
    else{
      DataLeft->AddData(x2,y2,z2,x1,y1,z1);
      DataLeft->Process();
      if (DataLeft->turn_ON==1){
        if (stim.time == 0)
          stimulate(1,stim_duration);
      }
    }*/

                /*    if (IS_MASTER){
      _acc[0] = x1;
      _acc[1] = y1;
      _acc[2] = z1;
      _gyro[0] = x2;
      _gyro[1] = y2;
      _gyro[2] = -z2;
      
      add_data(_acc, _gyro);
      alg_params = alg_calc();
          
      if (alg_process(alg_params)==1){
         LED2_OFF;
        if (stim.time == 0)
          stimulate(1,stim_duration);
      }
    }
    else{
      _acc[0] = x1;
      _acc[1] = y1;
      _acc[2] = z1;
      _gyro[0] = x2;
      _gyro[1] = y2;
      _gyro[2] = z2;
      
      add_data(_acc, _gyro);
      alg_params = alg_calc();
          
      if (alg_process(alg_params)==1){
        LED2_OFF;
        if (stim.time == 0)
          stimulate(1,stim_duration);
      }
    }*/

                //  LED2_OFF;

                alg_start = 0;
            }
            PCON |= 0x01; // idle
        }
    };
}

void delay(unsigned long i)
{
    while (i--)
        ;
}

void system_init(void)
{
    unsigned char i, Dummy;

    MEMMAP |= 0x01; // Memory = Flash
    SCS |= 0x03; // GPIO = FastIO

    PLLCFG &= ~0x7F;
    PLLCFG |= 0x24; // M = 4, P=2   0x24  CCLK = 12*5 = 60 MHz Fcco = 192 MHz
    PLLCON |= 0x03; // PLL Enabled at 60 MHz
    PLLFEED = 0xAA;
    PLLFEED = 0x55;
    delay(5);
    VPBDIV = 0x01; // div = 1, Peripheral works at processor frequency (60 MHz)
    PCONP = 0x172E;

    // Set MAM fully enable
    MAMCR_bit.MODECTRL = 0;
    MAMTIM_bit.CYCLES = 3;
    MAMCR_bit.MODECTRL = 2;
    // Memory map init flash memory is maped on 0 address
    MEMMAP_bit.MAP = 1;

    __disable_interrupt();

    // Functions Config
    PINSEL0 = PINSEL1 = 0;
    //PINSEL0 |= 0x0005;    // UART0

    //  GPIO config
    FIO0DIR0 = 0xD2; // P0.4-P0.6 OUTPUTS
    if (IS_MASTER)
      FIO0DIR1 = 0x15;
    else
      FIO0DIR1 = 0x35;
    FIO0DIR2 = 0x0C; // P0.18, P0.19, P0.21 OUTPUTS
    FIO0DIR3 = 0x00; // P0.31 OUTPUT LED
    FIO1DIR2 = 0x00; // P1.16 - LED, P1.20 - ADC_EN
    FIO1DIR3 = 0x03; // P1.25 - OUTPUT
    // GPIO clear
    FIO0CLR0 = 0xFF;
    FIO0CLR1 = 0xFF;
    FIO0CLR2 = 0xFF;
    FIO0CLR3 = 0xFF;
    FIO1CLR2 = 0xFF;
    FIO1CLR3 = 0xFF;

    // Switch of all peripherals
    ACC_CS_OFF;
    HYRO_CS_OFF;
    RF_CS_OFF;

    // Pin functions config
    PINSEL0 = 0x800E95C5; // EINT2, EINT3, PWM4, PWM2, SPI0, EINT1, UART0
    PINSEL1 = 0x05040000; // ADC

    // PWM config (PWM2 and PWM4 generates signals for push-pull)
    /* PWMTCR = 0x02;
  PWMTC = 0;
  PWMPR = 0;//0 - prescaler
  PWMPC = 0;
  PWMMCR = 0x02;
  PWMMR0 = 500;
  PWMPCR = 0x1414;
  PWMMR1 = 1;
  PWMMR2 = 2;
  PWMMR3 = 250;
  PWMMR4 = 251;
  PWMMR5 = 30;
  PWMMR6 = 40;
  PWMLER = 0x14;
  PWMTCR = 0x09;*/

    // Full-bridge
    PWMTCR = 0x02;
    PWMTC = 0;
    PWMPR = 1;
    PWMPC = 0;
    PWMMCR = 0x02;
    PWMMR0 = 500;
    PWMPCR = 0x1414;
    PWMMR1 = 251;
    PWMMR2 = 249;
    PWMMR3 = 1;
    PWMMR4 = 499;
    PWMMR5 = 30;
    PWMMR6 = 40;
    PWMLER = 0x14;
    PWMTCR = 0x09;

    // SPI0 init (for rf, hyro and accelerometer)
    S0SPCR = 0x0020;
    S0SPCCR = 64;
    for (i = 0; i < 8; i++) {
        Dummy = S0SPDR; // clear the RxFIFO
    }

    // SPI1 init (for sd-card)
    SSPCR0 = 0x0607;
    SSPCPSR = 0x02; //0x02;  // CLK_div
    SSPIMSC = 0;
    SSPCR1 = 0x02;
    for (i = 0; i < 8; i++) {
        Dummy = SSPDR; // clear the RxFIFO
    }

    // ADC init
    AD0CR = 0x00250E10; // установка разр€дности (25) и включение ј÷ѕ
    // установка делител€ тактовой частоты (0E) 60/15
    // выбор входного канала (01)

    // UART init at 115200 8-N-1
    U0LCR = 0x83;
    U0DLL = 30;
    U0DLM = 0;
    U0FDR = 0xC1;
    U0FCR = 0x01;
    U0LCR = 0x03;
    U0TER |= 0x80;
    U0IER |= 0x01;
    for (i = 0; i < 8; i++) {
        Dummy = U0RBR; /* clear the RxFIFO */
    }
    Timer0Init();
    Timer1Init();

    USBINTS &= ~0x80000000;
    VIC_Init();
    VIC_SetProtectionMode(UserandPrivilegedMode);

    // Timer0 interrupt
    /*  VIC_SetVectoredIRQ(EXT0_ISR,VIC_Slot1,VIC_EINT0);
  VIC_EnableInt(1<<VIC_EINT0);
  
  VIC_SetVectoredIRQ(EXT1_ISR,VIC_Slot2,VIC_EINT1);
  VIC_EnableInt(1<<VIC_EINT1);
  
  VIC_SetVectoredIRQ(EXT3_ISR,VIC_Slot3,VIC_EINT3);
  VIC_EnableInt(1<<VIC_EINT3);*/

    //  VIC_SetVectoredIRQ(RTC_ISR,VIC_Slot4,VIC_RTC);
    //  VIC_EnableInt(1<<VIC_RTC);

    VIC_SetVectoredIRQ(EXT1_ISR, VIC_Slot1, VIC_EINT1);
    VIC_EnableInt(1 << VIC_EINT1);

    VIC_SetVectoredIRQ(TIMER0_ISR, VIC_Slot2, VIC_TIMER0);
    VIC_EnableInt(1 << VIC_TIMER0);

    VIC_SetVectoredIRQ(TIMER1_ISR, VIC_Slot3, VIC_TIMER1);
    VIC_EnableInt(1 << VIC_TIMER1);

    //UART0 interrupt
    VIC_SetVectoredIRQ(UART0_ISR, VIC_Slot4, VIC_UART0);
    VIC_EnableInt(1 << VIC_UART0);

    VIC_SetVectoredIRQ(RTC_ISR, VIC_Slot5, VIC_RTC);
    VIC_EnableInt(1 << VIC_RTC);

    VIC_DisableNonVectoredIRQ();
    EXTMODE |= 0x0F;

    PCONP |= 0x0200;
    CCR = 0x2;
    ILR = 0x03;
    if ((YEAR > 2030) || (YEAR < 2010)) {

        HOUR = 9;
        SEC = 0;
        MIN = 0;
        DOM = 17;
        MONTH = 12;
        YEAR = 2013;
    }
    CIIR = 0x01;
    CCR = 0x11;
    PCONP &= ~0x0200;

    __enable_interrupt();
}

void soft_init(void)
{

    turned_off = false;
}

void peripheral_init(void)
{
    RF_ERR err;
    int8_t a_err, h_err;
#ifdef DBG_MSG
    str2uart("\fElectrical stimulator. firmware v 3.1 by Onischenko Denis, 06.2013\n\r");
    if (IS_MASTER) {
        str2uart("Dev. mode = Master\n\rCurrent Date/Time: ");
        int2uart(DOM);
        while (!U0LSR_bit.THRE)
            ;
        U0THR = '.';
        int2uart(MONTH);
        while (!U0LSR_bit.THRE)
            ;
        U0THR = '.';
        int2uart(YEAR);
        while (!U0LSR_bit.THRE)
            ;
        U0THR = '/';
        int2uart(HOUR);
        while (!U0LSR_bit.THRE)
            ;
        U0THR = ':';
        int2uart(MIN);
        while (!U0LSR_bit.THRE)
            ;
        U0THR = ':';
        int2uart(SEC);
        str2uart("\n\r");
    }
    else {
        str2uart("Dev. mode = Slave\n\r");
    }
#endif

    a_err = acc_init();
    h_err = hyro_init();
    err = rf_init();

#ifdef DBG_MSG
    str2uart("Init Peripheral modules...\n\r");

    str2uart("  Accelerometer...");
    if (a_err == -1)
        str2uart("ERROR\n\r");
    else
        str2uart("OK\n\r");

    str2uart("  Hyroscope...");
    if (h_err == -1)
        str2uart("ERROR\n\r");
    else
        str2uart("OK\n\r");

    str2uart("  RF-module...");
    if (err == RF_ERR_NOT_READY)
        str2uart("ERROR: NOT READY\n\r");
    if (err == RF_ERR_SPI)
        str2uart("ERROR: SPI COMMUNICATION\n\r");
    if (err == RF_OK)
        str2uart("OK\n\r");
    if ((a_err == 0) && (h_err == 0) && (err == RF_OK))
        str2uart("Peripheral modules was inited succesfully\n\r");
    else
        str2uart("One of modules failed. Could not continue, check PCB.");
#endif
    if (!((a_err == 0) && (h_err == 0) && (err == RF_OK))) {
#ifdef DBG_MSG
        while (1) {
            if (a_err != 0) {
                LED1_ON;
                delay(10000000);
                LED1_OFF;
                delay(10000000);
            }
            if (h_err != 0) {
                LED1_ON;
                delay(2500000);
                LED1_OFF;
                delay(2500000);
            }
            if (err != RF_OK) {
                LED1_ON;
                delay(700000);
                LED1_OFF;
                delay(700000);
            }
        }
#endif
    }
    else {
        rf_wr_strobe(CC1100_SIDLE);
        rf_wr_strobe(CC1100_SFRX);
        rf_wr_strobe(CC1100_SRX);
        rf.inited = true;
    }
}

//int8_t sd_prepare(uint8_t req){

//}

void Timer0Init(void)
{
    // Init timer
    // Reset and stop timer0
    T0TCR = 2;
    // Set timer counters mode - clock by PCLK
    T0CTCR = 0;
    // Set timer prescaler
    T0PR = 0;
    // Set timer period
    T0MR0 = 300000;
    T0MR1 = 10000;
    T0MR2 = 90000;
    T0MR3 = 270000;
    // Set mack action - interrupt by MACH0 enable, reset counter
    T0MCR = 0x24B;
    // No external action
    T0EMR = 0;

    T0TC = 0;
    // Enable timer0
    T0TCR = 1;
}

void Timer1Init(void)
{
    // Init timer
    // Reset and stop timer1
    T1TCR = 2;
    // Set timer counters mode - clock by PCLK
    T1CTCR = 0;
    // Set timer prescaler
    T1PR = 0;
// Set timer period
#ifdef BIPOLAR
    T1MR0 = 24000; // f = 2.5 kHz, period = 400 us
    T1MR1 = 6000; // set high voltage
    T1MR2 = 12000; // measure voltage and current
    T1MR3 = 18000; // turn off high voltage*/
#endif
#ifdef UNIPOLAR
    T1MR0 = 24000; // f = 2.5 kHz, period = 400 us
    T1MR1 = 12000; // set high voltage
    T1MR2 = 1000; // measure voltage and current
    T1MR3 = 18000; // turn off high voltage
#endif
    // Set mack action - interrupt by MACH0 enable, reset counter
    T1MCR = 0x24B;
    // No external action
    T1EMR = 0;

    T1TC = 0;
    // Enable timer1
    //T1TCR = 1;
}

void int2uart(unsigned int k)
{

    //  U0THR = k/1000+48;
    //  U0THR = (k%1000)/100+48;
    while (!U0LSR_bit.THRE)
        ;
    U0THR = (k % 100) / 10 + 48;
    while (!U0LSR_bit.THRE)
        ;
    U0THR = k % 10 + 48;
    //  U0THR = '\n';
    //  U0THR = '\r';
}

void str2uart(char* Buf)
{
    char* pBuf = Buf;
    while (*pBuf) {
        while (!U0LSR_bit.THRE)
            ;
        U0THR = *pBuf++;
    }
}

void send2uart(uint8_t cmd, uint8_t* Buf)
{
    uint8_t i, *pBuf = Buf;
    while (!U0LSR_bit.THRE)
        ;
    U0THR = cmd;
    for (i = 0; i < UART_PC_BUF_SIZE; i++) {
        while (!U0LSR_bit.THRE)
            ;
        U0THR = *pBuf++;
    }
}

void shdn()
{
    delay(5000);
    acc_exchange(0x20, 0x00);
    hyro_exchange(0x20, 0x00);
    enter_wor();
    PWMTCR = 0x02;
    // Reset and stop timer0
    T0TCR = 2;
    T1TCR = 2;
    VIC_DisableInt(1 << VIC_TIMER0);
    VIC_DisableInt(1 << VIC_TIMER1);
    VIC_DisableInt(1 << VIC_UART0);
    alg_start = false;
    PINSEL0 = 0x000000C0; // EINT2, EINT3, PWM4, PWM2, SPI0, EINT1, UART0
    PINSEL1 = 0x00000000; // ADC
    FIO0DIR0 |= 0x80;
    FIO0CLR0 = 0x83;
    FIO0DIR1 |= 0x01;
    FIO0CLR1 = 0x01;
    FIO0CLR2 = 0xC0;
    AD0CR &= ~0x00200000;
    USBINTS &= ~0x80000000;

    PWMTC = 0;
    PCONP = 0;
    EXTWAKE = 0x8002;

    PCONP |= 0x0200;
    CCR = 0x2;
    //  ILR = 0x03;
    CIIR = 0x02;
    CCR = 0x11;
    PCONP &= ~0x0200;

    //  PREINT = (int)((60000000UL/32768UL)-1)&((1<<14)-1);
    //  PREFRAC = (int)(60000000UL-(((unsigned long)PREINT+1UL)*32768UL));
    // LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    turned_off = true;
    waked = false;
    //waked = false;
    PCON = 0x1E;
}

void resume()
{
    EXTWAKE = 0x00;
    start_pll();
    VPBDIV = 0x01; // div = 1, Peripheral works at processor frequency (60 MHz)
    PCONP |= 0x152E;
    delay(1000);
    PINSEL0 = 0x800E95C5; // EINT2, EINT3, PWM4, PWM2, SPI0, EINT1, UART0
    PINSEL1 = 0x05040000; // ADC
    AD0CR = 0x00250E10;
    PWMTCR = 0x09;
    acc_init();
    hyro_init();
    rf_wakeup();
    LED2_ON;
    waked = false;

    PCONP |= 0x0200;
    CCR = 0x2;
    //ILR = 0x03;
    CIIR = 0x01;
    CCR = 0x11;
    PCONP &= ~0x0200;

    VIC_EnableInt(1 << VIC_TIMER0);
    VIC_EnableInt(1 << VIC_TIMER1);
    VIC_EnableInt(1 << VIC_UART0);
    Timer0Init();
    Timer1Init();
    //str2uart("Power restored\n\r");
}

void wake_for_read()
{
    EXTWAKE = 0x00;

    // start_pll();
    // delay(1000);

    //  VPBDIV = 0x01;  // div = 1, Peripheral works at processor frequency (60 MHz)
    PCONP |= 0x152E;
    PINSEL0 = 0x800E95C5; // EINT2, EINT3, PWM4, PWM2, SPI0, EINT1, UART0
    exit_wor();
    waked = true;
}

void sleep_from_read()
{
    re_enter_wor();
    PINSEL0 = 0x000000C0; // EINT2, EINT3, PWM4, PWM2, SPI0, EINT1, UART0
    PCONP = 0;
    //  CIIR = 0x02;
    EXTWAKE = 0x8002;
    //  ILR = 0x03;
    // LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    turned_off = true;
    waked = false;
    PCON = 0x1E;
}

void sleep_from_reset()
{
    enter_wor();
    PINSEL0 = 0x000000C0; // EINT2, EINT3, PWM4, PWM2, SPI0, EINT1, UART0
    PCONP = 0;
    EXTWAKE = 0x8002;
    // LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    turned_off = true;
    waked = false;
    PCON = 0x1E;
}

int8_t extract_rf_packet(uint8_t* pack, uint8_t* cmd_no, uint8_t* value)
{
    if ((pack[0] == 0) && (pack[3] == 125)) {
        if (pack[2] == pack[4]) {
            *cmd_no = pack[1];
            *value = pack[2];
            return 0;
        }
        else
            return -2;
    }
    return -1;
}

/*uint8_t transmit_cca(uint8_t adr, uint8_t* dat, uint8_t qty){
  uint8_t i, ch_status;
  uint8_t err,cnt;
  
  str2uart("CCA:\n\r");
  str2uart("Entering SRX:\n\r");
  go_srx_state();
  str2uart("In rx-state, wait RSSI:\n\r");
  err = rf_wr_burst(adr, dat, qty);
  delay(5000); // 500us
  str2uart("Monitoring CCA...\n\r");
  //(ch_status & 0x40)||(!(ch_status & 0x10)));
  monitor_cca();
  str2uart("Channel is clear, STX...\n\r");
  str2uart("Trying STX...\n\r");
  
  execute_stx();
  str2uart("Transmit done\n\r");
  return err;
}*/

void start_pll()
{
    PLLCFG &= ~0x7F;
    PLLCFG |= 0x24; // M = 4, P=2   0x24  CCLK = 12*5 = 60 MHz Fcco = 192 MHz
    PLLCON |= 0x03; // PLL Enabled at 60 MHz
    PLLFEED = 0xAA;
    PLLFEED = 0x55;
    delay(1000);
}

void stop_pll()
{
    PLLCFG &= ~0x7F;
    PLLCON &= ~0x03; // PLL Enabled at 60 MHz
    PLLFEED = 0xAA;
    PLLFEED = 0x55;
    delay(1000);
}

unsigned int erase()
{
    struct iap_in iap; // IAP input parameters
    unsigned int result[16]; // IAP results
    //  unsigned int save_VicInt;                // for saving of interrupt enable register

    __disable_interrupt();

#ifdef BYPASS_IAP
    stop_pll(); // IAP requires to run without PLL
#endif

    iap.cmd = 50; // IAP Command: Prepare Sectors for Write
    iap.par[0] = 8; //get_secnum (start);         // start sector
    iap.par[1] = 8; //get_secnum (end);           // end sector
    iap_entry(&iap, result); // call IAP function
    if (result[0])
        goto exit; // an error occured?

    iap.cmd = 52; // IAP command: Erase Flash
    iap.par[0] = 8; //get_secnum (start);         // start sector
    iap.par[1] = 8; //get_secnum (end);           // end sector
    iap.par[2] = CCLK; // CPU clock
    iap_entry(&iap, result); // call IAP function

exit:

#ifdef BYPASS_IAP
    start_pll(); // start PLL
#endif

    //VICIntEnable = save_VicInt;              // enable interrupts
    __enable_interrupt();
    return (result[0]);
}

/*
 * Program *data to flash_addr. number of bytes specified by size
 * Return:  IAP error code (0 when OK)
 * Note: 
 */
unsigned int program(void* data, unsigned int size)
{
    struct iap_in iap; // IAP input parameters
    unsigned int result[16]; // IAP results
    // unsigned int save_VicInt,i;                // for saving of interrupt enable register

    __disable_interrupt();

#ifdef BYPASS_IAP
    stop_pll(); // IAP requires to run without PLL
#endif

    iap.cmd = 50; // IAP Command: Prepare Sectors for Write
    iap.par[0] = 8; //get_secnum (flash_addr);    // start sector
    iap.par[1] = iap.par[0]; // end Sektor
    iap_entry(&iap, result); // call IAP function
    if (result[0]) {
        LED1_ON;
        goto exit; // an error occured?
    }
    iap.cmd = 51; // IAP Command: Copy RAM to Flash
    iap.par[0] = 0x8000; //(unsigned int) flash_addr;  // destination-addr
    iap.par[1] = (unsigned int)data; // source-addr
    iap.par[2] = size; // number of bytes
    iap.par[3] = CCLK; // CPU clock
    iap_entry(&iap, result); // call IAP function

exit:

#ifdef BYPASS_IAP
    start_pll(); // start PLL
#endif

    //VICIntEnable = save_VicInt;              // enable interrupts
    __enable_interrupt();
    return (result[0]);
}
