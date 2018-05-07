/***********************************************************************************
    Filename: cc1101_bsp.h

    Платформенно-независимые функции.

    (с) Онищенко Денис, март 2013, v 1.0
***********************************************************************************/

#ifndef CC1101_BSP_H
#define CC1101_BSP_H

#include "cc1101_reg.h"
#include "cc1101_drv.h"

#define RF_BUF_SIZE 64
#define USER_RF_BUF 8 //16
#define WAKEUP_BUF 3
#define CHANNEL *(reinterpret_cast<unsigned char*>(0x08004)) //20
#define RF_TIMEOUT 1000 // максимальный тайм-аут операции радиомодуля - 100 мс

RF_ERR rf_test_exchange(void); // Проверяет, есть ли обмен между микроконтроллером и радиомодулем по SPI
RF_ERR rf_init(void); // Инициализация радиомодуля
RF_ERR rf_init_rec(void);
RF_ERR rf_wr_strobe(uint8_t adr); // Запись строба в радиомодуль
RF_ERR rf_wr_reg(uint8_t adr, uint8_t dat); // Запись регистра
RF_ERR rf_rd_reg(uint8_t adr, uint8_t* dat); // Чтение регистра
RF_ERR rf_wr_burst(uint8_t adr, uint8_t* dat, uint8_t qty); // Запись нескольких байт за 1 раз
RF_ERR rf_rd_burst(uint8_t adr, uint8_t* dat, uint8_t qty); // Чтение нескольких байт за 1 раз

RF_ERR poll_satus(uint8_t strobe, uint8_t status);
RF_ERR go_idle_state();
RF_ERR go_srx_state();
RF_ERR execute_stx();
RF_ERR monitor_cca();
RF_ERR transmit_cca(uint8_t adr, uint8_t* dat, uint8_t qty);

RF_ERR enter_wor();
RF_ERR re_enter_wor();
RF_ERR exit_wor();
RF_ERR rf_wakeup();

#endif
