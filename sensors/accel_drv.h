/***********************************************************************************
    Filename: accel_drv.h

    Платформенно-зависимые функции.
    
    (с) Онищенко Денис, июнь 2013, v 1.0
***********************************************************************************/

#ifndef ACCEL_DRV_H
#define ACCEL_DRV_H

#include <stdint.h>
#include "iolpc2142.h"
#include <inarm.h>

#define ACC_CS_ON FIO1CLR3 = 0x01
#define ACC_CS_OFF FIO1SET3 = 0x01

uint8_t acc_exchange(uint8_t adr, uint8_t dat); // Функция обмена по SPI, без прерываний
int8_t acc_init();
int8_t acc_get(int8_t* x, int8_t* y, int8_t* z);

#endif
