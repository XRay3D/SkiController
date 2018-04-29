/***********************************************************************************
    Filename: hyro_drv.h

    Платформенно-зависимые функции.
    
    (с) Онищенко Денис, март 2013, v 1.0
***********************************************************************************/

#ifndef HYRO_DRV_H
#define HYRO_DRV_H

#include <stdint.h>
#include "iolpc2142.h"
#include <inarm.h>

#define HYRO_CS_ON FIO0CLR1 = 0x04
#define HYRO_CS_OFF FIO0SET1 = 0x04

uint8_t hyro_exchange(uint8_t adr, uint8_t dat); // Функция обмена по SPI, без прерываний
int8_t hyro_init();
int8_t hyro_get(int8_t* x, int8_t* y, int8_t* z);

#endif
