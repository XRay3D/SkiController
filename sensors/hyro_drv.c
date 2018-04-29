/***********************************************************************************
    Filename: cc1101_drv.c

    Платформенно-зависимые функции.

    (с) Онищенко Денис, июнь 2013, v 1.0
***********************************************************************************/

#include "hyro_drv.h"
#include "../app/main.h"

// Функция обмена по SPI, без прерываний
uint8_t hyro_exchange(uint8_t adr, uint8_t dat)
{ // Сюда вставить конкретную функцию обмена по
    unsigned char data; // SPI для данного процессора
    __disable_interrupt(); // Отключаем прерывания
    S0SPCR |= 0x0018;
    HYRO_CS_ON;
    S0SPDR = adr;
    while (!(S0SPSR & 0x80))
        ;
    data = S0SPDR;
    S0SPDR = dat;
    while (!(S0SPSR & 0x80))
        ;
    data = S0SPDR;
    delay(5);
    HYRO_CS_OFF;
    S0SPCR &= ~0x0018;
    __enable_interrupt();
    return data;
}

int8_t hyro_init()
{
    if (hyro_exchange(0x8F, 0x00) != 0xD3)
        return -1;
    hyro_exchange(0x20, 0xAF); // 0xC7
    hyro_exchange(0x21, 0x21);
    hyro_exchange(0x22, 0x00);
    hyro_exchange(0x23, 0x90); // 0x80 - 250 dps, 0x90 - 500 dps
    hyro_exchange(0x24, 0x02);

    /* hyro_exchange(0x20, 0x8F);  // 0xC7
  hyro_exchange(0x21, 0x21);
  hyro_exchange(0x22, 0x00);
  hyro_exchange(0x23, 0x90);
  hyro_exchange(0x24, 0x02);*/
    return 0;
}

int8_t hyro_get(int8_t* x, int8_t* y, int8_t* z)
{
    hyro_exchange(0x28 | 0x80, 0);
    *x = hyro_exchange(0x29 | 0x80, 0);
    hyro_exchange(0x2A | 0x80, 0);
    *y = hyro_exchange(0x2B | 0x80, 0);
    hyro_exchange(0x2C | 0x80, 0);
    *z = hyro_exchange(0x2D | 0x80, 0);
    return 0;
}
