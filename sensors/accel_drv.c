/***********************************************************************************
    Filename: accel_drv.c

    Платформенно-зависимые функции.

    (с) Онищенко Денис, июнь 2013, v 1.0
***********************************************************************************/

#include "accel_drv.h"
#include "../app/main.h"

// Функция обмена по SPI, без прерываний
uint8_t acc_exchange(uint8_t adr, uint8_t dat)
{ // Сюда вставить конкретную функцию обмена по
    unsigned char data; // SPI для данного процессора
    __disable_interrupt(); // Отключаем прерывания
    S0SPCR |= 0x0018;
    ACC_CS_ON;
    S0SPDR = adr;
    while (!(S0SPSR & 0x80))
        ;
    data = S0SPDR;
    S0SPDR = dat;
    while (!(S0SPSR & 0x80))
        ;
    data = S0SPDR;
    delay(5);
    ACC_CS_OFF;
    S0SPCR &= ~0x0018;
    __enable_interrupt();
    return data;
}

int8_t acc_init()
{
    if (acc_exchange(0x8F, 0x00) != 0x3B)
        return -1;
    acc_exchange(0x20, 0xC7); // 0xE7
    acc_exchange(0x21, 0x03); // 0x13
    acc_exchange(0x22, 0x00);

    /*  acc_exchange(0x20, 0xE7);  // 0xE7
  acc_exchange(0x21, 0x13);
  acc_exchange(0x22, 0x00);*/
    return 0;
}

int8_t acc_get(int8_t* x, int8_t* y, int8_t* z)
{
    *x = acc_exchange(0xA9, 0x00);
    *y = acc_exchange(0xAB, 0x00);
    *z = acc_exchange(0xAD, 0x00);
    return 0;
}
