/***********************************************************************************
    Filename: cc1101_drv.h
    Платформенно-зависимые функции.
    (с) Онищенко Денис, март 2013, v 1.0
***********************************************************************************/

#ifndef CC1101_DRV_H
#define CC1101_DRV_H

#include <stdint.h>
#include "iolpc2142.h"
#include <inarm.h>
#include "main.h"

#define DELAY_100us delay(1000)
#define RF_CS_ON         FIO1CLR3 = 0x02        // Ножка сигнала nCS (запись)
#define RF_CS_OFF        FIO1SET3 = 0x02
#define RF_nRDY          (FIO0PIN0 & 0x20)      // Ножка сигнала MISO контроллера (чтение)
#define RF_IRQ_ACTIVE    (FIO0PIN0 & 0x08)      // Ножка сигнала GDO = 0x06 радиомодуля
                                                // =прерывание при приеме или передаче
/*#define RF_CS_ON PORTC.B1  = 0     
#define RF_CS_OFF PORTC.B1 = 1
#define RF_nRDY PORTC.B4           // Ножка сигнала MISO контроллера (чтение)
#define RF_IRQ_ACTIVE PORTC.B6     // Ножка сигнала GDO = 0x06 радиомодуля
                                   // =прерывание при приеме или передаче*/

typedef enum {
    RF_OK = 0,                       // Функция выполнена без ошибок
    RF_ERR_NOT_READY,                // Радиомодуль не готов (не запустился кварц или встроенный опорник)
    RF_ERR_SPI,                      // Радиомодуль неправильно отвечает по SPI (КЗ или неправильно настроен SPI)
    RF_ERR_WR_SIZE,                  // При записи более 64 байт FIFO
    RF_ERR_TIMEOUT
} RF_ERR;

void rf_pwr_on_reset(void);             // Перезапуск радиомодуля при включении.
uint8_t rf_exchange(uint8_t snd_byte);  // Функция обмена по SPI, без прерываний
RF_ERR rf_wait_ready(void);             // Функция ожидания низкого уровня на ножке MISO
                                        // низкий уровень означает готовность передатчика


#endif
