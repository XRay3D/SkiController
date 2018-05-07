/***********************************************************************************
    Filename: cc1101_drv.h
    ������������-��������� �������.
    (�) �������� �����, ���� 2013, v 1.0
***********************************************************************************/

#ifndef CC1101_DRV_H
#define CC1101_DRV_H

#include <stdint.h>
#include "iolpc2142.h"
#include <inarm.h>
#include "main.h"

#define DELAY_100us delay(1000)
#define RF_CS_ON         FIO1CLR3 = 0x02        // ����� ������� nCS (������)
#define RF_CS_OFF        FIO1SET3 = 0x02
#define RF_nRDY          (FIO0PIN0 & 0x20)      // ����� ������� MISO ����������� (������)
#define RF_IRQ_ACTIVE    (FIO0PIN0 & 0x08)      // ����� ������� GDO = 0x06 �����������
                                                // =���������� ��� ������ ��� ��������
/*#define RF_CS_ON PORTC.B1  = 0     
#define RF_CS_OFF PORTC.B1 = 1
#define RF_nRDY PORTC.B4           // ����� ������� MISO ����������� (������)
#define RF_IRQ_ACTIVE PORTC.B6     // ����� ������� GDO = 0x06 �����������
                                   // =���������� ��� ������ ��� ��������*/

typedef enum {
    RF_OK = 0,                       // ������� ��������� ��� ������
    RF_ERR_NOT_READY,                // ����������� �� ����� (�� ���������� ����� ��� ���������� �������)
    RF_ERR_SPI,                      // ����������� ����������� �������� �� SPI (�� ��� ����������� �������� SPI)
    RF_ERR_WR_SIZE,                  // ��� ������ ����� 64 ���� FIFO
    RF_ERR_TIMEOUT
} RF_ERR;

void rf_pwr_on_reset(void);             // ���������� ����������� ��� ���������.
uint8_t rf_exchange(uint8_t snd_byte);  // ������� ������ �� SPI, ��� ����������
RF_ERR rf_wait_ready(void);             // ������� �������� ������� ������ �� ����� MISO
                                        // ������ ������� �������� ���������� �����������


#endif
