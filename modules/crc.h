#ifndef __CRC_INCLUDED
#define __CRC_INCLUDED

#include <stdint.h>

uint16_t CalcCRC16Table(uint16_t Length, uint8_t* CheckBuf);

#endif
