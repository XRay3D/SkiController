#ifndef UART_SERVICE_H
#define UART_SERVICE_H

#include <stdint.h>
#include "includes.h"

enum COMMAND {
    //Service functions
    PING, // return SOFTWARE_VERSION
    //User functions
    SET_GET_RF_SETTINGS,
    SET_GET_STIMULATION_SETTINGS,
    SET_GET_STATISTICS, //Statistics
    SET_GET_DATE_TIME,
    GET_BATTERY, //left + right
    GET_ACC_GYR, //AccGyr
    GET_STATUS,
    IMPULSE,
    ON_OFF,
    //Service functions
    BUFFER_OVERFLOW,
    WRONG_COMMAND,
    TEXTUAL_PARCEL, //for debug (may be)
    CRC_ERROR,
};

enum AccGyr {
    OFF,
    ON,
    SINGLE,
};

enum Statistics {
    RESET,
    GET_TRAINING,
    GET_PAUSE,
};
enum {
    RX_START = 0xAA55,
    TX_START = 0x55AA,
    TX_START0 = 0xAA,
    TX_START1 = 0x55,
};

enum {
    MIN_LENGHT = 5,
    EUSART_BUFFER_SIZE = 40,
};

#pragma pack(1) //выравнивание на 1, 2, 4, 8 или 16 байт (указывается вместо n)
typedef struct RfSettings_t {
    uint8_t channel;
    uint8_t address;
} RfSettings_t;

typedef struct Battery_t {
    uint8_t left;
    uint8_t right;
} Battery_t;

typedef struct StimulationSettings_t {
    uint8_t voltage;
    uint16_t duration;
    uint16_t delay;
    //int16_t leadTime;
} StimulationSettings_t;

typedef struct AccGyr_t { // считать состояния чипов, напряжения, версию прошивки
    struct {
        int8_t x;
        int8_t y;
        int8_t z;
    } acc;
    struct {
        int8_t x;
        int8_t y;
        int8_t z;
    } gyr;
} AccGyr_t;

typedef struct GetStatistics_t { // считать состояния чипов, напряжения, версию прошивки
    uint32_t timePause;
    uint32_t timeStimulatiion;
    float averageAmplitude;
    uint16_t steps;
} GetStatistics_t;

typedef struct SysInfo_t { //STATUS// считать состояния чипов, напряжения, версию прошивки
    uint8_t acc;
    uint8_t gyr;
    uint8_t rf;
    uint8_t battery_voltage;
    uint8_t impulse_voltage;
    uint8_t nLeft_right;
    uint8_t version;
} SysInfo_t;

typedef struct DateTime_t { // считать состояния чипов, напряжения, версию прошивки
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t dom;
} DateTime_t;

typedef struct Parcel_t {
    uint8_t start0;
    uint8_t start1;
    uint8_t size;
    uint8_t command;
    uint8_t data[0xFF];
    uint8_t crc() { return data[size - MIN_LENGHT]; }
} Parcel_t;

#pragma pack() //восстанавливает состояние по умолчанию (выравнивание на 32 бита)
/////////////////////

enum { POLYNOMIAL = 0x1D }; // x^8 + x^4 + x^3 + x^2 + 1

typedef enum {
    RX_RESET,
    RX_DATA_READY,
    RX_CRC_ERROR,
    RX_BUFFEROVERFLOW,
    RX_DATA_IS_RECEIVED,

    TX_RESET = 0,
    TX_DATA_IS_TRANSMITTED,
    TX_DATA_TRANSMIT_EXPECTED
} EusartState_e;

void UartService(void);

void TransmitAccGyr(const AccGyr_t& accGyr);

void TransmitParcel(const uint8_t command, const uint8_t* const data, const uint8_t len);
void TransmitParcel(const uint8_t command);
void TransmitText(const uint8_t* data);
uint8_t CheckRxParcel();
uint8_t CalcCrc(uint8_t* data);
void TransmitIsr(void);
void ReceiveIsr(void);

#endif // UART_SERVICE_H
