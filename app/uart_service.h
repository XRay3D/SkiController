#ifndef UART_SERVICE_H
#define UART_SERVICE_H

#include <stdint.h>
#include "includes.h"

enum COMMAND {
    //Service functions
    PING, //0 return SOFTWARE_VERSION
    //User functions
    IMPULSE, //1
    //
    GET_SETTINGS, //2
    SET_SETTINGS, //3
    //
    GET_STATISTICAL, //4
    //
    SET_SNIFFER_MODE, //5
    SET_SLEEP_MODE, //6
    //
    GET_SYSTEM_INFO, //7
    GET_ACC_AND_GYR_DATA, //8
    //
    GET_LEVEL_AND_DURATION_OF_STIMULATION, //9
    SET_LEVEL_AND_DURATION_OF_STIMULATION, //10
    //
    GET_RF_SETTINGS, //11
    SET_RF_SETTINGS, //12
    //
    GET_DATE, //13
    SET_DATE, //14
    //
    GET_PWM_AND_DURATION, //15
    SET_PWM_AND_DURATION, //16
    //Service functions
    BUFFER_OVERFLOW, //17
    WRONG_COMMAND, //18
    TEXTUAL_PARCEL, //19 for debug (may be)
    CRC_ERROR, //20
};

enum {
    POWER_OFF,
    POWER_ON,
};

enum SKI_SETTINGS {
    POWER,
    IMPULSE_AMPLITUDE, //1.2. Параметры импульса1.2.1. Установить амплитуду импульса (десятичное, Вольты)
    DURATION, //1.3. Продолжительность стимуляции 1.3.1. Целое число в мс от 100 до 300 мс
    LEAD_TIME, //1.4. Опережение включения/выключения момента стимуляции,//1.4.1. Знаковое целое, кол-во миллисекунд от -50 до 50 мс.
};

enum SKI_STATISTICAL {
    LEFT_BATTERY_CHARGE, //	2.1. Напряжение батареи 2 шт (левая и правая нога) (% заряда 0...100)
    RIGHT_BATTERY_CHARGE, //	2.1. Напряжение батареи 2 шт (левая и правая нога) (% заряда 0...100)
    SOFTWARE_VERSION, //	2.2. Версия ПО 1 шт. (строка)
    TIME_OF_PURE_STIMULATION, //	2.3. Время чистой стимуляции (секунды)
    AVERAGE_AMPLITUDE_OF_STIMULATION, //	2.4. Средняя амплитуда стимуляции (Вольты) - потом
    PURE_TRAINING_TIME_WITHOUT_PAUSES, //	2.5. Чистое время тренировки без пауз (секунды)
    TOTAL_TRAINING_TIME, //	2.6. Общее время тренировки (секунды)
    TOTAL_PAUSE_TIME, //	2.7. Общее время пауз (секунды)
    AVERAGE_FREQUENCY_OF_STEPS, //	2.9. Средняя частота шагов при стимуляции ??? (если нетрудоёмко)
};

#pragma pack(1) //выравнивание на 1, 2, 4, 8 или 16 байт (указывается вместо n)
typedef struct SkiSettings_t { // считать состояния чипов, напряжения, версию прошивки
    uint8_t power;
    uint16_t impulseAmplitude;
    uint16_t duration;
    int16_t leadTime;
} SkiSettings_t;

typedef struct SkiSettingsData_t {
    uint8_t type; //SKI_SETTINGS or SKI_STATISTICAL
    int16_t value;
} SkiSettingsData_t;

typedef struct SkiSysInfo_t { // считать состояния чипов, напряжения, версию прошивки
    uint8_t accelerometer;
    uint8_t gyroscope;
    uint8_t rf;
    uint8_t battery_voltage;
    uint8_t impulse_voltage;
    uint8_t master;
    uint8_t version;
} SkiSysInfo_t;

typedef struct SkiAccGyr_t { // считать состояния чипов, напряжения, версию прошивки
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
} SkiAccGyr_t;

typedef struct SkiRfSettings_t { // считать состояния чипов, напряжения, версию прошивки
    uint8_t freq0;
    uint8_t freq1;
    uint8_t freq2;
    uint8_t testb;
    uint8_t channel;
    uint8_t address;
} SkiRfSettings_t;

typedef struct SkiDate_t { // считать состояния чипов, напряжения, версию прошивки
    uint16_t year;
    uint8_t month;
    uint8_t dom;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} SkiDate_t;

typedef struct SkiPwmDuration_t { // считать состояния чипов, напряжения, версию прошивки
    uint8_t pwm;
    uint8_t duration;
} SkiPwmDuration_t;

typedef struct SkiStatistical_t { // считать состояния чипов, напряжения, версию прошивки
    uint8_t leftBatteryCharge; //	2.1. Напряжение батареи 2 шт (левая и правая нога) (% заряда 0...100)
    uint8_t rightBatteryCharge; //	2.1. Напряжение батареи 2 шт (левая и правая нога) (% заряда 0...100)
    uint8_t softwareVersion; //	2.2. Версия ПО 1 шт. (строка)
    uint16_t timeOfPureStimulation; //	2.3. Время чистой стимуляции (секунды)
    uint8_t averageAmplitudeOfStimulation; //	2.4. Средняя амплитуда стимуляции (Вольты) - потом
    uint16_t pureTrainingTimeWithoutPauses; //	2.5. Чистое время тренировки без пауз (секунды)
    uint16_t totalTrainingTime; //	2.6. Общее время тренировки (секунды)
    uint16_t totalPauseTime; //	2.7. Общее время пауз (секунды)
    uint8_t averageFrequencyOfSteps; //	2.9. Средняя частота шагов при стимуляции ??? (если нетрудоёмко)
} SkiStatistical_t;
#pragma pack()  //восстанавливает состояние по умолчанию (выравнивание на 32 бита)
/////////////////////
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

enum { POLYNOMIAL = 0x1D }; // x^8 + x^4 + x^3 + x^2 + 1

typedef struct Parcel_t {
    uint8_t start0;   
    uint8_t start1;
    uint8_t size;
    uint8_t command;
    uint8_t data[0xFF];
    uint8_t crc() { return data[size - 5 /*MIN_LENGHT*/]; }
} Parcel_t;

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
void TransmitParcel(const uint8_t command, const uint8_t* const data, const uint8_t len);
void TransmitParcel(const uint8_t command);
void TransmitText(const uint8_t* data);
uint8_t CheckRxParcel();
uint8_t CalcCrc(uint8_t* data);
void TransmitIsr(void);
void ReceiveIsr(void);

#endif // UART_SERVICE_H
