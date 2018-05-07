#include "uart_service.h"
#include "main.h"

uint8_t RxState;
uint8_t TxState;

const uint8_t crcArray[] = {
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53,
    0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
    0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4,
    0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19,
    0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
    0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D,
    0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7,
    0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
    0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75,
    0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8,
    0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
    0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F,
    0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66,
    0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
    0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1,
    0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C,
    0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

extern uart_t uart;

Parcel_t* const txParcel = reinterpret_cast<Parcel_t* const>(uart.tx_buf);
const Parcel_t* const rxParcel = reinterpret_cast<const Parcel_t*>(uart.rx_buf);

////////////////////////////////////////////////////////////////////
/// \brief RxPing
///
void RxPing()
{
    SysInfo_t sysInfo;
    TransmitParcel(PING, &sysInfo.version, sizeof(uint8_t));
}
////////////////////////////////////////////////////////////////////
/// \brief RxSetGetRfSettings
///
void RxSetGetRfSettings()
{
    RfSettings_t rf;
    if (rxParcel->size == MIN_LENGHT) {
        TransmitParcel(SET_GET_RF_SETTINGS, reinterpret_cast<uint8_t*>(&rf), sizeof(RfSettings_t));
    }
    else {
        rf = *reinterpret_cast<const RfSettings_t*>(rxParcel->data);
        TransmitParcel(SET_GET_RF_SETTINGS);
    }
}
////////////////////////////////////////////////////////////////////
/// \brief RxSetGetStimulationSettings
///
void RxSetGetStimulationSettings()
{
    StimulationSettings_t stimulationSettings;
    if (rxParcel->size == MIN_LENGHT) {
        TransmitParcel(SET_GET_STIMULATION_SETTINGS, reinterpret_cast<uint8_t*>(&stimulationSettings), sizeof(StimulationSettings_t));
    }
    else {
        stimulationSettings = *reinterpret_cast<const StimulationSettings_t*>(rxParcel->data);
        TransmitParcel(SET_GET_STIMULATION_SETTINGS);
    }
}
////////////////////////////////////////////////////////////////////
/// \brief RxSetGetStatistics
///
void RxSetGetStatistics()
{
    GetStatistics_t statisticsTraining;
    GetStatistics_t statisticsPause;
    switch (rxParcel->data[0]) {
    case RESET:
        memset(&statisticsTraining, 0, sizeof(GetStatistics_t));
        memset(&statisticsPause, 0, sizeof(GetStatistics_t));
        TransmitParcel(SET_GET_STATISTICS);
        return;
    case GET_TRAINING:
        TransmitParcel(SET_GET_STATISTICS, reinterpret_cast<uint8_t*>(&statisticsTraining), sizeof(GetStatistics_t));
        return;
    case GET_PAUSE:
        TransmitParcel(SET_GET_STATISTICS, reinterpret_cast<uint8_t*>(&statisticsPause), sizeof(GetStatistics_t));
        return;
    }
    TransmitParcel(WRONG_COMMAND);
}
////////////////////////////////////////////////////////////////////
/// \brief RxSetGetDateTime
///
void RxSetGetDateTime()
{
    if (rxParcel->size == MIN_LENGHT) {
        DateTime_t date;
        date.year = YEAR;
        date.month = MONTH;
        //date.day = DAY;
        date.hour = HOUR;
        date.minute = MIN;
        date.second = SEC;
        date.dom = DOM;
        TransmitParcel(SET_GET_DATE_TIME, reinterpret_cast<uint8_t*>(&date), sizeof(DateTime_t));
    }
    else {
        const DateTime_t* date = reinterpret_cast<const DateTime_t*>(rxParcel->data);
        YEAR = date->year;
        MONTH = date->month;
        //DAY = date->day;
        HOUR = date->hour;
        MIN = date->minute;
        SEC = date->second;
        DOM = date->dom;
        TransmitParcel(SET_GET_DATE_TIME);
    }
}
////////////////////////////////////////////////////////////////////
/// \brief RxGetBattery
///
void RxGetBattery()
{
    Battery_t battery;
    TransmitParcel(GET_BATTERY, reinterpret_cast<uint8_t*>(&battery), sizeof(Battery_t));
}
////////////////////////////////////////////////////////////////////
/// \brief RxGetAccGyr
///
void RxGetAccGyr()
{
    AccGyr_t accGyr;
    switch (rxParcel->data[0]) {
    case OFF: //
        TransmitParcel(GET_ACC_GYR);
        return;
    case ON: // отправлять каждое измерение
        TransmitParcel(GET_ACC_GYR);
        return;
    case SINGLE:
        TransmitParcel(GET_ACC_GYR, reinterpret_cast<uint8_t*>(&accGyr), sizeof(AccGyr_t));
        return;
    }
    TransmitParcel(WRONG_COMMAND);
}
////////////////////////////////////////////////////////////////////
/// \brief RxGetStatus
///
void RxGetStatus()
{
    SysInfo_t sysInfo;
    TransmitParcel(GET_STATUS, reinterpret_cast<uint8_t*>(&sysInfo), sizeof(SysInfo_t));
}
////////////////////////////////////////////////////////////////////
/// \brief RxImpulse
///
void RxImpulse()
{
    //подать импульс
    TransmitParcel(IMPULSE);
}
////////////////////////////////////////////////////////////////////
/// \brief RxOnOff
///
void RxOnOff()
{
    uint8_t onOff;
    if (rxParcel->size == MIN_LENGHT) {
        TransmitParcel(ON_OFF, &onOff, sizeof(uint8_t));
        return;
    }
    else {
        switch (rxParcel->data[0]) {
        case OFF:
            //выключить импульсы
            TransmitParcel(ON_OFF);
            return;
        case ON:
            //включить импульсы
            TransmitParcel(ON_OFF);
            return;
        }
        TransmitParcel(WRONG_COMMAND);
    }
}
////////////////////////////////////////////////////////////////////
static void (*const Callbacks[])(void) = {
    RxPing,
    RxSetGetRfSettings,
    RxSetGetStimulationSettings,
    RxSetGetStatistics,
    RxSetGetDateTime,
    RxGetBattery,
    RxGetAccGyr,
    RxGetStatus,
    RxImpulse,
    RxOnOff
};
////////////////////////////////////////////////////////////////////
/// \brief UartService
///
void UartService()
{
    switch (RxState) {
    case RX_DATA_READY:
        if (CheckRxParcel()) {
            if (rxParcel->command < BUFFER_OVERFLOW)
                Callbacks[rxParcel->command]();
            else
                TransmitParcel(WRONG_COMMAND);
            break;
        }
    case RX_CRC_ERROR:
        TransmitParcel(CRC_ERROR);
        break;
    case RX_BUFFEROVERFLOW:
        TransmitParcel(BUFFER_OVERFLOW);
        break;
    case RX_DATA_IS_RECEIVED:
    default:
        return;
    }
    RxState = RX_RESET;
}
////////////////////////////////////////////////////////////////////
/// \brief StartTransmit
///
void StartTransmit()
{
    TxState = TX_DATA_IS_TRANSMITTED;
    while (TxState)
        TransmitIsr();
}
////////////////////////////////////////////////////////////////////
/// \brief TransmitParcel
/// \param command
/// \param data
/// \param len
///
void TransmitParcel(const uint8_t command, const uint8_t* const data, const uint8_t len)
{
    while (TxState)
        ;
    txParcel->start0 = TX_START0;
    txParcel->start1 = TX_START1;
    txParcel->size = len + MIN_LENGHT;
    txParcel->command = command;
    for (uint8_t i = 0; i < len; ++i)
        txParcel->data[i] = data[i];
    txParcel->data[len] = CalcCrc(uart.tx_buf); //crc
    StartTransmit();
}
////////////////////////////////////////////////////////////////////
/// \brief TransmitParcel
/// \param command
///
void TransmitParcel(const uint8_t command)
{
    while (TxState)
        ;
    txParcel->start0 = TX_START0;
    txParcel->start1 = TX_START1;
    txParcel->size = MIN_LENGHT;
    txParcel->command = command;
    txParcel->data[0] = CalcCrc(uart.tx_buf); //crc
    StartTransmit();
}
////////////////////////////////////////////////////////////////////
/// \brief TransmitText
/// \param data
///
void TransmitText(const uint8_t* data)
{
    while (TxState)
        ;
    uint8_t i = 0;
    txParcel->start0 = TX_START0;
    txParcel->start1 = TX_START1;
    txParcel->command = TEXTUAL_PARCEL;
    while (data[i])
        txParcel->data[i] = data[i++]; //
    txParcel->size = i + 1 + MIN_LENGHT;
    txParcel->data[i] = CalcCrc(uart.tx_buf); //crc
    StartTransmit();
}
////////////////////////////////////////////////////////////////////
/// \brief CheckRxParcel
/// \return
///
uint8_t CheckRxParcel()
{
    return (rxParcel->data[rxParcel->size - MIN_LENGHT] == CalcCrc(uart.rx_buf));
}
////////////////////////////////////////////////////////////////////
/// \brief CalcCrc
/// \param data
/// \return
///
uint8_t CalcCrc(uint8_t* data)
{
    uint8_t crc8 = 0;
    const uint8_t len = data[2] - 1;
    uint8_t i = 0, j = 0;
    while (i < len) {
        crc8 ^= data[i++];
        crc8 = crcArray[crc8];
        //        j = 0;
        //        while (j++ < 8) {
        //            crc8 = (crc8 & 0x80) ? (crc8 << 1) ^ POLYNOMIAL : crc8 << 1;
        //        };
    }
    return crc8;
}
////////////////////////////////////////////////////////////////////
/// \brief TransmitIsr
///
void TransmitIsr()
{
    static uint8_t txCounter = 0;
    if (uart.tx_buf[2] == txCounter) {
        txCounter = 0;
        TxState = TX_RESET;
        return;
    }
    while (!U0LSR_bit.THRE)
        continue;
    U0THR = uart.tx_buf[txCounter++];
}
////////////////////////////////////////////////////////////////////
/// \brief ReceiveIsr
///
void ReceiveIsr()
{
    static uint8_t rxCounter = 0;
    uint8_t RCREG = U0RBR;

    uart.rx_buf[rxCounter] = RCREG;
    switch (rxCounter) {
    case 0:
        if (RCREG == 0x55)
            ++rxCounter;
        else
            RxState = RX_RESET;
        break;
    case 1:
        if (RCREG == 0xAA)
            ++rxCounter;
        else
            rxCounter = 0;
        break;
    default:
        ++rxCounter;
        RxState = RX_DATA_IS_RECEIVED;
        if (uart.rx_buf[2] <= EUSART_BUFFER_SIZE) {
            if (uart.rx_buf[2] == rxCounter) {
                RxState = RX_DATA_READY;
                rxCounter = 0;
            }
        }
        else {
            RxState = RX_BUFFEROVERFLOW;
            rxCounter = 0;
        }
    }
}
////////////////////////////////////////////////////////////////////
/// \brief TransmitAccGyr
/// \param accGyr
///
void TransmitAccGyr(const AccGyr_t& accGyr)
{
    TransmitParcel(GET_ACC_GYR, reinterpret_cast<const uint8_t*>(&accGyr), sizeof(AccGyr_t));
}
