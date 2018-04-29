#include "uart_service.h"
//#pragma pack(1) //выравнивание на 1, 2, 4, 8 или 16 байт (указывается вместо n)
uint8_t RxState;
uint8_t TxState;

extern uart_t uart;

Parcel_t* txParcel = reinterpret_cast<Parcel_t*>(uart.tx_buf);
const Parcel_t* const rxParcel = reinterpret_cast<const Parcel_t*>(uart.rx_buf);

void RxPing()
{
    TransmitParcel(PING);
}

void RxImpulse()
{
    //    if ((uart.rx_buf[0] == '5') && (uart.rx_buf[2] == '1') && (uart.rx_buf[1] == ';') && (uart.rx_buf[3] == ';')) {
    //        if (stim_test_cnt == 0) {
    //            stimulate(1, stim_duration);
    //            stim_test_cnt = 1;
    //        }
    //        else {
    //            //stim_right(stim_time_set);
    //            tr_stim_cmd = 1;
    //            stim_test_cnt = 0;
    //        }
    //    }

    //            case 5:
    //                stimulate(1, stim_duration);
    //                send2uart(uart.rx_cmd, uart.rx_buf);
    //                break;
    TransmitParcel(IMPULSE);
}

void RxGetSettings()
{
    int16_t data;
    //    switch (rxParcel->data[0]) {
    //    case POWER:
    //        //data = settings.enabled;
    //        break;
    //    case IMPULSE_AMPLITUDE:
    //        //data = settings.impulseAmplitude;
    //        break;
    //    case DURATION:
    //        //data = settings.duration;
    //        break;
    //    case LEAD_TIME:
    //        //data = settings.leadTime;
    //        break;
    //    default:
    //        TransmitParcel(WRONG_COMMAND);
    //        return;
    //    }
    TransmitParcel(GET_SETTINGS, reinterpret_cast<uint8_t*>(&data), sizeof(int16_t));
}

void RxSetSettings()
{
    const SkiSettingsData_t* const data = reinterpret_cast<const SkiSettingsData_t*>(rxParcel->data);
    //    switch (data->type) {
    //    case POWER:
    //        // stim enable or disable
    //        if ((uart.rx_buf[0] == '1') && (uart.rx_buf[2] == '1') && (uart.rx_buf[1] == ';') && (uart.rx_buf[3] == ';')) {
    //            if (uart.rx_buf[4] == '1') {
    //                stim.enabled = true;
    //            }
    //            else {
    //                stim.enabled = false;
    //            }
    //        }
    //        //settings.enabled = data->value;
    //        break;
    //    case IMPULSE_AMPLITUDE:
    //        // set voltage
    //        if ((uart.rx_buf[0] == '2') && (uart.rx_buf[2] == '1') && (uart.rx_buf[1] == ';') && (uart.rx_buf[3] == ';')) {
    //            if ((uart.rx_buf[5] == ';') || (uart.rx_buf[6] == ';') || (uart.rx_buf[7] == ';')) {
    //                if (uart.rx_buf[7] == ';')
    //                    stim.v_set = (uart.rx_buf[4] - 48) * 100 + (uart.rx_buf[5] - 48) * 10 + (uart.rx_buf[6] - 48);
    //                else {
    //                    if (uart.rx_buf[5] == ';')
    //                        stim.v_set = uart.rx_buf[4] - 48;
    //                    else
    //                        stim.v_set = (uart.rx_buf[4] - 48) * 10 + (uart.rx_buf[5] - 48);
    //                }
    //            }
    //            if (stim.v_set < V_MIN)
    //                stim.v_set = V_MIN;
    //            if (stim.v_set > V_MAX)
    //                stim.v_set = V_MAX;
    //            pwm = (stim.v_set - V_MIN) / V_STEP;
    //            //pwm = uart.rx_buf[0];
    //            v_set = V_STEP * pwm + V_MIN;
    //        }
    //        //settings.impulseAmplitude = data->value;
    //        break;
    //    case DURATION:
    //        // set duration
    //        if ((uart.rx_buf[0] == '3') && (uart.rx_buf[2] == '1') && (uart.rx_buf[1] == ';') && (uart.rx_buf[3] == ';')) {
    //            if ((uart.rx_buf[5] == ';') || (uart.rx_buf[6] == ';') || (uart.rx_buf[7] == ';')) {
    //                if (uart.rx_buf[7] == ';')
    //                    stim.time_ms = (uart.rx_buf[4] - 48) * 100 + (uart.rx_buf[5] - 48) * 10 + (uart.rx_buf[6] - 48);
    //                else {
    //                    if (uart.rx_buf[5] == ';')
    //                        stim.time_ms = uart.rx_buf[4] - 48;
    //                    else
    //                        stim.time_ms = (uart.rx_buf[4] - 48) * 10 + (uart.rx_buf[5] - 48);
    //                }
    //            }
    //            stim_duration = stim.time_ms / 8;
    //        }
    //        //settings.duration = data->value;
    //        break;
    //    case LEAD_TIME:
    //        // set phase
    //        /*if ((uart.rx_buf[0] == '4') && (uart.rx_buf[2] == '1') && (uart.rx_buf[1] == ';') && (uart.rx_buf[3] == ';')){
    //              if ((uart.rx_buf[5] == ';') || (uart.rx_buf[6] == ';') || (uart.rx_buf[7] == ';')){

    //                if (uart.rx_buf[7] == ';')
    //                  //stim.time_ms = (uart.rx_buf[4]-48)*100+(uart.rx_buf[5]-48)*10+(uart.rx_buf[6]-48);
    //                  stim.angle = -1*((uart.rx_buf[5]-48)*10+(uart.rx_buf[6]-48));
    //                else{
    //                  if (uart.rx_buf[5] == ';')
    //                    stim.angle = uart.rx_buf[4]-48;
    //                  else{
    //                    //stim.time_ms = (uart.rx_buf[4]-48)*10+(uart.rx_buf[5]-48);
    //                    if (uart.rx_buf[4] == '-'){
    //                      stim.angle = -1*(uart.rx_buf[5]-48);
    //                    }
    //                    else{
    //                      stim.angle = (uart.rx_buf[4]-48)*10+(uart.rx_buf[5]-48);
    //                    }
    //                  }
    //                }
    //              }
    //            }*/
    //        //settings.leadTime = data->value;
    //        break;
    //    default:
    //        TransmitParcel(WRONG_COMMAND);
    //        return;
    //    }
    TransmitParcel(SET_SETTINGS);
}
void RxGetStatistical()
{
    TransmitParcel(GET_STATISTICAL);
}

void RxSetSnifferMode()
{
    //            case 8:
    //                if (uart.rx_buf[0] == 1) {
    //                    accepted = 0;
    //                    declined = 0;
    //                    rf_snif = true;
    //                }
    //                else
    //                    rf_snif = false;
    //                break;
    TransmitParcel(SET_SNIFFER_MODE);
}
void RxSetSleepMode()
{
    //            case 7:
    //                if (uart.rx_buf[0] == 1)
    //                    turned_off = true;
    //                LED1_OFF;
    //                break;
    TransmitParcel(SET_SLEEP_MODE);
}

void RxGetSystemInfo()
{
    SkiSysInfo_t sysInfo;
    //    sysInfo.accelerometer = 1; // acc_init();
    //    sysInfo.gyroscope = 2; // hyro_init();
    //    sysInfo.rf = 3; // rf_init();
    //    sysInfo.battery_voltage = v_bat * 0.26;
    //    sysInfo.impulse_voltage = volt / 2.48;
    //    sysInfo.master = IS_MASTER;
    //    sysInfo.version = VERSION;
    TransmitParcel(GET_SYSTEM_INFO, reinterpret_cast<uint8_t*>(&sysInfo), sizeof(sysInfo));
}

void RxGetAccAndGyrData()
{
    SkiAccGyr_t accGyr;
    //    accGyr.acc.x = x1;
    //    accGyr.acc.y = y1;
    //    accGyr.acc.z = z1;
    //    accGyr.gyr.x = x2;
    //    accGyr.gyr.y = y2;
    //    accGyr.gyr.z = z2;
    TransmitParcel(GET_ACC_AND_GYR_DATA, reinterpret_cast<uint8_t*>(&accGyr), sizeof(accGyr));
}
void RxGetLevelAndDurationOfStimulation()
{
    TransmitParcel(GET_LEVEL_AND_DURATION_OF_STIMULATION);
}
void RxSetLevelAndDurationOfStimulation()
{
    TransmitParcel(SET_LEVEL_AND_DURATION_OF_STIMULATION);
}

void RxGetRfSettings()
{
    //            case 3 | 0x80:
    //                for (i = 0; i < 6; i++)
    //                    uart.tx_buf[i] = *(reinterpret_cast<unsigned char*>(0x08000 + i));
    //                send2uart(uart.rx_cmd, uart.tx_buf);
    //                break;
    TransmitParcel(GET_RF_SETTINGS);
}

void RxSetRfSettings()
{
    //            case 3:
    //                LED3_ON;
    //                for (i = 0; i < 6; i++)
    //                    vals[i] = uart.rx_buf[i];
    //                erase();
    //                program(vals, sizeof(vals));
    //                rf_init();
    //                ADR = *(reinterpret_cast<unsigned char*>(0x08005)) | ((!IS_MASTER) << 7);
    //                send2uart(uart.rx_cmd, uart.rx_buf);
    //                LED3_OFF;
    //                break;
    TransmitParcel(SET_RF_SETTINGS);
}

void RxGetDate()
{
    SkiDate_t date;
    date.year = YEAR;
    date.month = MONTH;
    date.dom = DOM;
    date.hour = HOUR;
    date.minute = MIN;
    date.second = SEC;
    TransmitParcel(SET_DATE, reinterpret_cast<uint8_t*>(&date), sizeof(date));
}
void RxSetDate()
{
    //    const SkiDate_t* const date = reinterpret_cast<const SkiDate_t*>(rxParcel->data);
    //    CCR = 0x2; // Reset the clock
    //    ILR = 0x3; // Clear the Interrupt Location Register
    //    YEAR = date->year;
    //    MONTH = date->month;
    //    DOM = date->dom;
    //    HOUR = date->hour;
    //    MIN = date->minute;
    //    SEC = date->second;
    //    CIIR = 0x01;
    //    CCR = 0x11;
    //    PCONP &= ~0x0200;
    TransmitParcel(SET_DATE);
}
void RxGetPwmAndDuration()
{
    //            case 6 | 0x80:
    //                uart.tx_buf[0] = pwm;
    //                uart.tx_buf[1] = 0;
    //                if (stim_duration <= 12)
    //                    uart.tx_buf[1] = 0;
    //                if ((stim_duration > 12) && (stim_duration <= 18))
    //                    uart.tx_buf[1] = 1;
    //                if ((stim_duration > 18) && (stim_duration <= 25))
    //                    uart.tx_buf[1] = 2;
    //                if ((stim_duration > 25) && (stim_duration <= 31))
    //                    uart.tx_buf[1] = 3;
    //                if ((stim_duration > 31) && (stim_duration <= 37))
    //                    uart.tx_buf[1] = 4;
    //
    //                for (i = 2; i < UART_PC_BUF_SIZE; i++)
    //                    uart.tx_buf[i] = 0;
    //                send2uart(uart.rx_cmd, uart.tx_buf);
    //                break;
    TransmitParcel(GET_PWM_AND_DURATION);
}
void RxSetPwmAndDuration()
{
    //            case 6:
    //                pwm = uart.rx_buf[0];
    //                v_set = V_STEP * pwm + V_MIN;
    //                if (v_set > V_MAX)
    //                    v_set = V_MAX;
    //
    //                // set_pwm(pwm);
    //                stim_duration = 12;
    //                if (uart.rx_buf[1] == 0)
    //                    stim_duration = 12;
    //                if (uart.rx_buf[1] == 1)
    //                    stim_duration = 18;
    //                if (uart.rx_buf[1] == 2)
    //                    stim_duration = 25;
    //                if (uart.rx_buf[1] == 3)
    //                    stim_duration = 31;
    //                if (uart.rx_buf[1] == 4)
    //                    stim_duration = 37;
    //                break;
    TransmitParcel(SET_PWM_AND_DURATION);
}

////////////////////////////////////////////////////////////////////
static void (*const Callbacks[])(void) = {
    RxPing,
    RxImpulse,
    RxGetSettings,
    RxSetSettings,
    RxGetStatistical,
    RxSetSnifferMode,
    RxSetSleepMode,
    RxGetSystemInfo,
    RxGetAccAndGyrData,
    RxGetLevelAndDurationOfStimulation,
    RxSetLevelAndDurationOfStimulation,
    RxGetRfSettings,
    RxSetRfSettings,
    RxGetDate,
    RxSetDate,
    RxGetPwmAndDuration,
    RxSetPwmAndDuration,
};

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

void StartTransmit()
{
    TxState = TX_DATA_IS_TRANSMITTED;
    while (TxState)
        TransmitIsr();
}

void TransmitParcel(const uint8_t command, const uint8_t* const data, const uint8_t len)
{
    while (TxState)
        ;
    txParcel->start0 =TX_START0; 
    txParcel->start1 =TX_START1;
    txParcel->size = len + MIN_LENGHT;
    txParcel->command = command;
    for (uint8_t i = 0; i < len; ++i)
        txParcel->data[i] = data[i];
    txParcel->data[len] = CalcCrc(uart.tx_buf); //crc
    StartTransmit();
}

void TransmitParcel(const uint8_t command)
{
    while (TxState)
        ;    
    txParcel->start0 =TX_START0; 
    txParcel->start1 =TX_START1;
    txParcel->size = MIN_LENGHT;
    txParcel->command = command;
    txParcel->data[0] = CalcCrc(uart.tx_buf); //crc
    StartTransmit();
}

void TransmitText(const uint8_t* data)
{
    while (TxState)
        ;
    uint8_t i = 0;
    txParcel->start0 =TX_START0; 
    txParcel->start1 =TX_START1;
    txParcel->command = TEXTUAL_PARCEL;
    while (data[i])
        txParcel->data[i] = data[i++]; //
    txParcel->size = i + 1 + MIN_LENGHT;
    txParcel->data[i] = CalcCrc(uart.tx_buf); //crc
    StartTransmit();
}

uint8_t CheckRxParcel()
{
    return (rxParcel->data[rxParcel->size - MIN_LENGHT] == CalcCrc(uart.rx_buf));
}

uint8_t CalcCrc(uint8_t* data)
{
    uint8_t crc8 = 0;
    const uint8_t len = data[2] - 1;
    uint8_t i = 0, j = 0;
    while (i < len) {
        crc8 ^= data[i++];
        j = 0;
        while (j++ < 8) {
            crc8 = (crc8 & 0x80) ? (crc8 << 1) ^ POLYNOMIAL : crc8 << 1;
        };
    }
    return crc8;
}

void TransmitIsr()
{
    static uint8_t txCounter = 0;
    if (uart.tx_buf[2] == txCounter) {
        txCounter = 0;
        TxState = TX_RESET;
        return;
    }
    while (!U0LSR_bit.THRE)
        ;
    U0THR = uart.tx_buf[txCounter++];
}

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
//#pragma pack()  //восстанавливает состояние по умолчанию (выравнивание на 32 бита)
