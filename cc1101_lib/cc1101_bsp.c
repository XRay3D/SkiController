/***********************************************************************************
    Filename: cc1101_bsp.c

    Платформенно-независимые функции.

    (с) Онищенко Денис, март 2013, v 1.0
***********************************************************************************/

#include "cc1101_drv.h"
#include "cc1101_bsp.h"

// Проверяет, есть ли обмен между микроконтроллером и радиомодулем по SPI
// Удобно использовать для проверки платы после пайки
RF_ERR rf_test_exchange(void)
{
    uint8_t x;
    rf_wr_reg(CC1100_IOCFG1, 0x55); // Запись в регистр некоторого значения
    rf_rd_reg(CC1100_IOCFG1, &x); // И затем чтение из этого регистра
    if (x == 0x55) // Если значения совпали
        return RF_OK; // Радиомодуль общается по SPI
    else // Если нет
        return RF_ERR_SPI; // Значит вероятно неправильно настроен SPI или КЗ на плате
}

// Инициализация радиомодуля
// Текущие настройки:
// частота: 460 МГц
// девиация: 20 кГц
// RX фильтр и ширина канала: 200 кГц
// Скорость обмена: 38400
// Кварц на плате: 27 МГц
// Мощность: +10 dBm
RF_ERR rf_init(void)
{
    uint8_t patable[8] = { TX_LVL_7, 0, 0, 0, 0, 0, 0, 0 }; // 0xC0
    RF_ERR err;

    rf_pwr_on_reset(); // Сброс при включении питания

    err = rf_test_exchange(); // Проверка связи по SPI
    if (err != RF_OK)
        return err;
    rf_wr_strobe(CC1100_SRES);
    rf_wr_reg(CC1100_FSCTRL1, 0x06); // Frequency synthesizer control.
    rf_wr_reg(CC1100_FSCTRL0, 0x00); // Frequency synthesizer control.

    //  rf_wr_reg(CC1100_FREQ2,    0x10);    // Frequency control word, high byte.      0x11 0F
    //  rf_wr_reg(CC1100_FREQ1,    0x25);    // Frequency control word, middle byte.    0x09 2F
    //  rf_wr_reg(CC1100_FREQ0,    0xED);    // Frequency control word, low byte.       0x7B 68
    rf_wr_reg(CC1100_FREQ2, *(reinterpret_cast<unsigned char*>(0x08000))); // Frequency control word, high byte.      0x11 0F
    rf_wr_reg(CC1100_FREQ1, *(reinterpret_cast<unsigned char*>(0x08001))); // Frequency control word, middle byte.    0x09 2F
    rf_wr_reg(CC1100_FREQ0, *(reinterpret_cast<unsigned char*>(0x08002))); // Frequency control word, low byte.       0x7B 68
    rf_wr_reg(CC1100_MDMCFG4, 0xCA); // Modem configuration.     8A
    rf_wr_reg(CC1100_MDMCFG3, 0x75); // Modem configuration.     75     E5
    rf_wr_reg(CC1100_MDMCFG2, 0x13); // Modem configuration.     03     13
    rf_wr_reg(CC1100_MDMCFG1, 0xA2); // Modem configuration.     A2     A2
    rf_wr_reg(CC1100_MDMCFG0, 0xE5); // Modem configuration.     E5     E5
    rf_wr_reg(CC1100_CHANNR, CHANNEL); // Channel number.
    rf_wr_reg(CC1100_DEVIATN, 0x41); // Modem deviation setting (when FSK modulation is enabled).  34

    rf_wr_reg(CC1100_FIFOTHR, 0x07); // Attenuation 0dB
    rf_wr_reg(CC1100_FREND1, 0x56); // Front end RX configuration.
    rf_wr_reg(CC1100_FREND0, 0x10); // Front end RX configuration.
    rf_wr_reg(CC1100_MCSM0, 0x11); // Main Radio Control State Machine configuration.
    rf_wr_reg(CC1100_MCSM1, 0x30); // Main Radio Control State Machine configuration.
    rf_wr_reg(CC1100_FOCCFG, 0x16); // Frequency Offset Compensation Configuration.
    rf_wr_reg(CC1100_BSCFG, 0x6C); // Bit synchronization Configuration.
    rf_wr_reg(CC1100_AGCCTRL2, 0x03); // AGC control.   0x43
    rf_wr_reg(CC1100_AGCCTRL1, 0x40); // AGC control.
    rf_wr_reg(CC1100_AGCCTRL0, 0x91); // AGC control.
    rf_wr_reg(CC1100_FSCAL3, 0xE9); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_FSCAL2, 0x2A); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_FSCAL1, 0); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_FSCAL0, 0x1F); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_FSTEST, 0x59); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_TEST2, 0x81); // Various test settings.
    rf_wr_reg(CC1100_TEST1, 0x35); // Various test settings.
    rf_wr_reg(CC1100_TEST0, 0x09); // Various test settings.
    //  rf_wr_reg(CC1100_TEST0,    *(reinterpret_cast<unsigned char*>(0x08003)));    // Various test settings.
    rf_wr_reg(CC1100_IOCFG2, 0x06); // GDO2 output pin configuration.
    rf_wr_reg(CC1100_IOCFG0, 0x34); // GDO0 output pin configuration.
    rf_wr_reg(CC1100_PKTCTRL1, 0x14); // Packet automation control.
    rf_wr_reg(CC1100_PKTCTRL0, 0x04); // Packet automation control.
    rf_wr_reg(CC1100_ADDR, 0); // Device address.

    //  rf_wr_reg(CC1100_MCSM2, 0x07);//0x03
    //  rf_wr_reg(CC1100_WOREVT1, 0x87);
    //  rf_wr_reg(CC1100_WOREVT0, 0x6B);//0x6B
    //rf_wr_reg(CC1100_IOCFG0,   0x2E);
    //  rf_wr_reg(CC1100_WORCTRL, 0x98);// F8

    rf_wr_reg(CC1100_PKTLEN, USER_RF_BUF); // Packet length.

    rf_wr_burst(CC1100_PATABLE, patable, 8); // Загрузка paTable (мощность передатчика)

    return RF_OK;
}

RF_ERR rf_init_rec(void)
{
    rf_wr_reg(CC1100_FSCTRL1, 0x06); // Frequency synthesizer control.
    rf_wr_reg(CC1100_FSCTRL0, 0x00); // Frequency synthesizer control.

    rf_wr_reg(CC1100_FREQ2, 0x11); // Frequency control word, high byte.
    rf_wr_reg(CC1100_FREQ1, 0x09); // Frequency control word, middle byte.
    rf_wr_reg(CC1100_FREQ0, 0x7B); // Frequency control word, low byte.
    rf_wr_reg(CC1100_MDMCFG4, 0x0D); // Modem configuration.     8A     7B    0x0E - 500k
    rf_wr_reg(CC1100_MDMCFG3, 0x3B); // Modem configuration.     75     E5
    rf_wr_reg(CC1100_MDMCFG2, 0x73); // Modem configuration.     03     13
    rf_wr_reg(CC1100_MDMCFG1, 0x42); // Modem configuration.     A2     A2
    rf_wr_reg(CC1100_MDMCFG0, 0xF8); // Modem configuration.     E5     E5

    rf_wr_reg(CC1100_CHANNR, 0); // Channel number.
    rf_wr_reg(CC1100_DEVIATN, 0x34); // Modem deviation setting (when FSK modulation is enabled).  34   46

    rf_wr_reg(CC1100_FIFOTHR, 0x00); // Front end RX configuration.
    rf_wr_reg(CC1100_FREND1, 0x56); // Front end RX configuration.
    rf_wr_reg(CC1100_FREND0, 0x10); // Front end RX configuration.
    rf_wr_reg(CC1100_MCSM0, 0x18); // Main Radio Control State Machine configuration.
    rf_wr_reg(CC1100_MCSM1, 0x00); // Main Radio Control State Machine configuration.
    rf_wr_reg(CC1100_FOCCFG, 0x16); // Frequency Offset Compensation Configuration.
    rf_wr_reg(CC1100_BSCFG, 0x6C); // Bit synchronization Configuration.
    rf_wr_reg(CC1100_AGCCTRL2, 0x43); // AGC control.
    rf_wr_reg(CC1100_AGCCTRL1, 0x40); // AGC control.
    rf_wr_reg(CC1100_AGCCTRL0, 0x91); // AGC control.
    rf_wr_reg(CC1100_FSCAL3, 0xE9); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_FSCAL2, 0x2A); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_FSCAL1, 0); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_FSCAL0, 0x1F); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_FSTEST, 0x59); // Frequency synthesizer calibration.
    rf_wr_reg(CC1100_TEST2, 0x81); // Various test settings.
    rf_wr_reg(CC1100_TEST1, 0x35); // Various test settings.
    rf_wr_reg(CC1100_TEST0, 0x09); // Various test settings.
    rf_wr_reg(CC1100_IOCFG2, 0x06); // GDO2 output pin configuration.
    rf_wr_reg(CC1100_IOCFG0, 0x34); // GDO0 output pin configuration.
    rf_wr_reg(CC1100_PKTCTRL1, 0x08); // Packet automation control. // 0x0C
    rf_wr_reg(CC1100_PKTCTRL0, 0x44); // Packet automation control.
    rf_wr_reg(CC1100_ADDR, 0); // Device address.
    rf_wr_reg(CC1100_PKTLEN, 9); // Packet length.
    rf_wr_reg(CC1100_PATABLE, TX_LVL_7);
    return RF_OK;
}

// Запись строба в радиомодуль (запись одного байта по SPI)
RF_ERR rf_wr_strobe(uint8_t adr)
{
    RF_ERR err;
    RF_CS_ON; // Выставляем CS (0)
    err = rf_wait_ready(); // Ожидание готовности передатчика
    if (err == RF_OK) // Передатчик готов до окончания тайм-аута
        rf_exchange(adr); // Отправить строб
    RF_CS_OFF; // Снимаем CS (1)
    return err;
}

// Запись регистра
RF_ERR rf_wr_reg(uint8_t adr, uint8_t dat)
{
    RF_ERR err;
    RF_CS_ON;
    err = rf_wait_ready();
    if (err == RF_OK) {
        rf_exchange(adr);
        rf_exchange(dat); // То же самое, только запись 2х байт
    }
    RF_CS_OFF;
    return err;
}

// Чтение регистра
RF_ERR rf_rd_reg(uint8_t adr, uint8_t* dat)
{
    RF_ERR err;
    RF_CS_ON;
    err = rf_wait_ready();
    if (err == RF_OK) { // То же самое, только выставляем флаг чтения
        rf_exchange(adr | CC1100_READ_SINGLE);
        *dat = rf_exchange(0);
    }
    RF_CS_OFF;
    return err;
}

// Запись нескольких байт за 1 раз
RF_ERR rf_wr_burst(uint8_t adr, uint8_t* dat, uint8_t qty)
{
    uint8_t i;
    RF_ERR err;
    RF_CS_ON;
    err = rf_wait_ready();
    if (err == RF_OK) {
        rf_exchange(adr | CC1100_WRITE_BURST); // Выставляем флаг потоковой записи
        if (qty > RF_BUF_SIZE) { // Защита от записи лишнего в FIFO
            err = RF_ERR_WR_SIZE;
            qty = RF_BUF_SIZE;
        }
        for (i = 0; i < qty; i++)
            rf_exchange(dat[i]);
    }
    RF_CS_OFF;
    return err;
}

// Чтение нескольких байт за 1 раз
RF_ERR rf_rd_burst(uint8_t adr, uint8_t* dat, uint8_t qty)
{
    uint8_t i;
    RF_ERR err;
    RF_CS_ON;
    err = rf_wait_ready();
    if (err == RF_OK) {
        rf_exchange(adr | CC1100_READ_BURST); // Выставляем флаг потокового чтения
        if (qty > RF_BUF_SIZE) { // Защита от чтения лишнего из FIFO
            err = RF_ERR_WR_SIZE;
            qty = RF_BUF_SIZE;
        }
        for (i = 0; i < qty; i++)
            dat[i] = rf_exchange(0);
    }
    RF_CS_OFF;
    return err;
}

RF_ERR poll_satus(uint8_t strobe, uint8_t status)
{
    uint8_t rf_status;
    uint16_t timeout = 0;
    // rf_wr_strobe(strobe);
    do {
        rf_wr_strobe(strobe);
        DELAY_100us;
        rf_rd_reg(CC1100_MARCSTATE | 0x40, &rf_status);
        timeout++;
        if (timeout > RF_TIMEOUT)
            return RF_ERR_TIMEOUT;
    } while ((rf_status != status));
    return RF_OK;
}

RF_ERR go_idle_state()
{
    return poll_satus(CC1100_SIDLE, 0x01);
}

RF_ERR go_srx_state()
{
    return poll_satus(CC1100_SRX, 0x0D);
}

RF_ERR execute_stx()
{
    return poll_satus(CC1100_STX, 0x13);
}

RF_ERR monitor_cca()
{
    uint8_t cnt, i;
    uint8_t rf_status;
    uint16_t timeout = 0;
    do {
        cnt = 0;
        for (i = 0; i < 10; i++) {
            DELAY_100us;
            rf_rd_reg(CC1100_PKTSTATUS | 0x40, &rf_status);
            if (rf_status & 0x40)
                cnt++;
        }
        timeout++;
        if (timeout > RF_TIMEOUT)
            return RF_ERR_TIMEOUT;
    } while (cnt != 0);
    return RF_OK;
}

RF_ERR transmit_cca(uint8_t adr, uint8_t* dat, uint8_t qty)
{
    RF_ERR err;
    err = go_srx_state();
    err = rf_wr_burst(adr, dat, qty);
    DELAY_100us;
    monitor_cca();
    execute_stx();
    return err;
}

RF_ERR enter_wor()
{
    RF_ERR err;
    rf_wr_strobe(CC1100_SFRX);

    //err = go_srx_state();

    rf_wr_reg(CC1100_MDMCFG2, 0x71); // Modem configuration.     03     13
    rf_wr_reg(CC1100_MDMCFG1, 0x72); // Modem configuration.     A2     A2

    rf_wr_reg(CC1100_PKTLEN, WAKEUP_BUF); // Packet length.
    err = go_idle_state();
    rf_wr_reg(CC1100_MCSM2, 0x18); //0x01
    rf_wr_reg(CC1100_AGCCTRL2, 0x03); // AGC control.   0x43
    rf_wr_reg(CC1100_MCSM0, 0x18); //0x11
    rf_wr_reg(CC1100_WOREVT1, 0x23); // 0x47 = 0.5s
    rf_wr_reg(CC1100_WOREVT0, 0x6B);
    rf_wr_reg(CC1100_IOCFG0, 0x2E);
    rf_wr_reg(CC1100_WORCTRL, 0x68); // 0x38
    rf_wr_strobe(CC1100_SWORRST);
    rf_wr_strobe(CC1100_SWOR);
    return err;
}

RF_ERR re_enter_wor()
{
    RF_ERR err;
    rf_wr_strobe(CC1100_SFRX);
    //  err = go_srx_state();
    rf_wr_reg(CC1100_WORCTRL, 0x68);
    err = go_idle_state();
    rf_wr_strobe(CC1100_SWORRST);
    rf_wr_strobe(CC1100_SWOR);
    return err;
}

RF_ERR exit_wor()
{
    rf_wr_reg(CC1100_WORCTRL, 0x98);
    go_idle_state();
    return RF_OK;
}

RF_ERR rf_wakeup()
{
    RF_ERR err;
    err = go_idle_state();
    // rf_wr_reg(CC1100_IOCFG2,   0x06);
    rf_wr_reg(CC1100_MCSM2, 0x07); //0x03
    rf_wr_reg(CC1100_AGCCTRL2, 0xFB); // AGC control.   0x43
    rf_wr_reg(CC1100_WOREVT1, 0x87);
    rf_wr_reg(CC1100_WOREVT0, 0x6B); //0x6B
    //rf_wr_reg(CC1100_IOCFG0,   0x2E);
    rf_wr_reg(CC1100_WORCTRL, 0x98); // F8
    rf_wr_reg(CC1100_MDMCFG2, 0x73); // Modem configuration.     03     13
    rf_wr_reg(CC1100_MDMCFG1, 0x22); // Modem configuration.     22     A2
    rf_wr_reg(CC1100_PKTLEN, USER_RF_BUF); // Packet length.
    return err;
}
