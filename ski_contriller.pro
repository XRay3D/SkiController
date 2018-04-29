TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += __STDC_CONSTANT_MACROS define __INT8_T_TYPE__ define __INT16_T_TYPE__ define __INT32_T_TYPE__


HEADERS += \
    app/algorithm.h \
    app/algoritm_beg.h \
    app/includes.h \
    app/main.h \
    cc1101_lib/cc1101_bsp.h \
    cc1101_lib/cc1101_drv.h \
    cc1101_lib/cc1101_reg.h \
    modules/crc.h \
    modules/LPC_Vic.h \
    sensors/accel_drv.h \
    sensors/hyro_drv.h \
    app/uart_service.h

SOURCES += \
    app/algorithm.c \
    app/algoritm_beg.c \
    cc1101_lib/cc1101_bsp.c \
    cc1101_lib/cc1101_drv.c \
    modules/crc.c \
    modules/LPC_Vic.c \
    sensors/accel_drv.c \
    sensors/hyro_drv.c \
    app/uart_service.cpp \
    app/main.cpp

INCLUDEPATH += "C:/Program Files/IAR Systems/Embedded Workbench 4.0 Evaluation/ARM/inc"
