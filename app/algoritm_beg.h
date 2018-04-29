//---------------------------------------------------------------------------

#ifndef _ALG_BEG
#define _ALG_BEG

#include <math.h>
#include <stdint.h>

#define AREA 25
#define AREA_COMPENSATOR 300
#define ANGULAR_RATE 0.009765625 * 2 // calculated for 250 dps 8bit gyroscope @ 200 Hz datarate
#define STEP_DETECT_LEVEL 500
#define TIME_AFTER_STEP 100
#define STIM_ANGLE 3
#define WORK_AMPLITUDE 30
#define MAX_PERIOD_TIME 300 // counts

typedef struct {
    float cur_angle;
    float stepper;
    float angle_amplitude;
    uint8_t increasing;
} alg_data_t;

typedef struct {
    float offset;
    float amplitude;
    uint8_t increasing;
} compensator_t;

void alg_reset();
void add_data(int8_t* acc_data, int8_t* gyro_data);
float delta_func(uint8_t area, float* prev_values, float new_value);
compensator_t gyroscope_compensator(uint16_t area, float* prev_values, float new_value);
alg_data_t alg_calc();
uint8_t alg_process(alg_data_t calc_data);

//---------------------------------------------------------------------------
#endif
