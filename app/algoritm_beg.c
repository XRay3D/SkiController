#include "algoritm_beg.h"

int8_t acc[3], gyro[3];
float ax_filtered = 0, KAX = 0.90, arez = 0;
float ay_filtered = 0, KAY = 0.90, arez_filt = 0;
float az_filtered = 0, KAZ = 0.90;
float gz_filtered = 0, KGZ = 0.95;
float summ_bedro_angle = 0, prev_angle = 0;
float prev_arez[AREA], prev_compensator[AREA_COMPENSATOR];

uint16_t periodic_cnt = 0, stepper_cnt = 0, stim_cnt = 0;
uint8_t periodic_found = 0, stepper_found = 0, stim_on_alg = 0;

void alg_reset()
{
    uint16_t i;
    ax_filtered = 0;
    arez = 0;
    ay_filtered = 0;
    arez_filt = 0;
    az_filtered = 0;
    gz_filtered = 0;
    summ_bedro_angle = 0;
    for (i = 0; i < AREA; i++)
        prev_arez[i] = 0;
    periodic_cnt = 0;
    periodic_found = 0;
    stepper_cnt = 0;
    stepper_found = 0;
    stim_on_alg = 0;
    prev_angle = 0;
    stim_cnt = 0;
}

void add_data(int8_t* acc_data, int8_t* gyro_data)
{
    int8_t i;
    for (i = 0; i < 3; i++) {
        acc[i] = acc_data[i];
        gyro[i] = gyro_data[i];
    }
}

float delta_func(uint8_t area, float* prev_values, float new_value)
{
    int i;
    float summ = 0;
    for (i = 0; i < area - 1; i++) {
        prev_values[i] = prev_values[i + 1];
    }
    prev_values[area - 1] = new_value;
    for (i = 0; i < area - 1; i++) {
        summ += fabs(prev_values[i] - prev_values[i + 1]);
    }
    return summ;
}

compensator_t gyroscope_compensator(uint16_t area, float* prev_values, float new_value)
{
    int i;
    float max, min;
    compensator_t comp_data;
    max = prev_values[0];
    min = prev_values[0];
    for (i = 0; i < area; i++) {
        if (i < area - 1)
            prev_values[i] = prev_values[i + 1];
        else
            prev_values[area - 1] = new_value;
        if (prev_values[i] > max)
            max = prev_values[i];
        if (prev_values[i] < min)
            min = prev_values[i];
    }
    comp_data.offset = max - (max - min) / 2;
    comp_data.amplitude = fabs(max - min);
    if (prev_values[AREA_COMPENSATOR - 1] > prev_values[AREA_COMPENSATOR - 40])
        comp_data.increasing = 1;
    else
        comp_data.increasing = 0;
    return comp_data;
}

alg_data_t alg_calc()
{
    alg_data_t out_data;
    compensator_t comp_data;
    //ax_filtered = (1.0-KAX)*(fabs((float)acc[0]))+KAX*ax_filtered;
    //ay_filtered = (1.0-KAY)*(fabs((float)acc[1]))+KAY*ay_filtered;
    arez = sqrt(((float)acc[1]) * ((float)acc[1]) + ((float)acc[2]) * ((float)acc[2]));
    arez_filt = delta_func(AREA, prev_arez, arez); // result pokaz postanovku nogi

    prev_angle = summ_bedro_angle;
    //gz_filtered = (1.0-KGZ)*(fabs((float)gyro[2]))+KGZ*gz_filtered;
    summ_bedro_angle += ((float)gyro[2]) * ANGULAR_RATE;
    comp_data = gyroscope_compensator(AREA_COMPENSATOR, prev_compensator, summ_bedro_angle);
    out_data.cur_angle = summ_bedro_angle - comp_data.offset;
    prev_angle -= comp_data.offset;
    out_data.angle_amplitude = comp_data.amplitude;
    out_data.increasing = comp_data.increasing;
    out_data.stepper = arez_filt;
    return out_data;
}

uint8_t alg_process(alg_data_t calc_data)
{
    /*if (stepper_found == 0)
  {
    if (calc_data.stepper > STEP_DETECT_LEVEL)
    {
      stepper_found = 1;
      stepper_cnt = 0;
    }
  }
  else
  {
    if (stepper_cnt < TIME_AFTER_STEP)
      stepper_cnt ++;
    else
      stepper_found = 0;
  }*/

    stepper_found = 1;
    stim_on_alg = 0;

    if (stim_on_alg == 0) {
        if (
            (stepper_found == 1) && (prev_angle < STIM_ANGLE) && (calc_data.cur_angle > STIM_ANGLE) && (calc_data.increasing == 1) && (calc_data.angle_amplitude > WORK_AMPLITUDE)) {
            if (periodic_found == 0) {
                periodic_found = 1;
                periodic_cnt = 0;
            }
            else {
                periodic_cnt = 0;
                stim_on_alg = 1;
                stim_cnt = 0;
            }
        }
    }
    if (periodic_found == 1) {
        if (periodic_cnt < MAX_PERIOD_TIME) {
            periodic_cnt++;
        }
        else {
            periodic_found = 0;
            periodic_cnt = 0;
        }
    }
    /*if (stim_on_alg == 1){
    if (stim_cnt < 10)
      stim_cnt ++;
    else
      stim_on_alg = 0;
  }*/
    return stim_on_alg;
}
