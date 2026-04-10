#pragma once

#include <stdint.h>

typedef struct
{
    float input_power[4];
    float wheel_speed[4];
    float total_power;
    float predict_output[4];
    uint8_t count;
} Power_Data_s;

void PowerControlupdate(uint16_t max_power_init, float reduction_ratio_init, float chassis_power_init);

float PowerInputCalc(float motor_speed, float motor_current_pridict, float motor_current_real, uint8_t motor_count);

float TotalPowerCalc(float input_power[]);

float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current);