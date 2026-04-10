#include "power_calc.h"
#include "arm_math.h"
#include "user_lib.h"
#include <stdlib.h>
#include <math.h>
#include "RLS.h"

float k1                   = 1.15041374e-07;//1.80413721e-08;
float k2                   = 1.79e-7;
float constant             = 0.8;
float torque_coefficient   = 2.94974577124e-06f; // (20/16384) * (0.3) /13 / (9.55)
float machine_power        = 0;
float current_power        = 0;
float speed_power          = 0;
float motor_current_output = 0;
float Power_Input          = 0;

float reduction_ratio, total_power;
uint16_t max_power = 0;
float chassis_power;

RLS_t *rls = NULL;

void PowerControlupdate(uint16_t max_power_init, float reduction_ratio_init, float chassis_power_init)
{
    max_power = max_power_init;
    chassis_power = chassis_power_init;
    if (reduction_ratio_init != 0) {
        reduction_ratio = reduction_ratio_init;
    } else {
        reduction_ratio = (187.0f / 3591.0f);
    }
}
float text_k1,text_c,text_k2,test_c;
float PowerInputCalc(float motor_speed, float motor_current_pridict, float motor_current_real, uint8_t motor_count)
{
        if (!motor_count)
    {
        text_k1 = 0.0f;
        text_k2 = 0.0f;
        machine_power = 0.0f;
    }
    machine_power += motor_current_real * torque_coefficient * motor_speed;
    text_k1 += motor_speed * motor_speed;
    text_k2 += motor_current_real * motor_current_real;
    // if (motor_count == 3)
    // {
    //     if (!rls)
    //     {
    //         float param[2] = {k1, k2};
    //         rls = RLS_Init(1e-5, 0.99999f);
    //         RLS_SetParamVector(rls, param);
    //     }
    //     else
    //     {
    //         if (chassis_power > 25.0f)
    //         {
    //             float sample[2] = {text_k1, text_k2};
    //             float *param;
    //             text_c = chassis_power - machine_power - 4.0 * constant;
    //             param = RLS_Update(rls, sample, text_c);
    //             k1 = param[0];
    //             k2 = param[1];
    //         }
    //         test_c = machine_power + 4.0 * constant + k1 * text_k1 + k2 * text_k2;
    //     }
    // }
    // P_input = I_cmd * C_t * w + k_1* w * w +k_2 * I_cmd * I_cmd
    float power_input = motor_current_pridict * torque_coefficient * motor_speed +
                        k1 * motor_speed * motor_speed +
                        k2 * motor_current_pridict * motor_current_pridict + constant;
                        text_c = motor_current_pridict * torque_coefficient * motor_speed;
                        
    Power_Input   = power_input;
    current_power = k2 * motor_current_pridict * motor_current_pridict + constant;
    speed_power   = k1 * motor_speed * motor_speed;

    return power_input;
}

float TotalPowerCalc(float input_power[])
{
    total_power = 0;
    for (int i = 0; i < 4; i++) {
        if (input_power[i] < 0) {
            continue;
        } else {
            total_power += input_power[i];
        }
    }
    return total_power;
}

float give_power;
float power_scale;
float torque_output;
float temp;
float CurrentOutputCalc(float motor_power, float motor_speed, float motor_current)
{
    if (total_power > max_power) {
        power_scale = max_power / total_power;
        give_power  = motor_power * power_scale;
        if (motor_power < 0) {
            if (motor_current > 15000) {
                motor_current = 15000;
            }
            if (motor_current < -15000) {
                motor_current = -15000;
            }
            return motor_current;
        }
        float a = k2;
        float b = motor_speed * torque_coefficient;
        float c = k1 * motor_speed * motor_speed - give_power + constant;
        if (motor_current > 0) {
             temp           = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
            motor_current_output = temp;
        } else {
             temp           = (-b - sqrtf(b * b - 4 * a * c)) / (2 * a);
            motor_current_output = temp;
        }
        if (motor_current_output > 15000) {
            motor_current_output = 15000;
        } else if (motor_current_output < -15000) {
            motor_current_output = -15000;
        }
        return motor_current_output;
    }
    if (motor_current > 15000) {
        motor_current = 15000;
    } else if (motor_current < -15000) {
        motor_current = -15000;
    }
    return motor_current;
}