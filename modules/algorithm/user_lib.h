/*
 * @Author       : Alliance
 * @Date         : 2023-09-08
 * @LastEditors  : HDC h2019dc@outlook.com
 * @LastEditTime : 2023-11-12
 * @FilePath     : \2024_Control_New_Framework_Base-dev-all\modules\algorithm\user_lib.h
 * @Description  :
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f407xx.h"
#include "arm_math.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#define MatAdd       arm_mat_add_f32
#define MatSubtract  arm_mat_sub_f32
#define MatMultiply  arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse   arm_mat_inverse_f32
void MatInit(mat *m, uint8_t row, uint8_t col);

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max)     \
    do {                             \
        if ((val) <= (min)) {        \
            (val) = (min);           \
        } else if ((val) >= (max)) { \
            (val) = (max);           \
        }                            \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do {                                \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do {                            \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief 返回一块干净的内存,不过仍然需要强制转换为你需要的类型
 *
 * @param size 分配大小
 * @return void*
 */
void *zmalloc(size_t size);

// 快速开方
float Sqrt(float x);
// 绝对值限制
float abs_limit(float num, float Limit);
// 判断符号位
float sign(float value);
// 浮点死区
float float_deadband(float Value, float minValue, float maxValue);
// 限幅函数
float float_constrain(float Value, float minValue, float maxValue);
// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue);
// 角度格式化为-180~180
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

uint16_t float_to_half(float f);

float half_to_float(uint16_t half);

void float_to_uint8_manual(float value, uint8_t *bytes);

float uint8_to_float_manual(uint8_t *bytes);

void int32_to_uint8_array(int32_t value, uint8_t *array);

int32_t uint8_array_to_int32(const uint8_t *array);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)
// 正弦扫频生成器
float sin_signal_generate(float F_start, float F_end, float repeat_time, uint8_t *SE_signal,float *F_out);

#endif
