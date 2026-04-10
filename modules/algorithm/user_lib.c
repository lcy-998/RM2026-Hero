/**
 ******************************************************************************
 * @file	 user_lib.c
 * @author  Wang Hongxi
 * @author  modified by neozng
 * @version 0.2 beta
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "stdlib.h"
#include "memory.h"
#include "user_lib.h"
#include "math.h"
#include "main.h"
#include "bsp_dwt.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

void *zmalloc(size_t size)
{
    void *ptr = malloc(size);
    memset(ptr, 0, size);
    return ptr;
}

// 快速开方
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0) {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 绝对值限制
float abs_limit(float num, float Limit)
{
    if (num > Limit) {
        num = Limit;
    } else if (num < -Limit) {
        num = -Limit;
    }
    return num;
}

// 判断符号位
float sign(float value)
{
    if (value >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue) {
        Value = 0.0f;
    }
    return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue) {
        return Input;
    }

    if (Input > maxValue) {
        float len = maxValue - minValue;
        while (Input > maxValue) {
            Input -= len;
        }
    } else if (Input < minValue) {
        float len = maxValue - minValue;
        while (Input < minValue) {
            Input += len;
        }
    }
    return Input;
}

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - (float)integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

// 三维向量归一化
float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

// 计算模长
float NormOf3d(float *v)
{
    return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 均值滤波,删除buffer中的最后一个元素,填入新的元素并求平均值
float AverageFilter(float new_data, float *buf, uint8_t len)
{
    float sum = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        buf[i] = buf[i + 1];
        sum += buf[i];
    }
    buf[len - 1] = new_data;
    sum += new_data;
    return sum / len;
}

void MatInit(mat *m, uint8_t row, uint8_t col)
{
    m->numCols = col;
    m->numRows = row;
    m->pData   = (float *)zmalloc(row * col * sizeof(float));
}
/**
 * @brief :  正弦扫频生成器
 * @param[in] F_start 起始频率
 * @param[in] F_end 终止频率
 * @param[in] repeat_time 周期重复次数
 * @param[out] *SE_signal 结束生成标志
 * @param[out] *F_out 当前频率
 * @return 正弦值（0~1）
 */
float sin_signal_generate(float F_start, float F_end, float repeat_time, uint8_t *SE_signal, float *F_out)
{
    static float F = 0;
    if (F == 0) F = F_start;
    static float lasttime = 0;
    // 频率超过限定，返回1
    if (F > F_end) {
        *SE_signal = 1;
        return 0;
    }
    // 保证sin初值为0
    if (lasttime == 0) lasttime = DWT_GetTimeline_s();

    float nowtime = DWT_GetTimeline_s();
    *F_out        = F;
    // 计算正弦值
    float cnt = arm_sin_f32(2 * PI * F * (nowtime - lasttime));
    // 频率递增
    if (nowtime - lasttime > ((1 / F) * repeat_time)) {
        if (F < 24)
            F += 0.5f;
        else if (F >= 24 && F <= 120)
            F += 2;
        else
            F += 4;

        lasttime = DWT_GetTimeline_s();
    }
    return cnt;
}

uint16_t float_to_half(float f) {
    uint32_t bit_pattern;
    memcpy(&bit_pattern, &f, sizeof(f));  // 获取float的二进制表示

    uint32_t sign = (bit_pattern >> 31) & 0x1;
    uint32_t exponent = (bit_pattern >> 23) & 0xFF;
    uint32_t mantissa = bit_pattern & 0x7FFFFF;

    // 处理非规范化数
    if (exponent == 0) {
        return (sign << 15); // 非规范化数
    }

    // 处理无穷大和NaN
    if (exponent == 0xFF) {
        return (sign << 15) | (0x1F << 10) | (mantissa >> 13);
    }

    // 规范化数的转换
    int half_exponent = exponent - 127 + 15;
    if (half_exponent > 0x1F) half_exponent = 0x1F;  // 最大指数
    if (half_exponent < 0) half_exponent = 0;  // 最小指数

    uint16_t half = (sign << 15) | (half_exponent << 10) | (mantissa >> 13);
    return half;
}
// 半精度浮点数转单精度浮点数
float half_to_float(uint16_t half) {
    uint32_t sign = (half >> 15) & 0x1;           // 符号位
    uint32_t exponent = (half >> 10) & 0x1F;      // 指数位
    uint32_t mantissa = half & 0x3FF;             // 尾数位

    // 处理非规范化数
    if (exponent == 0) {
        // 非规范化数处理（需要补零）
        return sign ? -0.0f : 0.0f;
    }

    // 处理无穷大和NaN
    if (exponent == 0x1F) {
        if (mantissa == 0) {
            return sign ? -INFINITY : INFINITY;
        } else {
            return NAN;
        }
    }

    // 规范化数的转换
    int32_t float_exp = (int32_t)(exponent) - 15 + 127; // 调整指数偏移量
    uint32_t float_mantissa = mantissa << 13;           // 扩展尾数至23位

    // 生成32位浮点数（符号 + 指数 + 尾数）
    uint32_t float_bits = (sign << 31) | (float_exp << 23) | float_mantissa;

    float result;
    memcpy(&result, &float_bits, sizeof(result));  // 将位模式转换为浮点数
    return result;
}
// 将 float 拆分为 4 个 uint8_t 字节（手动处理高低位）
void float_to_uint8_manual(float value, uint8_t *bytes) {
    uint32_t as_int;
    memcpy(&as_int, &value, sizeof(float)); // 将 float 转换为 uint32_t

    *bytes = (uint8_t)(as_int & 0xFF);         // 第 1 个字节（低位）
    *(bytes+1) = (uint8_t)((as_int >> 8) & 0xFF);  // 第 2 个字节
    *(bytes+2) = (uint8_t)((as_int >> 16) & 0xFF); // 第 3 个字节
    *(bytes+3) = (uint8_t)((as_int >> 24) & 0xFF); // 第 4 个字节（高位）
}

// 将 4 个 uint8_t 字节解析回 float（手动处理高低位）
float uint8_to_float_manual(uint8_t *bytes) {
    uint32_t as_int = 0;

    as_int |= ((uint32_t)bytes[0]);
    as_int |= ((uint32_t)bytes[1] << 8);
    as_int |= ((uint32_t)bytes[2] << 16);
    as_int |= ((uint32_t)bytes[3] << 24);

    float value;
    memcpy(&value, &as_int, sizeof(float)); // 将 uint32_t 转换回 float
    return value;
}

// 将 int32_t 转换为 uint8_t 数组（长度 4）
void int32_to_uint8_array(int32_t value, uint8_t *array) {
    array[0] = (uint8_t)((value >> 24) & 0xFF); // 高位字节
    array[1] = (uint8_t)((value >> 16) & 0xFF);
    array[2] = (uint8_t)((value >> 8) & 0xFF);
    array[3] = (uint8_t)(value & 0xFF);        // 低位字节
}

// 从 uint8_t 数组还原为 int32_t
int32_t uint8_array_to_int32(const uint8_t *array) {
    return (int32_t)(
        ((int32_t)array[0] << 24) | // 高位字节
        ((int32_t)array[1] << 16) |
        ((int32_t)array[2] << 8) |
        ((int32_t)array[3])
    );
}
