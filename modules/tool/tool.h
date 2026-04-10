#pragma once

#include "main.h"
#include "rm_referee.h"

#define bool      _Bool
#define true      1
#define false     0

#define RAMP_TIME 1500 // 1000

// 斜坡类型，计算WASD移动映射在底盘的速度
typedef struct ramp_t {
    int32_t count; // 计数值
    int32_t scale; // 规模
    float out;     // 输出
} ramp_t;

void ramp_init(ramp_t *ramp, int32_t scale);

float ramp_calc(ramp_t *ramp);
