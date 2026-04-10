#ifndef RLS_H
#define RLS_H

#include "arm_math.h"
#include <stdint.h>
#include <string.h>

// 定义维度为2，对应功率模型中的 k1 和 k2
#define RLS_DIM 2

// RLS 对象结构体
typedef struct {
    float lambda;            // 遗忘因子
    float delta;             // 初始非奇异值
    uint32_t updateCnt;      // 更新计数

    /* 矩阵数据存储区 */
    // 为了使用 arm_mat，我们需要原始数据 buffer
    float pData_trans[RLS_DIM * RLS_DIM];   // 转移矩阵 P 的数据
    float pData_gain[RLS_DIM];              // 增益向量 K 的数据
    float pData_params[RLS_DIM];            // 参数向量 theta 的数据
    float pData_default[RLS_DIM];           // 默认参数
    
    // 临时计算缓冲区 (避免频繁创建局部变量)
    float pData_temp_vec[RLS_DIM];          // 临时向量
    float pData_temp_mat[RLS_DIM * RLS_DIM]; // 临时矩阵
    float pData_temp_vec2[RLS_DIM];         // 临时向量2
    
    /* arm_matrix 实例 */
    arm_matrix_instance_f32 transMatrix;    // P (2x2)
    arm_matrix_instance_f32 gainVector;     // K (2x1)
    arm_matrix_instance_f32 paramsVector;   // theta (2x1)
    
    // 用于计算的临时矩阵实例
    arm_matrix_instance_f32 temp_vec;       // (2x1)
    arm_matrix_instance_f32 temp_mat;       // (2x2)
    arm_matrix_instance_f32 temp_vec2;      // (2x1)

    float output;            // 估计输出
} RLS_t;

/**
 * @brief 初始化RLS对象
 */
RLS_t *RLS_Init(float delta_, float lambda_);

/**
 * @brief 重置RLS状态
 */
void RLS_Reset(RLS_t *rls);

/**
 * @brief 设置默认参数向量
 */
void RLS_SetParamVector(RLS_t *rls, const float *params);

/**
 * @brief 执行一次RLS更新迭代
 * @param sampleVector  输入样本向量 (长度为2的数组)
 * @param actualOutput  实际测量输出值
 * @param currentTime   当前时间戳
 * @return 参数向量的指针
 */
const float* RLS_Update(RLS_t *rls, const float *sampleVector, float actualOutput);

/**
 * @brief 获取当前参数向量
 */
const float* RLS_GetParams(const RLS_t *rls);

#endif // RLS_H
