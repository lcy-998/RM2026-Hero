#include "RLS.h"
#include <stdlib.h>

RLS_t *RLS_Init(float delta_, float lambda_) 
{
    RLS_t *rls = NULL;
    rls = (RLS_t *)malloc(sizeof(RLS_t));
    memset(rls, 0, sizeof(RLS_t));

    rls->delta = delta_;
    rls->lambda = lambda_;
    rls->updateCnt = 0;
    rls->output = 0.0f;

    // 初始化默认参数为0
    memset(rls->pData_default, 0, sizeof(rls->pData_default));
    
    // 初始化 arm_matrix 实例
    // 行数、列数、数据指针
    arm_mat_init_f32(&rls->transMatrix, RLS_DIM, RLS_DIM, rls->pData_trans);
    arm_mat_init_f32(&rls->gainVector, RLS_DIM, 1, rls->pData_gain);
    arm_mat_init_f32(&rls->paramsVector, RLS_DIM, 1, rls->pData_params);
    
    // 初始化临时矩阵实例
    arm_mat_init_f32(&rls->temp_vec, RLS_DIM, 1, rls->pData_temp_vec);
    arm_mat_init_f32(&rls->temp_mat, RLS_DIM, RLS_DIM, rls->pData_temp_mat);
    arm_mat_init_f32(&rls->temp_vec2, RLS_DIM, 1, rls->pData_temp_vec2);

    RLS_Reset(rls);
    return rls;
}

void RLS_Reset(RLS_t *rls) {
    // P = I * delta
    memset(rls->pData_trans, 0, sizeof(rls->pData_trans));
    // 2x2 单位矩阵对角线赋值
    rls->pData_trans[0] = rls->delta; // [0,0]
    rls->pData_trans[3] = rls->delta; // [1,1]

    // Gain = 0
    memset(rls->pData_gain, 0, sizeof(rls->pData_gain));
    
    // Params = 0
    memset(rls->pData_params, 0, sizeof(rls->pData_params));
}

void RLS_SetParamVector(RLS_t *rls, const float *params) {
    memcpy(rls->pData_params, params, sizeof(rls->pData_params));
    memcpy(rls->pData_default, params, sizeof(rls->pData_default));
}

const float* RLS_Update(RLS_t *rls, const float *sampleVector, float actualOutput) {
    // 将输入数组封装为向量结构体
    // 为了使用 arm_mat 函数，我们需要把它伪装成一个矩阵实例
    // 我们复用 temp_vec2 作为输入向量 phi
    arm_matrix_instance_f32 phi;
    arm_mat_init_f32(&phi, RLS_DIM, 1, (float *)sampleVector); // 注意：这里强转去掉了const，仅用于计算，未修改内容

    /* 1. 计算增益向量 Gain = (P * phi) / (lambda + phi^T * P * phi) / lambda */
    
    // temp_vec = P * phi
    arm_mat_mult_f32(&rls->transMatrix, &phi, &rls->temp_vec);
    
    // temp_val = phi^T * temp_vec (即 phi^T * P * phi)
    float phi_t_P_phi;
    arm_mat_mult_f32(&phi, &rls->temp_vec, &rls->temp_vec2); // temp_vec2 此时是 1x1 矩阵(其实是 1x1 的 arm_matrix)，这里有点技巧
    // arm_mat_mult 结果是 1x1 矩阵存放在数组第一个元素
    // 注意：arm_mat_mult_f32 要求维度匹配。(2x1)^T * (2x1) 不能直接乘。
    // 所以这里应该用点积 arm_dot_prod_f32
    arm_dot_prod_f32(sampleVector, rls->temp_vec.pData, RLS_DIM, &phi_t_P_phi);
    
    // 分母 scalar_denom = lambda + phi_t_P_phi
    float scalar_denom = rls->lambda + phi_t_P_phi;
    
    // temp_vec = temp_vec / scalar_denom (即 P*phi / denom)
    arm_scale_f32(rls->temp_vec.pData, 1.0f / scalar_denom, rls->temp_vec.pData, RLS_DIM);
    
    // Gain = temp_vec / lambda
    arm_scale_f32(rls->temp_vec.pData, 1.0f / rls->lambda, rls->gainVector.pData, RLS_DIM);

    /* 2. 更新参数向量 theta = theta + Gain * (y - phi^T * theta) */
    
    // y_est = phi^T * theta
    float y_est;
    arm_dot_prod_f32(sampleVector, rls->paramsVector.pData, RLS_DIM, &y_est);
    
    float error = actualOutput - y_est;
    
    // theta = theta + Gain * error
    // arm_offset 是加常数，不符合。用 scale 加上向量
    // theta += gain * error
    // 先把 gain 缩放 error 倍存入 temp_vec
    arm_scale_f32(rls->gainVector.pData, error, rls->temp_vec.pData, RLS_DIM);
    // 再加到 theta 上
    arm_add_f32(rls->paramsVector.pData, rls->temp_vec.pData, rls->paramsVector.pData, RLS_DIM);

    /* 3. 更新协方差矩阵 P = (P - Gain * phi^T * P) / lambda */
    
    // 这里需要计算 Gain * (phi^T * P)
    // 注意：phi^T (1x2) * P (2x2) = Res (1x2)
    // 由于 arm_mat 不方便处理行向量，我们利用数学性质：(Gain * phi^T) * P
    // 外积：temp_mat = Gain * phi^T (2x1 * 1x2 = 2x2)
    // 使用 arm_mat_mult_f32 计算 (2x1) * (1x2)
    // phi 是 (2x1)，phi^T 我们可以用转置后的数据，但为了效率可以直接计算外积
    // CMSIS-DSP 没有直接的外积函数，但我们有 temp_vec = P * phi (步骤1已计算)
    // 实际上公式是：P_new = (P - K * phi^T * P) / lambda
    // 注意到 K * phi^T * P = K * (phi^T * P)
    // 我们在步骤1算过 phi^T * P 吗？没有，我们算的是 phi^T * (P * phi)。
    
    // 我们回退到标准公式：P_new = (P - K * phi^T * P) / lambda
    // 我们需要计算 phi^T * P。phi^T 是行向量。
    // 手动计算或转置 phi。
    // 由于维度很小(2x2)，我们手写这部分关键逻辑通常比调用库函数转置更快：
    // temp_mat[i][j] = K[i] * phi[j]
    
    // 但为了保持一致性，我们用矩阵乘法实现：
    // 定义一个 1x2 的矩阵代表 phi^T
    float phi_T_data[2] = {sampleVector[0], sampleVector[1]};
    arm_matrix_instance_f32 phi_T;
    arm_mat_init_f32(&phi_T, 1, RLS_DIM, phi_T_data);
    
    // 计算 phi^T * P -> 结果应为 1x2 矩阵。我们复用 temp_vec 的前两个字节? 不安全。
    // 使用 temp_vec2 (定义时为 2x1) 来存储? 不，维度不匹配。
    // 我们用一个技巧：P * phi 已经算过了。
    // 我们直接计算 (Gain * phi^T) * P
    
    // 步骤 A: 计算 temp_mat = Gain * phi^T (2x1 * 1x2 -> 2x2)
    // CMSIS 没法直接乘非方阵维度匹配的。
    // 既然是 2D，我们可以稍微手写一点或者用 sub_matrix 技巧。
    // 为了极致效率，这里直接手写 2x2 外积计算，这比调用复杂的矩阵乘法开销小：
    // temp_mat.pData[0] = gain[0]*phi[0]; temp_mat.pData[1] = gain[0]*phi[1];
    // temp_mat.pData[2] = gain[1]*phi[0]; temp_mat.pData[3] = gain[1]*phi[1];
    // 虽然是手写，但利用了 FPU 寄存器，效率极高。
    
    // 如果坚持用 arm_mat:
    // 我们需要计算 temp_mat (2x2) = Gain (2x1) * phi_T (1x2)
    // CMSIS-DSP 的 arm_mat_mult_f32 支持通用矩阵乘法。
    // 将 Gain 看作 (2x1), phi_T 看作 (1x2)。
    // 但 arm_mat_mult 要求结果矩阵也是 arm_matrix_instance。
    arm_matrix_instance_f32 gain_mat; // 伪装 Gain 为 2x1
    arm_mat_init_f32(&gain_mat, RLS_DIM, 1, rls->gainVector.pData);
    
    // 这里有个问题：CMSIS-DSP 的 arm_mat_mult_f32 在旧版本可能不支持非方阵乘法输出非方阵，但新版支持。
    // 结果是 2x2。
    arm_mat_mult_f32(&gain_mat, &phi_T, &rls->temp_mat); 
    // 此时 temp_mat = K * phi^T
    
    // 步骤 B: temp_mat2 = temp_mat * P
    // 我们需要另一个临时 2x2 空间。由于我们只要 P - (...)，我们可以直接操作。
    // 利用结合律：K * (phi^T * P)
    // phi^T (1x2) * P (2x2) = Res (1x2)
    float phi_T_P_data[2];
    arm_matrix_instance_f32 phi_T_P_res;
    arm_mat_init_f32(&phi_T_P_res, 1, RLS_DIM, phi_T_P_data);
    arm_mat_mult_f32(&phi_T, &rls->transMatrix, &phi_T_P_res);
    
    // 现在 temp_mat = K * (phi_T_P_res)
    // 即 temp_mat (2x2) = Gain (2x1) * phi_T_P_res (1x2)
    arm_mat_mult_f32(&gain_mat, &phi_T_P_res, &rls->temp_mat);
    
    // 步骤 C: P = (P - temp_mat) / lambda
    arm_sub_f32(rls->transMatrix.pData, rls->temp_mat.pData, rls->transMatrix.pData, RLS_DIM * RLS_DIM);
    arm_scale_f32(rls->transMatrix.pData, 1.0f / rls->lambda, rls->transMatrix.pData, RLS_DIM * RLS_DIM);

    rls->updateCnt++;
    return rls->paramsVector.pData;
}

const float* RLS_GetParams(const RLS_t *rls) {
    return rls->paramsVector.pData;
}