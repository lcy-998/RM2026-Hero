#ifndef DMmotor_H
#define DMmotor_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#define MIT_mode 0x000
#define DM_MOTOR_MX_CNT 4 // 最多允许4个DM电机使用多电机指令,挂载在一条总线上
// #define P_MIN -12.5f
// #define P_MAX 12.5f
// #define V_MIN -30.0f
// #define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
// #define T_MIN -10.0f
// #define T_MAX 10.0f

typedef struct 
{
	uint8_t mode;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
}motor_ctrl_t;
typedef struct 
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
	
	float last_pos;
	int32_t total_round;
	float total_position;
}motor_fbpara_t;

typedef struct 
{
	uint16_t disable_counter;
	uint16_t error_counter;
	uint8_t error_flag;
}Motor_Error_t;


typedef struct
{
    Motor_Type_e motor_type;        // 电机类型
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    CANInstance *motor_can_instance; // 电机CAN实例
   	motor_ctrl_t ctrl;
	motor_ctrl_t cmd;
    motor_fbpara_t measure;            // 电机测量值
   	Motor_Working_Type_e stop_flag; // 启停标志
    DaemonInstance *daemon;
	Motor_Error_t motor_error;
    uint32_t feed_cnt;
    float dt;
} DMMotorInstance;

// 电机回传信息结构体

/**
 * @brief 初始化DM电机
 *
 * @param config 电机配置
 * @return DMMotorInstance* 返回实例指针
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
void DMMotorControl();
void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorEnableMode(DMMotorInstance *motor);
void DMMotorSetTorque(DMMotorInstance *motor, float torque); // 设置力矩
void DMMotorStop(DMMotorInstance *motor);
void DMMotorEnable1(DMMotorInstance *motor);
void DMMotorDisableMode(DMMotorInstance *motor);
void DMMotorSetPos(DMMotorInstance *motor, float pos, float vel);
#endif // DMmotor_H
