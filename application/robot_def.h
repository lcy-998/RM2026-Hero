#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H
#include "stdint.h"
#include "ins_task.h"

#define CHASSIS_BOARD //底盘板
//#define GIMBAL_BOARD  //云台板

#define YAW_K                  0.00025f
#define PITCH_K                0.00025f

#define YAW_CHASSIS_ALIGN_ECD     3566 // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0    // ALIGN_ECD算云台偏转角度

#define PITCH_ECD_UP_ADD    1 // 云台抬升时编码器变化趋势,增为1,减为0 (陀螺仪变化方向应相同)

#define WHEEL_BASE             414.5   // 320.5   // 纵向轴距(前进后退方向)
#define TRACK_WIDTH            361   // 320.5   // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0     // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0     // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL           77   // 轮子半径
#define REDUCTION_RATIO_WHEEL  19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#define INS_YAW_ADDRESS_OFFSET   2 // 陀螺仪数据相较于云台的yaw的方向
#define INS_PITCH_ADDRESS_OFFSET 0 // 陀螺仪数据相较于云台的pitch的方向
#define INS_ROLL_ADDRESS_OFFSET  1 // 陀螺仪数据相较于云台的roll的方向

#define PUTTER_DOWN_OFFSET 6100.0f // 推杆下压的目标位置与推杆初始位置的差值,需要根据实车情况调整,当前值为上台阶所需的推杆行程
#define TRACK_WHEEL_REF 7600.0f    // 上台阶时履带的目标速度
#define TRACK_WHEEL_RADIUS 0.038f       // 履带轮的半径,用于计算履带线速度与角速度的关系
#define TRACK_WHEEL_TO_CENTER 0.3f // 履带到中心的距离



typedef enum {
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_REVERSE_ROTATE,    // 反方向小陀螺
} Chassis_Mode_e;

typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_MOTOR_MODE,     //编码器反馈模式，英雄部署模式可用
} Gimbal_Mode_e;

typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} Shoot_Mode_e;

typedef enum {
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} Friction_Mode_e;

typedef enum {
    LOAD_STOP = 0,  // 停止发射
    LOAD_1_BULLET,  // 单发
    LOAD_BURSTFIRE, // 连发
} Loader_Mode_e;

typedef enum {
    SUPERCAP_UNUSE = 0,
    SUPERCAP_USE
} SuperCap_Mode_e;

typedef enum {
    AUTO_AIM_OFF = 0,
    AUTO_AIM_ON
} Auto_Aim_Mode_e;

typedef enum {
    TRACK_WHEEL_OFF = 0,
    TRACK_WHEEL_ON
} Track_Wheel_Mode_e;

typedef enum {
    PUTTER_OFF = 0,
    PUTTER_ON
} Putter_Mode_e;

typedef struct
{
    float vx;                        // 前进方向速度
    float vy;                        // 横移方向速度
    float wz;                        // 旋转速度
    float offset_angle;              // 底盘和归中位置的夹角
    Chassis_Mode_e chassis_mode;
    SuperCap_Mode_e supercap_flag;  // 超电的标志位

    uint16_t power_buffer;           // 60焦耳缓冲能量
    uint16_t power_limit;            // 底盘功率限制

    Track_Wheel_Mode_e track_wheel_mode;
    float putter_offset;
    uint8_t is_power_on;           // 电管chassis口供电标志位 1->供电 0->断电
} Chassis_Ctrl_Cmd_s;

typedef struct
{
    float yaw_target_angle;
    float yaw_target_speed;
    float yaw_actual_angle;
    float yaw_actual_speed;
    float pitch_target_angle;
    float pitch_target_speed;
    Auto_Aim_Mode_e auto_aim_mode; 
    Gimbal_Mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

typedef struct
{
    Shoot_Mode_e shoot_mode;
    Loader_Mode_e load_mode;
    Friction_Mode_e friction_mode;
    uint16_t shooter_referee_heat;
} Shoot_Ctrl_Cmd_s;

typedef struct
{
    uint8_t ui_refresh_flag;
    Chassis_Mode_e chassis_mode;
    Gimbal_Mode_e gimbal_mode;
    Friction_Mode_e friction_mode;
    SuperCap_Mode_e supercap_mode;
    float chassis_attitude_angle;
    float chassis_real_power;
    float supercap_voltage;
    uint8_t cap_online_flag;
    uint16_t shooter_referee_heat;
} UI_Cmd_s;

typedef struct
{
    float real_vx;
    float real_vy;
    float real_wz;

    float chassis_real_power; // 底盘实际功率
    uint8_t cap_energy;        // 超电能量
    uint8_t cap_online_flag; // 超电在线标志位

    float putter_offset; //推杆推出长度

} Chassis_Upload_Data_s;

typedef struct
{
    uint16_t yaw_ecd;
    float yaw_motor_single_round_angle;
    uint16_t pitch_ecd;
    attitude_t *gimbal_imu_data;
} Gimbal_Upload_Data_s;

typedef struct
{

} Shoot_Upload_Data_s;

typedef struct
{

} UI_Upload_Data_s;

#pragma pack(1)
typedef struct
{
    uint8_t header; // 数据包头
    //底盘反馈
    float putter_offset; //推杆位置

    //裁判系统反馈
    float bullet_speed;
    uint8_t enermy_color;

    uint8_t tail; // 数据包尾
} Chassis_Board_Send_Packet_s;

typedef struct 
{
    uint8_t header; // 数据包头
    //底盘控制部分
    float cmd_vx;
    float cmd_vy;
    float cmd_wz;
    Chassis_Mode_e chassis_mode;
    SuperCap_Mode_e supercap_flag;
    float putter_offset;

    //云台控制部分
    float yaw_actual_angle;
    float yaw_actual_speed;
    float yaw_target_angle;
    float yaw_target_speed;
    Gimbal_Mode_e gimbal_mode;
    Auto_Aim_Mode_e auto_aim_mode;

    //发射控制部分
    Shoot_Mode_e shoot_mode;
    Loader_Mode_e load_mode;
    Friction_Mode_e friction_mode;

    // UI控制部分
    uint8_t ui_refresh_flag;

    uint8_t tail; // 数据包尾
} Gimbal_Board_Send_Packet_s;

typedef struct {
    uint8_t header[2]; // 数据包头
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint8_t tail; // 数据包尾
}NUC_Send_Packet_s;

typedef struct {
    uint8_t header[2]; // 数据包头
    uint8_t mode; //0:不控制，1:控制云台不开火,2:控制云台开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint8_t tail;
}NUC_Receive_Packet_s;
#pragma pack()

#endif
