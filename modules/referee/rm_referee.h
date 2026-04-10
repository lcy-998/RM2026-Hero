#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

// 屏幕宽度
#define SCREEN_WIDTH 1080
// 屏幕长度
#define SCREEN_LENGTH 1920

// 步兵机器人不同等级对应的功率（功率优先）
#define robot_power_level_1     60
#define robot_power_level_2     65
#define robot_power_level_3     70
#define robot_power_level_4     75
#define robot_power_level_5     80
#define robot_power_level_6     85
#define robot_power_level_7     90
#define robot_power_level_8     95
#define robot_power_level_9to10 100
#define robot_power_level_MAX   200 // 当机器人满级且获得最大功率增益（步兵机器人的功率上限）

extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
    uint8_t Robot_Color;        // 机器人颜色
    uint16_t Robot_ID;          // 本机器人ID
    uint16_t Cilent_ID;         // 本机器人对应的客户端ID
    uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
    referee_id_t referee_id;

    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;
    ext_game_state_t GameState;                            // 0x0001
    ext_game_result_t GameResult;                          // 0x0002
    ext_game_robot_HP_t GameRobotHP;                       // 0x0003
    ext_event_data_t EventData;                            // 0x0101
    ext_referee_warning_t RefereeWarning;                 // 0x0104
    ext_dart_info_t DartInfo;                              // 0x105
    ext_game_robot_status_t GameRobotStatus;                 // 0x0201
    ext_power_heat_data_t PowerHeatData;                   // 0x0202
    ext_game_robot_pos_t GameRobotPos;                     // 0x0203
    ext_buff_t BuffMusk;                              // 0x0204
    ext_hurt_data_t RobotHurt;                            // 0x0206
    ext_shoot_data_t ShootData;                            // 0x0207
    ext_projectile_allowance_t ProjectileAllowance;            // 0x0208
    ext_rfid_status_t Rfid_Status;                             // 0x0209
    ext_dart_client_cmd_t DdartClientCmd;                    // 0x020A
    ext_ground_robot_position_t GroundRobotPosition;         // 0x020B
    ext_radar_mark_data_t RadarMarkData;                     // 0x020C
    ext_sentry_info_t SentryInfo;                             // 0x020D
    ext_radar_info_t RadarInfo;                               // 0x020E
   // 0x0301
    ext_map_command_t MapCommand;                             // 0x0303
    ext_map_robot_data_t MapRobotData;                       // 0x0305
    ext_map_data_t MapData;                                   // 0x0307
    ext_custom_info_t CustomInfo;                             // 0x0308


    
  

    // 自定义交互数据的接收
    Communicate_ReceiveData_t ReceiveData;

    uint8_t init_flag;
} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
    uint32_t chassis_flag : 1;
    uint32_t friction_flag : 1;
    uint32_t Power_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    Chassis_Mode_e chassis_mode;             // 底盘模式
    Shoot_Mode_e shoot_mode;                 // 发射模式设置
    Friction_Mode_e friction_mode;           // 摩擦轮关闭
    uint8_t auto_rune;                       // 打符标志

    // 上一次的模式，用于flag判断
    Chassis_Mode_e chassis_last_mode;
    Shoot_Mode_e shoot_last_mode;
    Friction_Mode_e friction_last_mode;
    uint8_t last_rune;

} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

#endif // !REFEREE_H
