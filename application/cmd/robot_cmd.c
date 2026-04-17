// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "omni_UI.h"
// module
#include "buzzer.h"
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "referee_UI.h"
#include "referee_init.h"
#include "tool.h"
#include "super_cap.h"
#include "rm_referee.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "vofa.h"

#define RC_LOST (rc_data[TEMP].rc.switch_left == 0 && rc_data[TEMP].rc.switch_right == 0)
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)

static Publisher_t *gimbal_cmd_pub  ;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Publisher_t *ui_cmd_pub;        // UI控制消息发布者
static Subscriber_t *ui_feed_sub;      // UI反馈信息订阅者
static UI_Cmd_s ui_cmd_send;           // 传递给UI的控制信息
static UI_Upload_Data_s ui_fetch_data; // 从UI获取的反馈信息

static Chassis_Ctrl_Cmd_s chassis_cmd_send;
static Chassis_Upload_Data_s chassis_fetch_data;

#ifdef CHASSIS_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者

static referee_info_t *referee_data; // 用于获取裁判系统的数据
static HostInstance *rs485_chassis_board_instance; // 底盘板通信实例,初始化时返回
static Chassis_Board_Send_Packet_s chassis_board_send_data;
static Gimbal_Board_Send_Packet_s chassis_board_recv_data;
static SuperCapInstance *supercap;

static uint8_t gimbal_board_offline_flag = 0;
#endif

#ifdef GIMBAL_BOARD
static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
static HostInstance *rs485_gimbal_board_instance; // 云台板通信实例,初始化时返回
static HostInstance *usb_vision_instance; // 上位机接口
static NUC_Receive_Packet_s vision_recv_data;
static NUC_Send_Packet_s vision_send_packet;
static Chassis_Board_Send_Packet_s gimbal_board_recv_data;
static Gimbal_Board_Send_Packet_s gimbal_board_send_data;

static float pitch_control = 0.0f;
static float yaw_control = 0.0f;

static struct Communication_Flag_s{
    uint8_t vision_fire_advice;
    uint8_t vision_detect_flag;
    uint8_t vision_connect_flag;
    uint8_t chassis_board_offline_flag;
} communication_flag;
#endif

#ifdef CHASSIS_BOARD
static void ChassisBoardRecvOfflineCallback(void *instance)
{
    DaemonReload(rs485_chassis_board_instance->daemon);
    gimbal_board_offline_flag = 1;
}

static void ChassisBoardSend() // C->G
{
    chassis_board_send_data.header = 0xA5;

    chassis_board_send_data.bullet_speed = referee_data->ShootData.bullet_speed;
    chassis_board_send_data.enermy_color = referee_data->referee_id.Robot_Color;

    chassis_board_send_data.tail = 0x5A;

    HostSend(rs485_chassis_board_instance, &chassis_board_send_data, sizeof(chassis_board_send_data));
}

static void ChassisBoardRecvCallback() // G->C
{
    DaemonReload(rs485_chassis_board_instance->daemon);
    gimbal_board_offline_flag = 0;
    memcpy(&chassis_board_recv_data, rs485_chassis_board_instance->comm_instance, sizeof(chassis_board_recv_data));

    ChassisBoardSend();
}

static void  CalcOffsetAngle()
{
    static float angle;
    angle                               = gimbal_fetch_data.yaw_motor_single_round_angle;
#if YAW_ECD_GREATER_THAN_4096 // 如果大于180度
    if (angle < 180.0f + YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle =- (angle - YAW_ALIGN_ANGLE);
    else
        chassis_cmd_send.offset_angle =- (angle - YAW_ALIGN_ANGLE + 360.0f);
#else // 小于180度
    if (angle >= YAW_ALIGN_ANGLE - 180.0f && angle <= YAW_ALIGN_ANGLE + 180.0f) {
        chassis_cmd_send.offset_angle = -(angle - YAW_ALIGN_ANGLE);
    } else {
        chassis_cmd_send.offset_angle = -(angle - YAW_ALIGN_ANGLE - 360.0f);
    }
#endif
}


#endif

#ifdef GIMBAL_BOARD
static void GimbalBoardRecvOfflineCallback(void *instance)
{
    DaemonReload(rs485_gimbal_board_instance->daemon);
    communication_flag.chassis_board_offline_flag = 1;
}

static void GimbalBoardRecvCallback()
{
    DaemonReload(rs485_gimbal_board_instance->daemon);
    communication_flag.chassis_board_offline_flag = 0;
    memcpy(&gimbal_board_recv_data, rs485_gimbal_board_instance->comm_instance, sizeof(gimbal_board_recv_data));
}

static void GimbalBoardSend()
{
    gimbal_board_send_data.header = 0xA5;
    //底盘控制
    gimbal_board_send_data.cmd_vx = chassis_cmd_send.vx;
    gimbal_board_send_data.cmd_vy = chassis_cmd_send.vy;
    gimbal_board_send_data.cmd_wz = chassis_cmd_send.wz;
    gimbal_board_send_data.chassis_mode = chassis_cmd_send.chassis_mode;
    gimbal_board_send_data.supercap_flag = chassis_cmd_send.supercap_flag;

    //云台控制

    gimbal_board_send_data.yaw_actual_angle = gimbal_fetch_data.gimbal_imu_data->Yaw;
    gimbal_board_send_data.yaw_actual_speed = gimbal_fetch_data.gimbal_imu_data->Gyro[INS_YAW_ADDRESS_OFFSET];
    gimbal_board_send_data.yaw_target_angle = gimbal_cmd_send.yaw_target_angle;
    gimbal_board_send_data.yaw_target_speed = gimbal_cmd_send.yaw_target_speed;
    gimbal_board_send_data.gimbal_mode = gimbal_cmd_send.gimbal_mode;
    gimbal_board_send_data.auto_aim_mode = gimbal_cmd_send.auto_aim_mode;

    //发射控制
    gimbal_board_send_data.shoot_mode = shoot_cmd_send.shoot_mode;
    gimbal_board_send_data.load_mode = shoot_cmd_send.load_mode;
    gimbal_board_send_data.friction_mode = shoot_cmd_send.friction_mode;

    gimbal_board_send_data.ui_refresh_flag = ui_cmd_send.ui_refresh_flag;

    gimbal_board_send_data.tail = 0x5A;

    HostSend(rs485_gimbal_board_instance, &gimbal_board_send_data, sizeof(gimbal_board_send_data));
}

static void VisionOfflineCallback(void *instance)
{
    DaemonReload(usb_vision_instance->daemon); 
    communication_flag.vision_detect_flag = 0;
    communication_flag.vision_fire_advice = 0;
    communication_flag.vision_connect_flag = 0;
}

static void VisionRecvCallback()
{
    DaemonReload(usb_vision_instance->daemon); 
    memcpy(&vision_recv_data, usb_vision_instance->comm_instance, sizeof(vision_recv_data));

    communication_flag.vision_detect_flag = !(!vision_recv_data.mode);
    communication_flag.vision_fire_advice = (vision_recv_data.mode == 2);

    if (!communication_flag.vision_connect_flag)
    {
        
        communication_flag.vision_connect_flag = 1;
    }
}

static void VisionSendMessage()
{
    vision_send_packet.header[0] = 'C';
    vision_send_packet.header[1] = 'B';
    vision_send_packet.mode = 1;

    float q[4];
    EularAngleToQuaternion(gimbal_fetch_data.gimbal_imu_data->Yaw , gimbal_fetch_data.gimbal_imu_data->Pitch, gimbal_fetch_data.gimbal_imu_data->Roll, q);
    vision_send_packet.q[0] = q[0];
    vision_send_packet.q[1] = q[1];
    vision_send_packet.q[2] = q[2];
    vision_send_packet.q[3] = q[3];

    vision_send_packet.pitch = gimbal_fetch_data.gimbal_imu_data->Pitch * DEGREE_2_RAD;
    vision_send_packet.pitch_vel = gimbal_fetch_data.gimbal_imu_data->Gyro[INS_PITCH_ADDRESS_OFFSET] ;
    vision_send_packet.yaw = gimbal_fetch_data.gimbal_imu_data->Yaw * DEGREE_2_RAD;
    vision_send_packet.yaw_vel = gimbal_fetch_data.gimbal_imu_data->Gyro[INS_YAW_ADDRESS_OFFSET];

    vision_send_packet.bullet_speed = gimbal_board_recv_data.bullet_speed;
    vision_send_packet.tail = 0xff;
    
    HostSend(usb_vision_instance, &vision_send_packet, sizeof(vision_send_packet));
}

static void PitchAngleLimit()
{
    float limit_min, limit_max;
#if PITCH_INS_FEED_TYPE
    limit_min = -19.0f;//PITCH_LIMIT_ANGLE_DOWN * DEGREE_2_RAD;
    limit_max = 36.0f;//PITCH_LIMIT_ANGLE_UP * DEGREE_2_RAD;
#else
    limit_min = -30;//PITCH_LIMIT_ANGLE_DOWN;
    limit_max = 21;//PITCH_LIMIT_ANGLE_UP;
#endif

#if PITCH_ECD_UP_ADD // 云台抬升,反馈值增
    if (pitch_control > limit_max)
        pitch_control = limit_max;
    if (pitch_control < limit_min)
        pitch_control = limit_min;

#else
    if (pitch_control < limit_max)
        pitch_control = limit_max;
    if (pitch_control > limit_min)
        pitch_control = limit_min;
#endif
}

static void YawControlProcess()
{
    if (yaw_control - gimbal_fetch_data.gimbal_imu_data->Yaw > 180) {
        yaw_control -= 360;
    } else if (yaw_control - gimbal_fetch_data.gimbal_imu_data->Yaw < -180) {
        yaw_control += 360;
    }
}

static void RemoteControlSet()
{
    shoot_cmd_send.shoot_mode   = SHOOT_ON; // 发射机构常开
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    //滚轮下拨一下切换自瞄或关闭自瞄

    if (rc_data[TEMP].rc_update_flag == 1)
    {
        if (rc_data[TEMP].rc.dial > 250 && rc_data[LAST].rc.dial < 250)
        {
            if (gimbal_cmd_send.auto_aim_mode != AUTO_AIM_ON)
            {
                gimbal_cmd_send.auto_aim_mode = AUTO_AIM_ON;
            }
                
            else
                gimbal_cmd_send.auto_aim_mode = AUTO_AIM_OFF;
        }

        // if (rc_data[TEMP].rc.dial > 250 && rc_data[LAST].rc.dial < 250)
        // {
        //     if (SuperCap_flag_from_user != SUPERCAP_UNUSE)
        //     {
        //         SuperCap_flag_from_user = SUPERCAP_UNUSE;
        //     }
        //     else
        //     {
        //         SuperCap_flag_from_user = SUPERCAP_USE;
        //     }
        // }

        switch (rc_data[TEMP].rc.switch_left)
        {
            case RC_SW_UP:

                if (rc_data[LAST].rc.switch_left == RC_SW_MID)//左中到上开关摩擦轮
                {
                    if (shoot_cmd_send.friction_mode == FRICTION_ON)
                        shoot_cmd_send.friction_mode = FRICTION_OFF;
                    else
                        shoot_cmd_send.friction_mode = FRICTION_ON;
                }
                break;

            case RC_SW_DOWN:

                if (gimbal_cmd_send.auto_aim_mode == AUTO_AIM_OFF)
                {
                    if (rc_data[LAST].rc.switch_left == RC_SW_MID && shoot_cmd_send.friction_mode == FRICTION_ON)//左中到下且开摩擦轮时打弹
                    {
                        shoot_cmd_send.load_mode = LOAD_1_BULLET;
                    }
                }
                else
                {
                    if (vision_recv_data.mode == 2)
                    {
                        shoot_cmd_send.load_mode = LOAD_1_BULLET;
                    }
                }
                
                break;
            case RC_SW_MID:
                if (rc_data[LAST].rc.switch_left == RC_SW_DOWN)
                {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
                break;
        }

        switch (rc_data[TEMP].rc.switch_right)
        {
            case RC_SW_UP:
                if (rc_data[LAST].rc.switch_right == RC_SW_MID)
                {
                    if (chassis_cmd_send.chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW)
                        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
                    else
                        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
                }
                break;
            case RC_SW_DOWN:
                if (rc_data[LAST].rc.switch_right == RC_SW_MID)
                {
                    if (chassis_cmd_send.chassis_mode != CHASSIS_ROTATE)
                        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
                    else
                        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
                }
                break;
        }
        
        rc_data[TEMP].rc_update_flag = 0;
    }
    
    if (gimbal_cmd_send.auto_aim_mode == AUTO_AIM_ON && communication_flag.vision_detect_flag)
    {
        pitch_control = vision_recv_data.pitch * RAD_2_DEGREE;
        yaw_control = vision_recv_data.yaw * RAD_2_DEGREE;
    }
    else{
        pitch_control += /*0.1**/PITCH_K* (float)rc_data[TEMP].rc.rocker_l1 ;
        yaw_control -= /*0.05**/YAW_K * (float)rc_data[TEMP].rc.rocker_l_ ;
    }
    // 底盘参数
    chassis_cmd_send.vx = 70.0f * (float)rc_data[TEMP].rc.rocker_r1; // 水平方向
    chassis_cmd_send.vy = 70.0f * (float)rc_data[TEMP].rc.rocker_r_; // 竖直方向
    
    YawControlProcess();
    
     gimbal_cmd_send.yaw_target_angle   = yaw_control;
     gimbal_cmd_send.pitch_target_angle = pitch_control;    
}


static ramp_t fb_ramp;
static ramp_t lr_ramp;
static ramp_t slow_ramp;
static const float CHASSIS_SPEED_MAX = 40000.0F;
static void ChassisSet()
{
    // 底盘移动
    static float current_speed_x = 0;
    static float current_speed_y = 0;
    // 前后移动
    // 防止逃跑时关小陀螺按Ctrl进入慢速模式
    if (rc_data[TEMP].key[KEY_PRESS].w) {
        chassis_cmd_send.vx = (current_speed_x + (CHASSIS_SPEED_MAX - current_speed_x) * ramp_calc(&fb_ramp)); // vx方向待测
        ramp_init(&slow_ramp, RAMP_TIME);                                                                  // 2000
    } else if (rc_data[TEMP].key[KEY_PRESS].s) {
        chassis_cmd_send.vx = (current_speed_x + (-CHASSIS_SPEED_MAX - current_speed_x) * ramp_calc(&fb_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w) { // 防止逃跑关小陀螺进入慢速移动
        chassis_cmd_send.vx = (current_speed_x + (4000 - current_speed_x) * ramp_calc(&slow_ramp));
        ramp_init(&fb_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s) {
        chassis_cmd_send.vx = (current_speed_x + (-4000 - current_speed_x) * ramp_calc(&slow_ramp));
        ramp_init(&fb_ramp, RAMP_TIME);
    } else {
        chassis_cmd_send.vx = 0;
        ramp_init(&fb_ramp, RAMP_TIME);
    }

    // 左右移动
    if (rc_data[TEMP].key[KEY_PRESS].a) {
        chassis_cmd_send.vy = (current_speed_y + (CHASSIS_SPEED_MAX - current_speed_y) * ramp_calc(&lr_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS].d) {
        chassis_cmd_send.vy = (current_speed_y + (-CHASSIS_SPEED_MAX - current_speed_y) * ramp_calc(&lr_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a) {
        chassis_cmd_send.vy = (current_speed_y + (+4000 - current_speed_y) * ramp_calc(&fb_ramp));
        ramp_init(&lr_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d) {
        chassis_cmd_send.vy = (current_speed_y + (-4000 - current_speed_y) * ramp_calc(&fb_ramp));
        ramp_init(&lr_ramp, RAMP_TIME);
    } else {
        chassis_cmd_send.vy = 0;
        ramp_init(&lr_ramp, RAMP_TIME);
    }

    current_speed_x = chassis_cmd_send.vx;
    current_speed_y = chassis_cmd_send.vy;
}

static void GimbalSet()
{
    if(rc_data[TEMP].mouse.press_r)
    {
        gimbal_cmd_send.auto_aim_mode = AUTO_AIM_ON;
        if (communication_flag.vision_detect_flag)
        {
            pitch_control = vision_recv_data.pitch * RAD_2_DEGREE;
            yaw_control = vision_recv_data.yaw * RAD_2_DEGREE;
        }
        else
        {
            yaw_control -= rc_data[TEMP].mouse.x / 500.0f;
            pitch_control -= rc_data[TEMP].mouse.y / 800.0f;
        }
    }
    else
    {
        gimbal_cmd_send.auto_aim_mode = AUTO_AIM_OFF;
        yaw_control -= rc_data[TEMP].mouse.x / 500.0f;
        pitch_control -= rc_data[TEMP].mouse.y / 800.0f;
    }

    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    YawControlProcess();
    gimbal_cmd_send.yaw_target_angle   = yaw_control;
    gimbal_cmd_send.pitch_target_angle = pitch_control;
}

static void ShootSet()
{
    shoot_cmd_send.shoot_mode = SHOOT_ON;

    // 仅在摩擦轮开启时有效
    if (shoot_cmd_send.friction_mode == FRICTION_ON) 
    {
        // 打弹，单击左键单发，长按连发
        if (rc_data[TEMP].mouse.press_l) 
        {
            if (gimbal_cmd_send.auto_aim_mode == AUTO_AIM_OFF)
            {
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
            }
            else
            {
                if (communication_flag.vision_fire_advice)
                {
                    shoot_cmd_send.load_mode = LOAD_1_BULLET;
                }
                else
                {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
            }
        } 
        else 
        {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    } 
    else
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

static void KeyGetMode()
{
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) {
        case 1:
            if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
                chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            break;
        case 0:
            if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE)
                chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 2) {
        case 1:
            if (shoot_cmd_send.friction_mode != FRICTION_ON)
                shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].r) {
        case 1:
            ui_cmd_send.ui_refresh_flag = 1;
            break;
        case 0:
            ui_cmd_send.ui_refresh_flag = 0;
            break;
    }
    
    switch(rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2){
        case 1:
            chassis_cmd_send.supercap_flag = SUPERCAP_USE;
        break;
        case 0:
            chassis_cmd_send.supercap_flag = SUPERCAP_UNUSE;
        break;
    }
}

static void MouseKeySet()
{
    ChassisSet();
    GimbalSet();
    ShootSet();
    KeyGetMode();
}
#endif

static void EmergencyHandler()
{
    gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.friction_mode  = FRICTION_OFF;
    shoot_cmd_send.load_mode      = LOAD_STOP;
    shoot_cmd_send.shoot_mode     = SHOOT_OFF;
    chassis_cmd_send.supercap_flag = SUPERCAP_UNUSE;
    LOGERROR("[CMD] emergency stop!");
}

void RobotCMDInit()
{
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef CHASSIS_BOARD
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    ui_cmd_pub  = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    ui_feed_sub = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));

    referee_data = RefereeHardwareInit(&huart6);

    HostInstanceConf host_conf = {
        .usart_handle = &huart1,
        .callback  = ChassisBoardRecvCallback,
        .comm_mode = HOST_USART,
        .RECV_SIZE = sizeof(Gimbal_Board_Send_Packet_s),
        .daemon_conf = {
            .reload_count = 100,
            .callback = ChassisBoardRecvOfflineCallback,
        }
    };
    rs485_chassis_board_instance = HostInit(&host_conf);

    SuperCap_Init_Config_s supercap_config = {
        .can_config = {
            .can_handle = &hcan1,
        },
    };
    supercap = SuperCapInit(&supercap_config);
    SuperCapEnable(supercap);
    
#endif

#ifdef GIMBAL_BOARD
    rc_data = RemoteControlInit(&huart3);

    HostInstanceConf rs485_host_conf = {
        .usart_handle = &huart1,
        .callback  = GimbalBoardRecvCallback,
        .comm_mode = HOST_USART,
        .RECV_SIZE = sizeof(Chassis_Board_Send_Packet_s),
        .daemon_conf = {
            .reload_count = 100,
            .callback = GimbalBoardRecvOfflineCallback,
        }
    };
    rs485_gimbal_board_instance = HostInit(&rs485_host_conf);

    HostInstanceConf vision_host_conf = {

        .callback  = VisionRecvCallback,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = sizeof(NUC_Receive_Packet_s),
        .daemon_conf = {
            .reload_count = 5000,
            .callback = VisionOfflineCallback,
        }
    };
    usb_vision_instance = HostInit(&vision_host_conf); // 视觉通信串口
#endif

}

void RobotCMDTask()
{
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
#ifdef CHASSIS_BOARD
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
    SubGetMessage(ui_feed_sub, &ui_fetch_data);
    
    CalcOffsetAngle();

    chassis_cmd_send.vx = chassis_board_recv_data.cmd_vx;
    chassis_cmd_send.vy = chassis_board_recv_data.cmd_vy;
    chassis_cmd_send.wz = chassis_board_recv_data.cmd_wz;
    chassis_cmd_send.chassis_mode = chassis_board_recv_data.chassis_mode;
    chassis_cmd_send.supercap_flag = chassis_board_recv_data.supercap_flag;

    chassis_cmd_send.power_buffer = referee_data->PowerHeatData.chassis_power_buffer;
    chassis_cmd_send.power_limit = referee_data->GameRobotStatus.chassis_power_limit;

    gimbal_cmd_send.yaw_actual_angle = chassis_board_recv_data.yaw_actual_angle;
    gimbal_cmd_send.yaw_actual_speed = chassis_board_recv_data.yaw_actual_speed;
    gimbal_cmd_send.yaw_target_angle = chassis_board_recv_data.yaw_target_angle;
    gimbal_cmd_send.yaw_target_speed = chassis_board_recv_data.yaw_target_speed;

    gimbal_cmd_send.gimbal_mode = chassis_board_recv_data.gimbal_mode;
    gimbal_cmd_send.auto_aim_mode = chassis_board_recv_data.auto_aim_mode;

    shoot_cmd_send.shoot_mode = chassis_board_recv_data.shoot_mode;
    shoot_cmd_send.load_mode = chassis_board_recv_data.load_mode;
    shoot_cmd_send.friction_mode = chassis_board_recv_data.friction_mode;
    shoot_cmd_send.shooter_referee_heat = referee_data->PowerHeatData.shooter_42mm_heat;


    ui_cmd_send.ui_refresh_flag = chassis_board_recv_data.ui_refresh_flag;
    ui_cmd_send.chassis_mode = chassis_board_recv_data.chassis_mode;
    ui_cmd_send.gimbal_mode = chassis_board_recv_data.gimbal_mode;
    ui_cmd_send.friction_mode = chassis_board_recv_data.friction_mode;
    ui_cmd_send.chassis_attitude_angle = gimbal_fetch_data.yaw_motor_single_round_angle;

    if (gimbal_board_offline_flag)
    {
        EmergencyHandler();
    }

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
#endif

#ifdef GIMBAL_BOARD

    if (switch_is_up(rc_data[TEMP].rc.switch_left) && (switch_is_down(rc_data[TEMP].rc.switch_right))) // 遥控器拨杆右[上]左[下],键鼠控制
        MouseKeySet();
    else if (communication_flag.chassis_board_offline_flag || RC_LOST || (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))) {
        EmergencyHandler(); // 调试/疯车时急停
    }
    else {
        RemoteControlSet();
        PitchAngleLimit();
    }
    gimbal_cmd_send.pitch_target_speed = vision_recv_data.pitch_vel;
    gimbal_cmd_send.yaw_target_speed = vision_recv_data.yaw_vel;
    VisionSendMessage();
    GimbalBoardSend();
#endif
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}