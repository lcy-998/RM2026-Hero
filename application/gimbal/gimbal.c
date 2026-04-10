#include "stdio.h"

#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"

#include "referee_UI.h"
#include "controller.h"

static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

static attitude_t *gimbal_IMU_data;

#ifdef GIMBAL_BOARD
static DJIMotorInstance *pitch_motor;
static float pitch_current_feedforward = 0.0f;
#endif

#ifdef CHASSIS_BOARD
static DJIMotorInstance *yaw_motor;

#endif

void GimbalInit()
{
    gimbal_IMU_data = INS_Init();
#ifdef CHASSIS_BOARD
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 1.1,//1.8,//0.6, // 0.24, // 0.31, // 0.45
                .Ki            = 0,
                .Kd            = 0.007,//0.13,//0.07,
                .DeadBand      = 0.0f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 5,
            },
            .speed_PID = {
                .Kp            = 9000, // 18000, // 10500,//1000,//10000,// 11000
                .Ki            = 7000,    // 0
                .Kd            = 4,    // 10, // 30
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 12000,
                .MaxOut        = 16384,//25000, // 20000
                .Output_LPF_RC=1,//0.4,
                .CoefA=0.2,
                .CoefB=2,//0.3,
            },
            .other_angle_feedback_ptr = &gimbal_cmd_recv.yaw_actual_angle,
            .other_speed_feedback_ptr = &gimbal_cmd_recv.yaw_actual_speed,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag  =CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020};
        yaw_motor   = DJIMotorInit(&yaw_config);
#endif
#ifdef GIMBAL_BOARD
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 1.1 ,//0.16,//13, // 35, // 40, // 10
                .Ki            = 0.0,//0.5,
                .Kd            = 0.0,//0.1,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit  | PID_Derivative_On_Measurement,
                .IntegralLimit = 2,//10,
                .MaxOut        = 8,
                
            },
            .speed_PID = {
                .Kp            = -20000,//7500,//100,//6000, // 10500, // 13000,//10500,  // 10500
                .Ki            = -100,//1000,//2000, // 10000, // 10000
                .Kd            = 0,//5,    // 0
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement ,//| PID_ChangingIntegrationRate |PID_OutputFilter,
                .IntegralLimit = 1000,
                .MaxOut        = 16384,//24000,
                
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
           .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[INS_PITCH_ADDRESS_OFFSET],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020,
       
    }; 
    pitch_motor = DJIMotorInit(&pitch_config);
    DJIMotorStop(pitch_motor);
#endif
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

}

void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
#ifdef CHASSIS_BOARD
switch (gimbal_cmd_recv.gimbal_mode) {
        // ??
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            break;
        case GIMBAL_GYRO_MODE:        
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP, OTHER_FEED);
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            if (gimbal_cmd_recv.auto_aim_mode == AUTO_AIM_ON)
            {
                float error = gimbal_cmd_recv.yaw_target_angle - *yaw_motor->motor_controller.other_angle_feedback_ptr;
                if (error > 180.0f)
                {
                    gimbal_cmd_recv.yaw_target_angle = *yaw_motor->motor_controller.other_angle_feedback_ptr - 360.0 + error;
                }
                else if (error < -180.0)
                {
                    gimbal_cmd_recv.yaw_target_angle = *yaw_motor->motor_controller.other_angle_feedback_ptr + (360 + error);
                }
            }
            
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw_target_angle);
            
            break;
        case GIMBAL_MOTOR_MODE:
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP,MOTOR_FEED);
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw_target_angle); // yaw??pitch????robot_cmd?§Õ???????????
        break;
        default:
            break;
    }
    gimbal_feedback_data.yaw_ecd = yaw_motor->measure.ecd;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
#endif

#ifdef GIMBAL_BOARD
     switch (gimbal_cmd_recv.gimbal_mode) {
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(pitch_motor);
            break;
        case GIMBAL_GYRO_MODE:
            DJIMotorEnable(pitch_motor);
            DJIMotorChangeFeed(pitch_motor,SPEED_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP, OTHER_FEED);
            DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch_target_angle);
            break;
        case GIMBAL_MOTOR_MODE:
            DJIMotorEnable(pitch_motor);
            DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP,MOTOR_FEED);
            DJIMotorChangeFeed(pitch_motor,SPEED_LOOP,MOTOR_FEED);
            DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch_target_angle); 
            break;
        default:
            break;
    }

    gimbal_feedback_data.pitch_ecd = pitch_motor->measure.ecd;
#endif
    gimbal_feedback_data.gimbal_imu_data = gimbal_IMU_data;
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
