#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "tool.h"
#include "referee_UI.h"
#include "DMmotor.h"

static Publisher_t *shoot_pub;
static Subscriber_t *shoot_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;         
static Shoot_Upload_Data_s shoot_feedback_data; 

#ifdef CHASSIS_BOARD
static DMMotorInstance *loader;
static float loader_offset_angle = 24.55;
#endif

#ifdef GIMBAL_BOARD
static DJIMotorInstance *friction_l, *friction_r;
static float friction_feedforwardl = 100, friction_feedforwardr = -100;
static float fricl_speed = 0, limit_speed = 0, fricr_speed = 0;
const static float shoot_speed_target = 25000, shoot2_speed_target = 25000, limit_speed_target = 0;//26000->11.8//31000->14.1//25500->11.8(19degree)
#endif

#ifdef CHASSIS_BOARD
static float CalculateNextAngle(float current_angle)
{
    const float angle_step = PI / 3.0f;
    float target_angle, temp;
    uint16_t number;//与正方向相差几个PI/3
    temp = loader_offset_angle - current_angle + 0.2;
    number = (uint16_t)(temp / angle_step);
    target_angle = loader_offset_angle - ((float)(number + 1) * angle_step);
    return target_angle;
}
#endif

void ShootInit()
{
#ifdef CHASSIS_BOARD
    Motor_Init_Config_s loader_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x101,
            .rx_id = 0x11,
        },
        .motor_type = DM_4310,
        .controller_setting_init_config = {
            .control_range = {
                .P_max = 12.5,
                .V_max = 30,
                .T_max = 10,
            },
        },
        .motor_contro_type = ANGLE_LOOP_CONTRO,
    };
    loader = DMMotorInit(&loader_motor_config);
    DMMotorStop(loader);
#endif

#ifdef GIMBAL_BOARD
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1.0,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 15000,
                .MaxOut        = 10000,
            },

        },

        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            .feedforward_flag   = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508};

    friction_config.can_init_config.tx_id                                = 2; // 左摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag    = MOTOR_DIRECTION_REVERSE;
    friction_config.controller_param_init_config.current_feedforward_ptr = &friction_feedforwardr;
    friction_l                                                           = DJIMotorInit(&friction_config);
    // 三摩擦轮外加电机
    friction_config.can_init_config.tx_id                             = 1;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_config.controller_param_init_config.current_feedforward_ptr = &friction_feedforwardl;
    friction_r                                                      = DJIMotorInit(&friction_config);
    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
#endif
    shoot_cmd_recv.shoot_mode = SHOOT_ON;

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

}

static ramp_t fricl_on_ramp, fricr_on_ramp;
static ramp_t fricl_off_ramp, fricr_off_ramp;
static Loader_Mode_e last_mode;

void ShootTask()
{
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

#ifdef CHASSIS_BOARD
     if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        DMMotorStop(loader);
    } 
    else 
    {
        DMMotorEnable1(loader);
        if(loader->measure.state == 0)
        {
            loader->ctrl.pos_set = loader->measure.pos;
        }
    }

    float current_angle;
    switch (shoot_cmd_recv.load_mode) {
        case LOAD_STOP:
            last_mode = LOAD_STOP;
            break;
        case LOAD_1_BULLET:

            if (shoot_cmd_recv.friction_mode == FRICTION_OFF) break;

            if(last_mode == LOAD_STOP && shoot_cmd_recv.shooter_referee_heat < 100)
            {
                current_angle = loader->measure.total_position;
                loader->ctrl.vel_set = 5.0f;
                loader->ctrl.pos_set = CalculateNextAngle(current_angle);
            }
            last_mode = LOAD_1_BULLET;
            break;
        default:
            while (1); // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
#endif

#ifdef GIMBAL_BOARD
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
    } 
    else
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
    }

    static float shoot_speed = 0, shoot2_speed = 0, limit_shoot_speed;
    if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        fricl_speed  = (shoot_speed + (shoot_speed_target - shoot_speed) * ramp_calc(&fricl_on_ramp));
        fricr_speed = (shoot2_speed + (shoot2_speed_target - shoot2_speed) * ramp_calc(&fricr_on_ramp));
        ramp_init(&fricl_off_ramp, 3000);
        ramp_init(&fricr_off_ramp, 3000);
    } else if (shoot_cmd_recv.friction_mode == FRICTION_OFF) {
        fricl_speed  = (shoot_speed + (0 - shoot_speed) * ramp_calc(&fricl_off_ramp));
        fricr_speed = (shoot2_speed + (0 - shoot2_speed) * ramp_calc(&fricr_off_ramp));
        ramp_init(&fricl_on_ramp, 1000);
        ramp_init(&fricr_on_ramp, 1000);
    }
    DJIMotorSetRef(friction_l, fricl_speed);
    DJIMotorSetRef(friction_r, fricr_speed);
#endif
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
}

