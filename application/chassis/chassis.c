#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_init.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "arm_math.h"
#include "power_calc.h"
#include "tool.h"
#include "wattmeter.h"

#define HALF_WHEEL_BASE  (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)    // 半轮距
#define PERIMETER_WHEEL  (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

#ifdef CHASSIS_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
static PIDInstance *chassis_follow_pid;
static SuperCapInstance *supercap;
static WattmeterInstance *wattmeter;

static float chassis_vx, chassis_vy, chassis_vw; // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;
static float vxy_k = 1.0f, vw_k = 1.0f;
#endif

#ifdef GIMBAL_BOARD

#endif

#ifdef CHASSIS_BOARD
static float GetChassisVwFromPowerLimit(float power_limit)
{
    if (power_limit < 70.0f) return 3500.0f;
    if (power_limit < 75.0f) return 3750.0f;
    if (power_limit < 80.0f) return 4000.0f;
    if (power_limit < 85.0f) return 4175.0f;
    if (power_limit < 90.0f) return 4350.0f;
    if (power_limit < 95.0f) return 4525.0f;
    if (power_limit < 100.0f) return 4700.0f;
    if (power_limit < 105.0f) return 4900.0f;
    if (power_limit < 110.0f) return 5100.0f;
    if (power_limit < 120.0f) return 5300.0f;
    if (power_limit < 130.0f) return 5500.0f;
    if (power_limit < 140.0f) return 5700.0f;
    if (power_limit < 160.0f) return 6400.0f;
    if (power_limit < 250.0f) return 7000.0f;
    return 3000.0f;
}

static void MecanumCalculate()
{
    vt_lf = -chassis_vx - chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
}

float Power_Output;
const float buffer_energy_loop_kp = 2.0f;
const float cap_voltage_output_loop_kp = 3.0f;//放电时的kp
const float cap_voltage_input_loop_kp = 1.0f;//充电时的kp
float buffer_power_rectification, cap_power_rectification;

void Super_Cap_control()
 {
    float buffer_energy_target, buffer_energy_actual, cap_energy_target, cap_energy_actual;

    Power_Output = chassis_cmd_recv.power_limit;

    //缓冲能量环
    buffer_energy_actual = chassis_cmd_recv.power_buffer;
    buffer_energy_target = 50.0f;
    buffer_power_rectification = (buffer_energy_actual - buffer_energy_target) * buffer_energy_loop_kp;
    Power_Output += buffer_power_rectification;

    //电容能量环
    if (SuperCapIsOnline(supercap))
    {
        cap_energy_actual = SuperCapGetCapEnergy(supercap);
        if (chassis_cmd_recv.supercap_flag == SUPERCAP_USE)
            cap_energy_target = SUPERCAP_LOWER_THRESHOLD_ENERGY;
        else cap_energy_target = SUPERCAP_HIGHER_THRESHOLD_ENERGY;

        if (cap_energy_actual > cap_energy_target)//电压偏大放电
            cap_power_rectification = (cap_energy_actual - cap_energy_target) * cap_voltage_output_loop_kp;
        else cap_power_rectification = (cap_energy_actual - cap_energy_target) * cap_voltage_input_loop_kp;
        if (cap_power_rectification > 100.0f) cap_power_rectification = 100.0f;
        else if (cap_power_rectification < -50.0f) cap_power_rectification = -50.0f;
        Power_Output += cap_power_rectification;
        
        Power_Output -= 2.0f; //超电静态功耗
    }

    //底盘下电时超电失能
    if (!chassis_cmd_recv.is_power_on)
        SuperCapDisable(supercap);
    else SuperCapEnable(supercap);

    PowerControlupdate(Power_Output, 1.0f / REDUCTION_RATIO_WHEEL, wattmeter->power);

    SuperCapSetPowerLimit(supercap, chassis_cmd_recv.power_limit);

    SuperCapControl();

     // 设定速度参考值
     DJIMotorSetRef(motor_lf, vt_lf);
     DJIMotorSetRef(motor_rf, vt_rf);
     DJIMotorSetRef(motor_lb, vt_lb);
     DJIMotorSetRef(motor_rb, vt_rb);
 }
#endif

void ChassisInit()
{
    #ifdef CHASSIS_BOARD
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1.0, // 4.5
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 15000,
            }},
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    PID_Init_Config_s chassis_follow_pid_config = {
        .Kp            = 105.0f,
        .Ki            = 0.0f,
        .Kd            = 1.5f,
        .DeadBand      = 2.0f,
        .IntegralLimit = 3000.0f,
        .MaxOut        = 16384.0f,
        .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    };
    chassis_follow_pid = PIDRegister(&chassis_follow_pid_config);

    SuperCap_Init_Config_s supercap_config = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0x001,
            .rx_id      = 0x100,
        },
    };
    supercap = SuperCapRegister(&supercap_config);
    SuperCapEnable(supercap);

    Wattmeter_Init_Config_s wattmeter_config = {
        .can_config = {
            .rx_id = 0x212,
            .can_handle = &hcan1,
        }
    };
    wattmeter = WattmeterInit(&wattmeter_config);

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
}

void ChassisTask()
{
    #ifdef CHASSIS_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE){ 
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    static float offset_angle;
    static float sin_theta, cos_theta;
    static ramp_t rotate_ramp;
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW:
            chassis_cmd_recv.wz = 0;
            cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: 
            if (chassis_cmd_recv.offset_angle <= 90 && chassis_cmd_recv.offset_angle >= -90) // 0附近
                offset_angle =chassis_cmd_recv.offset_angle;
            else 
                offset_angle =(chassis_cmd_recv.offset_angle >= 0 ? chassis_cmd_recv.offset_angle - 180 : chassis_cmd_recv.offset_angle + 180);
            
            chassis_cmd_recv.wz = PIDCalculate(chassis_follow_pid, offset_angle, 0);
            cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
            chassis_cmd_recv.wz = 5000;
            cos_theta           = arm_cos_f32((chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD); // 矫正小陀螺偏心
            sin_theta           = arm_sin_f32((chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD);
            break;
            
        case CHASSIS_REVERSE_ROTATE:
            chassis_cmd_recv.wz = GetChassisVwFromPowerLimit(Power_Output);
            cos_theta           = arm_cos_f32((chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD); // 矫正小陀螺偏心
            sin_theta           = arm_sin_f32((chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD);
        default:
        break;
    }

    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    chassis_vx *= vxy_k;
    chassis_vy *= vxy_k;

    MecanumCalculate();

    Super_Cap_control();

    chassis_feedback_data.chassis_real_power = SuperCapGetChassisRealPower(supercap);
    chassis_feedback_data.cap_energy = SuperCapGetCapEnergy(supercap);
    chassis_feedback_data.cap_online_flag = SuperCapIsOnline(supercap);
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    #endif
}

