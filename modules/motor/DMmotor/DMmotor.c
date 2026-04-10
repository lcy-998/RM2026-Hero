#include "DMmotor.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"

uint8_t idm                                               = 0;
static DMMotorInstance *dmmotor_instance[DM_MOTOR_MX_CNT] = {NULL};
void DMMotorEnableMode(DMMotorInstance *motor);
void DMMotorErrorDetection(DMMotorInstance *motor);
void mit_ctrl(DMMotorInstance *motor);
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void DMMotorDecode(CANInstance *_instance)
{
    DMMotorInstance *motor     = (DMMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    motor_fbpara_t *measure = &motor->measure;
    uint8_t *rx_buff           = _instance->rx_buff;

    DaemonReload(motor->daemon); // 喂狗
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);


	measure->id = (rx_buff[0])&0x0F;
	measure->state = (rx_buff[0])>>4;
	measure->p_int=(rx_buff[1]<<8)|rx_buff[2];
	measure->v_int=(rx_buff[3]<<4)|(rx_buff[4]>>4);
	measure->t_int=((rx_buff[4]&0xF)<<8)|rx_buff[5];
	measure->pos = uint_to_float(measure->p_int, -motor->motor_settings.control_range.P_max, motor->motor_settings.control_range.P_max, 16); // (-12.5,12.5)
	measure->vel = uint_to_float(measure->v_int, -motor->motor_settings.control_range.V_max, motor->motor_settings.control_range.V_max, 12); // (-45.0,45.0)
	measure->tor = uint_to_float(measure->t_int, -motor->motor_settings.control_range.T_max, motor->motor_settings.control_range.T_max, 12);  // (-18.0,18.0)
	measure->Tmos = (float)(rx_buff[6]);
	measure->Tcoil = (float)(rx_buff[7]);

    if(measure->pos - measure->last_pos < -12.5f)
    {
        measure->total_round ++;
    }
    else if (measure->pos - measure->last_pos > 12.5)
    {
        measure->total_round --;
    }
    measure->total_position = measure->pos + measure->total_round * 25.0f;
    measure->last_pos = measure->pos;

    DMMotorErrorDetection(motor);
}

static void DMMotorLostCallback(void *motor_ptr)
{
    if (motor_ptr != NULL)
    {
        DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
        motor->measure.total_round = 0;
        motor->ctrl.pos_set = motor->measure.pos;
    }
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    // motor = (DRMotorInstance *)malloc(sizeof(DRMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
	// motor basic setting 电机基本设置
	motor ->ctrl.kp_set = 0;
	motor ->ctrl.kd_set = 0;//config->controller_param_init_config.speed_PID.Kd;
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;
    motor ->ctrl.mode= config->motor_contro_type; // 控制模式
    motor->motor_type = config->motor_type;
    config->can_init_config.id                  = motor;
    config->can_init_config.can_module_callback = DMMotorDecode;

    motor->motor_can_instance          = CANRegister(&config->can_init_config);

  DMMotorEnableMode(motor); // 使能电机模式,发送使能指令
  for (size_t i = 0; i < 10; i++)
    {
        CANTransmit(motor->motor_can_instance, 2);
    }
          motor->ctrl.vel_set = 0;
          motor->ctrl.tor_set = 0 ; //   默认关闭，避免出问题
    
    dmmotor_instance[idm++] = motor;

    Daemon_Init_Config_s daemon_config = {
        .callback     = DMMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 2, // 20ms
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}
void pos_ctrl(DMMotorInstance *motor)
{
    
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];
    
    
    pbuf=(uint8_t*)&motor->ctrl.pos_set;
    vbuf=(uint8_t*)&motor->ctrl.vel_set;

    data[0] = *pbuf;
    data[1] = *(pbuf+1);
    data[2] = *(pbuf+2);
    data[3] = *(pbuf+3);

    data[4] = *vbuf;
    data[5] = *(vbuf+1);
    data[6] = *(vbuf+2);
    data[7] = *(vbuf+3);
    
    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}
void DMMotorControl()
{
    float pid_measure, pid_ref;

    DMMotorInstance *motor;
    motor_fbpara_t *measure;
    Motor_Control_Setting_s *setting;
    Motor_Controller_s *motor_controller;

    for (size_t i = 0; i < idm; ++i) {
        motor   = dmmotor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        pid_ref = motor_controller->pid_ref;

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP) {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->pos;
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;
                motor->ctrl.vel_set = pid_ref;
        }

        // 电机反转判断
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) {
            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else
                pid_measure = measure->vel; // MOTOR_FEED,对速度闭环,单位为angle per sec
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
                pid_ref += *motor_controller->current_feedforward_ptr;
        motor->ctrl.tor_set = pid_ref;
        }

       

       

         if (motor->stop_flag == MOTOR_STOP) { 
            motor->ctrl.pos_set = 0 ;
            motor->ctrl.vel_set = 0 ;
            motor->ctrl.tor_set = 0 ;
            motor->ctrl.kp_set = 0 ;
            motor->ctrl.kd_set = 0 ;
         }

        // 发送
        // memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
        // memcpy(motor->motor_can_instance->tx_buff, &motor_controller->output_current, sizeof(float));
        // motor->motor_can_instance->tx_buff[6] = 0x01;
       switch (motor->ctrl.mode) // 控制模式
       {
       case 0:
        mit_ctrl(motor);
        break;
         case 1:
        pos_ctrl(motor);
       default:
        break;
       }
        CANTransmit(motor->motor_can_instance, 0.1);
    }
}


void DMMotorEnableMode(DMMotorInstance *motor)//
{
		uint8_t data[8];
		
		
		data[0] = 0xFF;
		data[1] = 0xFF;
		data[2] = 0xFF;
		data[3] = 0xFF;
		data[4] = 0xFF;
		data[5] = 0xFF;
		data[6] = 0xFF;
		data[7] = 0xFC;
		memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
        memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}

void DMMotorClearError(DMMotorInstance *motor)
{
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}

void DMMotorDisableMode(DMMotorInstance *motor)
{
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}

void mit_ctrl(DMMotorInstance *motor)//
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	
		

	pos_tmp = float_to_uint(motor->ctrl.pos_set,  -motor->motor_settings.control_range.P_max,  motor->motor_settings.control_range.P_max,  16);
	vel_tmp = float_to_uint(motor->ctrl.vel_set,  -motor->motor_settings.control_range.V_max,  motor->motor_settings.control_range.V_max,  12);
	kp_tmp  = float_to_uint(motor->ctrl.kp_set,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(motor->ctrl.kd_set,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(motor->ctrl.tor_set,  -motor->motor_settings.control_range.T_max,  motor->motor_settings.control_range.T_max,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}

uint8_t DMMotorIsOnline(DMMotorInstance *motor)
{
    return DaemonIsOnline(motor->daemon);
}

#define ERROR_TH           1000    // 异常判定阈值
#define DISABLE_SHORT_TH   1000    // 短时失能重连阈值
#define DISABLE_LONG_TH    10000   // 长时失能重连阈值

// 异常检测
void DMMotorErrorDetection(DMMotorInstance *motor)
{
    switch(motor->measure.state)
    {
        case 1:
            motor->motor_error.disable_counter = 0;
            motor->motor_error.error_counter = 0;
            motor->motor_error.error_flag = 0;
            break;
        case 0:
            motor->motor_error.disable_counter++;
            motor->motor_error.error_counter = 0;
            break;
        default:
            motor->motor_error.error_counter++;
            motor->motor_error.disable_counter = 0;
            break;
    }


    if ((motor->motor_error.disable_counter > DISABLE_SHORT_TH && 
            motor->motor_error.error_flag == 0)
        || (motor->motor_error.disable_counter > DISABLE_LONG_TH && 
            motor->motor_error.error_flag == 1))
    {
        DMMotorEnableMode(motor);
        CANTransmit(motor->motor_can_instance, 1);
        motor->motor_error.disable_counter = 0;
    }

    if (motor->motor_error.error_counter > ERROR_TH)
    {
        motor->motor_error.error_flag = 1;

        DMMotorClearError(motor);
        CANTransmit(motor->motor_can_instance, 1);

        DMMotorDisableMode(motor);
        CANTransmit(motor->motor_can_instance, 1);

        motor->motor_error.error_counter = 0;
    }
 }   


#undef ERROR_TH        
#undef DISABLE_SHORT_TH
#undef DISABLE_LONG_TH 

//DM电机软重启
// void DMMotorReset(DMMotorInstance* motor){
//    DMMotorEnableMode(motor); // 使能电机模式,发送使能指令
//     CANTransmit_once(motor->motor_can_instance->can_handle,
//                          (motor->motor_can_instance->tx_id) + 0x0000,
//                          motor->motor_can_instance->tx_buff, 2);
// }

void DMMotorStop(DMMotorInstance *motor)
{
     motor->stop_flag = MOTOR_STOP;
}

void DMMotorEnable1(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENABLED;
}

void DMMotorSetTorque(DMMotorInstance *motor, float torque)
{
    motor->ctrl.tor_set = torque;
}
void DMMotorSetPos(DMMotorInstance *motor, float pos, float vel)
{
    motor->ctrl.pos_set = pos;
    motor->ctrl.vel_set = vel;
}
