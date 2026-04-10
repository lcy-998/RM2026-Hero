#include "motor_task.h"
#include "LKmotor.h"
#include "HT04.h"
#include "dji_motor.h"
#include "dji_motor.h"
#include "step_motor.h"
#include "servo_motor.h"
#include "robot_def.h"
#include "DMmotor.h"

uint16_t g_cmd_set   = 2;
uint16_t* g_power_set=3000 ;
uint16_t g_vout_set  = 2300;
uint16_t g_iout_set  = 600;
void pm01_cmd_send( uint16_t new_cmd, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x600;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_cmd >> 8   );
	power_can_send_data[1] = (uint8_t)(new_cmd &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = 0x00;;//(save_flg == 0x01);			
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
void pm01_voltage_set( uint16_t new_voltage, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];


	power_tx_message.StdId = 0x602;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	power_can_send_data[0] = (uint8_t)(new_voltage >> 8   );
	power_can_send_data[1] = (uint8_t)(new_voltage &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = 0x00;;//(save_flg == 0x01);			

    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          设置输出电压
  * @param[in]      new_volt：新的电压值
                    save_flg: 0x00: 不保存至EEPROM  0x01: 保存至EEPROM
  * @retval         none
  */
 void pm01_current_set( uint16_t new_current, uint8_t save_flg )
 {
     uint32_t send_mail_box;
 
     CAN_TxHeaderTypeDef  power_tx_message;
     uint8_t              power_can_send_data[8];
 
     power_tx_message.StdId = 0x603;
     power_tx_message.IDE   = CAN_ID_STD;
     power_tx_message.RTR   = CAN_RTR_DATA;
     power_tx_message.DLC   = 0x04;
     power_can_send_data[0] = (uint8_t)(new_current >> 8   );
     power_can_send_data[1] = (uint8_t)(new_current &  0xFF);
     power_can_send_data[2] = 0x00;
     power_can_send_data[3] = 0x00;;//(save_flg == 0x01);			
     while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
     HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
     
 }
 /**
  * @brief          设置功率
  * @param[in]      new_power：新的功率值
                    save_flg: 0x00: 不保存至EEPROM  0x01: 保存至EEPROM
  * @retval         none
  */
void pm01_power_set( uint16_t new_power, uint8_t save_flg )
{
	uint32_t send_mail_box;

	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];

	power_tx_message.StdId = 0x601;
	power_tx_message.IDE   = CAN_ID_STD;
	power_tx_message.RTR   = CAN_RTR_DATA;
	power_tx_message.DLC   = 0x04;
	new_power*=100;
	power_can_send_data[0] = (uint8_t)(new_power >> 8   );
	power_can_send_data[1] = (uint8_t)(new_power &  0xFF);
	power_can_send_data[2] = 0x00;
	power_can_send_data[3] = 0x00;;//(save_flg == 0x01);			
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
	HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
	
}

void getSuper_cap()
{
    CAN_TxHeaderTypeDef  power_tx_message;
    uint8_t              power_can_send_data[8];

	uint32_t             send_mail_box;
    static uint8_t count = 0;
    static uint16_t  m_can_id  = 0x600;  /* 标识符  */
    switch (count)
    {
        case 0:
					case 1:
					case 2:
					case 3:
					case 4:
					case 5:
					case 6:
					case 7:
                    power_tx_message.StdId = m_can_id;       /* 发送相应的标识符 */
                    power_tx_message.IDE   = CAN_ID_STD;     /* 标准帧           */
                    power_tx_message.RTR   = CAN_RTR_REMOTE; /* 远程帧           */
                    power_tx_message.DLC   = 0x00;
                    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
                    HAL_CAN_AddTxMessage(&hcan2, &power_tx_message, power_can_send_data, &send_mail_box);
              break;
              case 8:			

            pm01_cmd_send( g_cmd_set, 0x00 );
            break;
					case 9:
					// g_power_set=30;
						pm01_power_set( g_power_set, 0x00 );
					  break;
					case 10:
					
										
					
						  pm01_voltage_set( g_vout_set, 0x00 );
					  break;
					case 11:
					
							
					
						  pm01_current_set( g_iout_set, 0x00 );
					  break;
    }
    
    count=(count+1) % 12;
    switch( count )
				{
					case 0: m_can_id = 0x600; break;
					case 1: m_can_id = 0x601; break;
					case 2: m_can_id = 0x602; break;
					case 3: m_can_id = 0x603; break;
					case 4: m_can_id = 0x610; break;
					case 5: m_can_id = 0x611; break;
					case 6: m_can_id = 0x612; break;
					case 7: m_can_id = 0x613; break;
				    case 8: m_can_id = 0x600; break; /* 写 */
					case 9: m_can_id = 0x601; break; /* 写 */
					case 10: m_can_id = 0x602; break; /* 写 */
					case 11: m_can_id = 0x603; break; /* 写 */

				}
}
void MotorControlTask()
{
	#ifdef CHASSIS_BOARD 
    static uint8_t cnt = 0;// 设定不同电机的任务频率
    // if(cnt%10==0) //100hz
    // {
	// 	getSuper_cap();
	// }
	// cnt=(cnt+1)%10;
	#endif
    DJIMotorControl();
    DMMotorControl();
    /* 如果有对应的电机则取消注释,可以加入条件编译或者register对应的idx判断是否注册了电机 */
    // DRMotorControl();
    // LKMotorControl();

    // legacy support
    // 由于ht04电机的反馈方式为接收到一帧消息后立刻回传,以此方式连续发送可能导致总线拥塞
    // 为了保证高频率控制,HTMotor中提供了以任务方式启动控制的接口,可通过宏定义切换
    // HTMotorControl();
    // 将所有的CAN设备集中在一处发送,最高反馈频率仅能达到500Hz,为了更好的控制效果,应使用新的HTMotorControlInit()接口

    //ServeoMotorControl();

    // StepMotorControl();
}
