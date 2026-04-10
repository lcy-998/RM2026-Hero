/**
 * @Author: HDC h2019dc@outlook.com
 * @Date: 2023-09-08 16:47:43
 * @LastEditors: HDC h2019dc@outlook.com
 * @LastEditTime: 2023-10-26 21:51:44
 * @FilePath: \2024_Control_New_Framework_Base-dev-all\application\robot.c
 * @Description:
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */
#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "buzzer.h"
#include "robot_test.h"
// #include "referee_UI.h"

#define ROBOT_DEF_PARAM_WARNING
// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#endif
// #include "omni_UI.h"
Vision_Recv_s *usb_test;
uint8_t power[52];
void RobotInit()
{
    
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();

    BSPInit();
    buzzer_one_note(Do_freq, 0.1f);
    RobotCMDInit();
    GimbalInit();
    ShootInit();
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    buzzer_one_note(Re_freq, 0.1f);
    buzzer_one_note(Mi_freq, 0.1f);
    C_board_LEDSet(0x00FF00);
    buzzer_one_note(Fa_freq, 0.1f);
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
    buzzer_one_note(So_freq, 0.1f);
    // UI_Init();
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
#endif

    // // 初始化完成,开启中断
     __enable_irq();
}
// #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart==&huart1)
//   {
//     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
//   }
// }
// #endif  
// #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart==&huart1)
//   {
//     HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
//   }
// }
// #endif  
void RobotTask()
{
 #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
    GimbalTask();
    ShootTask();
 #endif

 #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
 //   ChassisTask();
 #endif
    
}

// void Laser_Init()
// {
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // 使能GPIOC时钟
//     GPIOC->MODER &= ~(0x3 << (2 * 8));         // 清除PC8的模式位
//     GPIOC->MODER |= (0x1 << (2 * 8));          // 配置为输出模式 (01)
//     GPIOC->OTYPER &= ~(0x1 << 8);              // 配置为推挽输出（0）
//     GPIOC->OSPEEDR |= (0x3 << (2 * 8));        // 设置输出速度为高速
//     GPIOC->ODR &= (0x1 << 8);  // 设置PC8为高电平
   
// }
// void Laser_on()
// {
//     GPIOC->ODR |= (0x1 << 8);  // 设置PC8为高电平
// }
// void Laser_off()
// {
//     GPIOC->ODR &= (0x1 << 8);  // 设置PC8为高电平
// }