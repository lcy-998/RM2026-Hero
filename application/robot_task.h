/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "referee_init.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "user_lib.h"

#include "bsp_log.h"
#include "led.h"
#include "buzzer.h"
#include "ins_task.h"
#include "referee_UI.h"

#include "robot_cmd.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "motor_task.h"
#include "omni_UI.h"

//! 任务直接在cubeMX中配置,不再使用这种方式
__attribute__((noreturn)) void StartINSTASK(void *argument)
{
    UNUSED(argument);
    static uint32_t ins_time;
    static float ins_dt;
    LOGINFO("[freeRTOS] INS Task Start");
    while (1) {
        INS_Task();
        ins_dt = 1000 * DWT_GetDeltaT(&ins_time);
        if (ins_dt > 1.2f)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]ms", &ins_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void _RobotCMDTask(void *argument)
{
    static uint32_t cmd_time;
    static float cmd_dt;
    LOGINFO("[freeRTOS] RobotCMD Task Start");
    for (;;) {
        RobotCMDTask();
        cmd_dt = 1000 * DWT_GetDeltaT(&cmd_time);
        if (cmd_dt > 1.2f)
            LOGERROR("[freeRTOS] Robot Task is being DELAY! dt = [%f]ms", &cmd_dt);
        osDelay(1);
    }
}
__attribute__((noreturn)) void _ChassisTask(void *argument)
{
    for (;;) {
        ChassisTask();
        osDelay(1);
    }
}
float Gimbal_dt,Shoot_dt,motorControl_dt,Daemon_dt;
__attribute__((noreturn)) void _GimbalTask(void *argument)
{
    static uint32_t Gimbal_time;
    
    for (;;) {
        GimbalTask();
        Gimbal_dt = DWT_GetDeltaT(&Gimbal_time);
        osDelay(1);
    }
}
__attribute__((noreturn)) void _ShootTask(void *argument)
{
    static uint32_t Shoot_time;
    for (;;) {
        ShootTask();
        Shoot_dt = DWT_GetDeltaT(&Shoot_time);
        osDelay(1);
    }
}
__attribute__((noreturn)) void motorControlTask(void *argument)
{
    static uint32_t motorControl_time;
    for (;;) {
        MotorControlTask();
        motorControl_dt = DWT_GetDeltaT(&motorControl_time);
        osDelay(1);
    }
}
extern  UI_StaticInit();
__attribute__((noreturn)) void _UITask(void *argument)
{
    osDelay(10000); // 等待系统稳定后再初始化UI，避免资源竞争导致的异常
    UI_Init();
    for (;;) {
        
        UIDynamicRefresh();
        osDelay(5);
        MessageCommunication();
        osDelay(5);
    }
}

__attribute__((noreturn)) void _DaemonTask(void *argument)
{
    static uint32_t Daemon_time;

    for (;;) {
        DaemonTask();
        Daemon_dt = DWT_GetDeltaT(&Daemon_time);
        osDelay(1);
    }
}
