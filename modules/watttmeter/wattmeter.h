#ifndef WATTMETER_H
#define WATTMETER_H

#include "bsp_can.h"

typedef struct 
{
    float voltage;
    float current;
    float power;
    float power_max;

    CANInstance *can_instance;
}WattmeterInstance;

typedef struct 
{
    CAN_Init_Config_s can_config;
}Wattmeter_Init_Config_s;

WattmeterInstance *WattmeterInit(Wattmeter_Init_Config_s *config);


#endif