#include <memory.h>
#include <stdlib.h>
#include "wattmeter.h"

//功率计id 0x212

static WattmeterInstance *wattmeter = NULL;

static void WattmeterDecode(CANInstance *can_instance)
{
    uint8_t *rx_buffer = &can_instance->rx_buff;

    wattmeter->voltage = (float)((int16_t)(rx_buffer[1] << 8) | rx_buffer[0]) / 100.0;
    wattmeter->current = (float)((int16_t)(rx_buffer[3] << 8) | rx_buffer[2]) / 100.0;
    wattmeter->power = wattmeter->voltage * wattmeter->current;
    wattmeter->power_max = (wattmeter->power > wattmeter->power_max) ? wattmeter->power : wattmeter->power_max;
}

WattmeterInstance *WattmeterInit(Wattmeter_Init_Config_s *config)
{
    if (!wattmeter)
    {
        wattmeter = (WattmeterInstance *)malloc(sizeof(WattmeterInstance));
        memset(wattmeter, 0, sizeof(WattmeterInstance));
        
        config->can_config.can_module_callback = WattmeterDecode;
        config->can_config.id = wattmeter;
        wattmeter->can_instance = CANRegister(&config->can_config);
        return wattmeter;
    }
    
    return wattmeter;
}




