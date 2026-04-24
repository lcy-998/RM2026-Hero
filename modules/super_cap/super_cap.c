#include <memory.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "super_cap.h"

static SuperCapInstance *supercap = NULL;

void SuperCapEnable(SuperCapInstance *instance)
{
    instance->tx_data.enable_flag = SUPERCAP_ENABLE;
}

void SuperCapDisable(SuperCapInstance *instance)
{
    instance->tx_data.enable_flag = SUPERCAP_DISABLE;
}

void SuperCapSetPowerLimit(SuperCapInstance *instance, uint8_t power_limit)
{
    instance->tx_data.power_limit = power_limit;
}

static void SuperCapRxCallback(CANInstance *instance)
{
    DaemonReload(supercap->daemon_instance);
    memcpy(&supercap->rx_data, instance->rx_buff, sizeof(SuperCap_Rx_Data_s));
    return;
}

static void SuperCapLostCallback(void *instance)
{
    memset(&supercap->rx_data, 0, sizeof(SuperCap_Rx_Data_s));
    return;
}

uint8_t SuperCapIsOnline(SuperCapInstance *instance)
{
    return DaemonIsOnline(instance->daemon_instance);
}

SuperCapInstance *SuperCapRegister(SuperCap_Init_Config_s *config)
{
    if (!supercap)
    {
        supercap = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
        memset(supercap, 0, sizeof(SuperCapInstance));

        config->can_config.id = supercap;
        config->can_config.can_module_callback = SuperCapRxCallback;
        supercap->can_instance = CANRegister(&config->can_config);

        config->daemon_config.callback = SuperCapLostCallback;
        config->daemon_config.init_count = 200;
        config->daemon_config.owner_id = (void *)supercap;
        config->daemon_config.reload_count = 100;
        supercap->daemon_instance = DaemonRegister(&config->daemon_config);
    }
    return supercap;
}

void SuperCapTask(void)
{
    static uint8_t counter = 0;
    if (counter % 8 == 0) // 200ms周期发送一次控制命令,上位机控制频率较低
    {
         memcpy(supercap->can_instance->tx_buff, &supercap->tx_data, sizeof(SuperCap_Tx_Data_s));
         CANTransmit(supercap->can_instance, 1);
    }
    counter++;
}

float SuperCapGetChassisRealPower(SuperCapInstance *instance)
{
    return (float)(instance->rx_data.chassis_real_power << 1); // 接收的时候右移了一位,所以要左移还原
}

uint8_t SuperCapGetCapEnergy(SuperCapInstance *instance)
{
    return instance->rx_data.energy;
}

uint8_t SuperCapGetReadyFlag(SuperCapInstance *instance)
{
    return instance->rx_data.ready_flag;
}

