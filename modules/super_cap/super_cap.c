#include <memory.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "super_cap.h"

enum SuperCapStates{STATE_CMD_SET = 0x00, STATE_POWER_SET, STATE_VOLTAGE_SET, STATE_CURRENT_SET,
                    STATE_STATUS_READ, STATE_INPUT_READ, STATE_OUTPUT_READ, STATE_OTHER_READ,
                    STATE_CMD_READ} state;
static SuperCapInstance *supercap = NULL;
const CAN_TxHeaderTypeDef tx_header_table[] = {
    [0] = {.StdId = 0x600 , .DLC = 0x04 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA},//cmd_set
    [1] = {.StdId = 0x601 , .DLC = 0x04 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA},//power_set
    [2] = {.StdId = 0x602 , .DLC = 0x04 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA},//voltage_set
    [3] = {.StdId = 0x603 , .DLC = 0x04 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA},//current_set

    [4] = {.StdId = 0x610 , .DLC = 0x00 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_REMOTE},//status_read
    [5] = {.StdId = 0x611 , .DLC = 0x00 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_REMOTE},//input_read
    [6] = {.StdId = 0x612 , .DLC = 0x00 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_REMOTE},//output_read
    [7] = {.StdId = 0x613 , .DLC = 0x00 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_REMOTE},//temperature & time read

    // [8] = {.StdId = 0x600 , .DLC = 0x00 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_REMOTE},//cmd_read
    // [5] = {.StdId = 0x601 , .DLC = 0x00 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_REMOTE},//power_read
    // [6] = {.StdId = 0x602 , .DLC = 0x00 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_REMOTE},//voltage_read
    // [7] = {.StdId = 0x603 , .DLC = 0x00 , .IDE = CAN_ID_STD, .RTR = CAN_RTR_REMOTE},//current_read
};
static uint8_t can_tx_data[8];

void SuperCapEnable(SuperCapInstance *instance)
{
    instance->tx_data.enable_flag = 2;
}

void SuperCapDisable(SuperCapInstance *instance)
{
    instance->tx_data.enable_flag = 0;
}

void SuperCapSetPowerLimit(SuperCapInstance *instance, float power_limit)
{
    instance->tx_data.power_limit = power_limit;
}

void SuperCapSetVoltageLimit(SuperCapInstance *instance, float voltage_limit)
{
    instance->tx_data.voltage_limit = voltage_limit;
}

void SuperCapSetCurrentLimit(SuperCapInstance *instance, float current_limit)
{
    instance->tx_data.current_limit = current_limit;
}

void SuperCapRxCallback(CAN_RxHeaderTypeDef rx_config, uint8_t *recv_data)
{
    memcpy(supercap->can_instance->rx_buff, recv_data, rx_config.DLC);
    DaemonReload(supercap->daemon_instance);
    uint16_t temp;
    uint8_t *rx_buffer = supercap->can_instance->rx_buff;
    switch (rx_config.StdId)
    {
        case 0x610:
            temp = (uint16_t)rx_buffer[0] << 8 | rx_buffer[1];
            supercap->rx_data.status.all = temp;
            temp = (uint16_t)rx_buffer[2] << 8 | rx_buffer[3];
            supercap->rx_data.error_code = temp;
            break;

        case 0x611:
            temp = (uint16_t)rx_buffer[0] << 8 | rx_buffer[1];
            supercap->rx_data.input_power = (float)temp / 100.0f;
            temp = (uint16_t)rx_buffer[2] << 8 | rx_buffer[3];
            supercap->rx_data.input_vol = (float)temp / 100.0f;
            temp = (uint16_t)rx_buffer[4] << 8 | rx_buffer[5];
            supercap->rx_data.input_cur = (float)temp / 100.0f;
            break;

        case 0x612:
            temp = (uint16_t)rx_buffer[0] << 8 | rx_buffer[1];
            supercap->rx_data.output_power = (float)temp / 100.0f;
            if (supercap->rx_data.output_power > 200.0f) supercap->rx_data.output_power = 0.0f;
            temp = (uint16_t)rx_buffer[2] << 8 | rx_buffer[3];
            supercap->rx_data.output_vol = (float)temp / 100.0f;
            temp = (uint16_t)rx_buffer[4] << 8 | rx_buffer[5];
            supercap->rx_data.output_cur = (float)temp / 100.0f;
            break;

        case 0x613:
            temp = (uint16_t)rx_buffer[0] << 8 | rx_buffer[1];
            supercap->rx_data.temperature = (float)temp / 10.0f;
            temp = (uint16_t)rx_buffer[2] << 8 | rx_buffer[3];
            supercap->rx_data.total_time = temp;
            temp = (uint16_t)rx_buffer[4] << 8 | rx_buffer[5];
            supercap->rx_data.run_time = temp;
            break;
    }
}

static void SuperCapLostCallback(DaemonInstance *daemon_instance)
{

}

uint8_t SuperCapIsOnline(SuperCapInstance *instance)
{
    return DaemonIsOnline(instance->daemon_instance);
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config)
{
    if(!supercap)
    {
        supercap = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
        memset(supercap, 0, sizeof(SuperCapInstance));

        config->can_config.can_module_callback = NULL;
        config->can_config.id = (void *)supercap;
        supercap->can_instance = CANRegister(&config->can_config);

        config->daemon_config.callback = SuperCapLostCallback;
        config->daemon_config.init_count = 2000;
        config->daemon_config.owner_id = (void *)supercap;
        config->daemon_config.reload_count = 1000;
        supercap->daemon_instance = DaemonRegister(&config->daemon_config);

        CAN_FilterTypeDef filter_config = {
            .FilterMode = CAN_FILTERMODE_IDMASK,
            .FilterBank = 13,
            .FilterScale = CAN_FILTERSCALE_16BIT,
            .FilterFIFOAssignment = CAN_FILTER_FIFO0,
            .FilterActivation = CAN_FILTER_ENABLE,
            .SlaveStartFilterBank = 14,
            .FilterMaskIdHigh = 0x7EC << 5,
            .FilterMaskIdLow = 0x000,
            .FilterIdHigh = 0x600 << 5,
            .FilterIdLow = 0x000,
        };
        HAL_CAN_ConfigFilter(supercap->can_instance->can_handle, &filter_config);
        
        supercap->tx_data.power_limit = 100.0f;
        supercap->tx_data.voltage_limit = 24.0f;
        state = STATE_CMD_SET;
    }
    return supercap;
}

void SuperCapTask(void)
{
    uint16_t tx_buffer;
    switch (state)
    {
        case STATE_CMD_SET:
            can_tx_data[0] = 0x00;
            can_tx_data[1] = (uint8_t)supercap->tx_data.enable_flag;
            break;
        case STATE_POWER_SET:
            tx_buffer = (uint16_t)(supercap->tx_data.power_limit * 100.0f);
            can_tx_data[0] = (uint8_t)(tx_buffer >> 8);
            can_tx_data[1] = (uint8_t)(tx_buffer & 0xFF);
            break;
        case STATE_VOLTAGE_SET:
            tx_buffer = (uint16_t)(supercap->tx_data.voltage_limit * 100.0f);
            can_tx_data[0] = (uint8_t)(tx_buffer >> 8);
            can_tx_data[1] = (uint8_t)(tx_buffer & 0xFF);
            break;
        case STATE_CURRENT_SET:
            // tx_buffer = (uint16_t)(supercap->tx_data.current_limit * 100.0f);
            // can_tx_data[0] = (uint8_t)(tx_buffer >> 8);
            // can_tx_data[1] = (uint8_t)(tx_buffer & 0xFF);
            break;
        case STATE_STATUS_READ:
        case STATE_INPUT_READ:
        case STATE_OUTPUT_READ:
        case STATE_OTHER_READ:
            memset(can_tx_data, 0, sizeof(can_tx_data));
            break;
    }
    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(supercap->can_instance->can_handle, &tx_header_table[(size_t)state], can_tx_data, &tx_mailbox);
    
    state = (state + 1) % (sizeof(tx_header_table) / sizeof(CAN_TxHeaderTypeDef));
    if (state == STATE_CURRENT_SET) state = STATE_STATUS_READ;//跳过电流设置
}

float SuperCapGetChassisVoltage(SuperCapInstance *instance)
{
    return instance->rx_data.output_vol;
}
