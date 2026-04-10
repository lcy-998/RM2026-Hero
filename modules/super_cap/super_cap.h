#ifndef SUPER_CUP_H
#define SUPER_CUP_H

#include "bsp_can.h"
#include "daemon.h"

#define SUPERCAP_MAX_VOLTAGE 24.0f
#define SUPERCAP_MIN_VOLTAGE 16.0f
#define SUPERCAP_HIGHER_THRESHOLD_VOLTAGE 22.0f
#define SUPERCAP_LOWER_THRESHOLD_VOLTAGE 16.0f

typedef enum
{
    SUPERCAP_DISABLE = 0,
    SUPERCAP_OFF = 1,
    SUPERCAP_ENABLE = 2,
}SuperCap_Enable_Flag_e;

typedef struct 
{
    SuperCap_Enable_Flag_e enable_flag;
    float power_limit;
    float voltage_limit;
    float current_limit;
}SuperCap_Tx_Data_s;

typedef union
{
    struct 
    {
        uint16_t rdy:   1;  /*!< bit0    就绪     */
        uint16_t run:   1;  /*!< bit1    运行     */
        uint16_t alm:   1;  /*!< bit2    报警     */
        uint16_t pwr:   1;  /*!< bit3    电源开关 */
        uint16_t load:  1;  /*!< bit4    负载开关 */
        uint16_t cc:    1;  /*!< bit5    恒流     */
        uint16_t cv:    1;  /*!< bit6    恒压     */
        uint16_t cw:    1;  /*!< bit7    恒功率   */
        uint16_t revd:  7;  /*!< bit8-14 保留     */
        uint16_t flt:   1;  /*!< bit15   故障     */
    };
    uint16_t all;
}SuperCap_Status_u;

typedef enum
{
    SUPERCAP_NO_ERROR                   = 0,
    SUPERCAP_ERR_BAT_LOW_VOLTAGE        = 1,
    SUPERCAP_ERR_BAT_OVER_VOLTAGE       = 2,
    SUPERCAP_ERR_BAT_OVER_CURRENT       = 3,
    SUPERCAP_ERR_BAT_OVER_POWER         = 4,
    SUPERCAP_ERR_OVER_TEMP              = 5,
    SUPERCAP_ERR_LOW_TEMP               = 6,
    SUPERCAP_ERR_OUT_OVER_VOLTAGE       = 7,
    SUPERCAP_ERR_OUT_OVER_CURRENT       = 8,
    SUPERCAP_ERR_OUT_OVER_POWER         = 9,
    SUPERCAP_ERR_ZERO_DRIFT             = 10,
    SUPERCAP_ERR_REVERSE_CONNECTION     = 11,
    SUPERCAP_ERR_OUT_OF_CONTROL         = 12,
    SUPERCAP_ERR_CONNECTION             = 13,
    SUPERCAP_ERR_EEPROM                 = 14,
} SuperCap_Error_Code_e;

typedef struct 
{
    uint16_t error_code;
    SuperCap_Status_u status;
    float input_power;
    float input_vol;
    float input_cur;
    float output_power;
    float output_vol;
    float output_cur;
    float temperature;
    uint16_t total_time;    //hours
    uint16_t run_time;      //minutes
}SuperCap_Rx_Data_s;

typedef struct 
{
    CAN_Init_Config_s can_config;
    Daemon_Init_Config_s daemon_config;
}SuperCap_Init_Config_s;

typedef struct
{
    CANInstance *can_instance;
    DaemonInstance *daemon_instance;
    SuperCap_Tx_Data_s tx_data;
    SuperCap_Rx_Data_s rx_data;
}SuperCapInstance;

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config);
void SuperCapRxCallback(CAN_RxHeaderTypeDef rx_config, uint8_t *recv_data);
void SuperCapTask(void);
void SuperCapEnable(SuperCapInstance *instance);
void SuperCapDisable(SuperCapInstance *instance);
void SuperCapSetPowerLimit(SuperCapInstance *instance, float power_limit);
void SuperCapSetVoltageLimit(SuperCapInstance *instance, float voltage_limit);
void SuperCapSetCurrentLimit(SuperCapInstance *instance, float current_limit);
uint8_t SuperCapIsOnline(SuperCapInstance *instance);
float SuperCapGetChassisVoltage(SuperCapInstance *instance);

#endif