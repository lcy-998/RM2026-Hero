#ifndef __IMAGE_ROAD_H
#define __IMAGE_ROAD_H

#include "bsp_usart.h"
#include "referee_protocol.h"

#define IMAGE_ROAD_RC_SOF 0XA953

#pragma pack(1) // 按1字节对齐

typedef union
{

    uint8_t data[30];
} Custom_Receive_Packet_u;

typedef struct
{

    uint8_t data[300];
} Custom_Send_Packet_u;

typedef struct
{
    uint16_t header; // 数据包头
    uint16_t channel0;//r-
    uint16_t channel1;//r|
    uint16_t channel2;//l|
    uint16_t channel3;//l-
    uint8_t switch_button;//C:0 N:1 S:2
    uint8_t stop_button;//0:未按下 1:按下
    uint8_t left_button;
    uint8_t right_button;
    uint16_t dial;
} Image_Road_Cmd_t;

typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    Custom_Receive_Packet_u receive_packet;
    Custom_Send_Packet_u send_packet;
} image_road_info_t;

image_road_info_t *ImageRoadHardwareInit(UART_HandleTypeDef *usart_handle);

#pragma pack()

#endif
