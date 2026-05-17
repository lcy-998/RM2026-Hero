#include "daemon.h"
#include "crc_ref.h"
#include "image_road.h"

static USARTInstance *image_road_usart;
static DaemonInstance *image_road_daemon;
static image_road_info_t image_road_info;

static void ImageRoadRead(uint8_t *buff)
{
    uint16_t judge_length; // 统计一帧数据长度
    memcpy(&image_road_info.FrameHeader, buff, sizeof(xFrameHeader));
    if (buff[SOF] == REFEREE_SOF) {
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE) {
            // 统计一帧数据长度(byte),用于CR16校验
            judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE) {
                // 2个8位拼成16位int
                image_road_info.CmdID = (buff[6] << 8 | buff[5]);
                switch (image_road_info.CmdID) {
                    case ID_Custom_Recv_Info: // 0x0311
                        memcpy(&image_road_info.receive_packet, (buff + DATA_Offset), sizeof(image_road_info.receive_packet));
                        break;
                }
            }
        }
        if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + image_road_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5) { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
            ImageRoadRead(buff + sizeof(xFrameHeader) + LEN_CMDID + image_road_info.FrameHeader.DataLength + LEN_TAIL);
        }
    }
    else if (((uint16_t *)buff)[0] == IMAGE_ROAD_RC_SOF)
    {
        
    }
}

static void ImageRoadRxCallback(void)
{
    DaemonReload(image_road_daemon);
    ImageRoadRead(image_road_usart->recv_buff);
}

image_road_info_t *ImageRoadHardwareInit(UART_HandleTypeDef *usart_handle)
{
    USART_Init_Config_s usart_conf;
    usart_conf.module_callback   = ImageRoadRxCallback;
    usart_conf.usart_handle      = usart_handle;
    usart_conf.recv_buff_size    = 30; // 根据实际数据包大小调整
    image_road_usart = USARTRegister(&usart_conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback     = NULL, // 可以根据需要设置离线回调
        .owner_id     = image_road_usart,
        .reload_count = 100, // 0.1s没有收到数据,则认为丢失,重启串口接收
    };

    image_road_daemon = DaemonRegister(&daemon_conf);
    return &image_road_info;
}

void ImageRoadSend(void)
{
    USARTSend(image_road_usart, &image_road_info.send_packet, sizeof(image_road_info.send_packet), USART_TRANSFER_DMA);
}
