#include "rs485.h"

// HostInstance *HostInit(HostInstanceConf *host_conf,void *callback)
// {
//     if(idx >= Host_Instance_MX_CNT){
//         while(1)
//             LOGERROR("[master_process]You cant register more instance for host.");
//     }
//     HostInstance *_instance = (HostInstance *)malloc(sizeof(HostInstance));
//     memset(_instance,0,sizeof(HostInstance));
//     _instance->comm_mode = host_conf->comm_mode;
//     _instance->RECV_SIZE = host_conf->RECV_SIZE;
//     host_conf->callback=callback;
//     switch(host_conf->comm_mode){
//         case HOST_USART:
//             USART_Init_Config_s usart_conf;
//             usart_conf.module_callback = host_conf->callback;
//             usart_conf.recv_buff_size = host_conf->RECV_SIZE;
//             usart_conf.usart_handle = host_conf->usart_handle;
//             _instance->comm_instance = USARTRegister(&usart_conf);
//             break;
//         case HOST_VCP:
//            USB_Init_Config_s usb_conf = {.rx_cbk = host_conf->callback};
//            _instance->comm_instance = USBInit(usb_conf);
//             break;
//         default:
//             while (1)
//                 LOGERROR("[master_process]You must select correct mode for HOST.");
//     }

//     // 为上位机实例注册守护进程
//     Daemon_Init_Config_s daemon_conf = {
//         .callback = HostOfflineCallback, // 离线时调用的回调函数,会重启实例
//         .owner_id = _instance,
//         .reload_count = 10,
//     };
//     _instance->daemon = DaemonRegister(&daemon_conf);

//     host_instance[idx++] = _instance;

//     return _instance;
// }
