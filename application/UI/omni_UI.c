// app
#include "robot_def.h"
#include "omni_UI.h"
// module
#include "rm_referee.h"
#include "referee_protocol.h"
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "message_center.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "UI_interface.h"

#define Super_Cap_Charge_Voltage_MAX  23.0f // 电容最大电压

static Publisher_t *ui_pub;
static Subscriber_t *ui_sub;
static UI_Cmd_s ui_cmd_recv;
static UI_Upload_Data_s ui_feedback_data;

float cap_debug;
float heat_debug;
extern float diff2;
referee_info_t *referee_data_for_ui;

uint8_t UI_rune; // 自瞄打符标志位


uint8_t UI_Seq;                      // 包序号，供整个referee文件使用
static Graph_Data_t shoot_line[7];   // 射击准线
static Graph_Data_t crosshairs[4];   // 射击准心
static Graph_Data_t benchmark[5];   // 基准线
static Graph_Data_t state_circle[4]; // 圆形
static String_Data_t Char_State[6];  // 字符串
static Graph_Data_t Cap_voltage_arc;  
static Graph_Data_t Deviation_arc;
static Graph_Data_t Chassis_Power_arc; 
static Graph_Data_t Shoot_Heat_arc;
static Graph_Data_t Cap_voltage;      // 电容电压
static Graph_Data_t total_voltage;      // 电容电压
static Graph_Data_t Shoot_Local_Heat; // 射击本地热量
int linex,liney,line,linex1,liney1,width,size;
//查表计算总允许发射热量
static float getTotal_heat(uint8_t level){
    switch(level){
        case 1:
        return 100;break;
        case 2:
        return 140;break;
        case 3:
        return 180;break;
        case 4:
        return 220;break;
        case 5:
        return 260;break;
        case 6:
        return 300;break;
        case 7:
        return 340;break;
        case 8:
        return 380;break;
        case 9:
        return 420;break;
        case 10:
        return 500;break;
    }
}
//获得剩余热量
static float allow_shoot_heat(float total,uint16_t temp){
    float res;
    res=total-(float)temp;
    // res*=1000;
    return res;
}

static void UI_StaticInit()
{
    // 清空UI
    UIDelete(&referee_data_for_ui->referee_id, UI_Data_Del_ALL, 0);



   // 射击准心
    //UILineDraw(&crosshairs[0], "crosshairs0", UI_Graph_ADD, 9, UI_Color_Main, 6, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 25, SCREEN_LENGTH / 2 + 25, SCREEN_WIDTH / 2 + 25);
    // UILineDraw(&shoot_line[0], "ol0", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 + 5, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 44);
    // UILineDraw(&shoot_line[1], "ol1", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 75, SCREEN_WIDTH / 2 - 50, SCREEN_LENGTH / 2 - 30, SCREEN_WIDTH / 2 - 50);
    // UILineDraw(&shoot_line[2], "ol2", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 56, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 105);
    // UILineDraw(&shoot_line[3], "ol3", UI_Graph_ADD, 9, UI_Color_Yellow, 1, SCREEN_LENGTH / 2 - 20, SCREEN_WIDTH / 2 - 50, SCREEN_LENGTH / 2 + 25, SCREEN_WIDTH / 2 - 50);

    // UILineDraw(&shoot_line[4], "ol4", UI_Graph_ADD, 9, UI_Color_Green, 1, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 49, SCREEN_LENGTH / 2 - 25, SCREEN_WIDTH / 2 - 51);

    // UILineDraw(&shoot_line[5], "ol5", UI_Graph_ADD, 9, UI_Color_White, 2, 540, 150, 700, 300);
    // UILineDraw(&shoot_line[6], "ol6", UI_Graph_ADD, 9, UI_Color_White, 2, 1380, 150, 1220, 300);
    //基准线
    UILineDraw(&benchmark[0], "benchmark0", UI_Graph_ADD, 8, UI_Color_White, 10, 575, 530, 575, 550);
    UILineDraw(&benchmark[1], "benchmark1", UI_Graph_ADD, 9, UI_Color_White, 10,1345, 530, 1345, 550); 
    UIArcDraw(&benchmark[2], "benchmark2", UI_Graph_ADD, 7, UI_Color_White, 313, 316, 10, 956, 542, 383, 386);
    UIArcDraw(&benchmark[3], "benchmark3", UI_Graph_ADD, 6, UI_Color_White, 223, 226, 10, 955, 541, 383, 386);
    UIArcDraw(&benchmark[4], "benchmark4", UI_Graph_ADD, 5, UI_Color_White, 44, 47, 10, 961, 541, 383, 386);
    // 小陀螺
    sprintf(Char_State[0].show_Data, "Rotate");
    UICharDraw(&Char_State[0], "sc0", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 580, 125, "Rotate");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[0]);

    UICircleDraw(&state_circle[0], "oc0", UI_Graph_ADD, 9, UI_Color_White, 10, 620, 160, 10);
    // 卡弹
    sprintf(Char_State[1].show_Data, "Stuck");
    UICharDraw(&Char_State[1], "sc1", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 910, 210, "Stuck");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[1]);
    UICircleDraw(&state_circle[1], "oc1", UI_Graph_ADD, 9, UI_Color_White, 10, 960, 255, 10);
    // 摩擦轮
    sprintf(Char_State[2].show_Data, "Friction");
    UICharDraw(&Char_State[2], "sc2", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 880, 85, "Friction");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[2]);

    UICircleDraw(&state_circle[2], "oc2", UI_Graph_ADD, 9, UI_Color_White, 10, 960, 120, 10);
    // 电容
    sprintf(Char_State[3].show_Data, "Cap");
    UICharDraw(&Char_State[3], "sc3", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 1270, 125, "Cap");
    UICharRefresh(&referee_data_for_ui->referee_id, Char_State[3]);
    UICircleDraw(&state_circle[3], "oc3", UI_Graph_ADD, 9, UI_Color_White, 10, 1300, 160, 10);

    UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_ADD, 9, UI_Color_Green, 271, 273, 7, 956, 542, 383, 386);

    UIFloatDraw(&total_voltage, "of0", UI_Graph_ADD, 9, UI_Color_Green, 15, 3, 3, 740, 910, 24000);
    UIFloatDraw(&Cap_voltage, "of5", UI_Graph_ADD, 9, UI_Color_Pink, 15, 3, 3, 555, 550, (int32_t)(ui_cmd_recv.supercap_voltage * 1000));

    //底盘功率
    // UIArcDraw(&Chassis_Power_arc, "poutline", UI_Graph_ADD, 9, UI_Color_Green, 267, 269, 7, 956, 542, 383, 386);

    //热量
    UIArcDraw(&Shoot_Heat_arc, "heatline", UI_Graph_ADD, 9, UI_Color_Green, 87, 89, 7, 956, 542, 383, 386);
    //UIFloatDraw(&Shoot_Local_Heat, "of1", UI_Graph_ADD, 4, UI_Color_Green, 15, 3, 3, 1260, 540, (int32_t)(ui_cmd_recv.Shooter_heat * 1000));
    //底盘实时功率
    UIArcDraw(&Deviation_arc, "deviationline", UI_Graph_ADD, 4, UI_Color_Pink, 267, 269, 7, 956, 542, 390, 393);
    // 发送
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, shoot_line[0], shoot_line[1], shoot_line[2], shoot_line[3], shoot_line[4], shoot_line[5], shoot_line[6]);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, state_circle[0], state_circle[1], state_circle[2], state_circle[3], Cap_voltage, Shoot_Local_Heat,total_voltage);
    UIGraphRefresh(&referee_data_for_ui->referee_id,7,Cap_voltage_arc,Chassis_Power_arc,benchmark[0], benchmark[1],benchmark[2],benchmark[3],benchmark[4]);
    UIGraphRefresh(&referee_data_for_ui->referee_id,2,Deviation_arc, Shoot_Heat_arc);  

}


void UI_Init()
{
    ui_pub = PubRegister("ui_feed", sizeof(UI_Upload_Data_s));
    ui_sub = SubRegister("ui_cmd", sizeof(UI_Cmd_s));

    UI_StaticInit();
}

void UIDynamicRefresh()
{
    static uint8_t refresh_cnt = 0;
    refresh_cnt++;
    SubGetMessage(ui_sub, (void *)&ui_cmd_recv);
    if (ui_cmd_recv.ui_refresh_flag == 1) {
        UI_StaticInit();
    }
    // 小陀螺
    if (ui_cmd_recv.chassis_mode == CHASSIS_ROTATE) {
        UICircleDraw(&state_circle[0], "oc0", UI_Graph_Change, 9, UI_Color_Main, 10, 620, 160, 10);
    } else {
        UICircleDraw(&state_circle[0], "oc0", UI_Graph_Change, 9, UI_Color_White, 10, 620, 160, 10);
    }
    //卡弹
    // if (ui_cmd_recv.bullet_stuck == 1)
    // {
    //     UICircleDraw(&state_circle[1], "oc1", UI_Graph_ADD, 9, UI_Color_Pink, 10, 960, 255, 10);
    // }
    // else
    // {
    //     UICircleDraw(&state_circle[1], "oc1", UI_Graph_ADD, 9, UI_Color_White, 10, 960, 255, 10);
    // }
    //摩擦轮
    if (ui_cmd_recv.friction_mode == FRICTION_ON) {
        UICircleDraw(&state_circle[2], "oc2", UI_Graph_Change, 9, UI_Color_Main, 10, 960, 120, 10);
    } else {
        UICircleDraw(&state_circle[2], "oc2", UI_Graph_Change, 9, UI_Color_White, 10, 960, 120, 10);
    }
    // 电容&电容电压
    if (ui_cmd_recv.supercap_mode == SUPERCAP_UNUSE)
    {
        if (ui_cmd_recv.cap_online_flag == 0)
        {
            UICircleDraw(&state_circle[3], "oc3", UI_Graph_Change, 9, UI_Color_Pink, 10, 1300, 160, 10);
        }
        else
        {
            UICircleDraw(&state_circle[3], "oc3", UI_Graph_Change, 9, UI_Color_Green, 10, 1300, 160, 10);
        }
    }
    else
    {
        if (ui_cmd_recv.cap_online_flag == 0)
        {
            UICircleDraw(&state_circle[3], "oc3", UI_Graph_Change, 9, UI_Color_Pink, 10, 1300, 160, 10);
        }
        else
        {
            if (refresh_cnt % 10 < 5)
                UICircleDraw(&state_circle[3], "oc3", UI_Graph_Change, 9, UI_Color_Purplish_red, 10, 1300, 160, 10);
            else
                UICircleDraw(&state_circle[3], "oc3", UI_Graph_Change, 9, UI_Color_Green, 10, 1300, 160, 10);
        }
    }
    
    

    // if(ui_cmd_recv.supercap_voltage >= SUPERCAP_HIGHER_THRESHOLD_VOLTAGE){
    //     UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_Change, 9, UI_Color_Green, 271, 272 + 60 * (ui_cmd_recv.supercap_voltage - SUPERCAP_MIN_VOLTAGE) / (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE), 7, 956, 542, 383, 386);
    // }
    // else if (ui_cmd_recv.supercap_voltage >= SUPERCAP_LOWER_THRESHOLD_VOLTAGE){
    //     UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_Change, 9, UI_Color_Orange, 271, 272 + 60 * (ui_cmd_recv.supercap_voltage - SUPERCAP_MIN_VOLTAGE) / (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE), 7, 956, 542, 383, 386);
    // }
    // else if (ui_cmd_recv.supercap_voltage > SUPERCAP_LOWER_THRESHOLD_VOLTAGE)
    // {
    //     UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_Change, 9, UI_Color_Pink, 271, 272 + 60 * (ui_cmd_recv.supercap_voltage - SUPERCAP_MIN_VOLTAGE) / (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE), 7, 956, 542, 383, 386);
    // }
    // else
    // {
    //     UIArcDraw(&Cap_voltage_arc, "powerline", UI_Graph_Change, 9, UI_Color_Pink, 271, 272, 7, 956, 542, 383, 386);
    // }
    
    //底盘功率
    // if (ui_cmd_recv.Chassis_Ctrl_power < 1.0f) ui_cmd_recv.Chassis_Ctrl_power = 1.0f;
    // if (ui_cmd_recv.Chassis_Ctrl_power > 150.0f)
    // {
    //     UIArcDraw(&Chassis_Power_arc, "poutline", UI_Graph_Change, 9, UI_Color_Purplish_red, 268 - 60.0 * ui_cmd_recv.Chassis_Ctrl_power / 210.0f, 269, 7, 956, 542, 383, 386);
    // }
    // else if (ui_cmd_recv.Chassis_Ctrl_power > 100.0f)
    // {
    //     UIArcDraw(&Chassis_Power_arc, "poutline", UI_Graph_Change, 9, UI_Color_Green, 268 - 60.0 * ui_cmd_recv.Chassis_Ctrl_power / 210.0f, 269, 7, 956, 542, 383, 386);
    // }
    // else if (ui_cmd_recv.Chassis_Ctrl_power > 70.0f)
    // {
    //     UIArcDraw(&Chassis_Power_arc, "poutline", UI_Graph_Change, 9, UI_Color_Orange, 268 - 60.0 * ui_cmd_recv.Chassis_Ctrl_power / 210.0f, 269, 7, 956, 542, 383, 386);
    // }
    // else
    // {
    //     UIArcDraw(&Chassis_Power_arc, "poutline", UI_Graph_Change, 9, UI_Color_Cyan, 268 - 60.0 * ui_cmd_recv.Chassis_Ctrl_power / 210.0f, 269, 7, 956, 542, 383, 386);
    // }
    // UIFloatDraw(&Cap_voltage, "of5", UI_Graph_Change, 9, UI_Color_Pink, 15, 3, 3, 555, 550, (int32_t)(ui_cmd_recv.supercap_voltage));
    
    //热量
    if (ui_cmd_recv.shooter_referee_heat > 100.0f)
    {
        UIArcDraw(&Shoot_Heat_arc, "heatline", UI_Graph_Change, 9, UI_Color_Purplish_red,88 - 60 * ui_cmd_recv.shooter_referee_heat / 200.0f, 89, 7, 956, 542, 383, 386);
    }
    else
    {
        UIArcDraw(&Shoot_Heat_arc, "heatline", UI_Graph_Change, 9, UI_Color_Green,88 - 60 * ui_cmd_recv.shooter_referee_heat / 200.0f, 89, 7, 956, 542, 383, 386);
    }
    //UIFloatDraw(&Shoot_Local_Heat, "of1", UI_Graph_Change,  4, UI_Color_Green, 15, 3, 3, 1260, 540, (int32_t)(ui_cmd_recv.shooter_referee_heat * 1000));
    //底盘实时功率
    if (ui_cmd_recv.chassis_real_power > 120.0f) 
    {
        UIArcDraw(&Deviation_arc, "deviationline", UI_Graph_Change, 4, UI_Color_Purplish_red, 267 - 60.0 * ui_cmd_recv.chassis_real_power / 200.0f , 269, 7, 956, 542, 391, 394);
    }
    else
    {
        UIArcDraw(&Deviation_arc, "deviationline", UI_Graph_Change, 4, UI_Color_Cyan, 267 - 60.0 * ui_cmd_recv.chassis_real_power / 200.0f , 269, 7, 956, 542, 391, 394);
    }
    
    // 动态UI发送
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, state_circle[0], state_circle[1], state_circle[2], state_circle[3], Cap_voltage);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, Shoot_Local_Heat,shoot_line[4],Cap_voltage_arc,Chassis_Power_arc, Shoot_Heat_arc);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 1, Deviation_arc);

    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}
