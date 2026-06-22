#include "oled_app.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"

#define IPS200_TYPE     (IPS200_TYPE_SPI)

//y最大为30，x最大为20
void ips_sprintf(uint8_t x,uint8_t y,char* format,...)
{
    va_list arg;
    char str[30];
    va_start(arg,format);
    vsnprintf(str,sizeof(str),format,arg);
    va_end(arg);
    ips200_show_string(y*8,x*16,str);
}

void ips_proc(void)
{
    ips_sprintf(0, 0, "Roll:%.2f   ", global_roll - angle_mid);
    ips_sprintf(1, 0, "gyro_roll:%.2f   ", global_gyro_roll);
    ips_sprintf(2, 0, "yaw:%.2f   ", global_yaw);
    switch(choice_para)
    {
        case 0:
            ips_sprintf(3, 0, "Target_speed:%.2f m/s   ", target_motor_speed);
        break;
        case 1:
            ips_sprintf(3, 0, "gyro_kp:%.2f    ", Gyro_PID.kp);
        break;
    }

}

void ips_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
    ips200_clear();
}
