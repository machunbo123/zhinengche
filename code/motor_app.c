#include "motor_app.h"

volatile float target_motor_speed = 2.5f;

volatile int16 motor_rpm;//电机的转速，单位是rpm/min
volatile float motor_speed;//电机的速度，单位是m/s

//电机的目标转速
volatile int16 target_motor_rpm;

//初始化电机
void motor_init(void)
{
    small_driver_uart_init();
}

//给电机设定速度，设定值为m/s下发后自动转换为RPM
void motor_set_target_speed(float target_ms)
{
    // 只有在下发命令的这一瞬间，做一次浮点数运算，把 m/s 变回 RPM
    target_motor_rpm = (int16)(target_ms * SPEED_TO_RPM);
    small_driver_set_speed((int16)0, (int16)target_motor_rpm);
}

void motor_stop(void)
{
    target_motor_rpm = 0;
    small_driver_set_speed((int16)0, (int16)0);
}

// 设置占空比
void motor_set_duty(int16 duty)
{
    // 限幅 duty最大为-10000 ~ 10000 自行判断限幅
    if(duty > 5000) duty = 5000;
    if(duty < -5000) duty = -5000;

    small_driver_set_duty((int16)0, (int16)(duty));
}

char speed_str[30];

// 获取速度的函数
void motor_get_speed(void)
{
    // 1. 读取原始的 RPM
    motor_rpm = -(motor_value.receive_right_speed_data);

    // 3. 计算用于显示的 m/s 速度
    motor_speed = motor_rpm * RPM_TO_SPEED;
}
