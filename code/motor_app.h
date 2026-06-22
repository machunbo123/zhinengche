#ifndef CODE_MOTOR_APP_H_
#define CODE_MOTOR_APP_H_


#include "zf_common_headfile.h"

#define WHEEL_DIAMETER      0.064f       // 轮子直径
// 宏定义 RPM 转 m/s 的常数
#define RPM_TO_SPEED        (WHEEL_DIAMETER * PI / 60.0f)
// 宏定义：m/s 转 RPM 的反向常数
#define SPEED_TO_RPM        (60.0f / (WHEEL_DIAMETER * PI))

extern volatile int16 motor_rpm;          // 暴露给其他文件用
extern volatile float motor_speed;        // 改为 float 暴露给其他文件
extern volatile int16 target_motor_rpm;   // 暴露给 PID 用

extern volatile float target_motor_speed;

void motor_init(void);

void motor_set_target_speed(float target_ms);

// 立即清零软件目标，并向驱动板发送零速命令。
void motor_stop(void);

void motor_set_duty(int16 duty);

// 获取速度
void motor_get_speed(void);

#endif /* CODE_MOTOR_APP_H_ */
