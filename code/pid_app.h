#ifndef CODE_PID_APP_H_
#define CODE_PID_APP_H_


#include "zf_common_headfile.h"

void balance_pid_init(void);
void balance_control_task(void);
void stop_car(void);
void start_car(void);
extern volatile uint8_t is_control;
extern volatile float angle_mid;
extern volatile float target_angle;
extern volatile float target_gyro_roll;
extern volatile float gyro_pid_output;
extern volatile float target_yaw_smooth;

// 定义平衡用的串级 PID（三级架构）
extern PID_T Angle_PID;    // 中环：角度环 (10ms)
extern PID_T Gyro_PID;     // 内环：角速度环 (2ms)
extern PID_T Steering_PID; // 外环：转向环 — 航向→倾角 (20ms)

#endif /* CODE_PID_APP_H_ */
