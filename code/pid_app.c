#include "pid_app.h"

volatile uint8_t is_control = 0; //当为1时表示启动pid控制

PID_T Angle_PID;    // 中环：角度环
PID_T Gyro_PID;     // 内环：角速度环
PID_T Steering_PID; // 外环：转向环（航向→倾角）

#define HEADING_STEP    1.5f            // 目标航向每次外环调用允许变化的最大角度
#define FALL_PROTECT_ROLL_DEG    30.0f  //倾倒保护的角度值
#define GYRO_KP_BASE    2.0f            //动态调节角度环时的kp基准
#define GYRO_KP_MIN    0.05f            //动态调节角度环kp时的最小值

#define GYRO_KP_REFERENCE_SPEED    1.2f //动态调节kp时的基准速度
#define GYRO_KP_SPEED_GAIN    0.8f      //动态调节kp时的比例系数

//pid计算分频器
static uint8_t angle_tick = 0;
static uint8_t steer_tick = 0;

//动态更新角速度环kp系数的函数
static void update_gyro_kp_by_speed(void)
{
    //计算当前速度的绝对值
    float speed_abs = (motor_speed >= 0.0f) ? motor_speed : -motor_speed;
    //计算与基准速度的差值
    float speed_over = speed_abs - GYRO_KP_REFERENCE_SPEED;

    if(speed_over < 0.0f)
    {
        speed_over = 0.0f;
    }
    //动态计算当前角速度环kp
    float gyro_kp = GYRO_KP_BASE - GYRO_KP_SPEED_GAIN * speed_over;

    Gyro_PID.kp = pid_constrain(gyro_kp, GYRO_KP_MIN, GYRO_KP_BASE);
}

void balance_pid_init(void)
{
    //航向环
    pid_init(&Steering_PID, 0.20f, 0.0f, 0.2f, 0.0f, 15.0f);
    //角度环
    pid_init(&Angle_PID, 60.0f, 0.0f, 0.3f, 0.0f, 3000.0f);
    //角速度环
    pid_init(&Gyro_PID, 2.0f, 0.0f, 0.0f, 0.0f, 1500.0f);
}

//机械零点
volatile float angle_mid = 0.75f;
//航向环的输出值
volatile float target_angle=0.0f;
//角度环的输出值
volatile float target_gyro_roll=0.0f;
//角速度环的输出值
volatile float gyro_pid_output=0.0f;
//目标航向
float target_yaw=0.0f;
//平滑后的目标航向，航向环计算
volatile float target_yaw_smooth=0.0f;

static void balance_reset_runtime_state(void)
{
    pid_reset(&Steering_PID);
    pid_reset(&Angle_PID);
    pid_reset(&Gyro_PID);

    target_angle = 0.0f;
    target_gyro_roll = 0.0f;
    gyro_pid_output = 0.0f;
    angle_tick = 0;
    steer_tick = 0;
}

void stop_car(void)
{
    boolean interrupt_state;

    interrupt_state = disableInterrupts();
    is_control = 0;
    balance_reset_runtime_state();
    restoreInterrupts(interrupt_state);

    motor_stop();
    set_servo_duty(0);
}

void start_car(void)
{
    boolean interrupt_state;

    interrupt_state = disableInterrupts();
    target_yaw = global_yaw;
    target_yaw_smooth = global_yaw;
    restoreInterrupts(interrupt_state);

    motor_set_target_speed(target_motor_speed);

    interrupt_state = disableInterrupts();
    is_control = 1;
    restoreInterrupts(interrupt_state);
}

void balance_control_task(void)
{
    if(0 == is_control)
    {
        set_servo_duty(0);
        return;
    }

    if((global_roll > FALL_PROTECT_ROLL_DEG) || (global_roll < -FALL_PROTECT_ROLL_DEG))
    {
        stop_car();
        return;
    }

    update_gyro_kp_by_speed();

    if(++steer_tick>=10)
    {
        steer_tick=0;
        float target_yaw_error = normalize_angle(target_yaw - target_yaw_smooth);
        if(target_yaw_error > HEADING_STEP)
        {
            target_yaw_smooth = normalize_angle(target_yaw_smooth + HEADING_STEP);
        }
        else if(target_yaw_error < -HEADING_STEP)
        {
            target_yaw_smooth = normalize_angle(target_yaw_smooth - HEADING_STEP);
        }
        else
        {
            target_yaw_smooth = target_yaw;
        }
        float yaw_error = normalize_angle(global_yaw - target_yaw_smooth);

        target_angle = pid_calculate_by_error(&Steering_PID, yaw_error);
    }
    if(++angle_tick>=5)
    {
        angle_tick=0;
        pid_set_target(&Angle_PID, target_angle);
        target_gyro_roll=pid_calculate_positional(&Angle_PID, global_roll-angle_mid);
    }
    pid_set_target(&Gyro_PID, target_gyro_roll);
    gyro_pid_output = pid_calculate_positional(&Gyro_PID, global_gyro_roll);
    set_servo_duty((int16)(-gyro_pid_output));
}

