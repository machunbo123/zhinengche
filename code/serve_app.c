#include "serve_app.h"

int32_t serve_mid = 4980; // 数值小往左，数值大往右
int32_t serve_min = 3480;
int32_t serve_max = 6480;
volatile int32_t servo_final_duty = 4980;

void set_servo_duty(int16 pid_duty)
{
    int32_t final_duty = serve_mid + (int32_t)pid_duty;

    if(final_duty < serve_min)
    {
        final_duty = serve_min;
    }
    else if(final_duty > serve_max)
    {
        final_duty = serve_max;
    }

    servo_final_duty = final_duty;
    pwm_set_duty(ATOM1_CH1_P33_9, (uint32)final_duty);
}
