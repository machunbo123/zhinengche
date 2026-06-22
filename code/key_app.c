#include "key_app.h"
#include "zf_device_key.h"

static float key_num_limit(float value,float min,float max)
{
    if(value > max)
    {
        return max;
    }
    if(value < min)
    {
        return min;
    }
    return value;
}

void key_app_init(void)
{
    key_init(10);
}

uint8_t choice_para=0;//按键调参选择的参数，默认为速度
uint8_t choice_num=2; //需要调节参数的数量

void key_app_task(void)
{
    key_scanner();

    if(KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        start_car();
        key_clear_state(KEY_1);
    }

    if(KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        choice_para++;
        if(choice_para>=2) choice_para=0;
        key_clear_state(KEY_2);
    }

    if(KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        if(0 == is_control)
        {
            switch(choice_para)
            {
                case 0:
                    target_motor_speed = key_num_limit(target_motor_speed + 0.1,0.0f,3.0f);
                break;
                case 1:
                    Gyro_PID.kp=key_num_limit(Gyro_PID.kp+0.1f,0.0f,3.0f);
                break;
            }
        }
        key_clear_state(KEY_3);
    }

    if(KEY_SHORT_PRESS == key_get_state(KEY_4))
    {
        if(0 == is_control)
        {
            switch(choice_para)
            {
                case 0:
                    target_motor_speed = key_num_limit(target_motor_speed - 0.1,0.0f,3.0f);
                break;
                case 1:
                    Gyro_PID.kp=key_num_limit(Gyro_PID.kp-0.1f,0.0f,3.0f);
                break;
            }
        }
        key_clear_state(KEY_4);
    }
}
