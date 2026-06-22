#include "uart_app.h"
#include "zf_common_headfile.h"

#include <string.h>

#define VOFA_JUSTFLOAT_CHANNEL_COUNT    (10U)
#define VOFA_JUSTFLOAT_TAIL_SIZE        (4U)
#define VOFA_JUSTFLOAT_FRAME_SIZE       (VOFA_JUSTFLOAT_CHANNEL_COUNT * sizeof(float) + VOFA_JUSTFLOAT_TAIL_SIZE)

static const uint8 vofa_justfloat_tail[VOFA_JUSTFLOAT_TAIL_SIZE] = {0x00, 0x00, 0x80, 0x7f};

static uint8_t last_ch3_state = 0;

//通过无线串口模块向VOFA上位机发送JustFloat二进制遥测帧
void uart_app_vofa_task(void)
{
    float vofa_data[VOFA_JUSTFLOAT_CHANNEL_COUNT] =
    {
            target_motor_speed,
            motor_speed,
            target_angle,
            global_roll - angle_mid,
            target_gyro_roll,
            global_gyro_roll,
            gyro_pid_output,
            (float)is_control,
            Gyro_PID.kp,
            (float)servo_final_duty
    };
    uint8 vofa_frame[VOFA_JUSTFLOAT_FRAME_SIZE];

    // JustFloat格式：小端32位float数组 + 固定帧尾 00 00 80 7F，VOFA端需选择JustFloat协议。
    memcpy(vofa_frame, vofa_data, sizeof(vofa_data));
    memcpy(&vofa_frame[sizeof(vofa_data)], vofa_justfloat_tail, sizeof(vofa_justfloat_tail));
    wireless_uart_send_buffer(vofa_frame, (uint32)sizeof(vofa_frame));
}

//打印遥控接收到的数据
void uart_tran_remote_data(void)
{
    printf("CH1-CH6 data: ");
    for(int i = 0; i < 6; i++)
    {
        printf("%d ", uart_receiver.channel[i]);         // 串口输出6个通道数据
    }
    printf("\r\n");
}

//无线遥控器的任务函数
void uart_remote_task(void)
{
    if(1 == uart_receiver.finsh_flag)                            // 帧完成标志判断
    {
        if(1 == uart_receiver.state)                             // 遥控器失控状态判断
        {
            uint8_t current_ch3_state = 0;
            uint16_t channel3 = uart_receiver.channel[2];

            if(channel3 <= 600)
            {
                current_ch3_state = 1;
            }
            else if(channel3 >= 1000)
            {
                current_ch3_state = 2;
            }

            if(0 != current_ch3_state)
            {
                if(0 == last_ch3_state)
                {
                    last_ch3_state = current_ch3_state;
                }
                else if(current_ch3_state != last_ch3_state)
                {
                    stop_car();
                    last_ch3_state = current_ch3_state;
                }
            }
//            uart_tran_remote_data();
        }
        uart_receiver.finsh_flag = 0;                            // 帧完成标志复位
    }
}
