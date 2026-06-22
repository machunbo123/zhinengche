/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.8.0
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

// **************************** 代码区域 ****************************
extern void uart_receiver_callback(void);   // 遥控器 SBUS 串口接收中断回调（定义在 zf_device_uart_receiver.c）
#define LED1                    (P20_9)

int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    //屏幕初始化
    ips_init();
    //初始化led
    gpio_init(LED1, GPO, GPIO_HIGH,  GPO_PUSH_PULL);
    //舵机初始化，频率为330时对应的占空比值为1650-8250
    pwm_init(ATOM1_CH1_P33_9, 330, (uint32)serve_mid);
    //初始化imu,初始化成功时led熄灭
    (void)imu_init();
    //初始化平衡环
    balance_pid_init();
    //初始化无线串口模块,使用串口2
    wireless_uart_init();
    //初始化遥控器串口
    uart_receiver_init();
    key_app_init();
    //初始化电机
    motor_init();
    //开启周期定时器中断
    pit_ms_init(CCU60_CH1, 2);      //中断2，周期为2ms
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    //调度器初始化：放在所有初始化与双核同步之后，让 last_run 对齐到真正进入主循环的时刻，
    system_start();
    scheduler_init();
	while (TRUE)
	{
	    scheduler_run();
	}
}
//2ms定时器中断，执行pid任务
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    //解析陀螺仪数据
    imu_proc();
    //执行平衡控制任务
    balance_control_task();
}
//串口1接收中断，遥控器SBUS数据接收
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    uart_receiver_callback();                       // 遥控器串口接收回调函数
}
//串口3接收中断，速度数据传输后触发串口3中断，进行速度解析
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
//    gnss_uart_callback();                            // GPS串口回调函数
    //解析速度函数
    uart_control_callback();
}

#pragma section all restore
