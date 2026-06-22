#include "scheduler.h"

// 任务数量（仅在本文件内使用）
static uint8_t task_num;

typedef struct {
    void (*task_func)(void);
    uint32_t rate_ms;
    uint32_t last_run;
} task_t;

// 静态任务数组，每个任务包含任务函数、执行周期（毫秒）和上次运行时间（毫秒）
static task_t scheduler_task[] =
{
        //通过无线串口发送相关参数
        {uart_app_vofa_task,10,0},
        //获取电机速度数据
        {motor_get_speed,10,0},
        //遥控器数据处理函数
        {uart_remote_task,10,0},
        {key_app_task,10,0},
        //lcd显示
        {ips_proc,100,0}
};
/**
 * @brief 调度器初始化函数
 * 计算任务数组的元素个数，并将结果存储在 task_num 中
 */
void scheduler_init(void)
{
    uint32_t now_time = system_getval_ms();

    // 计算任务数组的元素个数，并将结果存储在 task_num 中
    task_num = sizeof(scheduler_task) / sizeof(task_t);

    // 所有任务从调度器初始化完成后开始计时，避免补跑初始化阶段错过的周期。
    for(uint8_t i = 0; i < task_num; i++)
    {
        scheduler_task[i].last_run = now_time;
    }
}

/**
 * @brief 调度器运行函数
 * 遍历任务数组，检查是否有任务需要执行。如果当前时间已经超过任务的执行周期，则执行该任务并更新上次运行时间
 */
void scheduler_run(void)
{
    // 获取当前的系统时间（毫秒），在循环外取一次，保证所有任务用同一时间戳
    uint32_t now_time = system_getval_ms();
    // 遍历任务数组中的所有任务
    for (uint8_t i = 0; i < task_num; i++)
    {
        // 检查当前时间是否达到任务的执行时间
        uint32_t elapsed_ms = now_time - scheduler_task[i].last_run;
        if(elapsed_ms >= scheduler_task[i].rate_ms)
        {
            // 跳过阻塞期间遗漏的周期，保留原有相位但不集中补跑任务。
            scheduler_task[i].last_run +=
                (elapsed_ms / scheduler_task[i].rate_ms) * scheduler_task[i].rate_ms;
            // 执行任务函数
            scheduler_task[i].task_func();
        }
    }
}






