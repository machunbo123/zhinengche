# AGENTS.md

本文件给代码代理提供本仓库的工作约束和代码结构说明。回答用户时使用中文。

## 项目概述

第 21 届全国大学生智能车竞赛单车定向赛道项目。车模为电动独轮自行车，后轮无刷电机驱动，前轮舵机转向，依靠向心力实现平衡和转弯。

- **车模**: N1，电机 BDS3620，舵机 BDS300，IMU660RB
- **MCU**: Infineon AURIX TC264D，双核 TriCore
- **框架**: 逐飞科技 SEEKFREE TC264 开源库 V3.4.1，基于 Infineon iLLD
- **当前工程名**: `E01_gpio_demo`，名称已过时，实际代码是单车平衡控制器

## 构建与烧录

本项目通过 AURIX Development Studio (ADS) IDE 构建和调试，无可直接独立使用的 Makefile 流程。

- **构建**: ADS 中导入工程后 Build，生成 `Debug/E01_gpio_demo.elf`
- **烧录/调试**: 使用 `.launch` 配置，调试器为 DAS JDS TriBoard TC3XX V2.0
- **清理**: 运行 `删除临时文件.bat` 删除 `Debug/`、`.ads/` 和 `.launch` 文件
- **测试**: 无自动化测试，最终验证依赖实际硬件

### 代理验证规则

- 不要自行编译或构建本项目。
- 不要调用 TASKING 编译器、生成的 Makefile、ADS 命令行构建或 ADS 无界面构建。
- TASKING 非商业许可证不支持独立运行编译器，生成的 Makefile 也需要由 ADS 集成环境处理。
- 代码修改后只做静态检查：声明、头文件依赖、调用位置、格式字符串、参数数量、共享变量和差异检查。
- 最终回复必须说明未执行编译，最终构建验证需要用户在 ADS 中手动完成。

## 目录与模块

- `user/cpu0_main.c`: CPU0 主入口、外设初始化、2ms PIT 控制中断、UART1/UART3 RX 中断。
- `user/cpu1_main.c`: CPU1 初始化后空循环，当前无应用逻辑。
- `user/isr.c`: 逐飞默认中断模板。`CCU60_CH1`、`UART1_RX`、`UART3_RX` 的实际应用中断已迁移到 `cpu0_main.c`，此文件内对应实现保持注释或空实现。
- `code/scheduler.c`: 主循环协作式调度器。
- `code/imu_app.c`: IMU660RB 初始化、零偏校准、轴映射、Mahony AHRS、欧拉角输出。
- `code/pid.c` / `code/pid.h`: 通用 PID 控制器。
- `code/pid_app.c`: 平衡控制三级串级 PID、启停控制、停车状态复位和 roll 倒地保护。
- `code/serve_app.c`: 舵机 PWM 限幅和输出。
- `code/motor_app.c`: 电机速度目标、RPM 与 m/s 换算、速度反馈缓存。
- `code/small_driver_uart_control.c`: CYT2BL3 驱动板 UART3 协议。
- `code/uart_app.c`: VOFA 遥测和 SBUS 遥控器任务。
- `code/key_app.c`: 4 个物理按键扫描与车辆启动/速度目标调整。
- `libraries/`: 逐飞库和 iLLD。除项目确有定制的驱动文件外，避免无关修改。

## 初始化流程

CPU0 当前初始化顺序在 `core0_main()` 中：

1. `clock_init()`、`debug_init()`
2. `ips_init()`
3. LED `P20_9` 初始化
4. 舵机 PWM：`ATOM1_CH1_P33_9`，330Hz，中值 `serve_mid`
5. `imu_init()`
6. `balance_pid_init()`
7. `wireless_uart_init()`
8. `uart_receiver_init()`
9. `key_app_init()`
10. `motor_init()`
11. `pit_ms_init(CCU60_CH1, 2)`
12. `cpu_wait_event_ready()`
13. `system_start()`
14. `scheduler_init()`
15. 主循环反复调用 `scheduler_run()`

`scheduler_init()` 必须位于 `system_start()` 之后，使任务 `last_run` 对齐到真实进入主循环后的毫秒时基。

## 时序架构

### 硬实时层

`CCU60_CH1` PIT 每 2ms 触发一次，ISR 定义在 `user/cpu0_main.c`：

```c
imu_proc();
balance_control_task();
```

该中断是姿态更新和平衡控制的核心路径。修改该路径时应避免阻塞、串口打印、大量浮点调试输出或任何可能导致 2ms 周期超时的逻辑。

### 软实时层

主循环调度器为协作式、非抢占式。若任务延迟，遗漏的周期会被跳过，不会集中补跑。

| 任务 | 周期 | 功能 |
| --- | --- | --- |
| `uart_app_vofa_task` | 10ms | 通过无线串口发送三环目标/实际值，VOFA JustFloat 二进制格式 |
| `motor_get_speed` | 10ms | 将 UART3 ISR 缓存的右轮 RPM 转换为 `motor_speed` |
| `uart_remote_task` | 10ms | 处理 SBUS 遥控器帧，CH3 档位变化时停车 |
| `key_app_task` | 10ms | 扫描按键，启动和调整目标速度 |
| `ips_proc` | 100ms | IPS200 显示 roll、yaw、目标速度、当前速度 |

## 中断与串口

- `CCU60_CH1`: 2ms 控制中断，在 `cpu0_main.c` 中定义。
- `UART1_RX`: SBUS 遥控器接收，在 `cpu0_main.c` 中调用 `uart_receiver_callback()`。
- `UART2_RX`: 无线串口模块，在 `user/isr.c` 中调用 `wireless_module_uart_handler()`。
- `UART3_RX`: CYT2BL3 电机驱动板速度反馈，在 `cpu0_main.c` 中调用 `uart_control_callback()`。
- `UART0`: 默认调试串口。

`user/isr.c` 中 UART1/UART3 的逐飞默认用途已经被项目改写，不要把这些中断恢复成摄像头或 GPS 默认回调。

## 控制架构

当前控制由 `code/pid_app.c` 实现，使用三级串级 PID：

- **外环 `Steering_PID`**: 每 20ms 运行一次，`Kp=0.20, Ki=0, Kd=0.2, limit=15`。使用 `normalize_angle(global_yaw - target_yaw_smooth)` 计算航向误差，并输出 `target_angle`。
- **中环 `Angle_PID`**: 每 10ms 运行一次，`Kp=100.0, Ki=0, Kd=0.3, limit=3000`。目标为 `target_angle`，反馈为 `global_roll - angle_mid`，输出 `target_gyro_roll`。
- **内环 `Gyro_PID`**: 每 2ms 运行一次，`Ki=0, Kd=0, limit=1500`。`Kp` 以 2.0 为基础，控制启用时按实际速度动态降低。目标为 `target_gyro_roll`，反馈为 `global_gyro_roll`，输出舵机 PWM 偏移。

重要细节：

- 控制启停变量为 `volatile uint8_t is_control`。
- 机械零点为 `volatile float angle_mid = 0.75f`。
- `start_car()` 会先设置 `target_yaw` 和 `target_yaw_smooth`，再下发 `target_motor_speed`，最后置 `is_control=1`。
- 当前代码里 `start_car()` 将 `target_yaw` 和 `target_yaw_smooth` 初始化为 `global_yaw`，与外环反馈同源，避免不断电重启时继承旧航向误差。
- `stop_car()` 会在短临界区内清 `is_control`，复位三级 PID 历史、`target_angle`、`target_gyro_roll` 和控制分频计数，再调用 `motor_stop()` 并让舵机回中。
- 已实现 roll 超限自动倒地保护：当 `is_control != 0` 且 `global_roll > 30.0f` 或 `global_roll < -30.0f` 时，`balance_control_task()` 会在 PID 计算前调用 `stop_car()`。
- 角速度内环 `Gyro_PID.kp` 使用实际速度 `motor_speed` 调度：`GYRO_KP_REFERENCE_SPEED` 及以下保持 `Kp=2.0`，超过后按 `kp = constrain(2.0f - GYRO_KP_SPEED_GAIN * (abs(motor_speed) - GYRO_KP_REFERENCE_SPEED), 0.05f, 2.0f)` 降低，不重置 PID 历史。
- `GYRO_KP_REFERENCE_SPEED` 当前为 `1.2f`，`GYRO_KP_SPEED_GAIN` 当前为 `0.8f`。
- VOFA 遥测每 10ms 以 JustFloat 二进制格式发送 10 个 `float` 通道，顺序为 `target_motor_speed,motor_speed,target_angle,global_roll-angle_mid,target_gyro_roll,global_gyro_roll,gyro_pid_output,is_control,Gyro_PID.kp,servo_final_duty`，帧长 44 字节，帧尾为 `{0x00,0x00,0x80,0x7f}`。VOFA+ 端需选择 JustFloat 协议，字节接收区按官方建议使用十六进制查看。

## 舵机与电机

### 舵机

- PWM 引脚：`ATOM1_CH1_P33_9`
- PWM 频率：330Hz
- 中值：`serve_mid = 4980`
- 限幅：`serve_min = 3480`，`serve_max = 6480`
- `set_servo_duty(pid_duty)` 将 PID 偏移叠加到中值并限幅后输出。
- `servo_final_duty` 记录 `set_servo_duty()` 限幅后的最终 PWM duty，供 VOFA 遥测查看实际下发值。

### 电机

- 驱动板：CYT2BL3
- 通信：UART3，460800 baud，0xA5 帧头，7 字节帧，和校验
- 当前使用右侧电机速度字段：`motor_rpm = -(motor_value.receive_right_speed_data)`
- 轮径：`WHEEL_DIAMETER = 0.064f`
- 速度单位换算：
  - `RPM_TO_SPEED = WHEEL_DIAMETER * PI / 60.0f`
  - `SPEED_TO_RPM = 60.0f / (WHEEL_DIAMETER * PI)`
- 默认目标速度：`target_motor_speed = 2.5f` m/s
- `motor_set_target_speed(float target_ms)` 接收 m/s，内部转换为 RPM 后发送速度闭环命令。
- `small_driver_get_speed()` 在初始化时发送一次速度查询命令，驱动板随后周期上报速度。
- 驱动板内部目标 RPM 按每 1ms 加减 0.5rpm 的阶梯斜坡变化；MCU 当前无法遥测驱动板内部目标 RPM，只能通过上报速度观察实际转速。

`small_driver_set_speed()` 内部短暂关闭中断，避免 2ms ISR 中零速帧和主循环速度帧交错。修改 UART3 下发逻辑时要保留这一并发保护思路。

## 按键与遥控器

按键定义来自 `libraries/zf_device/zf_device_key.h`：

| 按键 | 引脚 | 当前功能 |
| --- | --- | --- |
| KEY1 | P20_6 | 短按调用 `start_car()` |
| KEY2 | P20_7 | 切换停车状态下的调节参数：目标速度 / `Gyro_PID.kp` |
| KEY3 | P11_2 | 停车状态下对当前选中参数加 0.1，速度上限 3.0 m/s，`Gyro_PID.kp` 上限 3.0 |
| KEY4 | P11_3 | 停车状态下对当前选中参数减 0.1，速度下限 0.0 m/s，`Gyro_PID.kp` 下限 0.0 |

当前 KEY1 不是启停切换，只负责启动。停车由 `stop_car()` 提供，现有触发路径是 SBUS 遥控器 CH3 档位变化和 roll 超过 +30 度或低于 -30 度倒地保护。KEY3/KEY4 运行中只清按键状态，不改变目标速度或 `Gyro_PID.kp`。

SBUS 遥控器：

- UART：UART1
- 波特率：100000
- RX：`UART1_RX_P02_3`
- 通道数：6
- `uart_remote_task()` 仅在接收到完整帧且遥控器状态正常时处理 CH3。
- CH3 `<=600` 视为状态 1，`>=1000` 视为状态 2；状态从一个有效档位切到另一个有效档位时调用 `stop_car()`。

## IMU 流水线

`imu_init()`：

- 最多 10 次重试 `imu660rb_init()`。
- 启动时静止零偏校准：1000 个样本，每个样本间隔 2ms，最多 3 次重试。
- 零偏校准要求三轴陀螺原始跨度不超过约 3 dps。
- 使用加速度初始化姿态，要求加速度模长在 0.8g 到 1.2g。
- 成功后 `imu_ready=true`，LED `P20_9` 置高。

`imu_proc()`：

- 若 `imu_ready=false`，仅将 `global_gyro_roll` 清零并返回。
- 读取 IMU660RB 加速度和陀螺仪。
- 完成项目轴映射和陀螺零偏扣除。
- 使用 Mahony AHRS，`Kp=2.0, Ki=0.0`，更新 `global_pitch`、`global_roll`、`global_yaw` 和 `global_gyro_roll`。
- `normalize_angle()` 用于航向误差归一化到 `[-180, 180]`。

## PID 控制器

`PID_T` 支持位置式和增量式两种算法：

- `pid_init()` 默认令 `integral_limit = limit`，`d_filter_alpha = 1.0f`。
- `pid_calculate_positional()` 在 `Ki=0` 时会清零积分项。
- `pid_calculate_by_error()` 直接使用外部传入误差，适合已经处理角度环绕的航向误差。
- `pid_set_d_filter_alpha()` 当前只限制 0 到 1 范围内的滤波系数。
- 修改 PID 参数后，若会影响控制连续性，应调用 `pid_reset()` 清历史状态。

## 编码与编辑注意事项

- 根目录 `AGENTS.md` 使用 UTF-8。
- `.settings/org.eclipse.core.resources.prefs` 当前只显式指定了 `code/imu_app.c`、`code/small_driver_uart_control.c`、`user/cpu0_main.c` 为 UTF-8。
- 逐飞库和部分旧文件可能采用 ADS 默认编码。不要批量重编码，不要因工具显示乱码而重写无关文件。
- 源码中 ISR/主循环共享变量要保持 `volatile`，例如 `is_control`、`angle_mid`、`global_roll`、`global_gyro_roll`、`target_angle`、`target_gyro_roll`、`gyro_pid_output`、`target_yaw_smooth`、`motor_speed`、`target_motor_speed`、`servo_final_duty` 等。
- `zf_common_headfile.h` 会汇总包含项目头文件，修改头文件时仍需检查是否引入循环依赖或隐式声明。
- 保留逐飞库文件头部 GPL 声明和版权声明。

## 当前已知风险与待优化项

- roll 倒地保护只基于 `global_roll` 原始角度判断，未扣除 `angle_mid`。
- KEY1 只有启动，没有按键停车。
- 电机目标速度固定由按键调节，不随倾角、航向误差或轨迹状态动态联动。
- VOFA 当前通过 JustFloat 发送目标速度、实际速度、三环关键目标/反馈、角速度环输出、控制状态、当前内环 Kp 和最终舵机 PWM duty。
