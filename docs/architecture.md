# 整车架构与调度

核对日期：2026-11-12。功能基线：0dcaf8b。本文说明当前源码的调用关系，不代表设备正在运行同一版本。

## 1. 分层与入口

| 层次 | 主要文件 | 责任 |
| --- | --- | --- |
| 外设与启动 | [main.c](../gongxun_1/Core/Src/main.c)、[usart.c](../gongxun_1/Core/Src/usart.c)、[tim.c](../gongxun_1/Core/Src/tim.c) | 配置时钟、GPIO、DMA、串口、定时器，进入循环 |
| 驱动与协议 | [uart_dma.c](../gongxun_1/BSP/Src/uart_dma.c)、[Emm_V5.c](../gongxun_1/BSP/Src/Emm_V5.c)、[camera.c](../gongxun_1/BSP/Src/camera.c)、[hwt101.c](../gongxun_1/BSP/Src/hwt101.c)、[lidar.c](../gongxun_1/BSP/Src/lidar.c) | 收发字节、协议解包、反馈缓存及有效性 |
| 执行与算法 | [chassis.c](../gongxun_1/APP/Src/chassis.c)、[catch.c](../gongxun_1/APP/Src/catch.c)、[pid.c](../gongxun_1/APP/Src/pid.c)、[bfs.c](../gongxun_1/APP/Src/bfs.c) | 运动和机构命令、busy、PID、路径搜索 |
| 整车协调 | [system.c](../gongxun_1/Control/Src/system.c)、[task.c](../gongxun_1/Control/Src/task.c)、[control.c](../gongxun_1/Control/Src/control.c) | 发车条件、任务数据、比赛流程 |
| 人机交互 | [hmi_app.c](../gongxun_1/APP/Src/hmi_app.c)、lcd/ | 发车、停止、手动动作和状态显示 |
| 独立感知导航 | APP 下的 map_*，其中[map_obs.c](../gongxun_1/APP/Src/map_obs.c)处理点云/聚类 | 位姿、点云、道路状态和分段导航，按接入状态区别使用 |

追踪行为时按“上层在哪里调用 → 参数是什么 → 驱动何时发送 → 状态如何结束”的顺序阅读。驱动里有接口，不等于上层已经使用。

## 2. 初始化顺序

main() 完成 HAL、时钟和 MX_* 初始化后调用 system_init()：

1. 停止应用定时器，置 car_reset=false。
2. uart_init() 初始化调试、摄像头、IMU、雷达、屏幕和步进电机通信。
3. timer_init() 启动 TIM2、TIM6、TIM7。
4. 初始化底盘、抓放机构、提示装置。
5. 初始化比赛路径、任务数据、坐标变换、位姿和道路地图。
6. 启动雷达，完成提示，最后置 car_reset=true。

初始化含 HAL_Delay()，机构初始化也依赖周期服务，因此 system_init() 必须在主循环上下文执行。定时器先于部分上层初始化启动；system_control() 通过 car_run && car_reset 判断是否可推进比赛。这个条件只约束整车状态机，不能推导为所有底层中断都停止工作。

## 3. 实际调度表

| 执行位置 | 周期 | 当前调用 | 维护要点 |
| --- | --- | --- | --- |
| main() 的 while(1) | 尽快循环，非固定周期 | cam_task()、mv_task()，以及完成页切换 | 接收解析、视觉超时、挂起命令发送；扫码等待会占用这条循环 |
| TIM6 | 1 ms | hwt101_tick()、car_run_tick()、catch_tick() | 复位步骤、四轮发送、底盘/机构计时与部分反馈处理 |
| TIM7 | 5 ms | imu_task()、imu_get()、remind_control() | IMU 解析、连续角度和提示 |
| TIM2 | 10 ms | system_control()、debug_printf_task() | 比赛状态机、转向/跟踪控制、调试发送重试 |
| UART/DMA 回调 | 事件触发 | 各驱动接收/发送完成/错误回调 | 各串口路径不同，见通信说明 |

周期来自当前时钟和 tim.c 配置，不是实测最坏执行时间。这里没有采用 FreeRTOS 任务调度；函数名中的 task 表示服务函数。

cam_task() 同时执行 maix_result_task() 和 send_conduct_task()，停止调用会影响超时撤销和视觉命令发送。mv_task() 通过 service_for()/wait_qr() 同步等待，不能将整套程序描述为所有路径完全无阻塞。

## 4. 数据流

~~~mermaid
flowchart LR
    U[USART1 DMA回调] --> B[接收缓存]
    B --> C[主循环 cam_task]
    C --> R[视觉原始与窗口结果]
    R --> T[TIM2 比赛控制]
    I[TIM7 IMU与连续角度] --> T
    T --> P[PID与动作目标]
    P --> Q[电机发送队列]
    Q --> M[USART2 DMA与驱动器]
    M --> F[位置反馈缓存]
    F --> K[TIM6 机构处理]
~~~

底盘四轮航迹推算还需要显式执行 chassis_odom_tick()，当前 main() 没有调用它。反馈驱动存在不等于连续全局位姿正在更新。

视觉启动命令通常经 control.c 设置 is_send_color/is_send_circle/is_send_stack → 主循环 cam_task() → send_conduct_task() → maix_start_*()。部分停止命令仍直接在控制函数中发送，不能认为所有视觉发送都在主循环。

## 5. 比赛状态机

control_process() 先处理动作等待，再进入 step 分支：

| step | 含义 |
| --- | --- |
| 0 / 1 | 发车状态设置 / 等待 |
| 2 / 3 | 离开启停区 / 返回 |
| 4～8 | 规划去往启停区、扫码区、原料区、暂存区、粗加工区 |
| 9 | 到点角度校准与任务分派 |
| 10 / 11 | 常规道路 / 目标前最后一段 |
| 12 | 扫码 |
| 13 / 14 / 15 | 抓取 / 放置 / 再次拿取 |
| 16 | 雷达方向扫描 |
| 99 / default | 停止分支，需结合前面的重新发车判断理解 |

control_variate_init() 的目标顺序为：扫码 → 原料 → 粗加工 → 暂存 → 原料 → 粗加工 → 暂存 → 启停。阶段内会依据任务位置和批次执行子流程，这个数组不是每个机械动作的完整清单。

move_state、rotate_state、lift_state、stretch_state 表示上层等待相应动作。旋转等待每拍调用 car_rotate_yaw()，因为航向闭环要持续计算；位置动作通常下发一次，再等 busy 结束。子函数中的 static step 和计数器跨调用保留，维护复位/重新发车时需要同时核查。

## 6. 模块接入状态

| 模块 | 当前状态 | 可说明的能力 |
| --- | --- | --- |
| bfs_find_path() 与比赛状态机 | 已有实际调用 | 5×5 加权搜索、方向与转弯代价、动作协调 |
| lidar_get_obstacle() | 有比赛步骤调用；main 的 lidar_process() 被注释 | 有方向扫描/格子标记逻辑，尚不能确认持续消费新扫描 |
| chassis_odom_tick() | 实现存在，正式 main/定时器未周期调用 | 编码器增量航迹推算实现，接入和实测待确认 |
| map_obstacle_pose_task() | 实现及测试入口存在，正式 main 未调用 | 基于可信位姿的完整圈建图与方向观察 |
| map_nav_start()/map_nav_task() | 实现与主机测试存在，比赛入口未调用 | 独立道路分段导航、到位确认、异常停止 |
| lidar_replay/ | 独立电脑工具 | 导入、参数对比、问题帧复现及部分 C 算法对照 |

新旧规划模块的数据结构、坐标和控制所有权不同。接入道路导航需要选择底盘控制者、发布可信位姿并提供新扫描。本次整理保留现有接入状态。

## 7. 阅读与修改顺序

先读 main.c → system.c → timer.c；再看 control.h 的步骤定义和 control_process()；随后按运动、抓取、视觉、雷达进入模块。增加服务时同步写明调用位置、周期、单位、失效和退出条件。
