#include "headfile.h"

// 整车系统初始化 不可放入中断
void system_init(void)
{
    timer_disable();            // 先失能定时器
    HAL_Delay(100);             // 等待一会儿
    car_reset = false;          // 未完成初始化/复位

    task_init();                // 任务初始化
    timer_init();               // 定时器初始化
    light_buzzer_init();        // 灯光蜂鸣器初始化
    uart_init();                // 串口初始化   陀螺仪 串口屏 摄像头 串口调试 雷达

    tf_init();                  // 坐标变换初始化
    HAL_Delay(100);             // 等待一会儿
    light_on();                 // 开启补光灯

    lidar_start();              // 启动雷达

    step_motor_init();          // 闭环步进电机
    HAL_Delay(100);             // 等待一会儿

    chassis_init();             // 底盘装置
    catch_init();               // 抓取装置

    remind_init();              // 提示装置
    bfs_init();                 // 路径规划
    bfs_base_obstacle();        // 预设障碍物
    HAL_Delay(100);             // 等待一会儿

    init_ok_remind();           // 初始化完成提醒

    openmv_end = 0;
    car_reset = true;           // 系统完成初始化/复位

    // obstacle_map_init();
}

// 整车系统控制 10ms中断
void system_control(void)
{
    // 不具备发车条件，不执行
    if (!run_is_ok()) {
        return;
    }

    // 任务控制逻辑
    control_process();
}

// 是否具备发车条件
bool run_is_ok(void)
{
    return (car_run && car_reset);  // 初始化完成且发车指令
}

// ============ 定义全局变量 ============ //
// 车辆启停、复位标志位
bool car_run = false;           // 发车总开关
bool car_reset = false;         // 车辆复位标志 false表示未复位
// 发车位置
uint8_t start_location = 0;     // 0：启停区2  1：启停区1
