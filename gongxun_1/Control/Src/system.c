#include "system.h"

// 整车系统初始化
void system_init(void)
{
    timer_disable();    // 先失能定时器
    HAL_Delay(100);     // 等待一会儿

    timer_init();       // 定时器初始化
    my_hwt101_init();   // 陀螺仪
    my_hmi_init();      // 串口屏
    step_motor_init();  // 闭环步进电机
    HAL_Delay(100);     // 等待一会儿

    chassis_init();     // 底盘装置
    catch_init();       // 抓取装置
        // 摄像头
        // 路径规划
        // 雷达
    remind_init();      // 提示装置
    HAL_Delay(100);     // 等待一会儿

    car_reset = true;   // 系统完成初始化/复位
}

// 整车系统控制 10ms中断
void system_control(void)
{
    // 系统未初始化（复位），不执行
    if (!car_reset) {
        return;
    }

    // 任务控制逻辑
}

// ============ 定义全局变量 ============ //
// 车辆启停标志位
bool car_run = false;   // 发车总开关
bool car_reset = false; // 车辆复位标志 false表示未复位
