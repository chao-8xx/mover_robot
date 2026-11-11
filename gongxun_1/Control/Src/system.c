#include "headfile.h"

// 整车系统初始化 不可放入中断。
// 定时器会在部分机构/任务初始化完成前启动；car_reset 只阻止比赛状态机，不会停掉全部底层服务。
void system_init(void)
{
    // 失能定时器
    timer_disable();            // 先失能定时器
    car_reset = false;          // 未完成初始化/复位
    HAL_Delay(100);             // 等待一会儿

    // 外设初始化
    uart_init();                // 串口初始化   陀螺仪 串口屏 摄像头 串口调试 雷达 步进电机
    timer_init();               // 定时器初始化

    // 车身装置初始化
    chassis_init();             // 底盘装置
    catch_init();               // 抓取装置
    remind_init();              // 提示装置
    light_buzzer_init();        // 灯光蜂鸣器初始化
    HAL_Delay(100);             // 等待一会儿

    // 任务、算法初始化
    bfs_init();                 // 路径规划 ———— BFS 迪杰斯特拉
    bfs_base_obstacle();        // 预设障碍物
    HAL_Delay(100);             // 等待一会儿
    task_init();                // 任务初始化 ———— 抓取任务
    tf_init();                  // 坐标变换初始化 ———— 坐标变换及全局位姿
    map_pose_init();            // 等定位模块发布真实位姿 复位时清除上次结果
    map_update_init();          // 初始化道路状态和扫描确认记录
    HAL_Delay(100);             // 等待一会儿

    lidar_start();              // 启动雷达
    HAL_Delay(100);             // 等待一会儿

    init_ok_remind();           // 初始化完成提醒

    // 变量初始化
    openmv_end = 0;
    car_reset = true;           // 系统完成初始化/复位
}

// 整车系统控制 10ms中断
// 控制入口只在 car_run && car_reset 时执行；停止后的驱动处理不能仅依赖 control_process 的 step=99 分支。
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
