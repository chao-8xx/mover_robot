#include "headfile.h"

// 车辆总任务状态
Task_State task_state;


// 陀螺仪处理任务
void imu_task(void) {
    // 处理陀螺仪数据
    if(hwt101_rx_flag || hwt101_error_flag) {

        // 先清除标志位
        hwt101_error_flag = 0;
        hwt101_rx_flag = 0;

        hwt101_rx_handle();
    }
}

// 摄像头处理任务
void cam_task(void) {
    if (cam_rx_flag || cam_error_flag) {
        cam_error_flag = 0;
        cam_rx_flag = 0;  // 先清除标志位
        cam_rx_handle();
    }
    maix_result_task();  // 维护5帧VALID/STABLE 并处理视觉数据超时
}


// 障碍物检测任务
uint8_t obstacle_find_task(float car_x, float car_y, float car_yaw) {
    static uint8_t first_idle_scan = 0;     // 第一次空闲状态下的雷达扫描标志位
    static uint32_t last_scan_num = 0;      // 上一次雷达扫描编号

    Chassis_Mode chassis_mode;              // 底盘模式
    uint32_t tick;                          // 当前系统时间
    const lidar_scan_t *scan;               // 雷达扫描数据

    // 一直运行雷达扫瞄运行任务
    lidar_process();

    scan = lidar_get_scan();
    chassis_mode = car_get_state();

    // 若底盘非空闲 此时不进行点云更新和障碍物聚类
    if (chassis_mode != CHASSIS_IDLE) {
        first_idle_scan = 1;

        if (scan != NULL)
            last_scan_num = scan->scan_num;

        return 0;
    }

    // 还未完成第一圈扫描
    if (scan == NULL || scan->scan_num == 0)
        return 0;

    // 第一次底盘空闲时 记录当前雷达扫描编号 避免误判
    if (scan->scan_num == last_scan_num)
        return 0;

    last_scan_num = scan->scan_num;

    // 清除第一圈标志位
    if (first_idle_scan) {
        first_idle_scan = 0;
        return 0;
    }

    tick = HAL_GetTick();

    // 完整静止圈生成地图点云
    if (!point_cloud_update(scan, car_x, car_y, car_yaw, tick))
        return 0;

    // 进行障碍物聚类操作
    obstacle_cluster_update(&cloud, tick);

    return 1;
}