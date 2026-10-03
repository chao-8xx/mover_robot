#include "headfile.h"

// 车辆总任务状态
Task_State task_state;

// ==== 定义 ==== //
#define ANALYSIS_LEN        12                  // 解析数组长度
uint8_t analysis_task_data[ANALYSIS_LEN];       // 解析完后的数组

// ==== 声明 ==== //
extern uint8_t qr_data[QR_DATA_LEN + 1];        // 声明openmv接收数据数组

// 任务初始化
void task_init()
{
    // 控制变量初始化
    control_variate_init();

    // 解析数组清空
    for (int j = 0; j < ANALYSIS_LEN; j++) {
        analysis_task_data[j] = 0x00;
    }

    // 状态机清空
    for (int i = 0; i < 3; i++) {
        task_state.color_buf_one[i] = 0;
        task_state.area_buf_one[i] = 0;
        task_state.color_buf_two[i] = 0;
        task_state.area_buf_two[i] = 0;
    }

    // 车辆本身状态机初始化
    task_state.cur_x = 0;
    task_state.cur_y = 0;
    task_state.task_location = PARKING_2;   // 默认启停区2

    // 任务状态机初始化
    task_state.claw_state = LOOSE;          // 夹爪：松开
    task_state.turntable_pos = LOCATION_1;  // 转盘：一号位
    task_state.lift_height = PICK_HEIGHT;   // 升降装置：默认高度
    task_state.stretch_dist = ORIGIN_DIST;  // 伸缩装置：原点距离
    task_state.car_work = STOP;             // 车辆工作状态：停止
    // task_state.task_location = ;            // 位置：用串口屏设置起始坐标
}

// 任务解析 把接收到的C0、C1等颜色字节转化为状态
static void task_receive_analysis(void)
{
    uint8_t analysis_data[ANALYSIS_LEN] = {0};
    uint8_t pass = 0;

    for (int i = 0; i < ANALYSIS_LEN; i++) {
        if (i == 3 || i == 6 || i == 9) {  // 过滤掉任务数组里的符号 “+”
            pass += 1;
        }
        analysis_data[i] = qr_data[i + pass];   // 将过滤掉“+”的数据存入新数组
    }

    for (int j = 0; j < ANALYSIS_LEN; j++) {
        switch (analysis_data[j])
        {
            case MAIX_COLOR_RED:
                analysis_task_data[j] = TASK_RED;
                break;
            case MAIX_COLOR_YELLOW:
                analysis_task_data[j] = TASK_YELLOW;
                break;
            case MAIX_COLOR_BLUE:
                analysis_task_data[j] = TASK_BLUE;
                break;
            case MAIX_COLOR_GREEN:
                analysis_task_data[j] = TASK_GREEN;
                break;
            case MAIX_COLOR_BLACK:
                analysis_task_data[j] = TASK_BLACK;
                break;
            case MAIX_COLOR_LIGHTBLUE:
                analysis_task_data[j] = TASK_LOW_BLUE;
                break;
            case MAIX_RING_1:
                analysis_task_data[j] = AREA_1;
                break;
            case MAIX_RING_2:
                analysis_task_data[j] = AREA_2;
                break;
            case MAIX_RING_3:
                analysis_task_data[j] = AREA_3;
                break;
            default:
                analysis_task_data[j] = 0;
                break;
        }
    }
}

// 任务接收
void task_receive(void)
{
    // 解析数据
    task_receive_analysis();

    // 将解析后的数组赋值进状态机
    memcpy(task_state.color_buf_one, &analysis_task_data[0], 3);
    memcpy(task_state.area_buf_one, &analysis_task_data[3], 3);
    memcpy(task_state.color_buf_two, &analysis_task_data[6], 3);
    memcpy(task_state.area_buf_two, &analysis_task_data[9], 3);
}

// 任务执行总函数
void task_process(void)
{
    control_process();
}


// 陀螺仪处理任务
void imu_task(void) {
    // 处理陀螺仪数据
    if(hwt101_rx_flag || hwt101_error_flag) {

        // 先清除标志位
        // 错误标志由驱动清除；提前清零会跳过DMA恢复分支
        hwt101_rx_flag = 0;

        hwt101_rx_handle();
    }
}

// 摄像头处理任务
void cam_task(void) {
    if (cam_rx_flag || cam_error_flag) {
        // 保留错误标志给cam_rx_handle处理，接收通知仍先清除
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

// 从完整雷达扫描到道路状态的入口。当前安装标定测试不调用这个函数
uint8_t map_obstacle_task(float car_x, float car_y, float car_yaw, uint8_t pose_valid) {
    static uint8_t wait_new_scan = 2;  // 2: 记录旧圈；1: 丢弃跨越恢复时刻的圈
    static uint32_t skip_scan_num = 0;
    static uint32_t last_drop_num = 0;
    const lidar_scan_t *scan;
    uint32_t drop_num;

    // 坏包/接收错误会打断连续观察，恢复后要重新等完整静止圈
    lidar_process();
    drop_num = lidar_get_scan_drop_cnt();
    if (drop_num != last_drop_num) {
        last_drop_num = drop_num;
        map_update_clear_candidates();
        wait_new_scan = 2;
        skip_scan_num = 0;
    }

    if (!pose_valid || !map_update_is_ready() || car_get_state() != CHASSIS_IDLE ||
        !isfinite(car_x) || !isfinite(car_y) || !isfinite(car_yaw) ||
        map_check_area(car_x, car_y) != map_normal) {
        map_update_clear_candidates();
        wait_new_scan = 2;
        skip_scan_num = 0;
        return 0;
    }

    // 先记住已经发布的旧圈，再丢弃恢复后第一圈：它可能在恢复前就开始采集
    if (wait_new_scan == 2) {
        scan = lidar_get_scan();
        if (scan == NULL || scan->scan_num == 0)
            return 0;
        skip_scan_num = scan->scan_num;
        wait_new_scan = 1;
        return 0;
    }

    if (wait_new_scan == 1) {
        scan = lidar_get_scan();
        if (scan == NULL || scan->scan_num == skip_scan_num) return 0;
        skip_scan_num = scan->scan_num;
        wait_new_scan = 0;
        return 0;
    }

    if (skip_scan_num != 0) {
        scan = lidar_get_scan();
        if (scan == NULL || scan->scan_num == skip_scan_num) return 0;
        skip_scan_num = 0;
    }

    if (!obstacle_find_task(car_x, car_y, car_yaw)) return 0;
    // obstacle_find_task也会取DMA数据；若在这次处理里发生丢圈，不能先写地图再恢复
    drop_num = lidar_get_scan_drop_cnt();
    if (drop_num != last_drop_num) {
        last_drop_num = drop_num;
        map_update_clear_candidates();
        wait_new_scan = 2;
        skip_scan_num = 0;
        return 0;
    }
    return map_lidar_handle(1);
}
