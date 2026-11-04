#include "headfile.h"

// 车辆总任务状态
Task_State task_state;

// ==== 定义 ==== //
#define ANALYSIS_LEN        12                  // 解析数组长度
uint8_t analysis_task_data[ANALYSIS_LEN];       // 解析完后的数组
uint8_t openmv_start = 0;                       // openmv启动标志
uint8_t openmv_end = 0;                         // openmv结束标志

// ==== 声明 ==== //
extern uint8_t qr_data[QR_DATA_LEN + 1];        // 声明openmv接收数据数组

// 搬运任务初始化
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

    // 任务初始化未完成状态
    task_state.one_finish = 0;
    task_state.two_finish = 0;

    // 车辆本身状态机初始化
    task_state.cur_x = 0;
    task_state.cur_y = 0;
    task_state.task_location = PARKING_2;   // 默认启停区2

    // 任务状态机初始化
    task_state.claw_state = LOOSE;          // 夹爪：松开
    task_state.turntable_pos = LOCATION_1;  // 转盘：一号位
    task_state.car_work = STOP;             // 车辆工作状态：停止

    // 给串口屏发送任务标志
    task_state.send_task_hmi_ok = 0;

    // openmv工作状态初始化
    openmv_start = 0;
    openmv_end = 0;

    // 没码时 手动设置任务 后续可删 默认任务
    task_state.color_buf_one[0] = TASK_YELLOW; // 第一组 红黄蓝
    task_state.color_buf_one[1] = TASK_GREEN;
    task_state.color_buf_one[2] = TASK_BLUE;
    task_state.area_buf_one[0] = AREA_1;
    task_state.area_buf_one[1] = AREA_2;
    task_state.area_buf_one[2] = AREA_3;
    task_state.color_buf_two[0] = TASK_BLUE; // 第二组 蓝红黄
    task_state.color_buf_two[1] = TASK_YELLOW;
    task_state.color_buf_two[2] = TASK_GREEN;
    task_state.area_buf_two[0] = AREA_1;
    task_state.area_buf_two[1] = AREA_2;
    task_state.area_buf_two[2] = AREA_3;
    task_state.color_buf[0] = TASK_YELLOW;      // 总顺序
    task_state.color_buf[1] = TASK_GREEN;
    task_state.color_buf[2] = TASK_BLUE;
    task_state.color_buf[3] = TASK_BLUE;
    task_state.color_buf[4] = TASK_YELLOW;
    task_state.color_buf[5] = TASK_GREEN;

    // 用 color_buf[3+f]（第二组按位置排好序）而不是 color_buf_two[e]（扫描原始顺序）
    for (int e = 0; e < 3; e++) {          // e = put_num = 环位置
        task_state.turntable_seq[e] = 0;
        for (int f = 0; f < 3; f++) {      // f = 转盘槽
            if (task_state.color_buf[3 + f] == task_state.color_buf[e]) {
                task_state.turntable_seq[e] = f;
                break;
            }
        }
    }
}

// 任务解析 把接收到的C0、C1等颜色字节转化为状态
static void task_receive_analysis(void)
{
    uint8_t analysis_data[ANALYSIS_LEN] = {0};
    uint8_t idx = 0;

    // 先清一遍，避免上一次的残留
    memset(analysis_task_data, 0, sizeof(analysis_task_data));

    // 1) 把 15 字节里的 '+' 分隔符滤掉，剩下 12 个有效字节
    for (uint8_t i = 0; i < QR_DATA_LEN && idx < ANALYSIS_LEN; i++) {
        if (qr_data[i] == '+') {
            continue;
        }
        analysis_data[idx++] = qr_data[i];
    }

    // 2) 按位置区分颜色段和位置段
    //    颜色段：下标 0~2、6~8
    //    位置段：下标 3~5、9~11
    for (uint8_t j = 0; j < ANALYSIS_LEN; j++) {
        uint8_t c = analysis_data[j];
        uint8_t is_area = (j >= 3 && j < 6) || (j >= 9 && j < 12);

        if (is_area) {
            // 位置段：ASCII '1'~'3' 或 二进制 0xD1~0xD3
            switch (c) {
                case '1':
                case MAIX_RING_1:
                    analysis_task_data[j] = AREA_1;
                    break;
                case '2':
                case MAIX_RING_2:
                    analysis_task_data[j] = AREA_2;
                    break;
                case '3':
                case MAIX_RING_3:
                    analysis_task_data[j] = AREA_3;
                    break;
                default:
                    analysis_task_data[j] = 0;
                    break;
            }
        }
        else {
            // 颜色段：ASCII '1'~'6' 或 二进制 0xC1~0xC6
            switch (c) {
                case '1':
                case MAIX_COLOR_RED:
                    analysis_task_data[j] = TASK_RED;
                    break;
                case '2':
                case MAIX_COLOR_YELLOW:
                    analysis_task_data[j] = TASK_YELLOW;
                    break;
                case '3':
                case MAIX_COLOR_BLUE:
                    analysis_task_data[j] = TASK_BLUE;
                    break;
                case '4':
                case MAIX_COLOR_GREEN:
                    analysis_task_data[j] = TASK_GREEN;
                    break;
                case '5':
                case MAIX_COLOR_BLACK:
                    analysis_task_data[j] = TASK_BLACK;
                    break;
                case '6':
                case MAIX_COLOR_LIGHTBLUE:
                    analysis_task_data[j] = TASK_LOW_BLUE;
                    break;
                default:
                    analysis_task_data[j] = 0;
                    break;
            }
        }
    }
}

// 任务接收
uint8_t task_receive(void)
{
    // 解析数据
    task_receive_analysis();

    // 将解析后的数组赋值进状态机
    memcpy(task_state.color_buf_one, &analysis_task_data[0], 3);
    memcpy(task_state.area_buf_one, &analysis_task_data[3], 3);
    memcpy(task_state.color_buf_two, &analysis_task_data[6], 3);
    memcpy(task_state.area_buf_two, &analysis_task_data[9], 3);

    memset(task_state.color_buf, 0, sizeof(task_state.color_buf));   // 先清

    uint8_t i = 0,j = 0,k = 0,e = 0,f = 0;
    // 第一组整理顺序
    for (i = 0; i < 3; i++) {
        if (task_state.area_buf_one[i] == AREA_1) {
            task_state.color_buf[0] = task_state.color_buf_one[i];
        }
        else if (task_state.area_buf_one[i] == AREA_2) {
            task_state.color_buf[1] = task_state.color_buf_one[i];
        }
        else if (task_state.area_buf_one[i] == AREA_3) {
            task_state.color_buf[2] = task_state.color_buf_one[i];
        }
    }
    // 第二组整理顺序
    for (j = 0; j < 3; j++) {
        if (task_state.area_buf_two[j] == AREA_1) {
            task_state.color_buf[3] = task_state.color_buf_two[j];
        }
        else if (task_state.area_buf_two[j] == AREA_2) {
            task_state.color_buf[4] = task_state.color_buf_two[j];
        }
        else if (task_state.area_buf_two[j] == AREA_3) {
            task_state.color_buf[5] = task_state.color_buf_two[j];
        }
    }

    // 六个槽任意一个没填上（或值为 0）就算解析失败
    for (k = 0; k < 6; k++) {
        if (task_state.color_buf[k] < TASK_RED || task_state.color_buf[k] > TASK_LOW_BLUE)
            return 0;      // 失败
    }

    // 计算第二批的颜色 → 第一批的位置
    // 用 color_buf[3+f]（第二组按位置排好序）而不是 color_buf_two[e]（扫描原始顺序）
    for (e = 0; e < 3; e++) {          // e = put_num = 环位置
        task_state.turntable_seq[e] = 0;
        for (f = 0; f < 3; f++) {      // f = 转盘槽
            if (task_state.color_buf[3 + f] == task_state.color_buf[e]) {
                task_state.turntable_seq[e] = f;
                break;
            }
        }
    }
    return 1;              // 成功
}


// 陀螺仪处理任务
void imu_task(void) {
    // 处理陀螺仪数据
    if(hwt101_rx_flag || hwt101_error_flag) {

        // 先清除标志位
        // 错误标志由驱动清除  提前清零会跳过DMA恢复分支
        // hwt101_error_flag = 0;
        hwt101_rx_flag = 0;

        hwt101_rx_handle();
    }
}

// 摄像头处理任务
void cam_task(void) {
    if (cam_rx_flag || cam_error_flag) {
        // 保留错误标志给cam_rx_handle处理  接收通知仍先清除
        // cam_error_flag = 0;
        cam_rx_flag = 0;  // 先清除标志位
        cam_rx_handle();
    }
    maix_result_task();  // 维护5帧VALID/STABLE 并处理视觉数据超时

    send_conduct_task();      // 给maix发数据处理任务
}
// // OpenMV扫码处理任务
// void mv_task(void)
// {
//     if (cam_rx_flag || cam_error_flag) {
//         cam_rx_flag = 0;     // 清除接收通知
//         printf("123456789");
//         cam_rx_handle();     // 错误标志由驱动处理
//         printf("987654321");
//     }
//
//     // 收到完整任务码后  读取并打印一次
//     openmv_qr_task();
// }


// OpenMV -> F407 单项测试：只发送B1并等待一帧BB/LEN=15
void mv_task(void)
{
    if (openmv_start != 1 && openmv_end == 1) return;

    // 启动扫码并获取任务
    if (openmv_start == 1) {
        // 停止扫码并清空USART1残留
        openmv_send_stop_scan();
        // printf("[OpenMV] 停止扫码并清空USART1残留\r\n");
        service_for(STOP_GUARD_MS);
        cam_empty_rx();

        // 开始扫码
        printf("[OpenMV] 发送B1启动扫码\r\n");
        openmv_send_start_scan();
        wait_qr();

        // 立刻解析
        if (task_state.openmv_task_get_ok == 1) {
            uint8_t r = task_receive();
            printf("task_receive=%d\r\n", r);
            if (r == 1) {
                task_state.send_task_hmi_ok = 1;
                scan_send_task_hmi();
            }
        }

        openmv_send_stop_scan();
        service_for(STOP_GUARD_MS);
        cam_empty_rx();
        openmv_start = 0;
        openmv_end = 1;
    }

    scan_send_task_hmi();     // 给串口屏发送任务（写死，后续要删除）
}

// 给maix发数据处理任务
void send_conduct_task(void)
{
    if (my_control.is_send_color == 1) {
        send_color_conduct();
        my_control.is_send_color = 0;
    }
    if (my_control.is_send_circle == 1) {
        send_circle_conduct();
        my_control.is_send_circle = 0;
    }
    if (my_control.is_send_stack == 1) {
        send_stack_conduct();
        my_control.is_send_stack = 0;
    }
}

// 障碍物检测任务
uint8_t obstacle_find_task(float car_x, float car_y, float car_yaw)
{
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

// 非阻塞式发送小车当前状态至串口屏任务
void hmi_get_car_state_task(void)
{
    static uint8_t hmi_send_step = 0;

    // 未达到发送条件 退出
    if (hmi_state_send == 0) {
        return;
    }

    switch (hmi_send_step)
    {
        case 0:
            hmi_SetValue("yaw",(int)(my_yaw * 100.0f));
            hmi_send_step = 1;
            break;
        case 1:     // 进入冷却
            hmi_send_cold = 1;
            hmi_send_step = 2;
            break;
        case 2:     // 冷却结束后
            if (hmi_send_cold == 0) {
                hmi_SetValue("X",(int)(task_state.cur_x));
                hmi_send_cold = 1;  // 再次进入冷却，等待下个数据发送
                hmi_send_step = 3;
            }
            break;
        case 3:
            if (hmi_send_cold == 0) {
                hmi_SetValue("Y",(int)(task_state.cur_y));
                hmi_send_cold = 1;  // 再次进入冷却，等待下个数据发送
                hmi_send_step = 4;
            }
            break;
        case 4:
            if (hmi_send_cold == 0) {
                hmi_SetValue("X_",(int)(my_control.head_x));
                hmi_send_cold = 1;  // 再次进入冷却，等待下个数据发送
                hmi_send_step = 5;
            }
            break;
        case 5:
            if (hmi_send_cold == 0) {
                hmi_SetValue("Y_",(int)(my_control.head_y));
                hmi_send_step = 6;
            }
            break;
        case 6:
            hmi_send_step = 0;
            hmi_state_send = 0;
            hmi_send_cold = 0;
            break;
        default:
            break;
    }
}

// 读取实际位姿并更新雷达地图  位姿无效时仍接收雷达并清除待确认次数
uint8_t map_obstacle_pose_task(void) {
    map_pose_t pose;
    uint8_t valid = map_pose_get(HAL_GetTick(), &pose);
    return map_obstacle_task(pose.x_mm, pose.y_mm, pose.yaw_deg, valid);
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
