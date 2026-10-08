#include "headfile.h"

// 车辆总任务状态
Task_State task_state;

// ==== 定义 ==== //
#define ANALYSIS_LEN        12                  // 解析数组长度
uint8_t analysis_task_data[ANALYSIS_LEN];       // 解析完后的数组
uint8_t openmv_start = 0;                       // openmv启动标志
uint8_t openmv_end = 0;                         // openmv结束标志
uint8_t qr_ready = 0;                           // 0=无 1=解析成功 2=失败/超时

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

    // 没码时 手动设置任务 后续可删 默认任务
    task_state.color_buf_one[0] = TASK_RED; // 第一组 红绿蓝
    task_state.color_buf_one[1] = TASK_GREEN;
    task_state.color_buf_one[2] = TASK_BLUE;
    task_state.area_buf_one[0] = AREA_1;
    task_state.area_buf_one[1] = AREA_2;
    task_state.area_buf_one[2] = AREA_3;
    task_state.color_buf_two[0] = TASK_YELLOW; // 第二组 黄黑浅蓝
    task_state.color_buf_two[1] = TASK_BLACK;
    task_state.color_buf_two[2] = TASK_LOW_BLUE;
    task_state.area_buf_two[0] = AREA_1;
    task_state.area_buf_two[1] = AREA_2;
    task_state.area_buf_two[2] = AREA_3;
    task_state.color_buf[0] = TASK_RED;      // 总顺序
    task_state.color_buf[1] = TASK_GREEN;
    task_state.color_buf[2] = TASK_BLUE;
    task_state.color_buf[3] = TASK_YELLOW;
    task_state.color_buf[4] = TASK_BLACK;
    task_state.color_buf[5] = TASK_LOW_BLUE;
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

    uint8_t i = 0,j = 0;
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

    // ★ 六个槽任意一个没填上（或值为 0）就算解析失败
    for (uint8_t k = 0; k < 6; k++) {
        if (task_state.color_buf[k] < TASK_RED || task_state.color_buf[k] > TASK_LOW_BLUE)
            return 0;      // 失败
    }
    return 1;              // 成功
}


// 陀螺仪处理任务
void imu_task(void)
{
    // 处理陀螺仪数据
    if(hwt101_rx_flag || hwt101_error_flag) {
        // 先清除标志位
        hwt101_error_flag = 0;
        hwt101_rx_flag = 0;
        hwt101_rx_handle();
    }
}

// 摄像头处理任务
void cam_task(void)
{
    if (cam_rx_flag || cam_error_flag) {
        cam_error_flag = 0;
        cam_rx_flag = 0;  // 先清除标志位
        cam_rx_handle();
    }
    maix_result_task();  // 维护5帧VALID/STABLE 并处理视觉数据超时
}

// openmv扫码启动中止任务 放主循环
void openmv_work_task(void)
{
    static uint8_t  scan_wait = 0;
    static uint16_t wait_ms = 0;
    static uint8_t  retry = 0;

    // 请求开始扫码
    if (openmv_start) {
        openmv_start = 0;
        openmv_send_start_scan();     // 内部已清 qr_ok
        wait_ms = 0; retry = 0;
        scan_wait = 1;
    }

    // 等待二维码帧
    if (scan_wait) {
        qr_result_t qr;
        if (openmv_get_qr_result(&qr)) {     // 收到一帧 BB
            if (task_receive() == 1) {       // 立刻用 qr_data 解析进 task_state
                qr_ready = 1;
                scan_wait = 0;
            }
            else {
                qr_ready = 2;
            }
            openmv_send_stop_scan();         // 收到就停扫码，省得干扰后面
        }
        else if (++wait_ms >= 5000) {         // 超时
            wait_ms = 0;
            if (++retry < 3) {
                openmv_send_start_scan();    // 重发一次
            } else {
                openmv_send_stop_scan();
                qr_ready = 2;                // 失败，交给 control 决定怎么办
                scan_wait = 0;
            }
        }
    }

    // 主动要求停止
    if (openmv_end) {
        openmv_end = 0;                      // ★ 清自己，不是清 openmv_start
        openmv_send_stop_scan();
        scan_wait = 0;
    }
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
    if (my_control.is_send_onup == 1) {
        send_onup_conduct();
        my_control.is_send_onup = 0;
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
