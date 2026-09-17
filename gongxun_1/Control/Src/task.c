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
