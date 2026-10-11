#include "headfile.h"

// 发车点坐标，由屏幕 CC 命令设置，供后续路径规划使用
uint8_t hmi_start_x = 0;
uint8_t hmi_start_y = 0;
uint8_t hmi_lift_state = 0;     // 串口屏控制升降标志  1：可接收指令
uint8_t hmi_stretch_state = 0;  // 串口屏控制伸缩标志  1：可接收指令
int16_t hmi_lift_mm = 0;        // 串口屏控制升降距离 mm
int16_t hmi_stretch_mm = 0;     // 串口屏控制伸缩距离 mm

// 串口屏初始化
void my_hmi_init(void)
{
    hmi_init(&huart5);
}

// 触摸串口屏事件
void hmi_touch(void) {
    // led_toggle();
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
}

// 触摸串口屏发车事件
void hmi_start(void)
{
    car_run = true;
    if (car_run) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_RESET);
    }

}

// 触摸串口屏停车事件
void hmi_stop(void)
{
    car_run = false;
    if (!car_run) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_SET);
    }
}

// 完整帧在下面统一检查，这里只处理指令内容
static void hmi_frame_handle(const uint8_t *frame)
{
    if (frame[0] == 0x65) {         // 触摸事件：65 页码 控件 事件 FF FF FF
        if (frame[3] != 0x00) return; // 只处理按键弹起
        if ((frame[1] == 0x01 && frame[2] == 0x06) ||
            (frame[1] == 0x03 && frame[2] == 0x02) ||
            (frame[1] == 0x04 && frame[2] == 0x02)) {
            hmi_stop();
        }
        else if (frame[1] == 0x01 && frame[2] == 0x04) {
            hmi_start();
            car_reset = false;     // 置未复位标志，发车前复位一次
        }
        else if (frame[1] == 0x06 && frame[2] == 0x0F) {
            hwt101_yaw_reset();
            my_yaw = 0.0f;
        }
    }
    else if (frame[0] == 0xBB) {    // BB x y FF FF FF；EE EE 清除所有障碍
        uint8_t x = frame[1];
        uint8_t y = frame[2];
        if (x == 0xEE && y == 0xEE) {
            bfs_obstacle_clear();
        }
        else if (x < BFS_MAP_SIZE && y < BFS_MAP_SIZE) {
            bfs_set_obstacle(x, y, 1);
        }
    }
    else if (frame[0] == 0xCC) {    // CC 00/01 FF FF FF：选择发车点
        if (frame[1] > 0x01) return;
        hmi_start_x = frame[1] ? 4 : 0;
        hmi_start_y = 0;
        task_state.task_location = frame[1] ? PARKING_1 : PARKING_2;
        start_location = frame[1];
        task_state.cur_x = hmi_start_x;
        task_state.cur_y = hmi_start_y;
    }
    else if (frame[0] == 0xDD) {    // DD 11/22 正负 距离低字节 高字节 FF FF FF
        uint16_t mm = (uint16_t)(frame[3] | ((uint16_t)frame[4] << 8));
        if (frame[2] > 0x01 || mm > 32767U) return;
        int16_t distance = frame[2] ? (int16_t)mm : -(int16_t)mm;
        if (frame[1] == 0x11) {     // 升降
            hmi_lift_mm = distance;
            hmi_lift_state = 1;
        }
        else if (frame[1] == 0x22) { // 伸缩
            hmi_stretch_mm = distance;
            hmi_stretch_state = 1;
        }
    }
}

// 只用一个缓存和下标拼帧，支持半条指令和一次收到多条指令
static uint8_t hmi_frame[8];
static uint8_t hmi_frame_idx = 0;

void hmi_rx_reset(void) {
    hmi_frame_idx = 0;
}

void hmi_solve(UART_HandleTypeDef *huart, uint16_t Size) {
    (void)huart;
    if (Size > HMI_RX_SIZE) return;

    for (uint16_t i = 0; i < Size; i++) {
        uint8_t byte = hmi_rxbuffer[i];
        uint8_t frame_len;
        // 长度由帧头确定，不另外保存长度和时间标志
        switch (hmi_frame_idx ? hmi_frame[0] : byte) {
            case 0x65: frame_len = 7; break;
            case 0xBB: frame_len = 6; break;
            case 0xCC: frame_len = 5; break;
            case 0xDD: frame_len = 8; break;
            default: continue;
        }
        hmi_frame[hmi_frame_idx++] = byte;
        if (hmi_frame_idx == frame_len) {
            if (hmi_frame[frame_len - 1] == 0xFF &&
                hmi_frame[frame_len - 2] == 0xFF &&
                hmi_frame[frame_len - 3] == 0xFF) {
                hmi_frame_handle(hmi_frame);
            }
            hmi_rx_reset();
        }
    }
}

// 串口屏控制发送指令  放入1ms中断
void hmi_control_send_tick(void)
{
    // 上一帧还没发完就等下一拍，保证串口上有真正的帧间隔
    if (Emm_V5_Tx_Count() != 0) {
        return;
    }
    static uint8_t lift_time_gap = 0;
    static uint8_t stretch_time_gap = 0;

    // 升降
    if (hmi_lift_state) {
        // 升降每条指令隔3ms
        lift_time_gap ++;
        if (lift_time_gap >= 3) {
            catch_lift((float)hmi_lift_mm/10.0f);
            lift_time_gap = 0;
            hmi_lift_state = 0;
        }
    }
    // 伸缩
    if (hmi_stretch_state) {
        // 升降每条指令隔3ms
        stretch_time_gap ++;
        if (stretch_time_gap >= 3) {
            catch_stretch_pos((float)hmi_stretch_mm/10.0f);
            stretch_time_gap = 0;
            hmi_stretch_state = 0;
        }
    }
}

// hmi_SetValue("yaw",(int)(my_yaw * 100.0f));
