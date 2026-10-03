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

// 完整指令分发，接收批次不再当成协议帧
// 由下方流式拼帧入口调用
static void hmi_frame_handle(const uint8_t *frame, uint16_t Size)
{
    hmi_touch_t touch;
    if (Size < 5) return;
    // 循环调用 hmi_GetTouch
    if (frame[0] == 0x65) {
        for (uint16_t i = 0; i < Size; i++) {
            touch = hmi_GetTouch(frame[i]);

            if (touch.is_valid && touch.event_type == 0x00) {   // 事件有效且按键弹起
                if ((touch.page_id == 0x01 && touch.obj_id == 0x06) ||
                    (touch.page_id == 0x03 && touch.obj_id == 0x02) ||
                    (touch.page_id == 0x04 && touch.obj_id == 0x02)) {
                    hmi_stop();     // 停车事件触发
                    }
                else if (touch.page_id == 0x01 && touch.obj_id == 0x04) {
                    hmi_start();    // 发车事件触发
                    if (car_reset) {
                        car_reset = false;  // 置未复位标志，发车前复位一次
                    }
                }
                else if (touch.page_id == 0x06 && touch.obj_id == 0x0F) {
                    hwt101_yaw_reset();     // 陀螺仪偏航角复位
                    my_yaw = 0.0f;
                }
            }
        }
    }
    else if (frame[0] == 0xBB) {     // 设置障碍坐标事件
        // 帧格式：BB x y FF FF FF = 设置 (x,y) 为障碍；BB EE EE FF FF FF = 清除所有障碍坐标
        if (Size >= 6 && frame[Size - 1] == 0xFF &&
            frame[Size - 2] == 0xFF && frame[Size - 3] == 0xFF) {
            uint8_t x = frame[1];
            uint8_t y = frame[2];
            if (x == 0xEE && y == 0xEE) {
                bfs_obstacle_clear();       // 清除所有障碍
            }
            else if (x < BFS_MAP_SIZE && y < BFS_MAP_SIZE) {
                bfs_set_obstacle(x, y, 1);  // 标记为障碍
            }
        }
    }
    else if (frame[0] == 0xCC) {     // 设置发车坐标事件
        // 帧格式：CC 00 FF FF FF = 发车点 (0,0)；CC 01 FF FF FF = 发车点 (4,0)
        if (Size >= 5 && frame[1] <= 0x01 && frame[Size - 1] == 0xFF &&
            frame[Size - 2] == 0xFF && frame[Size - 3] == 0xFF) {
            if (frame[1] == 0x00) {  // 启停区2
                hmi_start_x = 0;
                hmi_start_y = 0;
                task_state.task_location = PARKING_2;
                start_location = 0; // 0:启停区2发车
            }
            else if (frame[1] == 0x01) { // 启停区1
                hmi_start_x = 4;
                hmi_start_y = 0;
                task_state.task_location = PARKING_1;
                start_location = 1; // 1:启停区1发车
            }
            // 设置起始坐标状态
            task_state.cur_x = hmi_start_x;
            task_state.cur_y = hmi_start_y;
        }
    }
    else if (frame[0] == 0xDD) {     // 串口屏控制抓取装置伸缩升降 低字节在前，高字节在后
        if (frame[1] == 0x11) {      // 升降
            // 帧格式：DD 11 01(1正0负) mm mm FF FF FF    （mm低8位在前，高8位在后）
            if (Size >= 8 && frame[Size - 1] == 0xFF &&
            frame[Size - 2] == 0xFF && frame[Size - 3] == 0xFF) {
                uint16_t mm;
                mm = (uint16_t)(frame[3] | ((uint16_t)frame[4] << 8));
                if (mm > 32767U) return; // 超出有符号距离范围，不让正负号溢出
                if (frame[2] == 0x01) { // 为正
                    hmi_lift_mm = (int16_t)mm;
                    hmi_lift_state = 1;
                }
                else if (frame[2] == 0x00) { // 为负
                    hmi_lift_mm = (int16_t)-mm;
                    hmi_lift_state = 1;
                }
            }
        }
        else if (frame[1] == 0x22) {  // 伸缩
            // 帧格式：DD 22 01(1正0负) mm mm FF FF FF
            if (Size >= 8 && frame[Size - 1] == 0xFF &&
            frame[Size - 2] == 0xFF && frame[Size - 3] == 0xFF) {
                uint16_t mm;
                mm = (uint16_t)(frame[3] | ((uint16_t)frame[4] << 8));
                if (mm > 32767U) return; // 超出有符号距离范围，不让正负号溢出
                if (frame[2] == 0x01) { // 为正
                    hmi_stretch_mm = (int16_t)mm;
                    hmi_stretch_state = 1;
                }
                else if (frame[2] == 0x00) { // 为负
                    hmi_stretch_mm = (int16_t)-mm;
                    hmi_stretch_state  = 1;
                }
            }
        }
    }
}

// 按协议长度跨DMA批次拼帧，同时支持一批数据里的多条指令
static uint8_t hmi_frame[8];
static uint8_t hmi_frame_idx = 0;
static uint8_t hmi_frame_len = 0;
static uint32_t hmi_frame_tick = 0;

void hmi_rx_reset(void) {
    hmi_frame_idx = 0;
    hmi_frame_len = 0;
}

void hmi_solve(UART_HandleTypeDef *huart, uint16_t Size) {
    uint32_t tick = HAL_GetTick();
    (void)huart;
    if (Size > HMI_RX_SIZE) return;
    if (hmi_frame_idx && tick - hmi_frame_tick > HMI_FRAME_TIMEOUT_MS) hmi_rx_reset();
    if (Size) hmi_frame_tick = tick;

    for (uint16_t i = 0; i < Size; i++) {
        uint8_t byte = hmi_rxbuffer[i];
        if (hmi_frame_idx == 0) {
            switch (byte) {
                case 0x65: hmi_frame_len = 7; break;
                case 0xBB: hmi_frame_len = 6; break;
                case 0xCC: hmi_frame_len = 5; break;
                case 0xDD: hmi_frame_len = 8; break;
                default: continue;
            }
        }
        hmi_frame[hmi_frame_idx++] = byte;
        if (hmi_frame_idx == hmi_frame_len) {
            if (hmi_frame[hmi_frame_len - 1] == 0xFF &&
                hmi_frame[hmi_frame_len - 2] == 0xFF &&
                hmi_frame[hmi_frame_len - 3] == 0xFF) {
                hmi_frame_handle(hmi_frame, hmi_frame_len);
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
