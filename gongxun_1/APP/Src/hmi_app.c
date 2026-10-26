#include "headfile.h"

// 发车点坐标，由屏幕 CC 命令设置，供后续路径规划使用
uint8_t hmi_start_x = 0;
uint8_t hmi_start_y = 0;
uint8_t hmi_lift_state = 0;     // 串口屏控制升降标志  1：可接收指令
uint8_t hmi_stretch_state = 0;  // 串口屏控制伸缩标志  1：可接收指令
int16_t hmi_lift_mm = 0;        // 串口屏控制升降距离 mm
int16_t hmi_stretch_mm = 0;     // 串口屏控制伸缩距离 mm

// 把小车状态显示在串口屏上 便于调试
uint8_t hmi_state_wait = 200;     // 整体状态发送间隔时间 ms
uint8_t hmi_state_send = 0;       // 发送状态标志位 1:发送
uint8_t hmi_send_cold = 0;        // 是否进入冷却 1:是
uint8_t hmi_send_cold_wait = 5;   // 冷却时间

// 急停状态
uint8_t hmi_rapid_stop = 0;       // 1:触发急停 所有电机急停

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
    hmi_lift_state = 0;
    hmi_stretch_state = 0;
    // 沿用设置发车点时建立的轮子基准  不在离开启停区前丢掉这段位移
}

// 触摸串口屏停车事件
void hmi_stop(void)
{
    car_run = false;
    my_control.step = 99;
    hmi_rapid_stop = 1;
}

// 触摸串口屏事件总处理
// 在hmi.c的hmi_rx_callback回调函数内调用
void hmi_solve(UART_HandleTypeDef *huart, uint16_t Size)
{
    hmi_touch_t touch;
    // 循环调用 hmi_GetTouch
    if (hmi_rxbuffer[0] == 0x65) {
        for (uint16_t i = 0; i < Size; i++) {
            touch = hmi_GetTouch(hmi_rxbuffer[i]);

            if (touch.is_valid && touch.event_type == 0x00) {   // 事件有效且按键弹起
                if ((touch.page_id == 0x01 && touch.obj_id == 0x06) ||
                    (touch.page_id == 0x03 && touch.obj_id == 0x02) ||
                    (touch.page_id == 0x04 && touch.obj_id == 0x02)) {
                    hmi_stop();     // 停车事件触发
                    }
                else if (touch.page_id == 0x01 && touch.obj_id == 0x04) {
                    hmi_start();    // 发车事件触发
                }
                else if (touch.page_id == 0x06 && touch.obj_id == 0x0F) {
                    hwt101_yaw_reset();     // 陀螺仪偏航角复位
                    my_yaw = 0.0f;
                }
            }
        }
    }
    else if (hmi_rxbuffer[0] == 0xBB) {     // 设置障碍坐标事件
        // 帧格式：BB x y FF FF FF = 设置 (x,y) 为障碍；BB EE EE FF FF FF = 清除所有障碍坐标
        if (Size >= 6 && hmi_rxbuffer[Size - 1] == 0xFF &&
            hmi_rxbuffer[Size - 2] == 0xFF && hmi_rxbuffer[Size - 3] == 0xFF) {
            uint8_t x = hmi_rxbuffer[1];
            uint8_t y = hmi_rxbuffer[2];
            if (x == 0xEE && y == 0xEE) {
                bfs_obstacle_clear();       // 清除所有障碍
            }
            else if (x < BFS_MAP_SIZE && y < BFS_MAP_SIZE) {
                bfs_set_obstacle(x, y, 1);  // 标记为障碍
            }
        }
    }
    else if (hmi_rxbuffer[0] == 0xCC) {     // 设置发车坐标事件
        // 帧格式：CC 00 FF FF FF = 发车点 (0,0)；CC 01 FF FF FF = 发车点 (4,0)
        // 上面保留旧比赛的格子编号；实际地图坐标以赛场图左下角为原点，X向右、Y向上
        if (Size >= 5 && hmi_rxbuffer[Size - 1] == 0xFF &&
            hmi_rxbuffer[Size - 2] == 0xFF && hmi_rxbuffer[Size - 3] == 0xFF) {
            if (hmi_rxbuffer[1] == 0x00) {  // 启停区2
                hmi_start_x = 0;
                hmi_start_y = 0;
                chassis_odom_origin_set(15.0f, 15.0f);  // 左下启停区2中心 cm
                task_state.task_location = PARKING_2;
                start_location = 0; // 0:启停区2发车
            }
            else if (hmi_rxbuffer[1] == 0x01) { // 启停区1
                hmi_start_x = 4;
                hmi_start_y = 0;
                chassis_odom_origin_set(240.0f - 15.0f, 15.0f);  // 右下启停区1中心 cm
                task_state.task_location = PARKING_1;
                start_location = 1; // 1:启停区1发车
            }
            // 设置起始坐标状态
            task_state.cur_x = hmi_start_x;
            task_state.cur_y = hmi_start_y;
        }
    }
    else if (hmi_rxbuffer[0] == 0xDD) {     // 串口屏控制抓取装置伸缩升降 低字节在前，高字节在后
        if (hmi_rxbuffer[1] == 0x11) {      // 升降
            // 帧格式：DD 11 01(1正0负) mm mm FF FF FF    （mm低8位在前，高8位在后）
            if (Size >= 8 && hmi_rxbuffer[Size - 1] == 0xFF &&
            hmi_rxbuffer[Size - 2] == 0xFF && hmi_rxbuffer[Size - 3] == 0xFF) {
                static int16_t mm = 0;
                mm = (int16_t)(hmi_rxbuffer[3] | (hmi_rxbuffer[4] << 8));
                if (hmi_rxbuffer[2] == 0x01) { // 为正
                    hmi_lift_mm = (int16_t)mm;
                    hmi_lift_state = 1;
                }
                else if (hmi_rxbuffer[2] == 0x00) { // 为负
                    hmi_lift_mm = (int16_t)-mm;
                    hmi_lift_state = 1;
                }
            }
        }
        else if (hmi_rxbuffer[1] == 0x22) {  // 伸缩
            // 帧格式：DD 22 01(1正0负) mm mm FF FF FF
            if (Size >= 8 && hmi_rxbuffer[Size - 1] == 0xFF &&
            hmi_rxbuffer[Size - 2] == 0xFF && hmi_rxbuffer[Size - 3] == 0xFF) {
                static int16_t mm = 0;
                mm = (int16_t)(hmi_rxbuffer[3] | (hmi_rxbuffer[4] << 8));
                if (hmi_rxbuffer[2] == 0x01) { // 为正
                    hmi_stretch_mm = (int16_t)mm;
                    hmi_stretch_state = 1;
                }
                else if (hmi_rxbuffer[2] == 0x00) { // 为负
                    hmi_stretch_mm = (int16_t)-mm;
                    hmi_stretch_state  = 1;
                }
            }
        }
        else if (hmi_rxbuffer[1] == 0x33) { // 舵机控制  01：云台  02：转盘  03：夹爪
            // 帧格式：DD 33 0x ... FF FF FF
            if (Size >= 7 && hmi_rxbuffer[Size - 1] == 0xFF &&
            hmi_rxbuffer[Size - 2] == 0xFF && hmi_rxbuffer[Size - 3] == 0xFF) {
                if (hmi_rxbuffer[2] == 0x01) {      // 01云台： DD 33 01 00 FF FF FF（00:放置 01:左 02:中 03:右）
                    if (hmi_rxbuffer[3] == 0x00) {  // 放置
                        holder_rotate(HOLDER_PUT);
                    }
                    else if (hmi_rxbuffer[3] == 0x01) { // 左
                        holder_rotate(HOLDER_LEFT);
                    }
                    else if (hmi_rxbuffer[3] == 0x02) { // 中
                        holder_rotate(HOLDER_CATCH);
                    }
                    else if (hmi_rxbuffer[3] == 0x03) { // 右
                        holder_rotate(HOLDER_RIGHT);
                    }
                }
                else if (hmi_rxbuffer[2] == 0x02) { // 02转盘：DD 33 02 00 FF FF FF（01:一号位 02:二号位 03:三号位 04:安全位）
                    if (hmi_rxbuffer[3] == 0x00) {  // 安全位置
                        turntable_rotate(SAFE_LOCATION);
                    }
                    else if (hmi_rxbuffer[3] == 0x01) { // 一号位
                        turntable_rotate(ONE_LOCATION);
                    }
                    else if (hmi_rxbuffer[3] == 0x02) { // 二号位
                        turntable_rotate(TWO_LOCATION);
                    }
                    else if (hmi_rxbuffer[3] == 0x03) { // 三号位
                        turntable_rotate(THREE_LOCATION);
                    }
                }
                else if (hmi_rxbuffer[2] == 0x03) { // 03夹爪：DD 33 03 00 FF FF FF（00:松开 01:抓取）
                    if (hmi_rxbuffer[3] == 0x00) {  // 松开
                        claw_ctrl(CLAW_LOOSE);
                    }
                    else if (hmi_rxbuffer[3] == 0x01) { // 夹取
                        claw_ctrl(CLAW_CATCH);
                    }
                }
                else if (hmi_rxbuffer[2] == 0x05) { // DD 33 05 00 FF FF FF舵机回零，除了夹爪保持闭合
                    holder_rotate(HOLDER_PUT);       // 云台舵机放置物料位置
                    turntable_rotate(SAFE_LOCATION); // 转盘舵机安全位置
                    claw_ctrl(CLAW_CATCH);           // 夹爪夹紧
                }
            }
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
        // 伸缩每条指令隔3ms
        stretch_time_gap ++;
        if (stretch_time_gap >= 3) {
            // catch_stretch_dist((float)hmi_stretch_mm/10.0f);    // 丝杆
            if (hmi_stretch_mm > 0 && hmi_stretch_mm < 350) {
                catch_stretch_distance((float)hmi_stretch_mm/10.0f);        // 齿轮齿条
            }
            stretch_time_gap = 0;
            hmi_stretch_state = 0;
        }
    }

    // 发送状态计时
    if (hmi_state_send == 0) {
        hmi_state_wait --;
        if (hmi_state_wait <= 0) {
            hmi_state_wait = 200;
            hmi_state_send = 1;
        }
    }
    // 发送冷却计时
    if (hmi_send_cold == 1) {
        hmi_send_cold_wait --;
        if (hmi_send_cold_wait <= 0) {
            hmi_send_cold_wait = 5;
            hmi_send_cold = 0;
        }
    }

    // 急停状态
    if (hmi_rapid_stop == 1) {
        Emm_V5_Stop_Now(0,false);    // 直接发送急停指令
    }
}



// hmi_SetValue("yaw",(int)(my_yaw * 100.0f));
