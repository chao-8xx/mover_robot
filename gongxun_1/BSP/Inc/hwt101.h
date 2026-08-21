#ifndef GONGXUN_1_HWT101_H
#define GONGXUN_1_HWT101_H

#include "headfile.h"

// HWT101 解析结果（角度单位 °，角速度单位 °/s，加速度单位 g）
typedef struct {
    float roll;         // 横滚角
    float pitch;        // 俯仰角
    float yaw;          // 偏航角
    float wx;           // X 轴角速度
    float wy;           // Y 轴角速度
    float wz;           // Z 轴角速度（偏航角速度）
    float ax;           // X 轴加速度
    float ay;           // Y 轴加速度
    float az;           // Z 轴加速度
    float yaw_accel;    // 偏航角加速度（由 wz 差分得到，单位 °/s²）
    uint32_t last_update_ms;  // 最近一次收到有效帧的时间
    uint8_t valid;            // 是否已经收到过有效数据
} HWT101_Data_t;

extern HWT101_Data_t hwt101;

// 初始化：保存串口句柄并开始 DMA 环形接收（IDLE 事件回调解析）
void hwt101_init(UART_HandleTypeDef *huart);

// DMA 收到一段数据后由 HAL_UARTEx_RxEventCallback 转发到这里
void hwt101_dma_rx_event(uint16_t size);

// 串口错误回调：复位解析状态并重新开始接收
void hwt101_uart_error_callback(UART_HandleTypeDef *huart);

// 获取偏航角（°）
float hwt101_get_yaw(void);

// 获取偏航角速度（°/s）
float hwt101_get_yaw_rate(void);

// 获取偏航角加速度（°/s²）
float hwt101_get_yaw_accel(void);

// 是否已经收到过有效陀螺仪数据
uint8_t hwt101_data_valid(void);

// 复位偏航角（非阻塞，只置标志；实际指令由 hwt101_tick 在 1ms 中断里发送）
void hwt101_yaw_reset(void);

// 中断安全版复位请求：可在任意中断里调用，不会阻塞
void hwt101_yaw_reset_request(void);

// 1ms 定时中断里调用，驱动“解锁→归零→保存”非阻塞状态机
void hwt101_tick(void);

// HWT101初始化（串口3） 初始化加偏航角复位
void my_hwt101_init(void);

// 全局变量
extern float yaw_rate;  // 全局偏航角角速度
extern float my_yaw;    // 全局偏航角角度

#endif //GONGXUN_1_HWT101_H
