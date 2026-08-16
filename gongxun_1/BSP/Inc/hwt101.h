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

// 初始化：保存串口句柄并开始单字节中断接收
void hwt101_init(UART_HandleTypeDef *huart);

// 串口接收完成回调（由 uart_dma.c 的 HAL_UART_RxCpltCallback 转发）
void hwt101_uart_rx_callback(UART_HandleTypeDef *huart);

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

// 复位偏航角（将当前偏航角清零）
void hwt101_yaw_reset(void);

// HWT101初始化（串口3） 初始化加偏航角复位
void my_hwt101_init(void);

#endif //GONGXUN_1_HWT101_H
