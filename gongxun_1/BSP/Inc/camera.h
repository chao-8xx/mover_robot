//
// Created by mechreuo on 2026/6/3.
//

#ifndef GONGXUN_1_CAMERA_H
#define GONGXUN_1_CAMERA_H

#include "headfile.h"

// 颜色指令码
#define MAIX_COLOR_NONE       0xC0
#define MAIX_COLOR_RED        0xC1
#define MAIX_COLOR_YELLOW     0xC2
#define MAIX_COLOR_BLUE       0xC3
#define MAIX_COLOR_GREEN      0xC4
#define MAIX_COLOR_BLACK      0xC5
#define MAIX_COLOR_LIGHTBLUE  0xC6

// 圆环数字指令码
#define MAIX_RING_NONE 0xD0
#define MAIX_RING_1    0xD1
#define MAIX_RING_2    0xD2
#define MAIX_RING_3    0xD3

// maix 定长通信协议帧
typedef struct
{
    uint8_t cmd;        // 命令码
    uint8_t target;     // 目标任务
    uint8_t ok;         // 结果标志位
    int16_t dx;         // X偏差值
    int16_t dy;         // Y偏差值
    uint32_t tick;      // 结果时间戳
    uint8_t new_data;   // 是否有新数据
} maix_vision_t;

extern maix_vision_t maix_vision;

///////////////////////////////////// 通信协议解析 //////////////////////////////
uint8_t check_sum(uint8_t cmd, uint8_t len, const uint8_t *p);

void openmv_byte_handle(uint8_t byte);
void maix_byte_handle(uint8_t byte);

uint8_t is_maix_cmd(uint8_t cmd);
int16_t read_i16_le(const uint8_t *p);
void maix_swap_rx_buffer(void);
void maix_parse_ready_frame(void);

///////////////////////////////////// openmv扫码任务 //////////////////////////////
void openmv_uart_init(UART_HandleTypeDef *huart);
void openmv_uart_rx_callback(UART_HandleTypeDef *huart);
void openmv_uart_error_callback(UART_HandleTypeDef *huart);
void openmv_send_start_scan(void);
void openmv_send_stop_scan(void);
void openmv_qr_task(void);

//////////////////////////////////// maix 视觉任务 ///////////////////////////////////////////
void maix_uart_init(UART_HandleTypeDef *huart);
void maix_uart_rx_callback(UART_HandleTypeDef *huart);
void maix_uart_error_callback(UART_HandleTypeDef *huart);
void maix_send_frame(uint8_t cmd, const uint8_t *data, uint8_t len);
void maix_stop(void);
void maix_start_blob(uint8_t color);
void maix_start_ring(uint8_t number);
void maix_start_onup(uint8_t color);
uint8_t maix_get_result(maix_vision_t *out);

#endif //GONGXUN_1_CAMERA_H
