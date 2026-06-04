//
// Created by mechreuo on 2026/6/3.
//

#ifndef GONGXUN_1_CAMERA_H
#define GONGXUN_1_CAMERA_H

#include "headfile.h"

uint8_t check_sum(uint8_t cmd, uint8_t len, const uint8_t *p);
void byte_handle(uint8_t byte);

void openmv_uart_init(UART_HandleTypeDef *huart);
void openmv_uart_rx_callback(UART_HandleTypeDef *huart);
void openmv_uart_error_callback(UART_HandleTypeDef *huart);
void openmv_send_start_scan(void);
void openmv_qr_task(void);


#endif //GONGXUN_1_CAMERA_H