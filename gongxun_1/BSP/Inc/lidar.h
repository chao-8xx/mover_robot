#ifndef __AX_LIDAR_H
#define __AX_LIDAR_H

#include "headfile.h"

extern uint16_t lidar_data[360];    // 雷达数据数组，存储360度的距离值

void lidar_init(UART_HandleTypeDef *huart);
void lidar_start(void);
void lidar_stop(void);
void lidar_change(void);
void lidar_data_handle(void);
void lidar_process(void);

#endif
