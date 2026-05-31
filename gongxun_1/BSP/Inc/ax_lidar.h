#ifndef __AX_LIDAR_H
#define __AX_LIDAR_H

#include "headfile.h"

extern uint16_t ax_lidar_data[360];

void AX_LIDAR_Init(UART_HandleTypeDef *huart);
void AX_LIDAR_Start(void);
void AX_LIDAR_Stop(void);
void LD_DataHandle(void);
void AX_LIDAR_RxCallback(UART_HandleTypeDef *huart);

#endif