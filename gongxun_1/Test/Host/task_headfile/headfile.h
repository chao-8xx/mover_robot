#ifndef GONGXUN_TASK_HOST_HEADFILE_H
#define GONGXUN_TASK_HOST_HEADFILE_H

// 保留工程的真实数据结构，硬件函数由 test_task.c 提供可控替身。
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

typedef struct UART_HandleTypeDef UART_HandleTypeDef;
#include "tf2d.h"
#include "lidar.h"
#include "obstacle_find.h"
#include "track_map.h"
#include "map_data.h"
#include "map_update.h"
#include "map_plan.h"
#include "map_lidar.h"
#include "catch.h"
#include "chassis.h"
#include "camera.h"
#include "task.h"

extern volatile uint8_t hwt101_rx_flag;
extern volatile uint8_t hwt101_error_flag;
void hwt101_rx_handle(void);
void control_variate_init(void);
void control_process(void);
uint32_t HAL_GetTick(void);

#endif
