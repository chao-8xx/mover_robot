#ifndef GONGXUN_MAP_HOST_HEADFILE_H
#define GONGXUN_MAP_HOST_HEADFILE_H

// 主机检查只提供类型与标准库，算法仍编译 APP / Test 中的原始 C 文件。
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
#include "map_stage_test.h"

#endif
