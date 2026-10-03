#ifndef HOST_CLUSTER_HEADFILE_H
#define HOST_CLUSTER_HEADFILE_H

#include <stdint.h>
#include <string.h>
#include <math.h>

// 主机验证只替换硬件头文件，点云、聚类和坐标变换仍编译工程原文件。
typedef struct { uint8_t unused; } UART_HandleTypeDef;
#include "tf2d.h"
#include "lidar.h"
#include "obstacle_find.h"

#endif
