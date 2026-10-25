#ifndef PERCEPTION_HOST_HEADFILE_H
#define PERCEPTION_HOST_HEADFILE_H

// 本机测试只替代整车头文件，使用实际点云、聚类、地图源码，不连接电机或串口
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
typedef struct { uint8_t unused; } UART_HandleTypeDef;
#include "lidar.h"
#include "map_tf2d.h"
#include "map_obs.h"
#include "map_track.h"
#include "map_data.h"
#include "map_update.h"
#include "map_plan.h"
#include "map_lidar.h"
#include "map_pose.h"
#include "chassis.h"
#include "map_nav.h"
extern float my_yaw;
uint32_t HAL_GetTick(void);

#endif
