#ifndef MAP_NAV_HOST_HEADFILE_H
#define MAP_NAV_HOST_HEADFILE_H

// 复用实际地图与位姿源码  只替代时钟和底盘接口
#include "../map_include/headfile.h"
#include "map_pose.h"
#include "map_tf2d.h"
#include "chassis.h"
#include "map_nav.h"

uint32_t HAL_GetTick(void);
extern float my_yaw;

#endif
