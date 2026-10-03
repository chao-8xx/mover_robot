#ifndef GONGXUN_1_MAP_LIDAR_H
#define GONGXUN_1_MAP_LIDAR_H

#include <stdint.h>

// 把已生成的雷达聚类写入动态地图；位姿无效时拒绝写入
uint8_t map_lidar_handle(uint8_t pose_valid);

#endif // GONGXUN_1_MAP_LIDAR_H
