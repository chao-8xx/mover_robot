#ifndef GONGXUN_1_MAP_LIDAR_H
#define GONGXUN_1_MAP_LIDAR_H

// 筛选小柱候选，用对应实际点更新道路；位姿无效时拒绝写入
uint8_t map_lidar_handle(uint8_t pose_valid);

// 桌面联调可指定观察框，只接收完整落在框内的候选；area=NULL表示不限制选区
// 观察框不是道路畅通证明，不自动放行
uint8_t map_lidar_handle_area(uint8_t pose_valid, const map_rect_t *area);

#endif // GONGXUN_1_MAP_LIDAR_H
