#ifndef GONGXUN_1_OBSTACLE_MAP_H
#define GONGXUN_1_OBSTACLE_MAP_H

#include <stdbool.h>
#include <stdint.h>

#define OBSTACLE_MAP_INVALID_CELL 0xFFU

typedef struct
{
    uint8_t grid_x;
    uint8_t grid_y;
    uint16_t road_mask;
} ObstacleMapProjection;

/* 初始化/清空障碍物地图，并把已知赛道恢复为可通行状态。 */
void obstacle_map_init(void);
void obstacle_map_reset(void);

/* 只计算障碍点位于哪个栅格/道路，不修改地图状态。 */
bool obstacle_map_project_world_point(float x_mm,
                                      float y_mm,
                                      ObstacleMapProjection *projection);

/*
 * 把 map 坐标系中的一个障碍点写入地图。
 * 返回 false：坐标无效或位于 2400 mm x 2400 mm 场地之外。
 * 返回 true ：栅格已标记为占用；road_mask 为受影响道路，禁行区中为 0。
 */
bool obstacle_map_mark_world_point(float x_mm,
                                   float y_mm,
                                   ObstacleMapProjection *projection);

#endif // GONGXUN_1_OBSTACLE_MAP_H
