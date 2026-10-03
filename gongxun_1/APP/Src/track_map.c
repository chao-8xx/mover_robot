//
// Created by mechreuo on 2026/9/23.
//

#include "track_map.h"

// N0~N2 在上方，N6~N8 在下方；坐标单位 mm
static const map_node_t map_nodes[MAP_NODE_COUNT] = {
    {275.0f, 2125.0f}, {1200.0f, 2125.0f}, {2125.0f, 2125.0f},
    {275.0f, 1200.0f}, {1200.0f, 1200.0f}, {2125.0f, 1200.0f},
    {275.0f, 275.0f},  {1200.0f, 275.0f},  {2125.0f, 275.0f}
};

// R0~R5 横向，R6~R11 纵向；与下面 5x5 的道路编号一致
static const map_road_t map_roads_info[12] = {
    {0, 1}, {1, 2}, {3, 4}, {4, 5}, {6, 7}, {7, 8},
    {0, 3}, {3, 6}, {1, 4}, {4, 7}, {2, 5}, {5, 8}
};



// 静态地图上的禁入区域
static const map_rect_t map_notgo_areas[MAP_FORBIDDEN_COUNT] = {
    { 550.0f, 1000.0f, 550.0f, 1000.0f },   // 左下禁入区
    { 1400.0f, 1850.0f, 550.0f, 1000.0f },  // 右下禁入区
    { 550.0f, 1000.0f, 1400.0f, 1850.0f },  // 左上禁入区
    { 1400.0f, 1850.0f, 1400.0f, 1850.0f }, // 右上禁入区
};

// 第一行对应赛场下方，第一列对应赛场左侧
static const map_cell_t map_cells[MAP_CELL_HEIGHT][MAP_CELL_WIDTH] =
{
    {
        {cell_node, 6}, {cell_road, 4}, {cell_node, 7},
        {cell_road, 5}, {cell_node, 8}
    },
    {
            {cell_road, 7}, {cell_forbidden, MAP_NO_ID},
            {cell_road, 9}, {cell_forbidden, MAP_NO_ID},
            {cell_road, 11}
    },
    {
            {cell_node, 3}, {cell_road, 2}, {cell_node, 4},
            {cell_road, 3}, {cell_node, 5}
    },
    {
            {cell_road, 6}, {cell_forbidden, MAP_NO_ID},
            {cell_road, 8}, {cell_forbidden, MAP_NO_ID},
            {cell_road, 10}
    },
    {
            {cell_node, 0}, {cell_road, 0}, {cell_node, 1},
            {cell_road, 1}, {cell_node, 2}
    }
};

// 单个轴上的毫米坐标转区域编号
// 调用前已经检查坐标位于场地范围内
static uint8_t map_axis_to_cell(float value)
{
    if (value < 550.0f)  return 0;
    if (value < 1000.0f) return 1;
    if (value < 1400.0f) return 2;
    if (value < 1850.0f) return 3;

    return 4;
}

uint8_t map_xy_to_cell(float x, float y,
                       uint8_t *cell_x, uint8_t *cell_y)
{
    if (cell_x == 0 || cell_y == 0)
        return 0;

    if (!(x >= 0.0f && x < MAP_WIDTH &&
          y >= 0.0f && y < MAP_HEIGHT))
        return 0;

    *cell_x = map_axis_to_cell(x);
    *cell_y = map_axis_to_cell(y);

    return 1;
}

uint8_t map_cell_get(uint8_t cell_x, uint8_t cell_y,
                     map_cell_t *cell)
{
    if (cell == 0 ||
        cell_x >= MAP_CELL_WIDTH ||
        cell_y >= MAP_CELL_HEIGHT)
        return 0;

    *cell = map_cells[cell_y][cell_x];
    return 1;
}


// 是否在静态地图内
static uint8_t map_can_in(float x, float y, const map_rect_t *map) {
    if (map == 0) return 0;

    return (x >= map->min_x && x <= map->max_x && y >= map->min_y && y <= map->max_y);
}

// 地图区域判定函数
map_area_t map_check_area(float x, float y) {
    // 出界区域判定
    if (!(x >= 0.0f && x < MAP_WIDTH && y >= 0.0f && y < MAP_HEIGHT)) {
        return map_outside;
    }

    // 禁止进入区域判定
    for (uint8_t area_index = 0; area_index < MAP_FORBIDDEN_COUNT; area_index++) {
        if (map_can_in(x, y, &map_notgo_areas[area_index])) {
            return map_forbidden;
        }
    }

    return map_normal;
}

// 读取节点的全局坐标
uint8_t map_node_get(uint8_t node_id, map_node_t *node) {
    if (node == 0 || node_id >= MAP_NODE_COUNT) return 0;
    *node = map_nodes[node_id];
    return 1;
}

// 读取道路连接的两个节点
uint8_t map_road_info_get(uint8_t road_id, map_road_t *road) {
    if (road == 0 || road_id >= 12) return 0;
    *road = map_roads_info[road_id];
    return 1;
}

// 根据两个相邻节点查找道路，正反方向相同
uint8_t map_road_find(uint8_t node_a, uint8_t node_b, uint8_t *road_id) {
    if (road_id == 0 || node_a >= MAP_NODE_COUNT || node_b >= MAP_NODE_COUNT) return 0;

    for (uint8_t i = 0; i < 12; i++) {
        if ((map_roads_info[i].node_a == node_a && map_roads_info[i].node_b == node_b) ||
            (map_roads_info[i].node_a == node_b && map_roads_info[i].node_b == node_a)) {
            *road_id = i;
            return 1;
        }
    }
    return 0;
}











