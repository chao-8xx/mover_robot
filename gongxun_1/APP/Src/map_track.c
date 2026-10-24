//
// Created by mechreuo on 2026/9/23.
//

#include "map_track.h"
/**
 *  静态赛道地图数据文件
 *  将2400mm * 2400mm地图划分为5*5的栅格区域 共9节点 12道路 4禁入区
 *  栅格编号从左下角开始  第一行对应赛场下方 第一列对应赛场左侧
 */

// 9节点枚举    N0~N2 在上方   N6~N8 在下方  坐标单位 mm
static const map_node_t map_nodes[MAP_NODE_COUNT] = {
    {275.0f, 2125.0f}, {1200.0f, 2125.0f}, {2125.0f, 2125.0f},
    {275.0f, 1200.0f}, {1200.0f, 1200.0f}, {2125.0f, 1200.0f},
    {275.0f, 275.0f},  {1200.0f, 275.0f},  {2125.0f, 275.0f}
};

// 12道路枚举     R0~R5 横向   R6~R11 纵向   与下面 5x5 的道路编号一致
static const map_road_t map_roads_info[12] = {
    {0, 1}, {1, 2}, {3, 4}, {4, 5}, {6, 7}, {7, 8},
    {0, 3}, {3, 6}, {1, 4}, {4, 7}, {2, 5}, {5, 8}
};



// X、Y使用同一组区域边界  坐标查询和障碍框归属都按这组边界判断
static const float map_axis_edges[6] = {0.0f, 550.0f, 1000.0f, 1400.0f, 1850.0f, 2400.0f};

// 5 * 5 栅格枚举  以左下角为原点      第一行对应赛场下方 第一列对应赛场左侧
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

// 单个轴上的毫米坐标转区域编号  调用前需检查坐标是否位于场地范围内
static uint8_t map_axis_to_cell(float value)
{
    for (uint8_t i = 0; i < MAP_CELL_WIDTH; i++) {
        if (value < map_axis_edges[i + 1]) return i;
    }
    return MAP_CELL_WIDTH - 1;
}

// 将毫米坐标映射到栅格编号  调用前需检查坐标是否位于场地范围内
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

// 读取栅格编号对应的栅格信息
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


// 读取一个区域的矩形边界
uint8_t map_cell_rect_get(uint8_t cell_x, uint8_t cell_y, map_rect_t *rect) {
    if (rect == 0 || cell_x >= MAP_CELL_WIDTH || cell_y >= MAP_CELL_HEIGHT)
        return 0;

    rect->min_x = map_axis_edges[cell_x];
    rect->max_x = map_axis_edges[cell_x + 1];
    rect->min_y = map_axis_edges[cell_y];
    rect->max_y = map_axis_edges[cell_y + 1];
    return 1;
}

// 地图区域判定与格子查询使用同一张表  避免边界一边算道路、一边算禁区
map_area_t map_check_area(float x, float y) {
    uint8_t cell_x, cell_y;
    map_cell_t cell;

    // 检查坐标是否在地图范围内
    if (!map_xy_to_cell(x, y, &cell_x, &cell_y) || !map_cell_get(cell_x, cell_y, &cell))
        return map_outside;

    return cell.cell_type == cell_forbidden ? map_forbidden : map_normal;
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

// 根据两个相邻节点查找道路 正反方向相同
uint8_t map_road_find(uint8_t node_a, uint8_t node_b, uint8_t *road_id) {
    if (road_id == 0 || node_a >= MAP_NODE_COUNT || node_b >= MAP_NODE_COUNT) return 0;

    // 遍历道路信息数组 查找匹配的道路
    for (uint8_t i = 0; i < 12; i++) {
        if ((map_roads_info[i].node_a == node_a && map_roads_info[i].node_b == node_b) ||
            (map_roads_info[i].node_a == node_b && map_roads_info[i].node_b == node_a)) {
            *road_id = i;
            return 1;
        }
    }
    return 0;
}











