#ifndef GONGXUN_1_TRACK_MAP_H
#define GONGXUN_1_TRACK_MAP_H

#include <stdint.h>

#define MAP_WIDTH   2400.0f
#define MAP_HEIGHT  2400.0f

#define MAP_FORBIDDEN_COUNT     4

#define MAP_CELL_WIDTH      5
#define MAP_CELL_HEIGHT     5
#define MAP_NO_ID           0xFF
#define MAP_NODE_COUNT      9


// 地图区域类型
typedef enum {
    map_outside = 0,
    map_normal,
    map_forbidden,
} map_area_t;

// 地图边界矩形结构体  禁入区
typedef struct {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
} map_rect_t;


// 栅格地图类型
typedef enum {
    cell_node = 0,
    cell_road,
    cell_forbidden
}map_cell_type_t;

// 栅格地图单元结构体
typedef struct {
    map_cell_type_t cell_type;   // 栅格类型
    uint8_t cell_id;             // 栅格ID  对于节点类型为节点ID 对于道路类型为道路ID 对于禁行区类型为禁行区ID
} map_cell_t;

// 节点和道路使用同一套毫米坐标 道路两端都是节点中心
typedef struct {
    float x;
    float y;
} map_node_t;

// 道路结构体  记录道路两端的节点ID
typedef struct {
    uint8_t node_a; // 道路起点节点ID
    uint8_t node_b; // 道路终点节点ID
} map_road_t;

map_area_t map_check_area(float x, float y);
uint8_t map_xy_to_cell(float x, float y, uint8_t *cell_x, uint8_t *cell_y);
uint8_t map_cell_get(uint8_t cell_x, uint8_t cell_y, map_cell_t *cell);
uint8_t map_cell_rect_get(uint8_t cell_x, uint8_t cell_y, map_rect_t *rect);
uint8_t map_node_get(uint8_t node_id, map_node_t *node);
uint8_t map_road_info_get(uint8_t road_id, map_road_t *road);
uint8_t map_road_find(uint8_t node_a, uint8_t node_b, uint8_t *road_id);




#endif // GONGXUN_1_TRACK_MAP_H
