//
// Created by mechreuo on 2026/9/23.
//

#include "headfile.h"




// 静态地图上的禁入区域
static const map_rect_t map_notgo_areas[MAP_FORBIDDEN_COUNT] = {
    { 550.0f, 1000.0f, 550.0f, 1000.0f },   // 左下禁入区
    { 1400.0f, 1850.0f, 550.0f, 1000.0f },  // 右下禁入区
    { 550.0f, 1000.0f, 1400.0f, 1850.0f },  // 左上禁入区
    { 1400.0f, 1850.0f, 1400.0f, 1850.0f }, // 右上禁入区
};


// 是否在静态地图内
static uint8_t map_can_in(float x, float y, const map_rect_t *map) {
    if (map == 0) return 0;

    return (x >= map->min_x && x <= map->max_x && y >= map->min_y && y <= map->max_y);
}

// 地图区域判定函数
map_area_t map_check_area(float x, float y) {
    // 出界区域判定
    if (!(x > 0.0f && x < MAP_WIDTH && y > 0.0f && y < MAP_HEIGHT)) {
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











