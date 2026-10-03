//
// Created by mechreuo on 2026/9/27.
//

#include "map_data.h"

#include <math.h>
#include <string.h>

// 赛道地图数据
static road_state_t map_roads[MAP_ROAD_COUNT];
static map_obs_t map_obs[MAP_OBS_MAX];


////////////////////////////////////////////////////// 内部调用函数 ///////////////////////
// 根据道路状态返回对应的道路类型
static uint8_t map_road_check(road_state_t state) {
    return (state == road_unknown ||
            state == road_blocked ||
            state == road_free);
}

// 根据障碍物状态和包围框坐标返回是否有效
static uint8_t map_obs_check(const map_obs_t *obs) {
    if (obs == 0)   return 0;

    if (obs->state == obs_no)
        return 1;

    if (obs->state != obs_unknown && obs->state != obs_ok)
        return 0;

    // 检查包围框坐标是否有效   拒绝NaN、无穷大等无效坐标
    if (!isfinite(obs->center_x) || !isfinite(obs->center_y) || !isfinite(obs->min_x) || !isfinite(obs->max_x) || !isfinite(obs->min_y) || !isfinite(obs->max_y))
        return 0;

    if (obs->min_x > obs->max_x || obs->min_y > obs->max_y)
        return 0;

    if (obs->center_x < obs->min_x || obs->center_x > obs->max_x ||
        obs->center_y < obs->min_y || obs->center_y > obs->max_y)
        return 0;

    return 1;
}


//////////////////////////////////////////////////////// 外部调用函数 ///////////////////////////
// 地图数据初始化
void map_data_init(void) {

    // 初始化道路状态数组
    for (uint8_t i = 0; i < MAP_ROAD_COUNT; i++) {
        map_roads[i] = road_unknown;
    }

    // 初始化障碍物数组
    memset(map_obs, 0, sizeof(map_obs));
}

// 设置指定道路的状态
uint8_t map_road_set(uint8_t road_id, road_state_t state) {
    if (road_id >= MAP_ROAD_COUNT) return 0;

    if (!map_road_check(state)) return 0;

    map_roads[road_id] = state;
    return 1;
}

// 获取指定道路的状态
uint8_t map_road_get(uint8_t road_id, road_state_t *state) {
    if (road_id >= MAP_ROAD_COUNT || state == NULL) return 0;

    *state = map_roads[road_id];
    return 1;
}




// 设置指定障碍物的状态和包围框
uint8_t map_obs_set(uint8_t obs_id, const map_obs_t *obs) {
    if (obs_id >= MAP_OBS_MAX) return 0;

    if (!map_obs_check(obs))    return 0;

    // 如果状态是无障碍物 则清除障碍物数据
    if (obs->state == obs_no) {
        return map_obs_clear(obs_id);
    }

    map_obs[obs_id] = *obs;
    return 1;
}

// 获取指定障碍物的状态
uint8_t map_obs_get(uint8_t obs_id, map_obs_t *obs) {
    if (obs_id >= MAP_OBS_MAX || obs == 0) return 0;

    *obs = map_obs[obs_id];
    return 1;
}

// 清除指定障碍物的状态和包围框
uint8_t map_obs_clear(uint8_t obs_id) {
    if (obs_id >= MAP_OBS_MAX) return 0;

    memset(&map_obs[obs_id], 0, sizeof(map_obs[obs_id]));
    return 1;
}
