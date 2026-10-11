//
// Created by mechreuo on 2026/9/27.
//

#ifndef GONGXUN_1_MAP_DATA_H
#define GONGXUN_1_MAP_DATA_H


#define MAP_ROAD_COUNT      12              // 12 条道路
#define MAP_OBS_MAX         20              // 与一圈雷达最多保存的聚类数相同


// 道路状态
typedef enum {
    road_unknown,           // 未知状态
    road_free,              // 可通行
    road_blocked            // 阻塞
} road_state_t;

// 障碍物状态
typedef enum {
    obs_no = 0,             // 无障碍物
    obs_unknown,            // 候选障碍物，还需要下一圈确认
    obs_ok                  // 已确认的障碍物
} obs_state_t;

// 全局地图中的障碍物结构体
typedef struct {
    float center_x;         // 全局坐标
    float center_y;

    float min_x;            // 障碍物包围框
    float min_y;
    float max_x;
    float max_y;

    uint32_t last_scan_num; // 最后见到他时的扫描编号
    uint32_t last_tick;     // 上次更新时刻 ms
    uint8_t seen_num;       // 候选阶段看到的次数；允许漏扫一圈
    uint8_t miss_num;       // 候选障碍物连续未看到的次数

    obs_state_t state;      // 状态
} map_obs_t;


void map_data_init(void);

uint8_t map_road_set(uint8_t road_id, road_state_t state);
uint8_t map_road_get(uint8_t road_id, road_state_t *state);

uint8_t map_obs_set(uint8_t obs_id, const map_obs_t *obs);
uint8_t map_obs_get(uint8_t obs_id, map_obs_t *obs);
uint8_t map_obs_clear(uint8_t obs_id);



#endif //GONGXUN_1_MAP_DATA_H
