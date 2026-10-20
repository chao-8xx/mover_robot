#ifndef GONGXUN_1_MAP_PLAN_H
#define GONGXUN_1_MAP_PLAN_H


#define MAP_COST_INF      0xFFFF    // 不可通行的道路
#define MAP_COST_FREE     1         // 已确认可通行的道路
#define MAP_COST_UNKNOWN  2         // 按总权重选路，可能用较长的已知路线代替较短的未知路线

// 未知路段的策略由上层选：允许搜索时 进路前仍需要停车扫描确认
typedef enum {
    map_unknown_stop = 0,     // 只走已确认可通行的道路
    map_unknown_allow         // 允许把未知道路纳入候选路径
} map_unknown_mode_t;

// 规划路径的道路状态
typedef enum {
    map_step_stop = 0,        // 道路封锁或路径无效  停止并重规划
    map_step_scan,            // 未知道路  停车确认后再进入
    map_step_go               // 已确认可通行
} map_step_t;

typedef struct {
    uint8_t nodes[MAP_NODE_COUNT];
    uint8_t roads[MAP_NODE_COUNT - 1];
    uint8_t node_count;
    uint8_t road_count;
} map_path_t;

// 搜索 N0~N8 的路径；道路状态变化后再次调用就是重规划
uint8_t map_plan_find(uint8_t start, uint8_t goal,
                      map_unknown_mode_t mode, map_path_t *path);

// 检查当前路径是否仍能按相同搜索策略使用；未知路仍需另行确认
uint8_t map_plan_check(const map_path_t *path, map_unknown_mode_t mode);

// 控制层在进入路径中的某条道路前调用，决定走、先扫描还是停止
map_step_t map_plan_road_state(const map_path_t *path, uint8_t road_index);

#endif // GONGXUN_1_MAP_PLAN_H
