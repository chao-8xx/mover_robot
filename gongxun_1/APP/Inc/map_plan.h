#ifndef GONGXUN_1_MAP_PLAN_H
#define GONGXUN_1_MAP_PLAN_H

#include "map_track.h"

#define MAP_COST_INF      0xFFFF    // 不可达代价  封锁道路不参与搜索
#define MAP_COST_FREE     1         // 已确认可通行的道路
#define MAP_COST_UNKNOWN  2         // 未知道路  按总代价选路  不额外计算转向代价

// 未知路段的策略由上层选：允许搜索时 进路前仍需要停车扫描确认
typedef enum
{
    map_unknown_stop = 0,     // 只走已确认可通行的道路
    map_unknown_allow         // 允许把未知道路纳入候选路径
} map_unknown_mode_t;

// 规划路径的道路状态
typedef enum
{
    map_step_stop = 0,        // 道路封锁或路径无效  停止并重规划
    map_step_scan,            // 未知道路  停车确认后再进入
    map_step_go               // 已确认可通行
} map_step_t;

// 路径结果  数组按起点到终点排列
typedef struct
{
    uint8_t nodes[MAP_NODE_COUNT];       // 依次经过的节点 包含起点和终点
    uint8_t roads[MAP_NODE_COUNT - 1];   // roads[i]连接nodes[i]与nodes[i+1]
    uint8_t node_count;                  // 路径节点数量  最多9个
    uint8_t road_count;                  // 路径道路数量  比节点数量少1
} map_path_t;

///////////////////////////////////// 路径规划接口 ////////////////////////////////////////
// Dijkstra搜索N0~N8的最小代价路径  保留已知路1、未知路2的权重
// 返回1表示找到路径  返回0时清空非空的path  起终点相同时只返回该节点
// 道路状态变化后从实际起点再次调用就是重规划  搜索期间由上层保持地图状态稳定
uint8_t map_plan_find(uint8_t start, uint8_t goal,
                      map_unknown_mode_t mode, map_path_t *path);

// 检查路径连接和道路状态是否仍符合搜索策略  不检查是否仍为最优路径
uint8_t map_plan_check(const map_path_t *path, map_unknown_mode_t mode);

// 进入道路前调用，tick传当前时间；FREE还需最新一圈的有效许可才返回GO
// 本接口不替代上层对位姿可信、实际停稳和行驶近场危险的判断
map_step_t map_plan_road_state(const map_path_t *path, uint8_t road_index, uint32_t tick);

// 任务观察几何：道路及前方节点的范围，和该范围在实际雷达坐标下的角区。
// 只做选区规划，不接收雷达、不累计候选、不控制底盘。
uint8_t map_plan_observe_area(uint8_t road_id, uint8_t next_node, map_rect_t *area);
uint8_t map_plan_observe_sector(const map_rect_t *area, float sensor_x, float sensor_y,
                               float sensor_yaw, float margin_deg, float *center, float *half_width);

#endif //GONGXUN_1_MAP_PLAN_H
