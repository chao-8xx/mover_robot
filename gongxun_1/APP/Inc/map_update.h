#ifndef GONGXUN_1_MAP_UPDATE_H
#define GONGXUN_1_MAP_UPDATE_H

#include <stdint.h>

// 这些值需要在现场看雷达效果再调，不代表障碍物已经能被稳定看到
#define MAP_MATCH_DISTANCE_MM   120.0f   // 两圈聚类中心最大匹配距离
#define MAP_DUP_DISTANCE_MM      30.0f   // 同一圈几乎重合的聚类只计一次
#define MAP_CONFIRM_SCAN_NUM    2        // 允许漏扫一圈，三圈中至少两圈看到才确认
#define MAP_CANDIDATE_MISS_NUM  2        // 候选连续漏掉两圈后删除
#ifndef MAP_CANDIDATE_TIMEOUT_MS
#define MAP_CANDIDATE_TIMEOUT_MS 1000U  // 停更超过1秒的旧候选不能与恢复后的一圈共同确认，现场可调整
#endif

// 一圈雷达聚类转换成的地图候选框，坐标单位 mm
typedef struct {
    float center_x;
    float center_y;
    float min_x;
    float min_y;
    float max_x;
    float max_y;
} map_scan_obs_t;

// 启动或停车重置时初始化；car_width 是含展开机构的外接正方形边长
// 按正方形对角线的一半膨胀，覆盖任意车头角度；无效参数不接受雷达更新
uint8_t map_update_init(float car_width_mm, float safety_mm);
uint8_t map_update_is_ready(void);
void map_update_clear_candidates(void); // 位姿/接收中断时只清候选，保留确认障碍和封锁道路

// 输入完整一圈聚类结果；相同扫描编号不会重复确认
uint8_t map_update_scan(const map_scan_obs_t *obs, uint8_t count,
                        uint32_t scan_num, uint32_t tick);

// 本模块确认障碍并封锁道路时递增，可供控制层判断是否需要重新规划
uint32_t map_update_change_get(void);

#endif // GONGXUN_1_MAP_UPDATE_H
