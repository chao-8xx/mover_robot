#include "headfile.h"


/**
 * 根据新观测更新障碍物和道路状态
 * 雷达聚类 → 本圈受影响的道路 → 连续两圈确认后封路
 *
 * 每条道路只记录连续受到障碍影响的圈数，不再配对、管理每一个物体
 * 同一圈里多个框影响同一条道路，也只能累计一次
 * 一圈没看到、跳过扫描、停更或主动清零后，都需要重新连续确认
 * 已封锁道路保持封锁；本文件不自动放行、不控制电机
 */

static uint8_t  map_ready = 0;      // 0=未初始化，1=已初始化可接受扫描
static uint8_t  map_has_scan = 0;   // 是否已经接收过至少一圈扫描（用于校验 scan_num 递增）
static float    map_safe_radius = 0.0f; // 每个障碍框向外膨胀的半径（含车体外接半径+安全余量）
static uint32_t map_last_scan = 0;  // 上一次接受扫描的编号（防止重复/回退）
static uint32_t map_last_tick = 0;  // 上一次接受扫描的时间戳（毫秒）
static uint32_t map_change_num = 0; // 新封锁道路的累计条数，供控制层判断是否要重规划
static uint8_t map_road_seen[MAP_ROAD_COUNT]; // 每条道路连续受到障碍影响的圈数

///////////////////////////////////// 内部调用函数 ////////////////////////////////////////
// 候选框合法性检查
static uint8_t map_box_check(const map_scan_obs_t *box) {
    if (box == 0)
        return 0;

    // 检查坐标是否为有限数值  避免 NaN 或无穷大
    if (!isfinite(box->center_x) || !isfinite(box->center_y) ||
        !isfinite(box->min_x) || !isfinite(box->min_y) ||
        !isfinite(box->max_x) || !isfinite(box->max_y))
        return 0;

    // 检查最小值是否小于最大值
    if (box->min_x > box->max_x || box->min_y > box->max_y)
        return 0;

    // 检查中心点是否在候选框内
    if (box->center_x < box->min_x || box->center_x > box->max_x ||
        box->center_y < box->min_y || box->center_y > box->max_y)
        return 0;

    return map_check_area(box->center_x, box->center_y) == map_normal;
}

// 候选框加上车体外接半径、安全余量后，与道路中心线是否相交 (是否压住某条道路)
static uint8_t map_box_hit_road(uint8_t road_id, const map_scan_obs_t *box) {
    map_road_t road;
    map_node_t a, b;
    float low, high;

    // 道路信息或端点信息不存在时
    if (!map_road_info_get(road_id, &road) || !map_node_get(road.node_a, &a) || !map_node_get(road.node_b, &b))
        return 0;

    // 水平道路
    if (a.y == b.y) {
        low = a.x < b.x ? a.x : b.x;
        high = a.x > b.x ? a.x : b.x;
        return (box->max_x + map_safe_radius >= low &&
                box->min_x - map_safe_radius <= high &&
                box->min_y - map_safe_radius <= a.y &&
                box->max_y + map_safe_radius >= a.y);
    }

    // 竖直道路
    low = a.y < b.y ? a.y : b.y;
    high = a.y > b.y ? a.y : b.y;
    return (box->max_y + map_safe_radius >= low &&
            box->min_y - map_safe_radius <= high &&
            box->min_x - map_safe_radius <= a.x &&
            box->max_x + map_safe_radius >= a.x);
}

///////////////////////////////////// 外部调用函数 ////////////////////////////////////////
// 初始化地图更新模块 设置车体宽度和安全余量
uint8_t map_update_init(float car_width_mm, float safety_mm) {
    map_ready = 0;

    // 检查参数合法性：车体宽度必须大于0，安全余量可以为0；都不能超过地图尺寸
    if (!isfinite(car_width_mm) || !isfinite(safety_mm) ||
        car_width_mm <= 0.0f || safety_mm < 0.0f ||
        car_width_mm > MAP_WIDTH || safety_mm > MAP_WIDTH) return 0;

    // 只有有效配置才重置地图；参数填错时保留已有的封锁道路
    map_data_init();
    map_has_scan = 0;
    map_last_scan = 0;
    map_last_tick = 0;
    map_change_num = 0;
    memset(map_road_seen, 0, sizeof(map_road_seen));
    map_safe_radius = car_width_mm * 0.70710678f + safety_mm;
    map_ready = 1;
    return 1;
}

uint8_t map_update_is_ready(void) {
    return map_ready;
}

// 中断连续观察时重新计数；已经封锁的道路保持不变
void map_update_clear_candidates(void) {
    memset(map_road_seen, 0, sizeof(map_road_seen));
}

uint8_t map_update_scan(const map_scan_obs_t *obs, uint8_t count,
                        uint32_t scan_num, uint32_t tick) {
    uint8_t road_hit[MAP_ROAD_COUNT] = {0}; // 本圈有哪些道路受到障碍影响

    if (!map_ready || count > MAP_OBS_MAX || (count != 0 && obs == 0) ||
        (map_has_scan && scan_num <= map_last_scan) ||
        (map_has_scan && tick - map_last_tick >= 0x80000000U)) return 0;

    // 编号不相邻或时间隔得太久，说明不再是连续观察，所有道路重新累计
    if (map_has_scan && (scan_num - map_last_scan != 1U ||
                        tick - map_last_tick > MAP_SCAN_TIMEOUT_MS))
        map_update_clear_candidates();

    // 第一步：只标记本圈受影响的道路；多个框影响同一条路也只标记为1
    for (uint8_t i = 0; i < count; i++) {
        if (!map_box_check(&obs[i])) continue;
        for (uint8_t road = 0; road < MAP_ROAD_COUNT; road++) {
            if (map_box_hit_road(road, &obs[i])) road_hit[road] = 1;
        }
    }

    // 第二步：每条道路每圈累计一次；本圈没有障碍就清零，连续达到次数才封路
    for (uint8_t road = 0; road < MAP_ROAD_COUNT; road++) {
        road_state_t state;
        if (!road_hit[road]) {
            map_road_seen[road] = 0;
            continue;
        }
        if (map_road_seen[road] < MAP_CONFIRM_SCAN_NUM) map_road_seen[road]++;
        if (map_road_seen[road] < MAP_CONFIRM_SCAN_NUM) continue;
        if (!map_road_get(road, &state) || state == road_blocked) continue;
        if (map_road_set(road, road_blocked)) map_change_num++;
    }

    map_has_scan = 1;
    map_last_scan = scan_num;
    map_last_tick = tick;
    return 1;
}

uint32_t map_update_change_get(void) {
    return map_change_num;
}
