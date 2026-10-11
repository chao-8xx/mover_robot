#include "headfile.h"



static uint8_t map_ready = 0;
static uint8_t map_has_scan = 0;
static float map_safe_radius = 0.0f;
static uint32_t map_last_scan = 0;
static uint32_t map_last_tick = 0;
static uint32_t map_change_num = 0;

///////////////////////////////////// 内部调用函数 ////////////////////////////////////////
// 检查候选框是否有效，不能把坏的坐标写到道路状态里
static uint8_t map_box_check(const map_scan_obs_t *box) {
    if (box == 0) return 0;
    if (!isfinite(box->center_x) || !isfinite(box->center_y) ||
        !isfinite(box->min_x) || !isfinite(box->min_y) ||
        !isfinite(box->max_x) || !isfinite(box->max_y)) return 0;
    if (box->min_x > box->max_x || box->min_y > box->max_y) return 0;
    if (box->center_x < box->min_x || box->center_x > box->max_x ||
        box->center_y < box->min_y || box->center_y > box->max_y) return 0;
    return map_check_area(box->center_x, box->center_y) == map_normal;
}

// 候选框加上车体外接半径、安全余量后，与道路中心线是否相交
static uint8_t map_box_hit_road(uint8_t road_id, const map_scan_obs_t *box) {
    map_road_t road;
    map_node_t a, b;
    float low, high;

    if (!map_road_info_get(road_id, &road) ||
        !map_node_get(road.node_a, &a) || !map_node_get(road.node_b, &b)) return 0;

    if (a.y == b.y) {
        low = a.x < b.x ? a.x : b.x;
        high = a.x > b.x ? a.x : b.x;
        return (box->max_x + map_safe_radius >= low &&
                box->min_x - map_safe_radius <= high &&
                box->min_y - map_safe_radius <= a.y &&
                box->max_y + map_safe_radius >= a.y);
    }

    low = a.y < b.y ? a.y : b.y;
    high = a.y > b.y ? a.y : b.y;
    return (box->max_y + map_safe_radius >= low &&
            box->min_y - map_safe_radius <= high &&
            box->min_x - map_safe_radius <= a.x &&
            box->max_x + map_safe_radius >= a.x);
}

// 只保存可能影响道路的物体，静态禁入区里的点不进入动态地图
static uint8_t map_box_hit_any_road(const map_scan_obs_t *box) {
    for (uint8_t road = 0; road < MAP_ROAD_COUNT; road++) {
        if (map_box_hit_road(road, box)) return 1;
    }
    return 0;
}

// 确认后的障碍物只封锁它影响的道路，不直接封锁整个 5x5 区域
static void map_block_road(const map_scan_obs_t *box) {
    road_state_t state;
    for (uint8_t road = 0; road < MAP_ROAD_COUNT; road++) {
        if (!map_box_hit_road(road, box)) continue;
        if (!map_road_get(road, &state) || state == road_blocked) continue;
        if (map_road_set(road, road_blocked)) map_change_num++;
    }
}

// 找上一圈位置最近、且这一圈尚未配对的记录
static uint8_t map_find_old(const map_scan_obs_t *box, const uint8_t *used,
                            uint8_t *obs_id) {
    float best = MAP_MATCH_DISTANCE_MM * MAP_MATCH_DISTANCE_MM;
    uint8_t found = 0;
    map_obs_t old;

    for (uint8_t i = 0; i < MAP_OBS_MAX; i++) {
        float dx, dy, distance;
        if (used[i] || !map_obs_get(i, &old) || old.state == obs_no) continue;
        dx = old.center_x - box->center_x;
        dy = old.center_y - box->center_y;
        distance = dx * dx + dy * dy;
        if (distance <= best) {
            best = distance;
            *obs_id = i;
            found = 1;
        }
    }
    return found;
}

// 同一圈中位置接近的两个聚类只算一次，防止一次扫描重复确认
static uint8_t map_seen_this_scan(const map_scan_obs_t *box, const uint8_t *used) {
    map_obs_t old;
    float limit = MAP_DUP_DISTANCE_MM * MAP_DUP_DISTANCE_MM;
    for (uint8_t i = 0; i < MAP_OBS_MAX; i++) {
        float dx, dy;
        if (!used[i] || !map_obs_get(i, &old)) continue;
        dx = old.center_x - box->center_x;
        dy = old.center_y - box->center_y;
        if (dx * dx + dy * dy <= limit) return 1;
    }
    return 0;
}

// 用空位保存新候选；满时只替换最旧的未确认候选
static uint8_t map_find_slot(const uint8_t *used, uint8_t *obs_id) {
    map_obs_t old;
    uint8_t found = 0;
    uint32_t oldest = 0;

    for (uint8_t i = 0; i < MAP_OBS_MAX; i++) {
        if (!map_obs_get(i, &old)) return 0;
        if (old.state == obs_no && !used[i]) {
            *obs_id = i;
            return 1;
        }
        if (old.state == obs_unknown && !used[i] &&
            (!found || old.last_scan_num < oldest)) {
            oldest = old.last_scan_num;
            *obs_id = i;
            found = 1;
        }
    }
    return found;
}

///////////////////////////////////// 外部调用函数 ////////////////////////////////////////
uint8_t map_update_init(float car_width_mm, float safety_mm) {
    map_ready = 0;
    if (!isfinite(car_width_mm) || !isfinite(safety_mm) ||
        car_width_mm <= 0.0f || safety_mm < 0.0f ||
        car_width_mm > MAP_WIDTH || safety_mm > MAP_WIDTH) return 0;

    // 只有有效配置才重置地图；参数填错时保留已有的封锁道路
    map_data_init();
    map_has_scan = 0;
    map_last_scan = 0;
    map_last_tick = 0;
    map_change_num = 0;
    map_safe_radius = car_width_mm * 0.70710678f + safety_mm;
    map_ready = 1;
    return 1;
}

uint8_t map_update_is_ready(void) {
    return map_ready;
}

// 中断连续观察时重新确认候选；已经确认的障碍和道路状态保持不变
void map_update_clear_candidates(void) {
    map_obs_t old;
    for (uint8_t i = 0; i < MAP_OBS_MAX; i++) {
        if (map_obs_get(i, &old) && old.state == obs_unknown) map_obs_clear(i);
    }
}

uint8_t map_update_scan(const map_scan_obs_t *obs, uint8_t count,
                        uint32_t scan_num, uint32_t tick) {
    uint8_t used[MAP_OBS_MAX] = {0};
    map_obs_t old;

    if (!map_ready || count > MAP_OBS_MAX || (count != 0 && obs == 0) ||
        (map_has_scan && scan_num <= map_last_scan) ||
        (map_has_scan && tick - map_last_tick >= 0x80000000U)) return 0;

    // 发布编号相邻也可能相隔很久，不能让停更前的候选一直保留确认次数
    for (uint8_t i = 0; i < MAP_OBS_MAX; i++) {
        if (map_obs_get(i, &old) && old.state == obs_unknown &&
            tick - old.last_tick > MAP_CANDIDATE_TIMEOUT_MS) map_obs_clear(i);
    }

    // 调用间如果跳过了完整扫描，候选也必须计入漏扫次数
    if (map_has_scan && scan_num > map_last_scan + 1) {
        uint32_t skipped = scan_num - map_last_scan - 1;
        for (uint8_t i = 0; i < MAP_OBS_MAX; i++) {
            if (!map_obs_get(i, &old) || old.state != obs_unknown) continue;
            if (old.miss_num >= MAP_CANDIDATE_MISS_NUM ||
                skipped >= (uint32_t)(MAP_CANDIDATE_MISS_NUM - old.miss_num))
                map_obs_clear(i);
            else {
                old.miss_num += (uint8_t)skipped;
                map_obs_set(i, &old);
            }
        }
    }

    // 每个聚类最多与一个旧障碍物配对，同一圈不会重复累计次数
    for (uint8_t i = 0; i < count; i++) {
        uint8_t id;
        uint8_t matched;
        map_obs_t now;
        if (!map_box_check(&obs[i]) || !map_box_hit_any_road(&obs[i])) continue;
        if (map_seen_this_scan(&obs[i], used)) continue;

        matched = map_find_old(&obs[i], used, &id);
        if (!matched && !map_find_slot(used, &id)) continue;

        memset(&now, 0, sizeof(now));
        if (matched) map_obs_get(id, &now);
        now.center_x = obs[i].center_x;
        now.center_y = obs[i].center_y;
        now.min_x = obs[i].min_x;
        now.min_y = obs[i].min_y;
        now.max_x = obs[i].max_x;
        now.max_y = obs[i].max_y;
        now.last_scan_num = scan_num;
        now.last_tick = tick;
        now.miss_num = 0;
        if (now.seen_num < 255) now.seen_num++;
        if (now.seen_num >= MAP_CONFIRM_SCAN_NUM) now.state = obs_ok;
        else now.state = obs_unknown;

        map_obs_set(id, &now);
        used[id] = 1;
        if (now.state == obs_ok) map_block_road(&obs[i]);
    }

    // 候选物体连续漏检会删除；已确认障碍物不因一次没扫到就放行
    for (uint8_t i = 0; i < MAP_OBS_MAX; i++) {
        if (used[i] || !map_obs_get(i, &old) || old.state != obs_unknown) continue;
        if (old.miss_num < 255) old.miss_num++;
        // 允许细小障碍漏扫一圈；连续漏掉两圈才删除候选
        if (old.miss_num >= MAP_CANDIDATE_MISS_NUM) map_obs_clear(i);
        else map_obs_set(i, &old);
    }

    map_has_scan = 1;
    map_last_scan = scan_num;
    map_last_tick = tick;
    return 1;
}

uint32_t map_update_change_get(void) {
    return map_change_num;
}
