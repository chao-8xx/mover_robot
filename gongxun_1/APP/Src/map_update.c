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

static uint8_t  map_ready = 0;                 // 是否可接受地图扫描
static uint8_t  map_has_scan = 0;              // 是否已经接收过至少一圈扫描（用于校验 scan_num 递增）
static uint8_t  map_scan_valid = 0;            // 最近一圈是否允许用于上层显式放行 中断后清零
static uint32_t map_last_scan = 0;             // 上一次接受扫描的编号（防止重复/回退）
static uint32_t map_last_tick = 0;             // 上一次接受扫描的时间戳（毫秒）
static uint32_t map_change_num = 0;            // 封路或显式放行的累计变更次数 供控制层检查路径
static uint8_t map_road_seen[MAP_ROAD_COUNT];  // 每条道路连续受到障碍影响的圈数
static uint32_t map_free_scan[MAP_ROAD_COUNT]; // 每条道路最近一次显式放行所依据的扫描编号

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

    // 中心在禁区或场外的框也可能伸入道路 是否影响道路交给后面的几何判断
    return 1;
}

// 障碍框碰到道路区域就标记本道路: 碰到节点区域就标记与节点相连的出口  跨区域的框会标记它实际覆盖的所有道路
static void map_box_mark_roads(const map_scan_obs_t *box, uint8_t *road_hit) {

    // 遍历地图网格  检查每个网格是否与障碍框相交
    for (uint8_t y = 0; y < MAP_CELL_HEIGHT; y++) {

        // 遍历每个网格  检查网格是否与障碍框相交
        for (uint8_t x = 0; x < MAP_CELL_WIDTH; x++) {
            map_cell_t cell;
            map_rect_t rect;

            // 检查网格是否有效  禁区或场外网格不参与道路碰撞判断
            if (!map_cell_get(x, y, &cell) || cell.cell_type == cell_forbidden ||
                !map_cell_rect_get(x, y, &rect))
                continue;

            // 检查障碍框与网格是否相交  不相交就跳过
            if (box->max_x < rect.min_x || box->min_x >= rect.max_x ||
                box->max_y < rect.min_y || box->min_y >= rect.max_y)
                continue;

            // 障碍框与网格相交  标记本道路或与节点相连的道路
            if (cell.cell_type == cell_road)
                road_hit[cell.cell_id] = 1;

            // 障碍框与节点相交  标记与节点相连的道路
            else {
                for (uint8_t road_id = 0; road_id < MAP_ROAD_COUNT; road_id++) {
                    map_road_t road;

                    // 检查道路是否与节点相连  道路的两个端点中有一个是该节点就标记本道路
                    if (map_road_info_get(road_id, &road) &&
                        (road.node_a == cell.cell_id || road.node_b == cell.cell_id))
                        road_hit[road_id] = 1;
                }
            }
        }
    }
}

///////////////////////////////////// 外部调用函数 ////////////////////////////////////////
// 初始化地图更新模块  按道路区域封锁  不再进行车体安全膨胀
void map_update_init(void) {
    map_ready = 0;
    map_data_init();
    map_has_scan = 0;
    map_scan_valid = 0;
    map_last_scan = 0;
    map_last_tick = 0;
    map_change_num = 0;
    memset(map_road_seen, 0, sizeof(map_road_seen));
    memset(map_free_scan, 0, sizeof(map_free_scan));
    map_ready = 1;
}

// 查询能否接收地图更新
uint8_t map_update_is_ready(void) {
    return map_ready;
}

// 清掉连续确认次数     中断连续观察时重新计数 已经封锁的道路保持不变
void map_update_clear_candidates(void) {
    memset(map_road_seen, 0, sizeof(map_road_seen));
    map_scan_valid = 0;  // 中断前的一圈也不能再拿来确认放行
}

// 预设矩形测试入口  只负责转换命中道路  和真实雷达共用同一套跨圈确认
uint8_t map_update_scan(const map_scan_obs_t *obs, uint8_t count, uint32_t scan_num, uint32_t tick, uint8_t complete) {
    uint8_t road_hit[MAP_ROAD_COUNT] = {0}; // 本圈有哪些道路受到障碍影响
    uint16_t road_mask = 0;                 // 本圈哪些道路受到障碍影响的位掩码

    // 检查参数合法性  只有在初始化后才接受扫描数据
    if (count > MAP_SCAN_OBS_MAX || (count != 0 && obs == 0)) {
        map_update_clear_candidates();
        return 0;
    }

    // 逐个候选框检查合法性并标记道路  不合法的候选框不影响道路状态
    for (uint8_t i = 0; i < count; i++) {
        if (!map_box_check(&obs[i])) {
            complete = 0;
            continue;
        }

        // 标记本道路或与节点相连的道路  跨区域的框会标记它实际覆盖的所有道路
        map_box_mark_roads(&obs[i], road_hit);
    }

    // 将本圈受影响的道路标记为位掩码  便于后续处理
    for (uint8_t road = 0; road < MAP_ROAD_COUNT; road++) {
        if (road_hit[road])
            road_mask |= (uint16_t)(1 << road);
    }

    return map_update_roads(road_mask, scan_num, tick, complete);
}

// 根据本圈实际命中的道路更新状态  不把聚类框内的空白当成障碍
uint8_t map_update_roads(uint16_t road_mask, uint32_t scan_num, uint32_t tick, uint8_t complete) {

    // 检查参数合法性  只有在初始化后才接受扫描数据
    if (!map_ready || scan_num == 0 || (road_mask >> MAP_ROAD_COUNT) != 0) {
        map_update_clear_candidates();
        return 0;
    }
    if ((map_has_scan && scan_num <= map_last_scan) ||
        (map_has_scan && tick - map_last_tick >= 0x80000000))
        return 0;

    // 判断是不是连续观察    编号不相邻或时间隔得太久  说明不再是连续观察  所有道路重新累计
    if (map_has_scan && (scan_num - map_last_scan != 1 ||
                        tick - map_last_tick > MAP_SCAN_TIMEOUT_MS))
        map_update_clear_candidates();

    // 多个点、多个候选影响同一条道路  road_mask中也只有一位  不会重复累计
    map_scan_valid = complete != 0;

    // 第二步：逐条更新道路的连续确认次数   每条道路每圈累计一次 本圈没有障碍就清零 连续达到次数才封路
    for (uint8_t road = 0; road < MAP_ROAD_COUNT; road++) {
        road_state_t state;

        // 本圈没有障碍影响就清零连续确认次数
        if ((road_mask & (1 << road)) == 0) {
            map_road_seen[road] = 0;
            continue;
        }

        // 有影响 记为一次
        if (map_road_seen[road] < MAP_CONFIRM_SCAN_NUM)
            map_road_seen[road]++;

        // 连续确认次数未达到要求就不封路
        if (map_road_seen[road] < MAP_CONFIRM_SCAN_NUM)
            continue;

        // 连续确认次数达到要求  封路  已经封锁的道路不再重复封锁
        if (!map_road_get(road, &state) || state == road_blocked)
            continue;

        // 记录封路的变化
        if (map_road_set(road, road_blocked))
            map_change_num++;
    }

    map_has_scan = 1;
    map_last_scan = scan_num;
    map_last_tick = tick;
    return 1;
}

// 上层已经确认整段道路被有效观察后 再用最近一圈结果显式确认放行
uint8_t map_update_confirm_free(uint8_t road_id, uint32_t scan_num, uint32_t tick) {
    road_state_t state;

    // 只认最新且未过期的一圈；本圈有障碍或已经封锁的道路不能放行
    if (!map_ready || !map_scan_valid || scan_num != map_last_scan ||
        tick - map_last_tick > MAP_SCAN_TIMEOUT_MS ||
        !map_road_get(road_id, &state) || state == road_blocked || map_road_seen[road_id] != 0)
        return 0;

    // 放行道路  只有状态发生变化才计入变更次数
    if (state != road_free) {
        if (!map_road_set(road_id, road_free))
            return 0;
        map_change_num++;
    }
    map_free_scan[road_id] = scan_num;
    return 1;
}

// 长期道路状态与本次通行许可分开:  更新、超时、中断或疑似障碍都会暂停旧许可
uint8_t map_update_road_ready(uint8_t road_id, uint32_t tick) {
    road_state_t state;

    return map_ready && map_scan_valid && tick - map_last_tick <= MAP_SCAN_TIMEOUT_MS &&
           map_road_get(road_id, &state) && state == road_free && map_road_seen[road_id] == 0 &&
           map_free_scan[road_id] == map_last_scan;
}

// 查询道路变更计数     封路和显式放行都会递增 直接调用map_road_set不计入这里
uint32_t map_update_change_get(void) {
    return map_change_num;
}
