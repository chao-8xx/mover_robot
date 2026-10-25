#include "headfile.h"
#include "lidar_map_desk_test.h"

static uint8_t desk_ready;

///////////////////////////////////// 固定位姿地图处理 ///////////////////////////////////
uint8_t lidar_map_desk_begin(void) {
    map_nav_observe_end();
    map_update_init();
    tf_init(); // 裸雷达使用零安装偏移，正式安装参数不在本测试中标定
    desk_ready = map_nav_observe_begin(LIDAR_MAP_DESK_CURRENT_NODE, LIDAR_MAP_DESK_NEXT_NODE);
    return desk_ready;
}

uint8_t lidar_map_desk_process(const lidar_scan_t *scan, uint32_t tick) {
    if (!desk_ready || scan == NULL || !scan->scan_num || tick - scan->end_time > MAP_SCAN_TIMEOUT_MS)
        return 0;
    // 保留整圈完成聚类，不先按道路或正面角区裁成小点组。
    if (!point_cloud_update_with_mask(scan, LIDAR_MAP_DESK_X, LIDAR_MAP_DESK_Y,
                                     LIDAR_MAP_DESK_YAW, tick, LIDAR_MAP_DESK_WHEEL_MASK)) return 0;
    obstacle_cluster_update(&cloud, tick);
    // 复用正式停车观察：道路/前方节点选区、实际点判路、同位置5圈中3圈确认。
    return map_nav_observe_update(1);
}

uint8_t lidar_map_desk_candidate(uint16_t index, lidar_map_desk_candidate_t *out) {
    if (out == NULL || index >= obstacle_result.cluster_count) return 0;
    const lidar_cluster_t *cluster = &obstacle_result.clusters[index];
    if (cluster->start_angle >= 360 || cluster->end_angle >= 360) return 0;
    memset(out, 0, sizeof(*out));
    out->center_cell.cell_id = MAP_NO_ID;
    uint8_t x, y;
    out->center_area = map_check_area(cluster->center_point.x, cluster->center_point.y);
    if (map_xy_to_cell(cluster->center_point.x, cluster->center_point.y, &x, &y))
        map_cell_get(x, y, &out->center_cell);
    map_rect_t area;
    if (!map_plan_observe_area(LIDAR_MAP_DESK_ROAD, LIDAR_MAP_DESK_NEXT_NODE, &area)) return 0;
    uint16_t span = (cluster->end_angle + 360U - cluster->start_angle) % 360U;
    for (uint16_t step = 0; step <= span; step++) {
        uint16_t angle = (cluster->start_angle + step) % 360U;
        const lidar_point_t *point = &cloud.points[angle];
        if (!point->valid) continue;
        if (angle >= 315 || angle <= 45) out->in_front = 1;
        if (!map_xy_to_cell(point->map_point.x, point->map_point.y, &x, &y)) {
            out->outside_points++;
            continue;
        }
        map_cell_t cell;
        if (!map_cell_get(x, y, &cell)) return 0;
        if (cell.cell_type == cell_forbidden) out->forbidden_points++;
        else if (cell.cell_type == cell_node) out->node_points++;
        else out->road_points++;
        uint16_t roads = map_lidar_point_roads(point->map_point.x, point->map_point.y);
        out->roads |= roads;
        if ((roads & (1U << LIDAR_MAP_DESK_ROAD)) &&
            point->map_point.x >= area.min_x && point->map_point.x < area.max_x &&
            point->map_point.y >= area.min_y && point->map_point.y < area.max_y)
            out->in_task = 1;
    }
    return 1;
}

#ifndef LIDAR_MAP_DESK_HOST_TEST
///////////////////////////////////// 开发板独立入口 /////////////////////////////////////
void lidar_map_desk_test_init(void) {
    debug_uart_init(&huart6);
    lidar_map_desk_begin();
    lidar_init(&huart4);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    lidar_start();
}

static uint8_t desk_key_released(uint32_t tick) {
    static uint8_t last_raw, stable;
    static uint32_t changed_tick;
    uint8_t raw = key_getstate(0);
    if (raw != last_raw) { last_raw = raw; changed_tick = tick; }
    if (raw != stable && tick - changed_tick >= 30U) {
        stable = raw;
        return stable == 0;
    }
    return 0;
}

static void desk_print_path(const map_path_t *path) {
    if (!path->node_count) { printf("[MAP PATH] NO PATH\r\n"); return; }
    printf("[MAP PATH] N%u", (unsigned int)path->nodes[0]);
    for (uint8_t i = 0; i < path->road_count; i++)
        printf(" -R%u-> N%u", (unsigned int)path->roads[i], (unsigned int)path->nodes[i + 1]);
    map_step_t step = map_plan_road_state(path, 0, HAL_GetTick());
    printf(" entry=%s\r\n", step == map_step_go ? "GO" : step == map_step_scan ? "SCAN" : "STOP");
}

void lidar_map_desk_test(void) {
    static uint8_t started, active, skip_scans;
    static uint32_t last_scan, last_drop, last_print, last_wait, last_change;
    static map_path_t path;
    static map_nav_observe_result_t observed; // 大快照不放在F407主循环栈上
    lidar_process();
    debug_printf_task();
    uint32_t tick = HAL_GetTick();
    const lidar_scan_t *scan = lidar_get_scan();
    if (!started) {
        started = 1;
        printf("===== LIDAR FIXED POSE MAP DESK TEST =====\r\n");
        printf("[MAP CONFIG] pose=(%.0f,%.0f,%.0f) N%u->N%u R%u wheel_mask=%u\r\n",
               LIDAR_MAP_DESK_X, LIDAR_MAP_DESK_Y, LIDAR_MAP_DESK_YAW,
               LIDAR_MAP_DESK_CURRENT_NODE, LIDAR_MAP_DESK_NEXT_NODE, LIDAR_MAP_DESK_ROAD,
               LIDAR_MAP_DESK_WHEEL_MASK);
        printf("[MAP FILTER] front=315~45 near=%.0f~%.0fmm points=%u~%u span_max=%.0fmm confirm=%u/%u\r\n",
               LIDAR_PILLAR_MIN_DISTANCE, LIDAR_PILLAR_MAX_DISTANCE,
               CLUSTER_MIN_POINT_COUNT, CLUSTER_MAX_POINT_COUNT, LIDAR_PILLAR_MAX_SIZE,
               MAP_OBSERVE_MIN_HITS, MAP_OBSERVE_WINDOW);
        printf("固定测试位姿和零安装偏移，不是实际定位；整圈聚类后按当前道路和前方节点筛选\r\n");
        printf("只接开发板和裸雷达；不启动电机、IMU、里程计或导航执行任务\r\n");
        printf("短按KEY1清空测试地图并开始；每组换位置后重新按KEY1，移走不自动解封\r\n");
    }
    if (desk_key_released(tick)) {
        active = lidar_map_desk_begin();
        skip_scans = 2;
        last_scan = scan ? scan->scan_num : 0;
        last_drop = lidar_get_scan_drop_cnt();
        last_change = last_print = 0;
        last_wait = tick;
        map_plan_find(LIDAR_MAP_DESK_CURRENT_NODE, LIDAR_MAP_DESK_NEXT_NODE, map_unknown_allow, &path);
        printf("[MAP RESET] active=%u 地图已清空，等待完整新圈；未发车\r\n", active);
        desk_print_path(&path);
        return;
    }
    if (!active) return;
    uint32_t drop = lidar_get_scan_drop_cnt();
    if (drop != last_drop || map_nav_observe_check_timeout(tick)) {
        last_drop = drop;
        map_nav_observe_reset();
        skip_scans = 2;
        last_scan = scan ? scan->scan_num : 0;
        printf("[MAP INTERRUPTED] reset pending window; blocked roads retained\r\n");
        return;
    }
    if (scan == NULL || !scan->scan_num || scan->scan_num == last_scan) {
        if (tick - last_wait >= 2000U) {
            last_wait = tick;
            printf("[MAP WAIT] accepted=%lu xor_err=%lu drop=%lu log_drop=%lu\r\n",
                   (unsigned long)cloud.scan_num, (unsigned long)lidar_get_xor_err_cnt(),
                   (unsigned long)drop, (unsigned long)debug_get_err_bytes());
        }
        return;
    }
    last_scan = scan->scan_num;
    last_wait = tick;
    if (skip_scans) { skip_scans--; return; }
    uint32_t before = DWT->CYCCNT;
    if (!lidar_map_desk_process(scan, tick)) return;
    uint32_t cost = (DWT->CYCCNT - before) / (SystemCoreClock / 1000000U);
    uint32_t change = map_update_change_get();
    uint8_t changed = change != last_change;
    if (changed) {
        last_change = change;
        if (!map_plan_check(&path, map_unknown_allow)) {
            printf("[MAP PATH] BLOCKED -> REPLAN\r\n");
            map_plan_find(LIDAR_MAP_DESK_CURRENT_NODE, LIDAR_MAP_DESK_NEXT_NODE, map_unknown_allow, &path);
            desk_print_path(&path);
        }
    }
    if (last_print && !changed && tick - last_print < 1000U) return;
    last_print = tick;
    map_nav_observe_get_result(&observed);
    uint16_t front = 0, printed = 0;
    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        lidar_map_desk_candidate_t info;
        if (lidar_map_desk_candidate(i, &info) && info.in_front) front++;
    }
    printf("[MAP FRAME] scan=%lu front=%u task=%u tracks=%u confirmed=%u pending=0x%03X block=0x%03X age=%lums cost=%luus overflow=%u\r\n",
           (unsigned long)cloud.scan_num, front, observed.frame_candidates, observed.candidate_count,
           observed.confirmed_count, observed.pending_roads, observed.confirmed_roads,
           (unsigned long)(HAL_GetTick() - cloud.scan_tick), (unsigned long)cost, observed.overflow);
    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        lidar_map_desk_candidate_t info;
        if (!lidar_map_desk_candidate(i, &info) || !info.in_front || printed >= 16) continue;
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];
        const char *region = info.center_area == map_outside ? "OUTSIDE" :
            info.center_area == map_forbidden ? "FORBIDDEN" :
            info.center_cell.cell_type == cell_node ? "NODE" : "ROAD";
        printf("[MAP CAND] C%u a=%u~%u p=%u near=%.0f map=(%.1f,%.1f) size=(%.1f,%.1f) center=%s:%u pts_R/N/F/O=%u/%u/%u/%u roads=0x%03X task=%u\r\n",
               i, cluster->start_angle, cluster->end_angle, cluster->point_count, cluster->nearest_distance,
               cluster->center_point.x, cluster->center_point.y, cluster->width, cluster->height,
               region, info.center_cell.cell_id, info.road_points, info.node_points,
               info.forbidden_points, info.outside_points, info.roads, info.in_task);
        printed++;
    }
    if (front > printed) printf("[MAP CAND] print_limited=%u front_total=%u\r\n", printed, front);
    for (uint8_t i = 0; i < MAP_OBSERVE_MAX_CANDIDATES; i++) {
        const map_nav_observe_candidate_t *candidate = &observed.candidates[i];
        if (!candidate->valid) continue;
        printf("[MAP TRACK] T%u map=(%.1f,%.1f) hits=%u/%u now=%u confirmed=%u last_scan=%lu\r\n",
               i, candidate->position.x, candidate->position.y, candidate->hits, MAP_OBSERVE_WINDOW,
               candidate->current_hit, candidate->hits >= MAP_OBSERVE_MIN_HITS,
               (unsigned long)candidate->last_hit_scan);
    }
    printf("[MAP ROAD]");
    for (uint8_t i = 0; i < MAP_ROAD_COUNT; i++) {
        road_state_t state;
        map_road_get(i, &state);
        printf(" R%u=%s", i, state == road_blocked ? "BLOCKED" : state == road_free ? "FREE" : "UNKNOWN");
    }
    printf(" changes=%lu xor_err=%lu drop=%lu log_drop=%lu\r\n", (unsigned long)change,
           (unsigned long)lidar_get_xor_err_cnt(), (unsigned long)drop,
           (unsigned long)debug_get_err_bytes());
}
#endif
