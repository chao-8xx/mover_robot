#include "headfile.h"

static uint8_t map_test_fail = 0;

static void map_test_check(uint8_t ok, const char *name) {
    if (ok) printf("[MAP TEST] PASS %s\r\n", name);
    else {
        printf("[MAP TEST] FAIL %s\r\n", name);
        map_test_fail++;
    }
}

// 不接雷达即可检查坐标、存储、道路封锁和重新规划
uint8_t map_stage_test(void) {
    uint8_t cell_x, cell_y, road_id;
    map_cell_t cell;
    map_node_t node;
    map_obs_t obs_a = {0}, obs_b = {0}, read_obs;
    road_state_t state;
    map_path_t old_path, new_path;
    map_scan_obs_t box = {1200.0f, 1625.0f, 1190.0f, 1615.0f,
                          1210.0f, 1635.0f};

    map_test_fail = 0;
    map_test_check(map_xy_to_cell(1200.0f, 1625.0f, &cell_x, &cell_y) &&
                   cell_x == 2 && cell_y == 3 &&
                   map_cell_get(cell_x, cell_y, &cell) &&
                   cell.cell_type == cell_road && cell.cell_id == 8,
                   "coordinate to R8");
    map_test_check(map_check_area(750.0f, 750.0f) == map_forbidden &&
                   map_check_area(2400.0f, 1200.0f) == map_outside &&
                   map_check_area(0.0f, 1200.0f) == map_normal,
                   "field and forbidden area");
    map_test_check(map_node_get(4, &node) && node.x == 1200.0f &&
                   node.y == 1200.0f &&
                   map_road_find(1, 4, &road_id) && road_id == 8,
                   "N4 and R8 topology");

    uint8_t node_seen[MAP_NODE_COUNT] = {0};
    uint8_t road_seen[MAP_ROAD_COUNT] = {0};
    uint8_t forbidden_count = 0;
    uint8_t all_cells_ok = 1;
    const float band_center[5] = {275.0f, 775.0f, 1200.0f, 1625.0f, 2125.0f};
    for (uint8_t y = 0; y < MAP_CELL_HEIGHT; y++) {
        for (uint8_t x = 0; x < MAP_CELL_WIDTH; x++) {
            if (!map_xy_to_cell(band_center[x], band_center[y], &cell_x, &cell_y) ||
                cell_x != x || cell_y != y || !map_cell_get(x, y, &cell)) {
                all_cells_ok = 0;
                continue;
            }
            if (cell.cell_type == cell_node && cell.cell_id < MAP_NODE_COUNT)
                node_seen[cell.cell_id]++;
            else if (cell.cell_type == cell_road && cell.cell_id < MAP_ROAD_COUNT)
                road_seen[cell.cell_id]++;
            else if (cell.cell_type == cell_forbidden && cell.cell_id == MAP_NO_ID)
                forbidden_count++;
            else all_cells_ok = 0;
        }
    }
    for (uint8_t i = 0; i < MAP_NODE_COUNT; i++)
        if (node_seen[i] != 1) all_cells_ok = 0;
    for (uint8_t i = 0; i < MAP_ROAD_COUNT; i++)
        if (road_seen[i] != 1) all_cells_ok = 0;
    map_test_check(all_cells_ok && forbidden_count == 4,
                   "all 25 cells and unique IDs");

    map_data_init();
    obs_a.state = obs_unknown;
    obs_a.center_x = obs_a.min_x = obs_a.max_x = 500.0f;
    obs_a.center_y = obs_a.min_y = obs_a.max_y = 500.0f;
    obs_b = obs_a;
    obs_b.center_x = obs_b.min_x = obs_b.max_x = 600.0f;
    map_test_check(map_obs_set(0, &obs_a) && map_obs_set(1, &obs_b) &&
                   map_obs_clear(0) && map_obs_get(1, &read_obs) &&
                   read_obs.center_x == 600.0f,
                   "clear one obstacle only");

    map_test_check(!map_update_init(0.0f, 50.0f) &&
                   !map_update_is_ready() &&
                   map_update_init(300.0f, 50.0f),
                   "width must be configured");
    map_test_check(map_plan_find(1, 4, map_unknown_allow, &old_path) &&
                   old_path.road_count == 1 && old_path.roads[0] == 8 &&
                   map_plan_road_state(&old_path, 0) == map_step_scan &&
                   !map_plan_find(1, 4, map_unknown_stop, &new_path),
                   "unknown road policy");
    map_test_check(map_plan_find(1, 1, map_unknown_stop, &new_path) &&
                   new_path.node_count == 1 && new_path.road_count == 0 &&
                   new_path.nodes[0] == 1,
                   "start equals goal");
    map_road_set(8, road_free);
    map_test_check(map_plan_find(1, 4, map_unknown_stop, &new_path) &&
                   new_path.road_count == 1 &&
                   map_plan_road_state(&new_path, 0) == map_step_go,
                   "confirmed free can go");
    map_road_set(8, road_unknown);

    map_test_check(map_update_scan(&box, 1, 1, 100) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   !map_update_scan(&box, 1, 1, 101),
                   "first scan and duplicate scan");
    map_test_check(map_update_scan(0, 0, 2, 200) &&
                   map_road_get(8, &state) && state == road_unknown &&
                   map_update_scan(&box, 1, 3, 300) &&
                   map_road_get(8, &state) && state == road_blocked &&
                   map_update_change_get() == 1,
                   "one miss then confirm R8");
    map_test_check(map_road_get(0, &state) && state == road_unknown &&
                   map_update_scan(0, 0, 4, 400) &&
                   map_road_get(8, &state) && state == road_blocked,
                   "other roads and blocked persistence");
    map_test_check(!map_plan_check(&old_path, map_unknown_allow) &&
                   map_plan_road_state(&old_path, 0) == map_step_stop &&
                   map_plan_find(1, 4, map_unknown_allow, &new_path) &&
                   new_path.road_count > 1 &&
                   map_plan_check(&new_path, map_unknown_allow),
                   "blocked road causes replan");

    // 两个相近的单圈聚类也只能算同一个候选
    map_update_init(300.0f, 50.0f);
    map_scan_obs_t same_scan[2] = {box, box};
    map_update_scan(same_scan, 2, 1, 100);
    map_test_check(map_road_get(8, &state) && state == road_unknown &&
                   map_obs_get(0, &read_obs) && read_obs.seen_num == 1,
                   "one scan cannot confirm twice");
    map_update_scan(&box, 1, 4, 400);
    map_test_check(map_road_get(8, &state) && state == road_unknown,
                   "missing scans remove candidate");

    // N4 附近有障碍时会同时封锁四个相邻出口
    box.center_x = 1200.0f;
    box.center_y = 1200.0f;
    box.min_x = 1190.0f;
    box.max_x = 1210.0f;
    box.min_y = 1190.0f;
    box.max_y = 1210.0f;
    map_update_init(300.0f, 50.0f);
    map_update_scan(&box, 1, 1, 100);
    map_update_scan(&box, 1, 2, 200);
    uint8_t center_blocked = 1;
    const uint8_t center_roads[4] = {2, 3, 8, 9};
    for (uint8_t i = 0; i < 4; i++) {
        if (!map_road_get(center_roads[i], &state) || state != road_blocked)
            center_blocked = 0;
    }
    map_test_check(center_blocked && map_update_change_get() == 4,
                   "N4 four exits");
    map_test_check(!map_plan_find(1, 4, map_unknown_allow, &new_path) &&
                   new_path.node_count == 0,
                   "N4 has no path");

    // 300 mm 正方形旋转后，角点可伸到中心线外约 212 mm
    box.center_x = box.min_x = box.max_x = 775.0f;
    box.center_y = box.min_y = box.max_y = 1915.0f;
    map_update_init(300.0f, 0.0f);
    map_update_scan(&box, 1, 1, 100);
    map_update_scan(&box, 1, 2, 200);
    map_test_check(map_road_get(0, &state) && state == road_blocked,
                   "rotated square safety radius");

    printf("[MAP TEST] %s\r\n", map_test_fail ? "FAILED" : "ALL PASS");
    return map_test_fail == 0;
}
