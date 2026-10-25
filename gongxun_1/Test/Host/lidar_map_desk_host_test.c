#include "headfile.h"
#include "lidar_map_desk_test.h"

static uint32_t tick = 100, scan_num;
static unsigned checks, failures;
float my_yaw;
uint32_t HAL_GetTick(void) { return tick; }
static void check(uint8_t ok, const char *name) {
    checks++;
    if (!ok) { failures++; printf("[MAP DESK TEST] FAIL %s\n", name); }
}
uint8_t car_is_busy(void) { return 0; }
Chassis_Mode car_get_state(void) { return CHASSIS_IDLE; }
void car_stop(void) { check(0, "desk test sent a chassis stop command"); }
void car_rotate_yaw(float target) { (void)target; check(0, "desk test rotated chassis"); }
void car_run_distance(int16_t speed, float x, float y) {
    (void)speed; (void)x; (void)y; check(0, "desk test moved chassis");
}
static uint8_t road_is(uint8_t road, road_state_t expected) {
    road_state_t state;
    return map_road_get(road, &state) && state == expected;
}
static uint8_t all_unknown(void) {
    for (uint8_t i = 0; i < MAP_ROAD_COUNT; i++) if (!road_is(i, road_unknown)) return 0;
    return 1;
}
static map_nav_observe_result_t observed(void) {
    map_nav_observe_result_t out;
    map_nav_observe_get_result(&out);
    return out;
}
static uint8_t frame(uint16_t angle, uint16_t distance) {
    lidar_scan_t scan = {0};
    scan.scan_num = ++scan_num;
    scan.end_time = tick += 100;
    if (distance) {
        scan.valid[angle] = 1;
        scan.distance[angle] = distance;
    }
    return lidar_map_desk_process(&scan, tick);
}
int main(void) {
    check(lidar_map_desk_begin() && all_unknown(), "begin selects N1 to N4 and clears test map");
    check(observed().road_id == 8 && observed().current_node == 1 && observed().next_node == 4,
          "desk observation owns the intended R8 task");
    check(frame(0, 500), "real raw scan passes shared coordinate and cluster code");
    check(obstacle_result.cluster_count == 1 && fabsf(cloud.points[0].map_point.x - 1200) < 0.01f &&
          fabsf(cloud.points[0].map_point.y - 1625) < 0.01f,
          "front 500mm transforms to map 1200,1625");
    lidar_map_desk_candidate_t info;
    check(lidar_map_desk_candidate(0, &info) && info.center_area == map_normal &&
          info.center_cell.cell_type == cell_road && info.center_cell.cell_id == 8 &&
          info.road_points == 1 && info.roads == (1U << 8) && info.in_task && info.in_front,
          "actual point diagnosis assigns the candidate to R8");
    check(road_is(8, road_unknown) && observed().candidates[0].hits == 1,
          "first scan remains pending");
    map_path_t path;
    check(map_plan_find(1, 4, map_unknown_allow, &path) && path.road_count == 1 && path.roads[0] == 8,
          "initial candidate route crosses R8");
    frame(0, 500);
    check(road_is(8, road_unknown), "two matching hits do not confirm yet");
    frame(0, 500);
    check(road_is(8, road_blocked) && observed().confirmed_roads == (1U << 8),
          "third same-position hit blocks R8 through the production window");
    check(!map_plan_check(&path, map_unknown_allow) && map_plan_find(1, 4, map_unknown_allow, &path),
          "blocking R8 invalidates the old route and permits a new route");
    uint8_t includes_r8 = 0;
    for (uint8_t i = 0; i < path.road_count; i++) if (path.roads[i] == 8) includes_r8 = 1;
    check(!includes_r8, "replanned path avoids the blocked road without executing chassis commands");
    frame(0, 0);
    check(!observed().candidates[0].current_hit && road_is(8, road_blocked),
          "removing the pillar does not fabricate a new return or unblock R8");
    check(lidar_map_desk_begin() && all_unknown() && !observed().candidate_count,
          "reset clears both blocked roads and tracked evidence");

    // 使用用户三轮摆放的代表角度与距离，不能继续受旧400~600mm框限制。
    const struct { uint16_t angle, distance; } pillars[] = {{30, 355}, {0, 527}, {335, 395}, {0, 900}};
    for (unsigned i = 0; i < sizeof(pillars) / sizeof(pillars[0]); i++) {
        lidar_map_desk_begin();
        for (unsigned f = 0; f < 3; f++) check(frame(pillars[i].angle, pillars[i].distance), "desk accepts a fresh pillar scan");
        check(road_is(8, road_blocked) && lidar_map_desk_candidate(0, &info) && info.in_task,
              "near angled and farther candidates reach the intended road or front node");
    }
    check(info.center_cell.cell_type == cell_node && info.center_cell.cell_id == 4 &&
          info.node_points == 1 && (info.roads & (1U << 8)),
          "actual front-node points retain their connected-road attribution");
    check(road_is(2, road_blocked) && road_is(3, road_blocked) &&
          road_is(8, road_blocked) && road_is(9, road_blocked),
          "confirmed N4 points affect all four connected roads under the existing node policy");

    // 同一候选仍符合第一层，地图层负责剔除禁区、其他道路和当前节点。
    const struct { uint16_t angle, distance; map_area_t area; map_cell_type_t type; uint8_t id; } background[] = {
        {330, 674, map_forbidden, cell_forbidden, MAP_NO_ID},
        {340, 1040, map_normal, cell_road, 2},
        {0, 220, map_normal, cell_node, 1},
        {0, 1300, map_normal, cell_road, 9}
    };
    for (unsigned i = 0; i < sizeof(background) / sizeof(background[0]); i++) {
        lidar_map_desk_begin();
        for (unsigned f = 0; f < 5; f++) frame(background[i].angle, background[i].distance);
        check(obstacle_result.cluster_count == 1 && lidar_map_desk_candidate(0, &info) &&
              info.center_area == background[i].area && info.center_cell.cell_type == background[i].type &&
              info.center_cell.cell_id == background[i].id && !info.in_task,
              "background candidate has the expected map region and is outside the R8 task");
        check(all_unknown() && !observed().frame_candidates && !observed().candidate_count,
              "excluded map regions do not gain confirmation votes or block roads");
    }

    // 道路边缘按实际点半开区间判断，包围框中心不代替点的归属。
    lidar_map_desk_begin();
    memset(&cloud, 0, sizeof(cloud)); memset(&obstacle_result, 0, sizeof(obstacle_result));
    cloud.points[0] = (lidar_point_t){.valid=1, .distance=500, .map_point={999,1625}};
    cloud.points[1] = (lidar_point_t){.valid=1, .distance=500, .map_point={1000,1625}};
    obstacle_result.cluster_count = 1;
    obstacle_result.clusters[0] = (lidar_cluster_t){.start_angle=0,.end_angle=1,.point_count=2,
        .center_point={999.5f,1625},.valid=1};
    check(lidar_map_desk_candidate(0, &info) && info.forbidden_points == 1 && info.road_points == 1 &&
          info.center_area == map_forbidden && info.in_task && info.roads == (1U << 8),
          "road-edge actual point at x1000 is selected even with the group center outside");
    cloud.points[0].map_point.x = -1;
    cloud.points[1].valid = 0;
    obstacle_result.clusters[0].center_point = (point2d_t){-1,1625};
    check(lidar_map_desk_candidate(0, &info) && info.center_area == map_outside &&
          info.outside_points == 1 && !info.roads && !info.in_task,
          "out-of-field actual points never acquire road attribution");
    check(!lidar_map_desk_candidate(1, &info) && !lidar_map_desk_candidate(0, NULL),
          "diagnosis rejects invalid indices and missing output");

    lidar_map_desk_begin();
    check(frame(40, 1000) && cloud.points[40].valid,
          "naked radar entry intentionally bypasses installed vehicle masks");
    lidar_scan_t scan = {0};
    scan.scan_num = ++scan_num; scan.end_time = tick += 100;
    scan.valid[40] = 1; scan.distance[40] = 1000;
    check(point_cloud_update(&scan, 1200, 2125, 180, tick) && !cloud.points[40].valid,
          "normal production entry still applies the configured vehicle mask");

    lidar_map_desk_begin();
    for (unsigned f = 0; f < 2; f++) frame(0, 500);
    scan = (lidar_scan_t){.scan_num=scan_num,.end_time=tick};
    check(!lidar_map_desk_process(&scan, tick) && observed().candidates[0].hits == 2,
          "duplicate scan cannot supply the third confirmation vote");
    scan_num++;
    frame(0, 500);
    check(road_is(8, road_unknown) && observed().candidates[0].hits == 1,
          "scan gaps discard pending evidence");
    frame(0, 500);
    tick += MAP_SCAN_TIMEOUT_MS + 1;
    check(map_nav_observe_check_timeout(tick) && !observed().candidate_count && road_is(8, road_unknown),
          "silent timeout clears the pending desk observation");
    scan.scan_num = ++scan_num; scan.end_time = tick - MAP_SCAN_TIMEOUT_MS - 1;
    check(!lidar_map_desk_process(&scan, tick) && !lidar_map_desk_process(NULL, tick),
          "old or absent scans cannot enter fixed-pose mapping");
    printf("[MAP DESK TEST] checks=%u failures=%u\n", checks, failures);
    return failures != 0;
}
