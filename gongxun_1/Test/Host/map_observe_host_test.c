#include "headfile.h"
#if defined(__has_include)
#if __has_include("map_nav.h")
#include "map_nav.h"
#define OBSERVE_AVAILABLE 1
#endif
#endif

static uint32_t tick = 100;
uint32_t HAL_GetTick(void) { return tick; }
#ifndef OBSERVE_AVAILABLE
int main(void) {
    puts("[OBSERVE TEST] FAIL task-selected same-position 3-of-5 confirmation is missing");
    return 1;
}
#else
static uint32_t scan_num;
static unsigned checks, failures;
static float car_x = 1200, car_y = 275, yaw;
static void check(uint8_t ok, const char *name) {
    checks++;
    if (!ok) { failures++; printf("[OBSERVE TEST] FAIL %s\n", name); }
}
// 合并到导航文件后仍只验证停车观察，任何底盘动作都应使本测试失败。
float my_yaw;
static uint8_t expected_stops;
uint8_t car_is_busy(void) { return 0; }
Chassis_Mode car_get_state(void) { return CHASSIS_IDLE; }
void car_stop(void) {
    if (expected_stops) expected_stops--;
    else check(0, "observation unexpectedly stopped chassis");
}
void car_rotate_yaw(float target) { (void)target; check(0, "observation unexpectedly rotated chassis"); }
void car_run_distance(int16_t speed, float x, float y) {
    (void)speed; (void)x; (void)y; check(0, "observation unexpectedly moved chassis");
}
static uint8_t road_is(uint8_t id, road_state_t expected) {
    road_state_t state;
    return map_road_get(id, &state) && state == expected;
}
static map_nav_observe_result_t result(void) {
    map_nav_observe_result_t out;
    memset(&out, 0, sizeof(out));
    map_nav_observe_get_result(&out);
    return out;
}
static void setup(void) {
    map_nav_observe_end();
    map_update_init();
    tf_init();
    memset(&cloud, 0, sizeof(cloud));
    memset(&obstacle_result, 0, sizeof(obstacle_result));
    car_x = 1200; car_y = 275; yaw = 0;
    check(map_nav_observe_begin(7, 4), "select adjacent north road");
}
static void point(float x, float y) {
    float a = atan2f(-(x - car_x), y - car_y) * 180.0f / TF_PI - yaw;
    while (a < 0) a += 360;
    uint16_t angle = (uint16_t)(a + 0.5f) % 360;
    cloud.points[angle].valid = 1;
    cloud.points[angle].map_point = (point2d_t){x, y};
    // 注入已完成TF的远距单点候选，单圈几何转换另有raw回归。
    cloud.points[angle].distance = 1300;
}
static uint8_t frame(uint8_t hit, float x, float y) {
    memset(cloud.points, 0, sizeof(cloud.points));
    if (hit) point(x, y);
    cloud.car_x = car_x; cloud.car_y = car_y; cloud.car_angle = yaw;
    cloud.scan_num = ++scan_num; cloud.scan_tick = tick += 100;
    cloud.update_flag++;
    obstacle_cluster_update(&cloud, tick);
    return map_nav_observe_update(1);
}
int main(void) {
    setup();
    check(frame(1, 1200, 800) && road_is(9, road_unknown), "first actual hit remains pending");
    check(!map_nav_observe_update(1) && result().frames == 1, "duplicate frame cannot add evidence");
    frame(0, 0, 0); frame(1, 1210, 810);
    check(road_is(9, road_unknown), "two intermittent hits cannot confirm");
    frame(1, 1190, 795);
    check(road_is(9, road_blocked) && result().confirmed_roads == (1U << 9),
          "third matching hit directly blocks without second confirmation");
    check(result().first_confirm_ms == 300, "confirmation time uses acquisition timestamps");
    check(result().candidates[0].current_hit && result().candidates[0].hits == 3,
          "diagnostic distinguishes current return and window count");
    frame(0, 0, 0);
    check(!result().candidates[0].current_hit && road_is(9, road_blocked), "miss never fabricates a fresh hit or unblocks");
    for (unsigned i = 0; i < 5; i++) frame(0, 0, 0);
    check(result().candidate_count == 0 && road_is(9, road_blocked), "expired pending tracks do not erase blocked map");

    setup();
    frame(1, 1200, 650); frame(1, 1200, 850); frame(1, 1200, 950);
    check(road_is(9, road_unknown), "different positions on one road cannot combine votes");
    setup();
    frame(1, 1200, 650); frame(1, 1200, 720); frame(1, 1200, 790);
    check(road_is(9, road_unknown), "fixed anchor prevents chain drift beyond matching radius");
    setup();
    frame(1, 1200, 800); frame(1, 1280, 800); frame(1, 1200, 800);
    check(road_is(9, road_blocked), "matching radius boundary is inclusive");
    setup();
    frame(1, 1200, 800); frame(1, 1281, 800); frame(1, 1200, 800);
    check(road_is(9, road_unknown), "outside matching radius starts separate evidence");
    setup();
    frame(1, 1200, 800); frame(0, 0, 0); frame(0, 0, 0); frame(0, 0, 0); frame(0, 0, 0);
    frame(1, 1200, 800); frame(1, 1200, 800);
    check(road_is(9, road_unknown), "hits outside the latest five scans cannot confirm");
    setup(); frame(1, 1200, 800); frame(1, 1200, 800); scan_num++;
    frame(1, 1200, 800);
    check(road_is(9, road_unknown) && result().frames == 1, "scan gap starts a new confirmation window");
    frame(1, 1200, 800); tick += MAP_SCAN_TIMEOUT_MS + 1;
    frame(1, 1200, 800);
    check(road_is(9, road_unknown) && result().frames == 1, "long acquisition gap clears old evidence");
    setup(); frame(1, 1200, 800); frame(1, 1200, 800);
    check(!map_nav_observe_update(0), "invalid pose interrupts observation");
    frame(1, 1200, 800);
    check(road_is(9, road_unknown), "pose recovery cannot reuse pending hits");
    setup(); frame(1, 1200, 800); frame(1, 1200, 800);
    check(map_nav_observe_begin(7, 4), "same request is idempotent");
    frame(1, 1200, 800);
    check(road_is(9, road_blocked), "repeated task selection does not erase window");
    setup(); frame(1, 1200, 800); frame(1, 1200, 800);
    check(map_nav_observe_begin(7, 6), "select backup west road");
    frame(1, 1200, 800);
    check(road_is(9, road_unknown) && result().frame_candidates == 0, "task direction switch excludes former road");
    check(!map_nav_observe_begin(7, 0) && !map_nav_observe_is_active(), "nonadjacent task is rejected and disables stale selection");

    setup(); yaw = 90;
    for (unsigned i = 0; i < 3; i++) frame(1, 1200, 800);
    check(road_is(9, road_blocked), "heading changes radar sector without changing selected world road");
    setup();
    frame(0, 0, 0);
    check(result().angle_start_deg > result().angle_end_deg, "north-facing observation sector crosses zero correctly");
    for (unsigned i = 0; i < 5; i++) frame(1, 800, 1200);
    check(road_is(2, road_unknown) && road_is(9, road_unknown), "off-task road cannot become a confirmed obstacle");
    check(!map_update_confirm_free(9, cloud.scan_num, tick), "empty selected sector cannot prove a road is free");

    setup(); frame(1, 1200, 800);
    obstacle_result.cloud_flag++;
    check(!map_nav_observe_update(1), "mismatched cloud and clusters are rejected");
    frame(1, 1200, 800); frame(1, 1200, 800);
    check(road_is(9, road_unknown), "mismatched frame breaks pending history");
    setup(); frame(1, 1200, 800); frame(1, 1200, 800);
    tick += MAP_SCAN_TIMEOUT_MS + 1;
    check(!map_nav_observe_update(1), "stale acquisition cannot confirm in a fast main loop");

    // 大聚类即使只有一部分落在道路角区，也不能先裁剪成小柱。
    setup();
    for (unsigned i = 0; i < 3; i++) {
        memset(cloud.points, 0, sizeof(cloud.points));
        for (unsigned j = 0; j < 13; j++) point(1125 + 25 * j, 800);
        cloud.car_x = car_x; cloud.car_y = car_y; cloud.car_angle = yaw;
        cloud.scan_num = ++scan_num; cloud.scan_tick = tick += 100; cloud.update_flag++;
        obstacle_cluster_update(&cloud, tick); map_nav_observe_update(1);
    }
    check(road_is(9, road_unknown), "full cluster size is checked before selecting sector points");

    // 已确认入口共享扫描去重，不再要求连续两圈，但不允许旧扫描封路。
    map_nav_observe_end(); map_update_init();
    check(map_update_confirmed_roads(1U << 8, 1U << 8, ++scan_num, tick += 100, 0)
          && road_is(8, road_blocked), "confirmed map entry blocks immediately");
    check(!map_update_confirmed_roads(1U << 9, 1U << 9, scan_num, tick, 0)
          && road_is(9, road_unknown), "duplicate confirmed message cannot block another road");
    check(!map_update_confirmed_roads(1U << MAP_ROAD_COUNT, 0, ++scan_num, tick += 100, 0),
          "confirmed entry rejects invalid road masks");

    // 疑似票不能被显式放行；选区处理后空帧也不伪造FREE。
    map_update_init();
    check(map_update_confirmed_roads(0, 1U << 9, ++scan_num, tick += 100, 1), "pending evidence reaches map");
    check(!map_update_confirm_free(9, scan_num, tick), "pending obstacle prevents explicit free");
    check(map_update_confirmed_roads(0, 0, ++scan_num, tick += 100, 0) &&
          !map_update_confirm_free(9, scan_num, tick), "selected empty frame cannot authorize explicit free");

    // 同圈多个候选碎片不能凑票。手动填入已分割聚类，仅检查跨圈关联。
    setup();
    for (unsigned f = 0; f < 2; f++) {
        memset(cloud.points, 0, sizeof(cloud.points));
        memset(&obstacle_result, 0, sizeof(obstacle_result));
        cloud.car_x = car_x; cloud.car_y = car_y; cloud.car_angle = yaw;
        cloud.scan_num = ++scan_num; cloud.scan_tick = tick += 100; cloud.update_flag++;
        obstacle_result.cloud_flag = cloud.update_flag;
        obstacle_result.cluster_count = 2;
        for (unsigned j = 0; j < 2; j++) {
            cloud.points[j].valid = 1;
            cloud.points[j].map_point = (point2d_t){1200 + 10 * j, 800};
            cloud.points[j].distance = 1300;
            obstacle_result.clusters[j] = (lidar_cluster_t){.start_angle=j, .end_angle=j,
                .point_count=1, .center_point={1200 + 10 * j,800}, .nearest_distance=1300, .valid=1};
        }
        check(map_nav_observe_update(1), "accept multiple compact candidates in one scan");
    }
    check(road_is(9, road_unknown) && result().candidates[0].hits == 2,
          "multiple fragments cannot supply multiple votes per scan");

    // 17个已分割位置超过16槽，容量必须显式标记，禁止静默覆盖旧证据。
    setup();
    memset(cloud.points, 0, sizeof(cloud.points));
    memset(&obstacle_result, 0, sizeof(obstacle_result));
    cloud.car_x = car_x; cloud.car_y = car_y; cloud.car_angle = yaw;
    cloud.scan_num = ++scan_num; cloud.scan_tick = tick += 100; cloud.update_flag++;
    obstacle_result.cloud_flag = cloud.update_flag;
    obstacle_result.cluster_count = MAP_OBSERVE_MAX_CANDIDATES + 1;
    for (unsigned j = 0; j < MAP_OBSERVE_MAX_CANDIDATES + 1; j++) {
        unsigned angle = (340 + 2 * j) % 360;
        point2d_t xy = {1050 + (j % 3) * 150, 600 + (j / 3) * 110};
        cloud.points[angle].valid = 1;
        cloud.points[angle].map_point = xy;
        cloud.points[angle].distance = 1300;
        obstacle_result.clusters[j] = (lidar_cluster_t){.start_angle=angle, .end_angle=angle,
            .point_count=1, .center_point=xy, .nearest_distance=1300, .valid=1};
    }
    check(map_nav_observe_update(1) && result().overflow && result().candidate_count == MAP_OBSERVE_MAX_CANDIDATES,
          "candidate overflow is explicit and does not overwrite tracked positions");

    setup();
    frame(1, 1200, 800);
    obstacle_result.truncated = 1;
    cloud.scan_num = ++scan_num; cloud.scan_tick = tick += 100; cloud.update_flag++;
    obstacle_result.cloud_flag = cloud.update_flag;
    check(map_nav_observe_update(1) && result().overflow, "upstream truncation remains visible");
    frame(1, 1200, 800);
    check(road_is(9, road_blocked), "truncated frame can still contribute actual observed danger");

    setup();
    car_x = 1200; car_y = 1200;
    check(!frame(1, 1200, 800), "wrong current node rejects a stale task selection");
    setup(); frame(1, 1200, 800); frame(1, 1200, 800);
    map_nav_observe_reset();
    check(map_nav_observe_is_active() && !map_nav_observe_update(1), "motion reset keeps direction but rejects old published cloud");
    frame(1, 1200, 800);
    check(road_is(9, road_unknown), "motion reset discards previous location votes");

    // 真实raw -> TF -> 单圈聚类 -> 观察窗口 -> 地图，不使用注入的假距离。
    setup();
    lidar_scan_t raw = {0};
    raw.valid[359] = raw.valid[0] = raw.valid[1] = 1;
    raw.distance[359] = raw.distance[0] = raw.distance[1] = 500;
    for (unsigned j = 0; j < 3; j++) {
        raw.scan_num = ++scan_num; raw.end_time = tick += 100;
        check(point_cloud_update(&raw, 1200, 275, 0, tick), "actual TF accepts a new raw scan");
        obstacle_cluster_update(&cloud, tick);
        check(map_nav_observe_update(1), "actual raw cluster enters selected road observation");
    }
    check(road_is(9, road_blocked) && result().candidates[0].point_count == 3,
          "raw three-point near cylinder confirms after three distinct scans");
    check(!map_nav_observe_get_result(NULL), "null diagnostic destination rejected");
    setup(); frame(1, 1200, 800); frame(1, 1200, 800);
    check(!map_nav_observe_check_timeout(tick + MAP_SCAN_TIMEOUT_MS), "observation remains current at timeout boundary");
    tick += MAP_SCAN_TIMEOUT_MS + 1;
    check(map_nav_observe_check_timeout(tick) && result().frames == 0 && result().candidate_count == 0 &&
          result().pending_roads == 0, "silent radar timeout clears evidence without a new scan");
    check(!map_nav_observe_check_timeout(tick), "timeout recovery barrier requested once");

    map_rect_t area;
    float center, half;
    check(map_plan_observe_area(9, 4, &area) && area.min_y == 550 && area.max_y == 1400,
          "planner returns the selected road and forward node area");
    check(!map_plan_observe_area(9, 5, &area), "planner rejects unrelated next node");
    check(!map_plan_observe_area(12, 4, &area) && !map_plan_observe_area(9, 4, NULL),
          "planner rejects invalid road or missing area output");
    check(map_plan_observe_area(9, 4, &area) &&
          map_plan_observe_sector(&area, 1200, 275, 0, 3, &center, &half) &&
          fabsf(center) < 0.01f && half > 36 && half < 40,
          "planner computes north radar sector from road geometry");
    check(map_plan_observe_sector(&area, 1200, 275, 90, 3, &center, &half) && fabsf(center + 90) < 0.01f,
          "planner converts world direction to current radar bearing");
    check(!map_plan_observe_sector(&area, NAN, 275, 0, 3, &center, &half) &&
          !map_plan_observe_sector(&area, 1200, 275, 0, -1, &center, &half),
          "planner rejects invalid view parameters");

    // 使用实际导航plan->scan状态验证方向归属，不执行底盘移动或转向。
    setup(); map_nav_init(); map_nav_observe_sync();
    check(map_nav_observe_is_active(), "idle navigation preserves a manually selected observation");
    map_pose_init();
    map_pose_update(1200, 275, 0, tick, 1);
    check(map_nav_start(7, 4, map_unknown_allow, 100), "navigation accepts the observation task start");
    map_nav_task();
    map_nav_observe_begin(7, 6);
    map_nav_observe_sync();
    check(result().current_node == 7 && result().next_node == 4 && result().road_id == 9,
          "navigation scan owns the next planned road selection");
    frame(1, 1200, 800); map_nav_observe_sync(); frame(0, 0, 0);
    frame(1, 1200, 800); map_nav_observe_sync(); frame(1, 1200, 800);
    check(road_is(9, road_blocked), "navigation synchronization preserves intermittent confirmation history");
    expected_stops = 1; // 主动重置活动导航应停车一次，不属于观察模块发车。
    map_nav_init();
    check(!expected_stops && !map_nav_observe_is_active(), "navigation reset releases its observation ownership");
    map_nav_observe_begin(7, 6); map_nav_observe_sync();
    check(result().next_node == 6, "manual selection survives after navigation ownership is released");
    printf("[OBSERVE TEST] checks=%u failures=%u context_bytes=%zu\n", checks, failures, sizeof(map_nav_observe_result_t));
    return failures != 0;
}
#endif
