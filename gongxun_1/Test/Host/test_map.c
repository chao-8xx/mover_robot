#include "headfile.h"

// 这里只构造聚类输入，不代表真实雷达已经看到了细杆。
point_cloud_t cloud = {0};
obstacle_result_t obstacle_result = {0};
static uint16_t failures = 0;
static uint16_t checks = 0;

static void check(uint8_t ok, const char *name) {
    checks++;
    printf("[MAP HOST] %s %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) failures++;
}

static map_scan_obs_t r8_box(void) {
    map_scan_obs_t box = {1200.0f, 1625.0f, 1190.0f, 1615.0f,
                          1210.0f, 1635.0f};
    return box;
}

static uint8_t road_is(uint8_t id, road_state_t expected) {
    road_state_t state;
    return map_road_get(id, &state) && state == expected;
}

static uint8_t obstacle_count(obs_state_t expected) {
    uint8_t count = 0;
    map_obs_t obs;
    for (uint8_t i = 0; i < MAP_OBS_MAX; i++) {
        if (map_obs_get(i, &obs) && obs.state == expected) count++;
    }
    return count;
}

static void test_candidate_lifetime(void) {
    map_scan_obs_t box = r8_box();
    map_obs_t obs;
    map_update_init(300.0f, 50.0f);
    check(map_update_scan(&box, 1, 1, 100) &&
          obstacle_count(obs_unknown) == 1 && road_is(8, road_unknown),
          "first hit remains candidate");
    check(map_update_scan(0, 0, 2, 200) && map_obs_get(0, &obs) &&
          obs.state == obs_unknown && obs.miss_num == 1,
          "one miss keeps candidate");
    check(map_update_scan(0, 0, 3, 300) &&
          obstacle_count(obs_unknown) == 0 && road_is(8, road_unknown),
          "two misses remove candidate");
    check(map_update_scan(&box, 1, 4, 400) &&
          obstacle_count(obs_unknown) == 1 && road_is(8, road_unknown),
          "hit after expiry starts new confirmation");
    check(map_update_scan(&box, 1, 5, 500) &&
          obstacle_count(obs_ok) == 1 && road_is(8, road_blocked),
          "second fresh hit blocks R8");
    uint32_t changes = map_update_change_get();
    check(map_update_scan(0, 0, 100, 10000) &&
          obstacle_count(obs_ok) == 1 && road_is(8, road_blocked) &&
          map_update_change_get() == changes,
          "confirmed obstacle and blocked road persist");

    map_update_init(300.0f, 50.0f);
    check(map_update_scan(&box, 1, 1, 100) &&
          map_update_scan(&box, 1, 3, 300) && road_is(8, road_blocked),
          "one skipped scan still permits confirmation");
    map_update_init(300.0f, 50.0f);
    check(map_update_scan(&box, 1, 1, 100) &&
          map_update_scan(&box, 1, 4, 400) && road_is(8, road_unknown),
          "two skipped scans expire old candidate");
}

static void test_configuration_and_input(void) {
    map_scan_obs_t box = r8_box();
    map_update_init(300.0f, 50.0f);
    map_update_scan(&box, 1, 1, 100);
    map_update_scan(&box, 1, 2, 200);
    check(!map_update_init(NAN, 50.0f) && !map_update_is_ready() &&
          road_is(8, road_blocked) && !map_update_scan(&box, 1, 3, 300),
          "invalid config disables updates and preserves blocking");
    check(map_update_init(300.0f, 50.0f) && map_update_is_ready() &&
          road_is(8, road_unknown) && obstacle_count(obs_ok) == 0,
          "valid explicit reset clears map");
    check(map_update_scan(0, 0, 1, 100) && road_is(8, road_unknown),
          "empty scan does not claim road free");
    check(!map_update_scan(&box, 1, 1, 101) &&
          !map_update_scan(&box, 1, 0, 102) &&
          !map_update_scan(0, 1, 2, 200) &&
          !map_update_scan(&box, MAP_OBS_MAX + 1, 2, 200),
          "duplicate backwards null and oversized scans reject");
    box.center_x = NAN;
    check(map_update_scan(&box, 1, 2, 200) &&
          obstacle_count(obs_unknown) == 0 && road_is(8, road_unknown),
          "nonfinite box does not enter map");
    box = r8_box();
    box.min_x = box.max_x + 1.0f;
    check(map_update_scan(&box, 1, 3, 300) &&
          obstacle_count(obs_unknown) == 0,
          "inverted box does not enter map");
    box = r8_box();
    box.center_x = box.min_x = box.max_x = 775.0f;
    box.center_y = box.min_y = box.max_y = 775.0f;
    check(map_update_scan(&box, 1, 4, 400) &&
          obstacle_count(obs_unknown) == 0,
          "static forbidden center is excluded");
    check(map_check_area(NAN, 1200.0f) == map_outside &&
          map_check_area(1200.0f, INFINITY) == map_outside,
          "nonfinite map coordinates reject");
}

static void set_adapter_scan(uint32_t scan_num, uint32_t flag) {
    map_scan_obs_t box = r8_box();
    memset(&cloud, 0, sizeof(cloud));
    memset(&obstacle_result, 0, sizeof(obstacle_result));
    cloud.scan_num = scan_num;
    cloud.update_flag = flag;
    obstacle_result.cloud_flag = flag;
    obstacle_result.cluster_count = 1;
    obstacle_result.update_tick = scan_num * 100;
    obstacle_result.clusters[0].valid = 1;
    obstacle_result.clusters[0].center_point.x = box.center_x;
    obstacle_result.clusters[0].center_point.y = box.center_y;
    obstacle_result.clusters[0].min_x = box.min_x;
    obstacle_result.clusters[0].min_y = box.min_y;
    obstacle_result.clusters[0].max_x = box.max_x;
    obstacle_result.clusters[0].max_y = box.max_y;
}

static void test_lidar_adapter(void) {
    map_update_init(300.0f, 50.0f);
    set_adapter_scan(1, 1);
    check(!map_lidar_handle(0) && obstacle_count(obs_unknown) == 0,
          "invalid pose never writes cluster into map");
    obstacle_result.cloud_flag = 2;
    check(!map_lidar_handle(1) && obstacle_count(obs_unknown) == 0,
          "cluster from different cloud rejects");
    obstacle_result.cloud_flag = 1;
    obstacle_result.cluster_count = CLUSTER_MAX_COUNT + 1;
    check(!map_lidar_handle(1), "oversized cluster list rejects");
    obstacle_result.cluster_count = 1;
    check(map_lidar_handle(1) && road_is(8, road_unknown) &&
          obstacle_count(obs_unknown) == 1,
          "valid current cluster enters candidate map");
    check(!map_lidar_handle(1) && road_is(8, road_unknown),
          "same cloud cannot confirm twice");
    set_adapter_scan(2, 2);
    check(map_lidar_handle(1) && road_is(8, road_blocked),
          "second cloud confirms road blocking");
    map_update_init(0.0f, 50.0f);
    set_adapter_scan(3, 3);
    check(!map_lidar_handle(1) && road_is(8, road_blocked),
          "unconfigured map rejects lidar adapter");
}

static void test_path_input(void) {
    map_path_t path = {0};
    path.node_count = 1;
    path.nodes[0] = MAP_NO_ID;
    check(!map_plan_check(&path, map_unknown_allow) &&
          !map_plan_check(&path, map_unknown_stop),
          "invalid single node path rejects");
    path.nodes[0] = MAP_NODE_COUNT;
    check(!map_plan_check(&path, map_unknown_allow),
          "first node outside map rejects");
    path.nodes[0] = 4;
    check(map_plan_check(&path, map_unknown_allow) &&
          map_plan_check(&path, map_unknown_stop),
          "valid single node path accepts");
    path.node_count = 2;
    path.road_count = 1;
    path.nodes[1] = MAP_NO_ID;
    check(!map_plan_check(&path, map_unknown_allow),
          "invalid node in longer path rejects");
}

static void test_time_and_pause(void) {
    map_scan_obs_t box = r8_box();
    map_obs_t old;
    map_update_init(300.0f, 50.0f);
    map_update_scan(&box, 1, 1, 0);
    map_update_scan(&box, 1, 2, MAP_CANDIDATE_TIMEOUT_MS + 1);
    check(road_is(8, road_unknown) && map_obs_get(0, &old) && old.seen_num == 1,
          "long reception gap expires candidate even when scan IDs are adjacent");
    map_update_scan(&box, 1, 3, MAP_CANDIDATE_TIMEOUT_MS + 2);
    check(road_is(8, road_blocked), "fresh second observation confirms after timeout");
    box.center_x = box.min_x = box.max_x = 775.0f;
    box.center_y = box.min_y = box.max_y = 2125.0f;
    map_update_scan(&box, 1, 4, MAP_CANDIDATE_TIMEOUT_MS + 3);
    map_update_clear_candidates();
    check(obstacle_count(obs_unknown) == 0 && obstacle_count(obs_ok) == 1 &&
          road_is(8, road_blocked) && map_update_change_get() == 1,
          "pause clears candidates and preserves confirmed blocking");
    box = r8_box();
    map_update_init(300.0f, 50.0f);
    map_update_scan(&box, 1, 1, UINT32_MAX - 50);
    map_update_scan(&box, 1, 2, 25);
    check(road_is(8, road_blocked), "tick wrap preserves a fresh candidate");
    check(!map_update_scan(&box, 1, 3, 24), "backwards timestamp rejects without changing map");
}

int main(void) {
    check(map_stage_test(), "existing embedded map stage checks");
    test_candidate_lifetime();
    test_configuration_and_input();
    test_lidar_adapter();
    test_path_input();
    test_time_and_pause();
    printf("[MAP HOST] RESULT %u passed, %u failed\n",
           (unsigned)(checks - failures), (unsigned)failures);
    return failures ? 1 : 0;
}
