#include "headfile.h"

static uint32_t scan_num;
static uint32_t tick;
static unsigned int checks, failures;

uint32_t HAL_GetTick(void) { return tick; }

static void check(uint8_t ok, const char *name) {
    checks++;
    if (!ok) failures++;
    printf("[PILLAR TEST] %s %s\n", ok ? "PASS" : "FAIL", name);
}

static uint8_t road_is(uint8_t id, road_state_t expected) {
    road_state_t state;
    return map_road_get(id, &state) && state == expected;
}

static uint8_t all_unknown(void) {
    for (uint8_t i = 0; i < MAP_ROAD_COUNT; i++)
        if (!road_is(i, road_unknown)) return 0;
    return 1;
}

static void start_case(void) {
    map_update_init();
    memset(&cloud, 0, sizeof(cloud));
    memset(&obstacle_result, 0, sizeof(obstacle_result));
}

static void point(uint16_t angle, float x, float y) {
    cloud.points[angle].valid = 1;
    cloud.points[angle].distance = 500.0f;
    cloud.points[angle].map_point = (point2d_t){x, y};
}

static void small_pillar(void) {
    point(10, 1190, 1625);
    point(11, 1200, 1620);
    point(12, 1210, 1625);
}

static uint8_t frame(void) {
    cloud.scan_num = ++scan_num;
    cloud.scan_tick = tick += 100;
    cloud.update_flag++;
    obstacle_cluster_update(&cloud, tick);
    return map_lidar_handle(1);
}

int main(void) {
    start_case(); small_pillar();
    check(frame() && road_is(8, road_unknown), "first pillar scan stays pending");
    check(!map_lidar_handle(1) && road_is(8, road_unknown), "duplicate scan cannot confirm twice");
    check(frame() && road_is(8, road_blocked), "second pillar scan blocks R8");

    start_case();
    point(10, 1100, 1625); point(11, 1200, 1625); point(12, 1300, 1625);
    frame(); frame();
    check(obstacle_result.cluster_count == 1 && obstacle_result.clusters[0].point_count == 3,
          "large ordinary cluster remains available for diagnosis");
    check(all_unknown(), "large cluster is not a pillar road obstacle");

    // 三点都在场外；小包围框覆盖场内N6的一角，不能凭框内空白封R4/R7
    start_case();
    point(10, -10, 30); point(11, -10, -10); point(12, 30, -10);
    frame(); frame();
    check(all_unknown(), "outside points cannot block roads through empty box interior");

    // 三点只落在R2/R9；框的右上角压到N4，不应封锁没有测距点的R3/R8
    start_case();
    point(10, 990, 1010); point(11, 990, 990); point(12, 1010, 990);
    frame(); frame();
    check(road_is(2, road_blocked) && road_is(9, road_blocked), "actual points may affect two roads");
    check(road_is(3, road_unknown) && road_is(8, road_unknown), "empty node corner cannot block other exits");

    start_case();
    point(10, 1200, 1200); point(11, 1205, 1200); point(12, 1210, 1200);
    frame(); frame();
    check(road_is(2, road_blocked) && road_is(3, road_blocked) &&
          road_is(8, road_blocked) && road_is(9, road_blocked), "actual node points block its four exits");

    start_case();
    for (uint16_t i = 0; i < 12; i++) point(10 + i, 1180 + i * 3, 1625);
    frame(); frame();
    check(road_is(8, road_blocked), "more than ten compact points remain a pillar candidate");

    start_case();
    point(359, 1190, 1625); point(0, 1200, 1620); point(1, 1210, 1625);
    frame(); frame();
    check(obstacle_result.cluster_count == 1 && road_is(8, road_blocked), "pillar spanning zero degrees uses all its points");

    start_case(); small_pillar(); frame();
    memset(cloud.points, 0, sizeof(cloud.points)); frame();
    small_pillar(); frame();
    check(road_is(8, road_unknown), "missed pillar resets consecutive confirmation");
    frame();
    check(road_is(8, road_blocked), "pillar confirms again after two consecutive hits");

    start_case(); point(10, 1200, 1625); point(11, 1210, 1625);
    frame(); frame();
    check(obstacle_result.cluster_count == 0 && all_unknown(), "existing three point minimum is unchanged");

    start_case(); small_pillar(); frame();
    check(!map_lidar_handle(0), "invalid pose rejects map input");
    frame();
    check(road_is(8, road_unknown), "pose interruption clears pending hit");
    frame();
    check(road_is(8, road_blocked), "fresh observations recover after pose interruption");

    start_case(); small_pillar(); frame();
    obstacle_result.cloud_flag++;
    check(!map_lidar_handle(1), "mismatched cluster and cloud are rejected");
    frame();
    check(road_is(8, road_unknown), "mismatch clears pending hit");

    start_case(); frame();
    check(all_unknown(), "empty observation never automatically frees a road");
    check(map_update_confirm_free(8, cloud.scan_num, tick), "explicit free interface is retained");

    start_case();
    point(10, 1150, 1625); point(11, 1200, 1625); point(12, 1250, 1625);
    frame(); frame();
    check(road_is(8, road_blocked), "trial size boundary is inclusive");
    start_case();
    point(10, 1149, 1625); point(11, 1200, 1625); point(12, 1250, 1625);
    frame(); frame();
    check(all_unknown(), "candidate above trial size is rejected");
    start_case();
    point(10, 1200, 1625); point(11, 1201, 1625); point(12, 1202, 1625);
    frame(); frame();
    check(road_is(8, road_blocked), "partial small surface has no strict size minimum");

    start_case(); small_pillar();
    for (unsigned int i = 10; i <= 12; i++) cloud.points[i].distance = 5000;
    frame(); frame();
    check(all_unknown(), "points beyond configured candidate range are not selected");
    start_case(); small_pillar();
    for (unsigned int i = 10; i <= 12; i++) cloud.points[i].distance = 40;
    frame(); frame();
    check(all_unknown(), "points below configured minimum distance are not selected");

    // 第21组使普通聚类结果截断；已保存的候选仍能封路，但不允许显式放行
    start_case();
    for (uint16_t i = 0; i < 21; i++) {
        point(i * 10, 1190, 1625);
        point(i * 10 + 1, 1200, 1620);
        point(i * 10 + 2, 1210, 1625);
    }
    frame(); frame();
    check(obstacle_result.cluster_count == CLUSTER_MAX_COUNT && obstacle_result.truncated &&
          road_is(8, road_blocked), "truncated result still blocks observed danger");
    check(!map_update_confirm_free(0, cloud.scan_num, tick), "truncated result cannot authorize free");

    start_case(); small_pillar(); frame();
    map_rect_t outside_area = {0, 100, 0, 100};
    cloud.scan_num = ++scan_num;
    cloud.scan_tick = tick += 100;
    check(map_lidar_handle_area(1, &outside_area) && road_is(8, road_unknown),
          "desk observation area excludes a candidate and resets pending hit");
    frame();
    check(road_is(8, road_unknown), "outside-area frame did not confirm an old hit");
    map_rect_t pillar_area = {1000, 1400, 1525, 1725};
    cloud.scan_num = ++scan_num;
    cloud.scan_tick = tick += 100;
    check(map_lidar_handle_area(1, &pillar_area) && road_is(8, road_blocked),
          "desk area uses the same actual-point confirmation as production");
    pillar_area.min_x = NAN;
    check(!map_lidar_handle_area(1, &pillar_area), "invalid desk area is rejected");

    start_case(); small_pillar(); frame();
    check(!map_update_roads((uint16_t)(1U << MAP_ROAD_COUNT), ++scan_num, tick + 1, 1),
          "invalid road mask is rejected");
    frame();
    check(road_is(8, road_unknown), "invalid mask clears pending confirmation");

    check(!obstacle_cluster_is_pillar(NULL), "null cluster is rejected");
    lidar_cluster_t invalid = {.valid = 1, .point_count = 3, .nearest_distance = 500, .width = NAN};
    check(!obstacle_cluster_is_pillar(&invalid), "nonfinite candidate size is rejected");

    // 使用真实点云变换源码连接原始扫描，而非只手工填写地图坐标
    start_case(); tf_init();
    lidar_scan_t raw = {0};
    raw.valid[359] = raw.valid[0] = raw.valid[1] = 1;
    raw.distance[359] = raw.distance[0] = raw.distance[1] = 500;
    for (unsigned int i = 0; i < 2; i++) {
        raw.scan_num = ++scan_num;
        raw.end_time = tick += 100;
        point_cloud_update(&raw, 1200, 2125, 180, tick);
        obstacle_cluster_update(&cloud, tick);
        map_lidar_handle(1);
    }
    check(cloud.valid_count == 3 && road_is(8, road_blocked),
          "raw scan through TF and clustering blocks the expected road");
    start_case(); memset(&raw, 0, sizeof(raw));
    raw.valid[40] = raw.valid[41] = raw.valid[42] = 1;
    raw.distance[40] = raw.distance[41] = raw.distance[42] = 500;
    raw.scan_num = ++scan_num;
    raw.end_time = tick += 100;
    point_cloud_update(&raw, 1200, 2125, 180, tick);
    obstacle_cluster_update(&cloud, tick);
    map_lidar_handle(1);
    check(cloud.valid_count == 0 && all_unknown(), "existing wheel mask remains effective");

    printf("[PILLAR TEST] checks=%u failures=%u\n", checks, failures);
    return failures != 0;
}
