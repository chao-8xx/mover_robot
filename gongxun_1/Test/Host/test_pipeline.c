#include "headfile.h"

static uint16_t test_count = 0;
static uint16_t fail_count = 0;

static void check(uint8_t ok, const char *name) {
    test_count++;
    if (!ok) fail_count++;
    printf("[PIPELINE] %s %s\n", ok ? "PASS" : "FAIL", name);
}

static uint8_t road_is(uint8_t id, road_state_t expected) {
    road_state_t state;
    return map_road_get(id, &state) && state == expected;
}

// 人工完整圈：前方425mm处的三点跨过0度，不代表D6能扫到真实细杆。
static void make_scan(lidar_scan_t *scan, uint32_t number) {
    memset(scan, 0, sizeof(*scan));
    scan->scan_num = number;
    scan->raw_count = scan->valid_count = 3;
    scan->valid[359] = scan->valid[0] = scan->valid[1] = 1;
    scan->distance[359] = scan->distance[0] = scan->distance[1] = 425;
}

int main(void) {
    lidar_scan_t scan;
    map_path_t old_path, new_path;
    uint8_t cell_x, cell_y;
    map_cell_t cell;

    tf_init();
    check(map_update_init(300.0f, 50.0f) &&
          map_plan_find(1, 4, map_unknown_allow, &old_path) &&
          old_path.road_count == 1 && old_path.roads[0] == 8,
          "initial unknown path uses R8 and still needs scan");

    make_scan(&scan, 1);
    check(point_cloud_update(&scan, 1200.0f, 1200.0f, 0.0f, 100) &&
          cloud.scan_num == 1 && cloud.valid_count == 3 &&
          fabsf(cloud.points[0].map_point.x - 1200.0f) < 0.01f &&
          fabsf(cloud.points[0].map_point.y - 1625.0f) < 0.01f,
          "raw scan transforms front target into world R8");
    obstacle_cluster_update(&cloud, 100);
    check(obstacle_result.cluster_count == 1 &&
          obstacle_result.clusters[0].point_count == 3 &&
          obstacle_result.cloud_flag == cloud.update_flag &&
          map_xy_to_cell(obstacle_result.clusters[0].center_point.x,
                         obstacle_result.clusters[0].center_point.y,
                         &cell_x, &cell_y) &&
          map_cell_get(cell_x, cell_y, &cell) &&
          cell.cell_type == cell_road && cell.cell_id == 8,
          "seam cluster and map region agree on R8");
    check(!map_lidar_handle(0) && map_lidar_handle(1) &&
          road_is(8, road_unknown) && !map_lidar_handle(1),
          "invalid pose and repeated scan cannot confirm obstacle");

    make_scan(&scan, 2);
    point_cloud_update(&scan, 1200.0f, 1200.0f, 0.0f, 200);
    obstacle_cluster_update(&cloud, 200);
    check(map_lidar_handle(1) && road_is(8, road_blocked) &&
          map_update_change_get() == 1,
          "second raw scan confirms and blocks only R8");
    check(!map_plan_check(&old_path, map_unknown_allow) &&
          map_plan_road_state(&old_path, 0) == map_step_stop &&
          map_plan_find(1, 4, map_unknown_allow, &new_path) &&
          new_path.road_count > 1 && map_plan_check(&new_path, map_unknown_allow),
          "updated map invalidates old path and permits alternate path");

    memset(&scan, 0, sizeof(scan));
    scan.scan_num = 3;
    point_cloud_update(&scan, 1200.0f, 1200.0f, 0.0f, 300);
    obstacle_cluster_update(&cloud, 300);
    check(cloud.valid_count == 0 && obstacle_result.cluster_count == 0 &&
          map_lidar_handle(1) && road_is(8, road_blocked),
          "empty new scan clears cloud but retains confirmed blocking");

    map_update_init(300.0f, 50.0f);
    for (uint32_t number = 4; number <= 5; number++) {
        make_scan(&scan, number);
        point_cloud_update(&scan, 1200.0f, 1200.0f, -90.0f, number * 100);
        obstacle_cluster_update(&cloud, number * 100);
        map_lidar_handle(1);
    }
    check(road_is(3, road_blocked) && road_is(8, road_unknown) &&
          fabsf(cloud.points[0].map_point.x - 1625.0f) < 0.01f &&
          fabsf(cloud.points[0].map_point.y - 1200.0f) < 0.01f,
          "body yaw minus90 projects front east and blocks R3");

    printf("[PIPELINE] RESULT %u passed, %u failed\n",
           (unsigned)(test_count - fail_count), (unsigned)fail_count);
    return fail_count != 0;
}
