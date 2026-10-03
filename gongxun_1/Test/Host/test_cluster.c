#include <stdio.h>
#include "headfile.h"

static uint16_t test_count;
static uint16_t fail_count;

static void check(uint8_t pass, const char *name) {
    test_count++;
    if (!pass) fail_count++;
    printf("%s %s\n", pass ? "PASS" : "FAIL", name);
}

static void clear_cloud(void) {
    memset(&cloud, 0, sizeof(cloud));
    cloud.update_flag = 1;
}

static void put_point(uint16_t angle, float x, float y) {
    cloud.points[angle].valid = 1;
    cloud.points[angle].distance = 1000.0f;
    cloud.points[angle].map_point.x = x;
    cloud.points[angle].map_point.y = y;
    cloud.valid_count++;
}

static void put_polar(uint16_t angle) {
    point2d_t point = tf_lidar_polar_to_xy(angle, 1000);
    put_point(angle, point.x, point.y);
}

static void run_cluster(void) {
    obstacle_cluster_update(&cloud, 1234);
}

static void test_mask(void) {
    lidar_scan_t scan;
    memset(&scan, 0, sizeof(scan));
    scan.scan_num = 1;
    scan.valid[0] = 1;
    scan.distance[0] = 1000;
    #if LIDAR_WHEEL_MASK_ENABLE
    check(point_cloud_update(&scan, 500.0f, 500.0f, 0.0f, 10) &&
          cloud.valid_count == 0 && !cloud.points[0].valid,
          "enabled zero to zero wheel range masks one angle");
    #else
    check(point_cloud_update(&scan, 500.0f, 500.0f, 0.0f, 10) &&
          cloud.valid_count == 1 && cloud.points[0].valid,
          "default wheel mask retains zero degree");
    #endif
    check(!point_cloud_update(&scan, 500.0f, 500.0f, 0.0f, 11),
          "same scan is not processed twice");
    scan.scan_num++;
    scan.valid[0] = 0;
    scan.valid[90] = scan.valid[180] = scan.valid[270] = 1;
    scan.distance[90] = scan.distance[180] = scan.distance[270] = 1000;
    check(point_cloud_update(&scan, 500.0f, 500.0f, 0.0f, 12) &&
          !cloud.points[0].valid && cloud.valid_count == 3 &&
          fabsf(cloud.points[90].map_point.x + 500.0f) < 0.01f &&
          fabsf(cloud.points[270].map_point.x - 1500.0f) < 0.01f,
          "new scan clears old point and uses right X forward Y");
}

static void test_seam(void) {
    clear_cloud();
    put_polar(358); put_polar(359); put_polar(0); put_polar(1);
    run_cluster();
    check(obstacle_result.cluster_count == 1 &&
          obstacle_result.clusters[0].point_count == 4 &&
          fabsf(obstacle_result.clusters[0].center_point.x - 8.724f) < 0.1f &&
          obstacle_result.clusters[0].min_x < -17.4f &&
          obstacle_result.clusters[0].max_x > 34.8f &&
          obstacle_result.clusters[0].nearest_distance == 1000.0f,
          "seam keeps two plus two points");

    clear_cloud();
    put_polar(359); put_polar(0); put_polar(1);
    run_cluster();
    check(obstacle_result.cluster_count == 1 &&
          obstacle_result.clusters[0].point_count == 3,
          "seam keeps one plus two points");

    clear_cloud();
    for (uint16_t angle = 355; angle <= 358; angle++) put_polar(angle);
    for (uint16_t angle = 1; angle <= 4; angle++) put_polar(angle);
    run_cluster();
    check(obstacle_result.cluster_count == 1 &&
          obstacle_result.clusters[0].point_count == 8 &&
          obstacle_result.clusters[0].start_angle == 355 &&
          obstacle_result.clusters[0].end_angle == 4,
          "seam tolerates two missing angles");

    clear_cloud();
    for (uint16_t angle = 354; angle <= 357; angle++) put_polar(angle);
    for (uint16_t angle = 1; angle <= 4; angle++) put_polar(angle);
    run_cluster();
    check(obstacle_result.cluster_count == 2,
          "seam splits at three missing angles");
}

static void test_gap(void) {
    clear_cloud();
    put_point(10, 0, 0); put_point(11, 10, 0); put_point(14, 20, 0);
    run_cluster();
    check(obstacle_result.cluster_count == 1 &&
          obstacle_result.clusters[0].point_count == 3,
          "linear run tolerates two missing angles");

    clear_cloud();
    put_point(10, 0, 0); put_point(11, 10, 0); put_point(15, 20, 0);
    run_cluster();
    check(obstacle_result.cluster_count == 0,
          "linear run splits at three missing angles");

    clear_cloud();
    put_point(10, 0, 0); put_point(11, 100, 0); put_point(12, 200, 0);
    run_cluster();
    check(obstacle_result.cluster_count == 1,
          "distance exactly 100 mm is connected");

    clear_cloud();
    put_point(10, 0, 0); put_point(11, 100.125f, 0); put_point(12, 200.25f, 0);
    run_cluster();
    check(obstacle_result.cluster_count == 0,
          "distance above 100 mm is split");
}

static void test_limits(void) {
    uint8_t has_seam;
    clear_cloud();
    run_cluster();
    check(obstacle_result.cluster_count == 0,
          "empty scan publishes empty result");

    clear_cloud();
    for (uint16_t angle = 0; angle < 360; angle++) put_point(angle, 0, 1000);
    run_cluster();
    check(obstacle_result.cluster_count == 1 &&
          obstacle_result.clusters[0].point_count == 360,
          "full circle without a break makes one cluster");

    clear_cloud();
    for (uint16_t angle = 0; angle < 360; angle++) {
        float x = angle < 90 || angle >= 270 ? 0.0f :
                  angle < 180 ? 1000.0f : 2000.0f;
        put_point(angle, x, 1000);
    }
    run_cluster();
    has_seam = 0;
    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++)
        if (obstacle_result.clusters[i].point_count == 180) has_seam = 1;
    check(obstacle_result.cluster_count == 3 && has_seam,
          "full circle starts at a distance break and keeps seam group");

    clear_cloud();
    for (uint16_t group = 0; group < 20; group++) {
        put_point(group * 6, group * 1000.0f, 1000);
        put_point(group * 6 + 1, group * 1000.0f + 10, 1000);
    }
    run_cluster();
    check(obstacle_result.cluster_count == 0,
          "all two point noise is rejected");

    clear_cloud();
    for (uint16_t group = 0; group < 20; group++) {
        for (uint16_t point = 0; point < 3; point++)
            put_point(group * 6 + point, group * 1000.0f + point * 10, 1000);
    }
    run_cluster();
    check(obstacle_result.cluster_count == CLUSTER_MAX_COUNT &&
          obstacle_result.clusters[19].point_count == 3,
          "twenty clusters fit result capacity");
    for (uint16_t point = 0; point < 3; point++)
        put_point(120 + point, 21000.0f + point * 10, 1000);
    run_cluster();
    check(obstacle_result.cluster_count == CLUSTER_MAX_COUNT,
          "twenty first cluster cannot overflow capacity");

    clear_cloud();
    put_polar(358); put_polar(359); put_polar(0); put_polar(1);
    for (uint16_t group = 0; group < 19; group++) {
        for (uint16_t point = 0; point < 3; point++)
            put_point(10 + group * 6 + point, 10000.0f + group * 1000.0f + point * 10, 1000);
    }
    run_cluster();
    has_seam = 0;
    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++)
        if (obstacle_result.clusters[i].point_count == 4) has_seam = 1;
    check(obstacle_result.cluster_count == CLUSTER_MAX_COUNT && has_seam,
          "twenty cluster capacity retains complete seam group");
    check(obstacle_result.cloud_flag == cloud.update_flag &&
          obstacle_result.update_tick == 1234,
          "cluster retains source cloud and timestamp");
}

static void test_circle(void) {
    uint8_t all_pass = 1;
    // 把同一组点移动到每个角度位置，圆周上任意边界都应得到相同的分组。
    for (uint16_t shift = 0; shift < 360; shift++) {
        clear_cloud();
        for (uint16_t angle = 355; angle <= 358; angle++)
            put_point((angle + shift) % 360, (angle - 355) * 10.0f, 1000);
        for (uint16_t angle = 1; angle <= 4; angle++)
            put_point((angle + shift) % 360, (angle + 3) * 10.0f, 1000);
        run_cluster();
        if (obstacle_result.cluster_count != 1 || obstacle_result.clusters[0].point_count != 8 ||
            fabsf(obstacle_result.clusters[0].center_point.x - 35.0f) > 0.01f)
            all_pass = 0;
    }
    check(all_pass, "two missing angle group is invariant at all 360 circle positions");
}

static void test_invalid_pose(void) {
    lidar_scan_t scan = {0};
    scan.scan_num = 3;
    scan.valid[90] = 1;
    scan.distance[90] = 1000;
    uint32_t old_flag = cloud.update_flag;
    check(!point_cloud_update(&scan, NAN, 0, 0, 100) && cloud.update_flag == old_flag,
          "nonfinite pose rejects without publishing a poisoned cloud");
    check(point_cloud_update(&scan, 0, 0, 0, 100),
          "same scan can retry after correcting invalid pose");
    scan.scan_num = 4;
    tf_base.tx = NAN;
    check(!point_cloud_update(&scan, 0, 0, 0, 101), "nonfinite installation rejects cloud");
    tf_transform_set(&tf_base, 0, 0, 0);
    check(point_cloud_update(&scan, 0, 0, 0, 101), "corrected installation can retry same scan");
    scan.scan_num = 0;
    check(!point_cloud_update(&scan, 0, 0, 0, 102), "unpublished scan zero cannot create cloud");
}

int main(void) {
    tf_init();
    test_mask();
    test_invalid_pose();
    test_seam();
    test_gap();
    test_limits();
    test_circle();
    printf("cluster tests: %u total, %u failed\n", test_count, fail_count);
    return fail_count != 0;
}
