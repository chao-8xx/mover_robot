#include "headfile.h"

// 只验证任务层何时允许一圈进入地图；雷达、TF、聚类另有各自检查。
point_cloud_t cloud = {0};
obstacle_result_t obstacle_result = {0};
static lidar_scan_t scan = {0};
static uint8_t have_scan = 1;
static Chassis_Mode mode = CHASSIS_IDLE;
static uint32_t point_cloud_calls = 0;
static uint16_t failures = 0;
static uint16_t checks = 0;

// 完整 task.c 中未被本套检查调用的外设入口，保持链接独立。
uint8_t qr_data[QR_DATA_LEN + 1];
volatile uint8_t hwt101_rx_flag;
volatile uint8_t hwt101_error_flag;
volatile uint8_t cam_rx_flag;
volatile uint8_t cam_error_flag;
void control_variate_init(void) {}
void control_process(void) {}
void hwt101_rx_handle(void) {}
void cam_rx_handle(void) {}
void maix_result_task(void) {}

void lidar_process(void) {}

const lidar_scan_t *lidar_get_scan(void) {
    return have_scan ? &scan : 0;
}

Chassis_Mode car_get_state(void) {
    return mode;
}

uint32_t HAL_GetTick(void) {
    return scan.end_time;
}

uint8_t point_cloud_update(const lidar_scan_t *source,
                           float car_x, float car_y, float car_yaw,
                           uint32_t tick) {
    point_cloud_calls++;
    cloud.scan_num = source->scan_num;
    cloud.update_flag++;
    cloud.car_x = car_x;
    cloud.car_y = car_y;
    cloud.car_angle = car_yaw;
    cloud.update_tick = tick;
    return 1;
}

void obstacle_cluster_update(const point_cloud_t *source, uint32_t tick) {
    memset(&obstacle_result, 0, sizeof(obstacle_result));
    obstacle_result.cloud_flag = source->update_flag;
    obstacle_result.update_tick = tick;
    obstacle_result.cluster_count = 1;
    obstacle_result.clusters[0].valid = 1;
    obstacle_result.clusters[0].center_point.x = 1200.0f;
    obstacle_result.clusters[0].center_point.y = 1625.0f;
    obstacle_result.clusters[0].min_x = 1190.0f;
    obstacle_result.clusters[0].max_x = 1210.0f;
    obstacle_result.clusters[0].min_y = 1615.0f;
    obstacle_result.clusters[0].max_y = 1635.0f;
}

static void check(uint8_t ok, const char *name) {
    checks++;
    printf("[TASK HOST] %s %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) failures++;
}

static void publish_scan(uint32_t number) {
    scan.scan_num = number;
    scan.start_time = number * 100;
    scan.end_time = scan.start_time + 100;
}

static uint8_t run_map(uint8_t pose_valid) {
    return map_obstacle_task(1200.0f, 1200.0f, 0.0f, pose_valid);
}

static void test_pose_recovery(void) {
    map_update_init(300.0f, 50.0f);
    publish_scan(10);
    check(obstacle_find_task(1200.0f, 1200.0f, 0.0f) &&
          map_lidar_handle(1) && cloud.scan_num == 10,
          "seed known stationary scan 10");

    // 位姿无效期间车移动，scan 11 已完成；scan 12 还在跨越恢复时刻。
    mode = CHASSIS_MOVE;
    publish_scan(11);
    uint32_t before = point_cloud_calls;
    check(!run_map(0) && point_cloud_calls == before,
          "invalid moving pose does not generate cloud");
    mode = CHASSIS_IDLE;
    check(!run_map(1) && point_cloud_calls == before,
          "recovery records already published scan 11");
    publish_scan(12);
    check(!run_map(1) && point_cloud_calls == before && cloud.scan_num == 10,
          "scan 12 crossing pose recovery must be discarded");
    publish_scan(13);
    check(run_map(1) && point_cloud_calls == before + 1 && cloud.scan_num == 13,
          "first entire scan after recovery is scan 13");
    check(!run_map(1) && point_cloud_calls == before + 1,
          "duplicate completed scan never updates twice");
}

static void test_configuration_and_pose_gates(void) {
    uint32_t before = point_cloud_calls;
    map_update_init(0.0f, 50.0f);
    publish_scan(14);
    check(!run_map(1) && point_cloud_calls == before,
          "unconfigured map does not generate cloud");
    map_update_init(300.0f, 50.0f);
    check(!map_obstacle_task(NAN, 1200.0f, 0.0f, 1) &&
          !map_obstacle_task(1200.0f, 1200.0f, INFINITY, 1) &&
          !map_obstacle_task(775.0f, 775.0f, 0.0f, 1) &&
          !map_obstacle_task(2400.0f, 1200.0f, 0.0f, 1) &&
          point_cloud_calls == before,
          "nonfinite forbidden and outside poses reject");
    have_scan = 0;
    check(!run_map(1) && point_cloud_calls == before,
          "recovery without completed scan waits");
    have_scan = 1;
    check(!run_map(1) && point_cloud_calls == before,
          "configured recovery records old scan 14");
    publish_scan(15);
    check(!run_map(1) && point_cloud_calls == before,
          "configured recovery discards crossing scan 15");
    publish_scan(16);
    check(run_map(1) && point_cloud_calls == before + 1,
          "configured recovery accepts clean scan 16");
}

static void test_observed_movement(void) {
    uint32_t before = point_cloud_calls;
    mode = CHASSIS_MOVE;
    publish_scan(17);
    check(!run_map(1) && point_cloud_calls == before,
          "movement with valid pose still blocks map update");
    mode = CHASSIS_IDLE;
    check(!run_map(1) && point_cloud_calls == before,
          "new stop records completed scan 17");
    publish_scan(18);
    check(!run_map(1) && point_cloud_calls == before,
          "new stop discards scan 18 crossing movement");
    publish_scan(19);
    check(run_map(1) && point_cloud_calls == before + 1 && cloud.scan_num == 19,
          "new stop accepts clean stationary scan 19");
}

int main(void) {
    test_pose_recovery();
    test_configuration_and_pose_gates();
    test_observed_movement();
    printf("[TASK HOST] RESULT %u passed, %u failed\n",
           (unsigned)(checks - failures), (unsigned)failures);
    return failures ? 1 : 0;
}
