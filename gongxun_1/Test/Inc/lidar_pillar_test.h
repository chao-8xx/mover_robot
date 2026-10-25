#ifndef GONGXUN_1_LIDAR_PILLAR_TEST_H
#define GONGXUN_1_LIDAR_PILLAR_TEST_H

// 正面315~45度，统计距离统一使用map_obs.h中的220~1320mm径向距离。
#define LIDAR_PILLAR_TEST_CENTER_DEG       0
#define LIDAR_PILLAR_TEST_HALF_WIDTH_DEG  45
#define LIDAR_PILLAR_TEST_SCANS           100

typedef struct {
    uint16_t center_deg, half_width_deg;
} lidar_pillar_config_t;
#define LIDAR_PILLAR_CONFIG_DEFAULT {LIDAR_PILLAR_TEST_CENTER_DEG, LIDAR_PILLAR_TEST_HALF_WIDTH_DEG}

typedef struct {
    uint32_t scan_num, scan_tick;
    uint16_t raw_samples, raw_zero, raw_target_points, integer_target_points;
    uint16_t native_points, legacy_points;
    uint8_t native_hit, legacy_hit;
    point2d_t native_center;
    float native_span_mm;
    uint16_t raw_nearest_mm, raw_farthest_mm;
} lidar_pillar_frame_t;

// hits/max_miss/first_hit_ms顺序：原始点、整数格点、原始点聚类候选、360格候选。
typedef struct {
    uint32_t samples, last_scan, start_tick;
    uint32_t hits[4], run_miss[4], max_miss[4], first_hit_ms[4];
    uint32_t raw_hist[4];       // 目标原始点数0、1、2、3及以上的圈数
    uint32_t missing_scans;
    uint32_t total_process_us, max_process_us;
} lidar_pillar_stats_t;

// 仅供独立测试：复用全局cloud/obstacle_result做360格对照，不与正式建图同时调用。
uint8_t lidar_pillar_measure(const lidar_raw_sector_t *raw, const lidar_scan_t *scan,
                            const lidar_pillar_config_t *config, lidar_pillar_frame_t *result);
void lidar_pillar_stats_reset(lidar_pillar_stats_t *stats);
uint8_t lidar_pillar_stats_add(lidar_pillar_stats_t *stats, const lidar_pillar_frame_t *frame,
                              uint32_t process_us);
void lidar_pillar_test_init(void);
void lidar_pillar_test(void);

#endif
