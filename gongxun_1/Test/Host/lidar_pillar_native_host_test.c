#include "headfile.h"

static uint8_t *dma_buffer;
static uint16_t dma_size, dma_write;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size) {
    dma_buffer = buffer; dma_size = size; dma_write = 0;
    uart->hdmarx->remaining = size;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size) {
    (void)uart; (void)buffer; (void)size; return HAL_OK;
}

#if !defined(LIDAR_RAW_SECTOR_MAX) || !__has_include("lidar_pillar_test.h")
int main(void) {
    puts("[NATIVE TEST] FAIL original sector samples and native pillar measurement are missing");
    return 1;
}
#else
#include "lidar_pillar_test.h"
static unsigned checks, failures;
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef uart = {&dma};
static void check(int ok, const char *name) {
    checks++;
    if (!ok) { failures++; printf("[NATIVE TEST] FAIL %s\n", name); }
}
static void send_packet(uint8_t start, uint16_t first_q6, uint16_t last_q6,
                        const uint16_t *distances, uint8_t count, uint8_t corrupt) {
    uint8_t packet[85] = {0xAA, 0x55, start, count};
    uint16_t a = (uint16_t)((first_q6 << 1) | 1U);
    uint16_t b = (uint16_t)((last_q6 << 1) | 1U);
    packet[4] = (uint8_t)a; packet[5] = (uint8_t)(a >> 8);
    packet[6] = (uint8_t)b; packet[7] = (uint8_t)(b >> 8);
    uint16_t cs = 0x55AA ^ (uint16_t)(start | ((uint16_t)count << 8)) ^ a ^ b;
    for (uint8_t i = 0; i < count; i++) {
        uint16_t offset = 10 + 3 * i, d = distances[i];
        uint8_t intensity = (uint8_t)(20 + i);
        packet[offset] = (uint8_t)((intensity & 63U) << 2);
        packet[offset + 1] = (uint8_t)(((d & 63U) << 2) | (intensity >> 6));
        packet[offset + 2] = (uint8_t)(d >> 6);
        cs ^= packet[offset];
        cs ^= (uint16_t)(packet[offset + 1] | ((uint16_t)packet[offset + 2] << 8));
    }
    if (corrupt) cs ^= 0x100;
    packet[8] = (uint8_t)cs; packet[9] = (uint8_t)(cs >> 8);
    for (uint16_t i = 0; i < 10U + 3U * count; i++) {
        dma_buffer[dma_write] = packet[i];
        dma_write = (uint16_t)((dma_write + 1U) % dma_size);
        dma.remaining = (uint16_t)(dma_size - dma_write);
    }
    lidar_process();
}
static void boundary(void) {
    uint16_t zero = 0;
    tick += 100;
    send_packet(1, 180U * 64U, 180U * 64U, &zero, 1, 0);
}
static void setup(uint16_t center, uint16_t half) {
    lidar_init(&uart);
    check(lidar_raw_sector_start(center, half), "valid sector config");
    boundary();
}

static void test_capture(void) {
    uint16_t d[3] = {2000, 1980, 1990}, zero = 0;
    lidar_init(&uart); boundary();
    send_packet(0, 646, 691, d, 3, 0); boundary();
    check(lidar_raw_sector_get() == NULL, "capture disabled by default");
    setup(10, 2);
    send_packet(0, 646, 691, d, 3, 0);
    send_packet(0, 736, 736, &zero, 1, 0); boundary();
    const lidar_raw_sector_t *raw = lidar_raw_sector_get();
    const lidar_scan_t *scan = lidar_get_scan();
    check(raw && raw->scan_num == scan->scan_num, "sector and integer scan share identity");
    check(raw && raw->count == 4 && !raw->overflow, "three subdegree returns and zero all preserved");
    if (!raw) return;
    check(raw->samples[0].angle_q6 == 646 && raw->samples[1].angle_q6 == 669 &&
          raw->samples[2].angle_q6 == 691, "subdegree angles preserved without merging");
    check(raw->samples[0].distance == 2000 && raw->samples[1].distance == 1980 &&
          raw->samples[2].intensity == 22 && raw->samples[3].distance == 0,
          "original distances intensity and no-return survive");
    check(scan->distance[10] == 1980 && scan->valid[10] && !scan->valid[11],
          "legacy nearest value unchanged");
    check(sizeof(lidar_raw_sample_t) == 6, "compact raw sample is six bytes");

    check(lidar_raw_sector_start(0, 2), "capture can change sector");
    send_packet(0, 0, 0, d, 1, 0); boundary();
    check(lidar_raw_sector_get() == NULL, "config change excludes old partial scan");
    send_packet(0, 359U * 64U + 32U, 32U, d, 3, 0); boundary();
    raw = lidar_raw_sector_get();
    check(raw && raw->count == 3 && raw->samples[0].angle_q6 == 359U * 64U + 32U &&
          raw->samples[1].angle_q6 == 0 && raw->samples[2].angle_q6 == 32,
          "capture crosses zero without losing samples");
    check(!lidar_raw_sector_start(360, 2) && !lidar_raw_sector_start(0, 181),
          "invalid sector config rejected");

    setup(0, 5);
    for (unsigned i = 0; i < LIDAR_RAW_SECTOR_MAX + 5U; i++) send_packet(0, 0, 0, d, 1, 0);
    boundary(); raw = lidar_raw_sector_get();
    check(raw && raw->count == LIDAR_RAW_SECTOR_MAX && raw->overflow == 5,
          "overflow counted without overwriting stored samples");
    lidar_pillar_config_t cfg = LIDAR_PILLAR_CONFIG_DEFAULT;
    cfg.half_width_deg = 5;
    lidar_pillar_frame_t result;
    check(!lidar_pillar_measure(raw, lidar_get_scan(), &cfg, &result), "overflow cannot be a complete measurement");
    if (raw) {
        lidar_raw_sector_t complete = *raw;
        complete.overflow = 0;
        check(lidar_pillar_measure(&complete, lidar_get_scan(), &cfg, &result),
              "same snapshot without overflow is measurable");
    }

    setup(0, 8);
    uint32_t dropped = lidar_get_scan_drop_cnt();
    send_packet(0, 0, 0, d, 1, 0); send_packet(0, 64, 64, d, 1, 1);
    boundary();
    check(lidar_raw_sector_get() == NULL && lidar_get_scan_drop_cnt() > dropped,
          "bad packet discards diagnostic partial scan");
    send_packet(0, 0, 0, d, 1, 0); boundary(); raw = lidar_raw_sector_get();
    check(raw && raw->count == 1, "capture restarts cleanly after bad packet");
    lidar_raw_sector_stop();
    check(lidar_raw_sector_get() == NULL, "stop disables diagnostic results");
}

static void test_native_measurement(void) {
    lidar_pillar_config_t cfg = LIDAR_PILLAR_CONFIG_DEFAULT;
    lidar_pillar_frame_t result;
    uint16_t d[3] = {1300, 1300, 1300}, zero = 0;
    setup(10, 2); cfg.center_deg = 10; cfg.half_width_deg = 2;
    send_packet(0, 646, 691, d, 3, 0); boundary();
    check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result), "paired sector is measurable");
    check(result.raw_target_points == 3 && result.integer_target_points == 1 &&
          result.native_points == 3 && result.legacy_points == 1,
          "native clustering uses all three real points versus one binned point");
    check(result.native_hit && result.legacy_hit, "one to eight points are accepted in both paths");
    check(result.native_center.x < -220.0f && result.native_center.x > -250.0f,
          "fractional bearing is used in actual coordinates");

    setup(10, 2);
    d[0] = d[1] = d[2] = 500;
    send_packet(0, 646, 691, d, 3, 0); boundary();
    check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result), "near comparison is measurable");
    check(result.native_hit && result.legacy_hit && result.native_points == 3 && result.legacy_points == 1,
          "near three real samples and one binned sample both pass the calibrated limits");

    setup(0, 8); cfg = (lidar_pillar_config_t)LIDAR_PILLAR_CONFIG_DEFAULT;
    cfg.half_width_deg = 8;
    d[0] = d[1] = d[2] = 1300;
    send_packet(0, 16, 32, d, 2, 0);
    send_packet(0, 359U * 64U + 32U, 359U * 64U + 48U, d, 2, 0); boundary();
    check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result) && result.native_points == 4,
          "native cluster joins physical neighbors across scan array boundary");

    setup(0, 8); send_packet(0, 0, 0, &zero, 1, 0); boundary();
    check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result) &&
          result.raw_zero == 1 && !result.raw_target_points && !result.native_hit && !result.legacy_hit,
          "zero return is diagnosed but cannot create a point or candidate");
    cfg.half_width_deg = 181;
    check(!lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result), "invalid measurement config rejected");
    cfg = (lidar_pillar_config_t)LIDAR_PILLAR_CONFIG_DEFAULT;
    cfg.half_width_deg = 8;
    lidar_scan_t wrong = *lidar_get_scan(); wrong.scan_num++;
    check(!lidar_pillar_measure(lidar_raw_sector_get(), &wrong, &cfg, &result), "different scan snapshots cannot be compared");

    setup(40, 8); cfg.center_deg = 40;
    send_packet(0, 40U * 64U, 40U * 64U, d, 1, 0); boundary();
    check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result) && result.legacy_hit && result.native_hit,
          "independent bench comparison does not apply vehicle wheel masks");
}

static void test_calibrated_limits(void) {
    lidar_pillar_config_t cfg = LIDAR_PILLAR_CONFIG_DEFAULT;
    lidar_pillar_frame_t result;
    cfg.center_deg = 0; cfg.half_width_deg = 8;
    const uint16_t distances[] = {219, 220, 1320, 1321};
    for (unsigned i = 0; i < sizeof(distances) / sizeof(distances[0]); i++) {
        setup(0, 8);
        send_packet(0, 0, 0, &distances[i], 1, 0); boundary();
        int accepted = distances[i] >= 220 && distances[i] <= 1320;
        check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result) &&
              result.raw_samples == 1 && result.raw_target_points == (unsigned)accepted &&
              result.integer_target_points == (unsigned)accepted,
              "raw capture is preserved while statistics use the calibrated radial range");
        check(result.native_hit == accepted && result.legacy_hit == accepted &&
              obstacle_result.cluster_count == (unsigned)accepted,
              "native and integer clusters share inclusive calibrated distance limits");
    }
    uint16_t d[9] = {500, 500, 500, 500, 500, 500, 500, 500, 500};
    for (unsigned count = 8; count <= 9; count++) {
        setup(0, 8);
        send_packet(0, 356U * 64U, 4U * 64U, d, (uint8_t)count, 0); boundary();
        check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result) &&
              result.raw_target_points == count && result.integer_target_points == count,
              "all real points are counted across zero before filtering");
        check(result.native_hit == (count == 8) && result.legacy_hit == (count == 8) &&
              obstacle_result.cluster_count == (count == 8),
              "eight points pass and nine points are rejected whole in both paths");
    }
}

static void test_front_sector(void) {
    lidar_pillar_config_t cfg = LIDAR_PILLAR_CONFIG_DEFAULT;
    lidar_pillar_frame_t result;
    check(cfg.center_deg == 0 && cfg.half_width_deg == 45,
          "default observation sector is inclusive 315 through zero to 45 degrees");
    const struct {
        uint16_t angle, distance;
        uint8_t accepted;
    } cases[] = {
        {315, 220, 1}, {315, 1320, 1}, {45, 220, 1}, {45, 1320, 1},
        {330, 573, 1}, {30, 1300, 1}, {0, 573, 1},
        {314, 500, 0}, {46, 500, 0}, {315, 219, 0}, {45, 1321, 0}, {0, 0, 0}
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        setup(cfg.center_deg, cfg.half_width_deg);
        send_packet(0, cases[i].angle * 64U, cases[i].angle * 64U, &cases[i].distance, 1, 0);
        boundary();
        check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result),
              "wide front sector snapshot is measurable");
        check(result.raw_target_points == cases[i].accepted && result.integer_target_points == cases[i].accepted &&
              result.native_hit == cases[i].accepted && result.legacy_hit == cases[i].accepted &&
              obstacle_result.cluster_count == cases[i].accepted,
              "radial statistics and candidates agree at side angles and distance boundaries");
        lidar_pillar_stats_t stats;
        lidar_pillar_stats_reset(&stats);
        check(lidar_pillar_stats_add(&stats, &result, 0) &&
              stats.hits[0] == cases[i].accepted && stats.hits[1] == cases[i].accepted &&
              stats.hits[2] == cases[i].accepted && stats.hits[3] == cases[i].accepted,
              "front candidate enters all four hit statistics without the old 200mm window");
    }

    // 聚类仍先收集完整点组，统计范围不能先裁掉越界点而改变候选判断。
    setup(0, 45);
    uint16_t d[2] = {1320, 1350};
    send_packet(0, 0, 64, d, 2, 0); boundary();
    check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result) &&
          result.raw_target_points == 1 && result.native_points == 2 && result.legacy_points == 2 &&
          result.native_hit && result.legacy_hit,
          "cluster nearest distance controls acceptance even when another point exceeds 1320mm");
    setup(0, 45); d[0] = 219; d[1] = 230;
    send_packet(0, 0, 64, d, 2, 0); boundary();
    check(lidar_pillar_measure(lidar_raw_sector_get(), lidar_get_scan(), &cfg, &result) &&
          result.raw_target_points == 1 && !result.native_hit && !result.legacy_hit &&
          !obstacle_result.cluster_count,
          "below-range nearest point rejects the complete group without clipping it");

    // 正面90度按约0.9度间隔回放101个采样，扩角后不得被旧64点容量排除。
    setup(0, 45);
    uint16_t distance = 600;
    for (unsigned i = 0; i <= 100; i++) {
        uint16_t angle = (uint16_t)((315U * 64U + i * 90U * 64U / 100U) % (360U * 64U));
        send_packet(0, angle, angle, &distance, 1, 0);
    }
    boundary();
    const lidar_raw_sector_t *raw = lidar_raw_sector_get();
    check(raw && raw->count == 101 && !raw->overflow &&
          lidar_pillar_measure(raw, lidar_get_scan(), &cfg, &result) && result.raw_target_points == 101,
          "90 degree raw capture preserves more than 64 samples without overflow");
    check(!result.native_hit && !result.legacy_hit && !obstacle_result.cluster_count,
          "wide dense background is not split into accepted eight point fragments");
}

static void test_statistics(void) {
    lidar_pillar_stats_t stats;
    lidar_pillar_stats_reset(&stats);
    lidar_pillar_frame_t frame = {0};
    for (unsigned i = 0; i < 100; i++) {
        frame.scan_num = i + 1; frame.scan_tick = i * 100;
        frame.raw_target_points = i % 2 ? 0 : 2;
        frame.integer_target_points = frame.raw_target_points ? 1 : 0;
        frame.native_hit = i % 2 == 0; frame.legacy_hit = i % 4 == 0;
        check(lidar_pillar_stats_add(&stats, &frame, 17), "new scan counted once");
        check(!lidar_pillar_stats_add(&stats, &frame, 17), "duplicate scan does not inflate hits");
    }
    check(stats.samples == 100 && stats.hits[0] == 50 && stats.hits[1] == 50 &&
          stats.hits[2] == 50 && stats.hits[3] == 25, "four stage hit rates use real fresh observations");
    check(stats.raw_hist[0] == 50 && stats.raw_hist[2] == 50, "zero and two-point frequencies retained");
    check(stats.max_miss[0] == 1 && stats.max_miss[3] == 3, "longest consecutive misses measured per stage");
    check(stats.first_hit_ms[0] == 0 && stats.total_process_us == 1700 && stats.max_process_us == 17,
          "first hit and processing cost are measured separately");
    frame.scan_num += 3; frame.scan_tick += 300;
    check(lidar_pillar_stats_add(&stats, &frame, 20) && stats.missing_scans == 2,
          "unobserved published scans are counted as gaps rather than synthetic misses");
    lidar_pillar_stats_reset(&stats);
    frame.scan_num = 1; frame.scan_tick = 100;
    frame.raw_target_points = frame.integer_target_points = frame.native_hit = frame.legacy_hit = 0;
    lidar_pillar_stats_add(&stats, &frame, 0);
    check(stats.first_hit_ms[0] == UINT32_MAX, "no first hit remains explicitly absent");
}
int main(void) {
    tf_init(); test_capture(); test_native_measurement(); test_calibrated_limits(); test_front_sector(); test_statistics();
    printf("[NATIVE TEST] checks=%u failures=%u\n", checks, failures);
    return failures != 0;
}
#endif
