#include "headfile.h"
#include "lidar_pillar_test.h"

// 只保存本角区的局部XY，不分配第二套整圈点云；整数格对照复用已有全局cloud。
static point2d_t native_xy[LIDAR_RAW_SECTOR_MAX];
static uint8_t native_valid[LIDAR_RAW_SECTOR_MAX];

///////////////////////////////////// 原始点局部处理 /////////////////////////////////////
static uint8_t pillar_in_sector(uint16_t angle_q6, const lidar_pillar_config_t *config) {
    int32_t delta = (int32_t)angle_q6 - (int32_t)config->center_deg * 64;
    if (delta > 180 * 64) delta -= 360 * 64;
    if (delta < -180 * 64) delta += 360 * 64;
    return delta >= -(int32_t)config->half_width_deg * 64 &&
           delta <= (int32_t)config->half_width_deg * 64;
}

// 插值的是已有正余弦表对应坐标，不增加采样点；一个真实采样只产生一个XY。
static point2d_t pillar_fractional_xy(uint16_t angle_q6, uint16_t distance) {
    uint16_t degree = angle_q6 / 64U;
    float fraction = (float)(angle_q6 % 64U) / 64.0f;
    point2d_t a = tf_lidar_polar_to_xy(degree, distance);
    point2d_t b = tf_lidar_polar_to_xy((degree + 1U) % 360U, distance);
    a.x += fraction * (b.x - a.x);
    a.y += fraction * (b.y - a.y);
    return a;
}

// 按真实径向距离统计，不再使用固定200mm、左右100mm的矩形窗口。
static uint8_t pillar_in_target(uint16_t distance) {
    return distance >= LIDAR_PILLAR_MIN_DISTANCE && distance <= LIDAR_PILLAR_MAX_DISTANCE;
}

static uint8_t pillar_connected(const lidar_raw_sector_t *raw, uint16_t previous, uint16_t current) {
    uint16_t gap = (uint16_t)((raw->samples[current].angle_q6 + 360U * 64U -
                              raw->samples[previous].angle_q6) % (360U * 64U));
    float dx = native_xy[current].x - native_xy[previous].x;
    float dy = native_xy[current].y - native_xy[previous].y;
    // 旧算法允许缺2个整数角格，这里对应相邻真实采样角度差不超过3度。
    return gap <= (CLUSTER_MAX_MISS_ANGLE + 1U) * 64U &&
           dx * dx + dy * dy <= CLUSTER_MAX_POINT_GAP * CLUSTER_MAX_POINT_GAP;
}

static void pillar_cluster_add(lidar_cluster_t *cluster, point2d_t point, uint16_t distance) {
    if (!cluster->point_count) {
        cluster->center_point = point;
        cluster->min_x = cluster->max_x = point.x;
        cluster->min_y = cluster->max_y = point.y;
        cluster->nearest_distance = distance;
    } else {
        float count = (float)(cluster->point_count + 1U);
        cluster->center_point.x += (point.x - cluster->center_point.x) / count;
        cluster->center_point.y += (point.y - cluster->center_point.y) / count;
        cluster->min_x = fminf(cluster->min_x, point.x);
        cluster->max_x = fmaxf(cluster->max_x, point.x);
        cluster->min_y = fminf(cluster->min_y, point.y);
        cluster->max_y = fmaxf(cluster->max_y, point.y);
        cluster->nearest_distance = fminf(cluster->nearest_distance, distance);
    }
    cluster->point_count++;
}

static void pillar_cluster_finish(lidar_cluster_t *cluster, lidar_pillar_frame_t *result) {
    if (!cluster->point_count) return;
    cluster->width = cluster->max_x - cluster->min_x;
    cluster->height = cluster->max_y - cluster->min_y;
    cluster->valid = 1;
    // 原始点与360格共用220~1320mm、1~8点及100mm跨度筛选。
    if (obstacle_cluster_is_pillar(cluster)) {
        result->native_hit = 1;
        if (cluster->point_count > result->native_points) {
            result->native_points = cluster->point_count;
            result->native_center = cluster->center_point;
            result->native_span_mm = fmaxf(cluster->width, cluster->height);
        }
    }
    memset(cluster, 0, sizeof(*cluster));
}

uint8_t lidar_pillar_measure(const lidar_raw_sector_t *raw, const lidar_scan_t *scan,
                            const lidar_pillar_config_t *config, lidar_pillar_frame_t *result) {
    if (result == NULL) return 0;
    memset(result, 0, sizeof(*result));
    if (raw == NULL || scan == NULL || config == NULL || raw->scan_num == 0 ||
        raw->scan_num != scan->scan_num || raw->overflow || raw->count > LIDAR_RAW_SECTOR_MAX ||
        raw->center_deg != config->center_deg || raw->half_width_deg != config->half_width_deg ||
        config->center_deg >= 360 || config->half_width_deg > 180) return 0;

    result->scan_num = raw->scan_num;
    result->scan_tick = raw->end_time;
    result->raw_samples = raw->count;
    uint16_t first = UINT16_MAX, last = UINT16_MAX;
    memset(native_valid, 0, sizeof(native_valid));
    for (uint16_t i = 0; i < raw->count; i++) {
        const lidar_raw_sample_t *sample = &raw->samples[i];
        if (sample->distance == 0) result->raw_zero++;
        if (sample->angle_q6 >= 360U * 64U || !pillar_in_sector(sample->angle_q6, config) ||
            sample->distance < LIDAR_MIN_DISTANCE || sample->distance > LIDAR_MAX_DISTANCE) continue;
        native_xy[i] = pillar_fractional_xy(sample->angle_q6, sample->distance);
        native_valid[i] = 1;
        if (first == UINT16_MAX) first = i;
        last = i;
        if (pillar_in_target(sample->distance)) {
            if (!result->raw_target_points || sample->distance < result->raw_nearest_mm)
                result->raw_nearest_mm = sample->distance;
            if (sample->distance > result->raw_farthest_mm) result->raw_farthest_mm = sample->distance;
            result->raw_target_points++;
        }
    }

    if (first != UINT16_MAX) {
        // 先找真正断开处，跨0度的一组真实点也要完整聚类后再筛选。
        uint16_t start = first, previous = last;
        for (uint16_t i = 0; i < raw->count; i++) {
            if (!native_valid[i]) continue;
            if (!pillar_connected(raw, previous, i)) { start = i; break; }
            previous = i;
        }
        lidar_cluster_t cluster = {0};
        previous = UINT16_MAX;
        for (uint16_t step = 0; step < raw->count; step++) {
            uint16_t i = (uint16_t)((start + step) % raw->count);
            if (!native_valid[i]) continue;
            if (previous != UINT16_MAX && !pillar_connected(raw, previous, i))
                pillar_cluster_finish(&cluster, result);
            pillar_cluster_add(&cluster, native_xy[i], raw->samples[i].distance);
            previous = i;
        }
        pillar_cluster_finish(&cluster, result);
    }

    // 同一角区、同一圈、同一局部坐标下，调用正式360格聚类源码作对照。
    // 台架两路都不使用整车遮挡；本函数只在独立入口运行，不写地图。
    uint32_t next_flag = cloud.update_flag + 1U;
    memset(&cloud, 0, sizeof(cloud));
    cloud.update_flag = next_flag;
    cloud.scan_num = scan->scan_num;
    cloud.scan_tick = scan->end_time;
    for (uint16_t angle = 0; angle < 360; angle++) {
        uint16_t distance = scan->distance[angle];
        if (!scan->valid[angle] || !pillar_in_sector(angle * 64U, config) ||
            distance < LIDAR_MIN_DISTANCE || distance > LIDAR_MAX_DISTANCE) continue;
        lidar_point_t *point = &cloud.points[angle];
        point->valid = 1;
        point->distance = distance;
        point->map_point = tf_lidar_polar_to_xy(angle, distance);
        cloud.valid_count++;
        if (pillar_in_target(distance)) result->integer_target_points++;
    }
    obstacle_cluster_update(&cloud, scan->end_time);
    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];
        if (obstacle_cluster_is_pillar(cluster)) {
            result->legacy_hit = 1;
            if (cluster->point_count > result->legacy_points) result->legacy_points = cluster->point_count;
        }
    }
    return 1;
}

#ifndef LIDAR_PILLAR_HOST_TEST
///////////////////////////////////// 独立台架入口 ///////////////////////////////////////
// 不调用system_init，不开启整车定时器，也不调用电机、里程计、导航和地图更新。
void lidar_pillar_test_init(void) {
    debug_uart_init(&huart6);
    tf_init();
    lidar_init(&huart4);
    lidar_raw_sector_start(LIDAR_PILLAR_TEST_CENTER_DEG, LIDAR_PILLAR_TEST_HALF_WIDTH_DEG);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    lidar_start();
}

// 主循环直接对KEY1作30ms消抖，独立模式不用整车按键任务或阻塞等待。
static uint8_t pillar_key_released(uint32_t tick) {
    static uint8_t last_raw = 0, stable = 0;
    static uint32_t changed_tick = 0;
    uint8_t raw = key_getstate(0);
    if (raw != last_raw) { last_raw = raw; changed_tick = tick; }
    if (raw != stable && tick - changed_tick >= 30U) {
        stable = raw;
        return stable == 0;
    }
    return 0;
}

static void pillar_test_prompt(uint8_t stage) {
    if (stage >= 6) {
        printf("[PILLAR TEST] DONE 三轮背景/有柱采集结束；重新开始请复位\r\n");
    } else {
        printf("[PILLAR TEST] 第%u轮 %s：固定雷达，人离开扫描平面，短按KEY1采%u个新扫描\r\n",
               (unsigned int)(stage / 2U + 1U), stage % 2U ? "放入原样小黑柱" : "移走小黑柱测背景",
               (unsigned int)LIDAR_PILLAR_TEST_SCANS);
    }
}

static void pillar_test_report(uint8_t stage, const lidar_pillar_stats_t *stats,
                                uint32_t excluded, uint32_t receiver_breaks,
                                uint32_t xor_start, uint32_t drop_start, uint32_t log_start) {
    static const char *names[4] = {"RAW", "INTEGER", "NATIVE", "LEGACY"};
    printf("[PILLAR 100] round=%u mode=%s scans=%lu excluded=%lu gaps=%lu breaks=%lu\r\n",
           (unsigned int)(stage / 2U + 1U), stage % 2U ? "PRESENT" : "BACKGROUND",
           (unsigned long)stats->samples, (unsigned long)excluded,
           (unsigned long)stats->missing_scans, (unsigned long)receiver_breaks);
    for (uint8_t i = 0; i < 4; i++) {
        // 首次命中时间从本组第一个有效扫描结束算起；-1表示本组一直没命中。
        printf("[PILLAR RATE] %s hit=%lu/%lu max_miss=%lu first_hit_ms=%ld\r\n", names[i],
               (unsigned long)stats->hits[i], (unsigned long)stats->samples,
               (unsigned long)stats->max_miss[i], stats->first_hit_ms[i] == UINT32_MAX ? -1L :
               (long)stats->first_hit_ms[i]);
    }
    printf("[PILLAR HIST] raw_points_0/1/2/3plus=%lu/%lu/%lu/%lu\r\n",
           (unsigned long)stats->raw_hist[0], (unsigned long)stats->raw_hist[1],
           (unsigned long)stats->raw_hist[2], (unsigned long)stats->raw_hist[3]);
    printf("[PILLAR COST] compare_avg_us=%lu max_us=%lu xor_err=%lu drop=%lu log_drop=%lu\r\n",
           (unsigned long)(stats->samples ? stats->total_process_us / stats->samples : 0),
           (unsigned long)stats->max_process_us,
           (unsigned long)(lidar_get_xor_err_cnt() - xor_start),
           (unsigned long)(lidar_get_scan_drop_cnt() - drop_start),
           (unsigned long)(debug_get_err_bytes() - log_start));
}

static void pillar_test_print_example(const lidar_raw_sector_t *raw, const lidar_pillar_config_t *config) {
    uint16_t printed = 0, found = 0;
    // 每组只在首次原始命中时打印一个快照；最多16点，超过时明确报告，统计仍使用全部点。
    for (uint16_t i = 0; i < raw->count; i++) {
        const lidar_raw_sample_t *sample = &raw->samples[i];
        if (sample->angle_q6 >= 360U * 64U || !pillar_in_sector(sample->angle_q6, config) ||
            !pillar_in_target(sample->distance)) continue;
        point2d_t point = pillar_fractional_xy(sample->angle_q6, sample->distance);
        found++;
        if (printed >= 16) continue;
        printf("[PILLAR SAMPLE] scan=%lu a=%.3f d=%u I=%u xy=(%.1f,%.1f)\r\n",
               (unsigned long)raw->scan_num, (float)sample->angle_q6 / 64.0f,
               (unsigned int)sample->distance, (unsigned int)sample->intensity, point.x, point.y);
        printed++;
    }
    if (found > printed) printf("[PILLAR SAMPLE] print_limited=%u total_target=%u\r\n",
                                (unsigned int)printed, (unsigned int)found);
}

void lidar_pillar_test(void) {
    static const lidar_pillar_config_t config = LIDAR_PILLAR_CONFIG_DEFAULT;
    static lidar_pillar_stats_t stats;
    static uint8_t started = 0, stage = 0, collecting = 0, skip_new_scans = 0;
    static uint32_t last_scan = 0, last_drop = 0, excluded = 0, receiver_breaks = 0;
    static uint32_t xor_start = 0, drop_start = 0, log_start = 0, last_wait_tick = 0;
    uint32_t tick = HAL_GetTick();
    lidar_process();
    debug_printf_task();
    uint8_t pressed = pillar_key_released(tick);
    const lidar_scan_t *scan = lidar_get_scan();
    const lidar_raw_sector_t *raw = lidar_raw_sector_get();

    if (!started) {
        started = 1;
        printf("===== LIDAR PILLAR NATIVE / 360 TEST =====\r\n");
        printf("[PILLAR CONFIG] angle=%u~%u center=%u +/-%u range=%.0f~%.0fmm\r\n",
               (unsigned int)((config.center_deg + 360U - config.half_width_deg) % 360U),
               (unsigned int)((config.center_deg + config.half_width_deg) % 360U),
               (unsigned int)config.center_deg, (unsigned int)config.half_width_deg,
               LIDAR_PILLAR_MIN_DISTANCE, LIDAR_PILLAR_MAX_DISTANCE);
        printf("[PILLAR FILTER] near=%.0f~%.0fmm points=%u~%u span_max=%.0fmm\r\n",
               LIDAR_PILLAR_MIN_DISTANCE, LIDAR_PILLAR_MAX_DISTANCE,
               (unsigned int)CLUSTER_MIN_POINT_COUNT, (unsigned int)CLUSTER_MAX_POINT_COUNT,
               LIDAR_PILLAR_MAX_SIZE);
        printf("台架两路均不屏蔽车体；原始点容量=%u；不补点、不保持旧命中、不写地图\r\n",
               (unsigned int)LIDAR_RAW_SECTOR_MAX);
        printf("RAW/INTEGER=观察角区220~1320mm内有真实点；NATIVE/LEGACY=本圈通过小柱筛选；背景命中为误报候选\r\n");
        pillar_test_prompt(stage);
    }
    if (stage >= 6) return;
    if (!collecting) {
        if (!pressed) return;
        collecting = 1;
        skip_new_scans = 2; // 丢弃摆放时刻附近的扫描，避免把半圈运动回波带入统计
        last_scan = scan ? scan->scan_num : 0;
        lidar_pillar_stats_reset(&stats);
        excluded = receiver_breaks = 0;
        xor_start = lidar_get_xor_err_cnt();
        last_drop = drop_start = lidar_get_scan_drop_cnt();
        log_start = debug_get_err_bytes();
        last_wait_tick = tick;
        printf("[PILLAR TEST] collecting mode=%s\r\n", stage % 2U ? "PRESENT" : "BACKGROUND");
        return;
    }

    uint32_t drop = lidar_get_scan_drop_cnt();
    if (drop != last_drop) {
        last_drop = drop;
        receiver_breaks++;
        memset(stats.run_miss, 0, sizeof(stats.run_miss));
    }
    if (scan == NULL || scan->scan_num == 0 || scan->scan_num == last_scan) {
        if (tick - last_wait_tick >= 2000U) {
            last_wait_tick = tick;
            printf("[PILLAR WAIT] accepted=%lu xor_ok=%lu xor_err=%lu drop=%lu log_drop=%lu\r\n",
                   (unsigned long)stats.samples, (unsigned long)lidar_get_xor_ok_cnt(),
                   (unsigned long)lidar_get_xor_err_cnt(), (unsigned long)drop,
                   (unsigned long)debug_get_err_bytes());
        }
        return;
    }
    last_scan = scan->scan_num;
    last_wait_tick = tick;  // 只有连续2秒没有新扫描才提示WAIT，正常圈间等待不刷屏
    if (skip_new_scans) { skip_new_scans--; return; }
    uint32_t before = DWT->CYCCNT;
    lidar_pillar_frame_t frame;
    uint8_t valid = lidar_pillar_measure(raw, scan, &config, &frame);
    uint32_t process_us = (DWT->CYCCNT - before) / (SystemCoreClock / 1000000U);
    if (!valid) {
        excluded++;
        memset(stats.run_miss, 0, sizeof(stats.run_miss));
        if (excluded == 1 || excluded % 20U == 0)
            printf("[PILLAR EXCLUDED] scan=%lu raw_scan=%lu count=%u overflow=%u total=%lu\r\n",
                   (unsigned long)scan->scan_num, (unsigned long)(raw ? raw->scan_num : 0),
                   (unsigned int)(raw ? raw->count : 0), (unsigned int)(raw ? raw->overflow : 0),
                   (unsigned long)excluded);
        return;
    }
    if (!lidar_pillar_stats_add(&stats, &frame, process_us)) return;
    if (frame.raw_target_points && stats.hits[0] == 1) pillar_test_print_example(raw, &config);
    if (stats.samples <= 3 || stats.samples % 20U == 0) {
        printf("[PILLAR FRAME] scan=%lu n=%lu sector=%u zero=%u target_raw=%u integer=%u "
               "native=%u/%u legacy=%u/%u d=%u~%u span=%.1f clusters=%u cost=%luus\r\n",
               (unsigned long)frame.scan_num, (unsigned long)stats.samples,
               (unsigned int)frame.raw_samples, (unsigned int)frame.raw_zero,
               (unsigned int)frame.raw_target_points, (unsigned int)frame.integer_target_points,
               (unsigned int)frame.native_hit, (unsigned int)frame.native_points,
               (unsigned int)frame.legacy_hit, (unsigned int)frame.legacy_points,
               (unsigned int)frame.raw_nearest_mm, (unsigned int)frame.raw_farthest_mm,
               frame.native_span_mm, (unsigned int)obstacle_result.cluster_count, (unsigned long)process_us);
        // 打印本角区筛选后的360格候选，与LEGACY命中统计使用同一结果。
        for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
            const lidar_cluster_t *cluster = &obstacle_result.clusters[i];
            printf("[PILLAR CLUSTER] C%u a=%u~%u p=%u near=%.0f xy=(%.1f,%.1f) size=(%.1f,%.1f)\r\n",
                   (unsigned int)i, (unsigned int)cluster->start_angle, (unsigned int)cluster->end_angle,
                   (unsigned int)cluster->point_count, cluster->nearest_distance,
                   cluster->center_point.x, cluster->center_point.y, cluster->width, cluster->height);
        }
    }
    if (stats.samples >= LIDAR_PILLAR_TEST_SCANS) {
        pillar_test_report(stage, &stats, excluded, receiver_breaks, xor_start, drop_start, log_start);
        collecting = 0;
        stage++;
        pillar_test_prompt(stage);
    }
}
#endif

///////////////////////////////////// 逐圈统计，不保持旧命中 ///////////////////////////////
void lidar_pillar_stats_reset(lidar_pillar_stats_t *stats) {
    if (stats == NULL) return;
    memset(stats, 0, sizeof(*stats));
    for (uint8_t i = 0; i < 4; i++) stats->first_hit_ms[i] = UINT32_MAX;
}

uint8_t lidar_pillar_stats_add(lidar_pillar_stats_t *stats, const lidar_pillar_frame_t *frame,
                              uint32_t process_us) {
    if (stats == NULL || frame == NULL || !frame->scan_num ||
        (stats->samples && (int32_t)(frame->scan_num - stats->last_scan) <= 0)) return 0;
    if (!stats->samples) stats->start_tick = frame->scan_tick;
    else if (frame->scan_num - stats->last_scan != 1U) {
        stats->missing_scans += frame->scan_num - stats->last_scan - 1U;
        // 未观察的圈既不补命中，也不伪装成已知漏检；连续漏检段重新开始。
        memset(stats->run_miss, 0, sizeof(stats->run_miss));
    }
    uint8_t hit[4] = {frame->raw_target_points != 0, frame->integer_target_points != 0,
                      frame->native_hit, frame->legacy_hit};
    for (uint8_t i = 0; i < 4; i++) {
        if (hit[i]) {
            stats->hits[i]++;
            stats->run_miss[i] = 0;
            if (stats->first_hit_ms[i] == UINT32_MAX)
                stats->first_hit_ms[i] = frame->scan_tick - stats->start_tick;
        } else {
            stats->run_miss[i]++;
            if (stats->run_miss[i] > stats->max_miss[i]) stats->max_miss[i] = stats->run_miss[i];
        }
    }
    uint16_t bucket = frame->raw_target_points < 3 ? frame->raw_target_points : 3;
    stats->raw_hist[bucket]++;
    stats->total_process_us += process_us;
    if (process_us > stats->max_process_us) stats->max_process_us = process_us;
    stats->last_scan = frame->scan_num;
    stats->samples++;
    return 1;
}
