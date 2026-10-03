#include "headfile.h"

// 雷达的单圈结果只在这里转换，地图更新模块不依赖雷达和 HAL
uint8_t map_lidar_handle(uint8_t pose_valid) {
    map_scan_obs_t boxes[CLUSTER_MAX_COUNT];
    uint8_t count = 0;

    if (!pose_valid || !map_update_is_ready() ||
        obstacle_result.cloud_flag == 0 ||
        obstacle_result.cloud_flag != cloud.update_flag ||
        obstacle_result.cluster_count > CLUSTER_MAX_COUNT) return 0;

    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];
        if (!cluster->valid) continue;
        boxes[count].center_x = cluster->center_point.x;
        boxes[count].center_y = cluster->center_point.y;
        boxes[count].min_x = cluster->min_x;
        boxes[count].min_y = cluster->min_y;
        boxes[count].max_x = cluster->max_x;
        boxes[count].max_y = cluster->max_y;
        count++;
    }

    return map_update_scan(boxes, count, cloud.scan_num, obstacle_result.update_tick);
}
