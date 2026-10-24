#include "headfile.h"

/**
 * @brief 处理雷达数据并更新地图
 * @param pose_valid: 位姿是否有效的标志
 * @return uint8_t: 返回更新结果  0表示未更新  非0表示更新成功
 */

// 一个实际点落在道路时标记本道路 落在节点时  沿用相连出口全部受影响的策略
static uint16_t map_lidar_point_roads(const point2d_t *point) {
    uint8_t x, y;
    map_cell_t cell;
    uint16_t road_mask = 0;

    if (!map_xy_to_cell(point->x, point->y, &x, &y) || !map_cell_get(x, y, &cell))
        return 0;
    if (cell.cell_type == cell_road)
        return (uint16_t)(1U << cell.cell_id);
    if (cell.cell_type == cell_node) {
        for (uint8_t i = 0; i < MAP_ROAD_COUNT; i++) {
            map_road_t road;
            if (map_road_info_get(i, &road) && (road.node_a == cell.cell_id || road.node_b == cell.cell_id))
                road_mask |= (uint16_t)(1U << i);
        }
    }
    return road_mask;
}

// 雷达的单圈结果只在这里转换
uint8_t map_lidar_handle_area(uint8_t pose_valid, const map_rect_t *area) {
    uint16_t road_mask = 0;
    uint8_t complete = !obstacle_result.truncated;

    if (!pose_valid || !map_update_is_ready() ||
        obstacle_result.cloud_flag == 0 ||
        obstacle_result.cloud_flag != cloud.update_flag ||
        obstacle_result.cluster_count > CLUSTER_MAX_COUNT) {
        map_update_clear_candidates();
        return 0;
    }

    if (area != NULL && (!isfinite(area->min_x) || !isfinite(area->max_x) ||
        !isfinite(area->min_y) || !isfinite(area->max_y) ||
        area->min_x > area->max_x || area->min_y > area->max_y)) {
        map_update_clear_candidates();
        return 0;
    }

    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];
        if (!obstacle_cluster_is_pillar(cluster)) continue;
        if (cluster->start_angle >= LIDAR_ANGLE_COUNT || cluster->end_angle >= LIDAR_ANGLE_COUNT) {
            complete = 0;
            continue;
        }
        if (area != NULL && (cluster->min_x < area->min_x || cluster->max_x > area->max_x ||
            cluster->min_y < area->min_y || cluster->max_y > area->max_y)) continue;

        // 起止角度标记了这一组点的范围；按环形角度遍历，兼容359度跨到0度
        // 只使用真正有效的点，不填满包围框，也不额外复制整份点云
        uint16_t span = (cluster->end_angle + LIDAR_ANGLE_COUNT - cluster->start_angle) % LIDAR_ANGLE_COUNT;
        for (uint16_t step = 0; step <= span; step++) {
            uint16_t angle = (cluster->start_angle + step) % LIDAR_ANGLE_COUNT;
            const lidar_point_t *point = &cloud.points[angle];
            if (!point->valid) continue;
            if (!isfinite(point->map_point.x) || !isfinite(point->map_point.y)) {
                complete = 0;
                continue;
            }
            road_mask |= map_lidar_point_roads(&point->map_point);
        }
    }

    return map_update_roads(road_mask, cloud.scan_num, cloud.scan_tick, complete);
}

uint8_t map_lidar_handle(uint8_t pose_valid) {
    return map_lidar_handle_area(pose_valid, NULL);
}
