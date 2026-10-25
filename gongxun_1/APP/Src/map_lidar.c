#include "headfile.h"

/**
 * 将合格候选中的实际点转换成道路命中结果
 *
 *
 */

// 一个地图点影响哪些道路      一个实际点落在道路时标记本道路 落在节点时  沿用相连出口全部受影响的策略
uint16_t map_lidar_point_roads(float x_mm, float y_mm) {
    uint8_t x, y;
    map_cell_t cell;
    uint16_t road_mask = 0;

    // 先把实际点转换成地图格子坐标  再获取格子信息
    if (!map_xy_to_cell(x_mm, y_mm, &x, &y) || !map_cell_get(x, y, &cell))
        return 0;

    // 落在道路格子时只标记本道路
    if (cell.cell_type == cell_road)
        return (uint16_t)(1 << cell.cell_id);

    // 落在节点格子时标记所有相连道路
    if (cell.cell_type == cell_node) {
        for (uint8_t i = 0; i < MAP_ROAD_COUNT; i++) {
            map_road_t road;
            if (map_road_info_get(i, &road) && (road.node_a == cell.cell_id || road.node_b == cell.cell_id))
                road_mask |= (uint16_t)(1 << i);
        }
    }
    return road_mask;
}

// 处理一圈聚类结果
uint8_t map_lidar_handle_area(uint8_t pose_valid, const map_rect_t *area) {
    uint16_t road_mask = 0;
    uint8_t complete = !obstacle_result.truncated;

    // 检查位姿有效性、点云更新、聚类结果有效性和区域参数合法性
    if (!pose_valid || !map_update_is_ready() ||
        obstacle_result.cloud_flag == 0 ||
        obstacle_result.cloud_flag != cloud.update_flag ||
        obstacle_result.cluster_count > CLUSTER_MAX_COUNT) {
        map_update_clear_candidates();
        return 0;
    }

    // 检查区域参数合法性：min_x <= max_x, min_y <= max_y, 坐标为有限数
    if (area != NULL && (!isfinite(area->min_x) || !isfinite(area->max_x) ||
        !isfinite(area->min_y) || !isfinite(area->max_y) ||
        area->min_x > area->max_x || area->min_y > area->max_y)) {
        map_update_clear_candidates();
        return 0;
    }

    // 遍历聚类结果  按起止角度标记有效点  转换成道路命中结果
    for (uint16_t i = 0; i < obstacle_result.cluster_count; i++) {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];

        if (!obstacle_cluster_is_pillar(cluster))
            continue;
        if (cluster->start_angle >= LIDAR_ANGLE_COUNT || cluster->end_angle >= LIDAR_ANGLE_COUNT) {
            complete = 0;
            continue;
        }
        if (area != NULL && (cluster->min_x < area->min_x || cluster->max_x > area->max_x ||
            cluster->min_y < area->min_y || cluster->max_y > area->max_y))
            continue;

        // 起止角度标记了这一组点的范围  按环形角度遍历  兼容359度跨到0度  只使用真正有效的点  不填满包围框  也不额外复制整份点云
        uint16_t span = (cluster->end_angle + LIDAR_ANGLE_COUNT - cluster->start_angle) % LIDAR_ANGLE_COUNT;
        for (uint16_t step = 0; step <= span; step++) {
            uint16_t angle = (cluster->start_angle + step) % LIDAR_ANGLE_COUNT;
            const lidar_point_t *point = &cloud.points[angle];

            if (!point->valid)
                continue;
            if (!isfinite(point->map_point.x) || !isfinite(point->map_point.y)) {
                complete = 0;
                continue;
            }
            road_mask |= map_lidar_point_roads(point->map_point.x, point->map_point.y);
        }
    }

    // 更新道路状态  只要有一条道路被标记就更新  没有标记就清空候选
    return map_update_roads(road_mask, cloud.scan_num, cloud.scan_tick, complete);
}

// 处理一圈聚类结果  不限制区域
uint8_t map_lidar_handle(uint8_t pose_valid) {
    return map_lidar_handle_area(pose_valid, NULL);
}
