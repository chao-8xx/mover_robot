//
// Created by mechreuo on 2026/9/5.
//

#include "headfile.h"

// 初始化结构体
point_cloud_t cloud = {0};
obstacle_result_t obstacle_result = {0};

static uint32_t last_cloud_scan_num = 0;  // 上一次点云更新使用的雷达扫描编号


///////////////////////////////////// 相邻点聚类 //////////////////////////////////////
// 判断一个角度是否在指定的起始角度和结束角度之间 考虑跨0度的情况
static uint8_t get_angle_right(uint16_t angle, uint16_t start, uint16_t end) {
    if (start <= end) {
        return (angle >= start && angle <= end);
    }

    return (angle >= start || angle <= end);
}

// 判断当前角度是否被小车四个轮子占用的角度范围内
static uint8_t get_wheel_blocked(uint16_t angle) {
    // 未标定时四组0~0只是占位值，不能把真正的0度回波误删。
    if (!LIDAR_WHEEL_MASK_ENABLE)
        return 0;

    if (get_angle_right(angle, LEFT_FORNT_WHEEL_START, LEFT_FORNT_WHEEL_END))
        return 1;
    if (get_angle_right(angle, RIGHT_FORNT_WHEEL_START, RIGHT_FORNT_WHEEL_END))
        return 1;
    if (get_angle_right(angle, LEFT_REAR_WHEEL_START, LEFT_REAR_WHEEL_END))
        return 1;
    if (get_angle_right(angle, RIGHT_REAR_WHEEL_START, RIGHT_REAR_WHEEL_END))
        return 1;

    return 0;
}

////////////////////////////////////// 点云生成 ////////////////////////////////////////
// 点云生成   遍历360度雷达数据 并把有效点依次变换到map坐标系
uint8_t point_cloud_update(const lidar_scan_t *scan, float car_x, float car_y, float car_yaw, uint32_t tick){
    point2d_t p_lidar;
    point2d_t p_base;
    point2d_t p_map;

    // 避免重复更新同一帧点云
    if (scan == NULL || scan->scan_num == 0 || scan->scan_num == last_cloud_scan_num ||
        !isfinite(car_x) || !isfinite(car_y) || !isfinite(car_yaw) ||
        !isfinite(tf_base.tx) || !isfinite(tf_base.ty) ||
        !isfinite(tf_base.sin_yaw) || !isfinite(tf_base.cos_yaw)) return 0;

    last_cloud_scan_num = scan->scan_num;

    cloud.valid_count = 0;
    cloud.car_x = car_x;
    cloud.car_y = car_y;
    cloud.car_angle = car_yaw;

    // 地图坐标系随小车实时位姿更新，同一帧点云共用同一个变换
    tf_transform_set(&tf_map, car_x, car_y, car_yaw);

    // 更新点云数据
    for (uint16_t angle = 0; angle < 360; angle++) {
        lidar_point_t *point = &cloud.points[angle];
        uint16_t distance = scan->distance[angle];

        // 每次更新前先清空该角度的旧结果
        point->valid = 0;
        point->distance = distance;
        point->map_point.x = 0.0f;
        point->map_point.y = 0.0f;

        // 跳过被小车四个轮子遮挡的角度
        if (get_wheel_blocked(angle))
            continue;

        // 当前角度无效点 跳过
        if (scan->valid[angle] == 0) {
            continue;
        }

        // 雷达距离限位 过滤无效距离
        if (distance < (uint16_t)LIDAR_MIN_DISTANCE || distance > (uint16_t)LIDAR_MAX_DISTANCE) {
            // cloud.points[angle].vaild = 0;
            continue;
        }

        // 雷达极坐标 -> lidar坐标 -> base坐标 -> map坐标
        p_lidar = tf_lidar_polar_to_xy(angle, distance);
        p_base = tf_transform_point(p_lidar, &tf_base);
        p_map = tf_transform_point(p_base, &tf_map);

        // 更新点云数据结构体
        point->map_point = p_map;
        point->valid = 1;
        cloud.valid_count++;
    }

    cloud.update_flag++;
    cloud.scan_num = scan->scan_num;
    cloud.update_tick = tick;

    return 1;
}


///////////////////////////////////// 相邻点聚类 //////////////////////////////////////
// 清空一个聚类对象
static void cluster_clear(lidar_cluster_t *cluster)
{
    if (cluster == NULL)
        return;

    memset(cluster, 0, sizeof(*cluster));   // 清零整个结构体
}


// 使用第一个有效点初始化创建新的临时聚类
static void cluster_start(lidar_cluster_t *cluster,uint16_t angle,const lidar_point_t *point) {
    if (cluster == NULL || point == NULL)
        return;

    cluster_clear(cluster);

    // 使用第一个有效点初始化聚类属性 后续聚类点会增量更新这些属性
    cluster->start_angle = angle;
    cluster->end_angle = angle;
    cluster->point_count = 1;

    cluster->center_point = point->map_point;
    cluster->min_x = point->map_point.x;
    cluster->max_x = point->map_point.x;
    cluster->min_y = point->map_point.y;
    cluster->max_y = point->map_point.y;
    cluster->nearest_distance = point->distance;
}


// 将一个相邻点加入当前临时聚类，并增量更新中心和包围框
static void cluster_add_point(lidar_cluster_t *cluster,uint16_t angle,const lidar_point_t *point) {
    uint16_t new_count;

    if (cluster == NULL || point == NULL || cluster->point_count == 0)
        return;

    new_count = cluster->point_count + 1;
    cluster->end_angle = angle;

    // 增量平均值    避免为每个聚类额外保存全部点坐标 节省内存
    cluster->center_point.x += (point->map_point.x - cluster->center_point.x) / (float)new_count;
    cluster->center_point.y += (point->map_point.y - cluster->center_point.y) / (float)new_count;

    // 更新包围框和最近点距离
    if (point->map_point.x < cluster->min_x)
        cluster->min_x = point->map_point.x;
    if (point->map_point.x > cluster->max_x)
        cluster->max_x = point->map_point.x;
    if (point->map_point.y < cluster->min_y)
        cluster->min_y = point->map_point.y;
    if (point->map_point.y > cluster->max_y)
        cluster->max_y = point->map_point.y;
    if (point->distance < cluster->nearest_distance)
        cluster->nearest_distance = point->distance;

    cluster->point_count = new_count;
}


// 判断相邻两个雷达点在空间中是否足够接近  连通性判断  待后续更新优化为根据不同距离来设置不同的阈值分辨率
static uint8_t cluster_points_connected(const lidar_point_t *previous,const lidar_point_t *current) {
    float dx;
    float dy;
    float distance_square;
    const float gap_square = CLUSTER_MAX_POINT_GAP * CLUSTER_MAX_POINT_GAP;

    if (previous == NULL || current == NULL || previous->valid == 0 || current->valid == 0)
        return 0;

    // 计算两个点在地图坐标系下的距离平方
    dx = current->map_point.x - previous->map_point.x;
    dy = current->map_point.y - previous->map_point.y;
    distance_square = dx * dx + dy * dy;

    // 比较距离平方 不调用 sqrtf 开方 降低单片机计算量
    return distance_square <= gap_square;
}


// 保存有效聚类  完成一个临时聚类，满足最小点数后保存到结果数组
static void cluster_finish(lidar_cluster_t *cluster)
{
    lidar_cluster_t *final;

    if (cluster == NULL || cluster->point_count == 0)
        return;

    // 仅当聚类点数达到阈值且结果数组未满时才保存
    if (cluster->point_count >= CLUSTER_MIN_POINT_COUNT && obstacle_result.cluster_count < CLUSTER_MAX_COUNT) {

        // 计算聚类的包围框宽高
        cluster->width = cluster->max_x - cluster->min_x;
        cluster->height = cluster->max_y - cluster->min_y;
        cluster->valid = 1;

        // 将聚类结果复制到全局结果数组中
        final = &obstacle_result.clusters[obstacle_result.cluster_count];
        *final = *cluster;
        obstacle_result.cluster_count++;
    }

    cluster_clear(cluster);
}


// 从真正断开的位置开始遍历，让跨0度的一组点先完整收集，再判断最小点数。
static uint16_t cluster_start_angle(const point_cloud_t *source) {
    uint16_t first_angle = 0;
    uint16_t last_angle = LIDAR_ANGLE_COUNT - 1;
    uint16_t miss_angle_num = 0;
    const lidar_point_t *pre_point;

    while (first_angle < LIDAR_ANGLE_COUNT && !source->points[first_angle].valid)
        first_angle++;
    if (first_angle == LIDAR_ANGLE_COUNT)
        return 0;

    while (!source->points[last_angle].valid)
        last_angle--;

    // 首尾本来就断开时，仍按原来的角度顺序处理。
    miss_angle_num = first_angle + LIDAR_ANGLE_COUNT - 1 - last_angle;
    if (miss_angle_num > CLUSTER_MAX_MISS_ANGLE ||
        !cluster_points_connected(&source->points[last_angle], &source->points[first_angle]))
        return first_angle;

    pre_point = &source->points[first_angle];
    miss_angle_num = 0;
    for (uint16_t angle = first_angle + 1; angle <= last_angle; angle++) {
        const lidar_point_t *point = &source->points[angle];
        if (!point->valid) {
            miss_angle_num++;
            continue;
        }
        if (miss_angle_num > CLUSTER_MAX_MISS_ANGLE || !cluster_points_connected(pre_point, point))
            return angle;
        pre_point = point;
        miss_angle_num = 0;
    }

    // 整圈都连通时，任选起点都只会得到一个完整聚类。
    return first_angle;
}


// 按雷达角度顺序分割点云 生成候选障碍物聚类
void obstacle_cluster_update(const point_cloud_t *cloud, uint32_t tick){
    lidar_cluster_t current_cluster;
    const lidar_point_t *pre_point = NULL;

    uint8_t real_cluster = 0;         // 当前存在聚类标志位
    uint8_t miss_angle_num = 0;       // 连续无效点计数
    uint16_t start_angle;
    uint32_t next_update_flag = obstacle_result.update_flag + 1;// 递增更新序号

    if (cloud == NULL)
        return;

    // 清除上一帧结果 同时保留递增的更新序号
    memset(&obstacle_result, 0, sizeof(obstacle_result));
    cluster_clear(&current_cluster);

    obstacle_result.update_flag = next_update_flag;
    obstacle_result.cloud_flag = cloud->update_flag;
    obstacle_result.update_tick = tick;

    start_angle = cluster_start_angle(cloud);
    // 从断开处绕一圈，0/359边界与其他位置使用相同的缺点和距离规则。
    for (uint16_t step = 0; step < LIDAR_ANGLE_COUNT; step++){
        uint16_t angle = (start_angle + step) % LIDAR_ANGLE_COUNT;
        const lidar_point_t *current_point = &cloud->points[angle]; // 更新当前点指针

        // 少量缺点暂时保留聚类，缺点本身不计入有效点数。
        if (current_point->valid == 0){
            if (real_cluster){
                miss_angle_num ++;

                // 连续无效点多于等于3个时 认为当前聚类已经结束
                if (miss_angle_num > CLUSTER_MAX_MISS_ANGLE) {
                    cluster_finish(&current_cluster);

                    real_cluster = 0;
                    miss_angle_num = 0;
                    pre_point = NULL;
                }
            }

            continue;
        }

        // 若当前没有真实聚类  直接使用该点开启新聚类
        if (!real_cluster) {
            cluster_start(&current_cluster, angle, current_point);
            real_cluster = 1;
        }
        else if (cluster_points_connected(pre_point, current_point)) {
            cluster_add_point(&current_cluster, angle, current_point);
        }
        else {
            // 不满足相邻点连通性时 即为当前点与上一点距离过大：保存旧聚类并开启新聚类
            cluster_finish(&current_cluster);
            cluster_start(&current_cluster, angle, current_point);
            real_cluster = 1;
        }
        miss_angle_num = 0;             // 重置连续无效点计数
        pre_point = current_point;      // 更新上一点指针
    }

    // 处理遍历结束时仍未保存的最后一个聚类
    if (real_cluster)
        cluster_finish(&current_cluster);
}
