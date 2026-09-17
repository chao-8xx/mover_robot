//
// Created by mechreuo on 2026/9/5.
//

#include "headfile.h"

// 初始化结构体
point_cloud_t cloud = {0};
obstacle_result_t obstacle_result = {0};


////////////////////////////////////// 点云生成 ////////////////////////////////////////
// 点云生成   遍历360度雷达数据 并把有效点依次变换到map坐标系
void point_cloud_update(float car_x, float car_y, float car_yaw, uint32_t tick){
    point2d_t p_lidar;
    point2d_t p_base;
    point2d_t p_map;

    cloud.valid_count = 0;
    cloud.car_x = car_x;
    cloud.car_y = car_y;
    cloud.car_angle = car_yaw;

    // 地图坐标系随小车实时位姿更新，同一帧点云共用同一个变换
    tf_transform_set(&tf_map, car_x, car_y, car_yaw);

    // 更新点云数据
    for (uint16_t angle = 0; angle < 360; angle++) {
        uint16_t distance = lidar_data[angle];
        lidar_point_t *point = &cloud.points[angle];

        // 每次更新前先清空该角度的旧结果
        point->distance = distance;
        point->valid = 0;
        point->map_point.x = 0.0f;
        point->map_point.y = 0.0f;

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
    cloud.update_tick = tick;
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


// 合并跨越359度和0度的同一组雷达点
static void cluster_merge(const point_cloud_t *source) {
    lidar_cluster_t *first; // 指向第一个聚类
    lidar_cluster_t *last;  // 指向最后一个聚类
    uint16_t total_count;   // 合并后的总点数

    // 聚类大于等于2时才进行合并
    if (source == NULL || obstacle_result.cluster_count < 2)
        return;

    first = &obstacle_result.clusters[0];
    last = &obstacle_result.clusters[obstacle_result.cluster_count - 1];

    // 只有聚类确实分别接触0度和359度时才允许首尾合并
    if (first->start_angle != 0 || last->end_angle != 359 ||
        !cluster_points_connected(&source->points[359],&source->points[0]))
        return;


    total_count = first->point_count + last->point_count;

    // 按各自点数加权  重新计算合并后的中心位置
    first->center_point.x = (first->center_point.x * (float)first->point_count + last->center_point.x * (float)last->point_count) / (float)total_count;
    first->center_point.y = (first->center_point.y * (float)first->point_count + last->center_point.y * (float)last->point_count) / (float)total_count;

    first->start_angle = last->start_angle;
    first->point_count = total_count;

    // 更新包围框和最近点距离
    if (last->min_x < first->min_x)     first->min_x = last->min_x;
    if (last->max_x > first->max_x)     first->max_x = last->max_x;
    if (last->min_y < first->min_y)     first->min_y = last->min_y;
    if (last->max_y > first->max_y)     first->max_y = last->max_y;
    if (last->nearest_distance < first->nearest_distance)   first->nearest_distance = last->nearest_distance;
    first->width = first->max_x - first->min_x;
    first->height = first->max_y - first->min_y;

    // 最后一项已合并到第一项  减少一次聚类的有效数量
    cluster_clear(last);
    obstacle_result.cluster_count--;
}


// 按雷达角度顺序分割点云 生成候选障碍物聚类
void obstacle_cluster_update(const point_cloud_t *cloud, uint32_t tick){
    lidar_cluster_t current_cluster;
    const lidar_point_t *pre_point = NULL;
    uint8_t exist_cluster = 0;         // 当前存在聚类标志位
    uint32_t next_update_flag = obstacle_result.update_flag + 1;// 递增更新序号

    if (cloud == NULL)
        return;

    // 清除上一帧结果，同时保留递增的更新序号
    memset(&obstacle_result, 0, sizeof(obstacle_result));
    cluster_clear(&current_cluster);

    obstacle_result.update_flag = next_update_flag;
    obstacle_result.cloud_flag = cloud->update_flag;
    obstacle_result.update_tick = tick;

    // 遍历360度雷达点云  按相邻点距离聚类
    for (uint16_t angle = 0; angle < 360; angle++){
        const lidar_point_t *current_point = &cloud->points[angle]; // 更新当前点指针

        // 无效角度会结束当前连续聚类   待优化：可考虑将无效点也纳入聚类，但不计入点数，避免因单个无效点导致聚类中断
        if (current_point->valid == 0){
            if (exist_cluster){
                cluster_finish(&current_cluster);    // 若之前有真实聚类 则调用 cluster_finish 保存
                exist_cluster = 0;
            }

            pre_point = NULL;
            continue;
        }

        // 若当前没有真实聚类  直接使用该点开启新聚类
        if (!exist_cluster) {
            cluster_start(&current_cluster, angle, current_point);
            exist_cluster = 1;
        }
        else if (cluster_points_connected(pre_point, current_point)) {
            cluster_add_point(&current_cluster, angle, current_point);
        }
        else {
            // 不满足相邻点连通性时 即为当前点与上一点距离过大：保存旧聚类并开启新聚类
            cluster_finish(&current_cluster);
            cluster_start(&current_cluster, angle, current_point);
            exist_cluster = 1;
        }
        pre_point = current_point;      // 更新上一点指针
    }

    // 处理遍历结束时仍未保存的最后一个聚类
    if (exist_cluster)
        cluster_finish(&current_cluster);

    cluster_merge(cloud);
}
