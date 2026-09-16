#include "headfile.h"


// LiDAR 测试函数，按键1开始扫描，按键1长按停止扫描，扫描结果通过虚拟串口CDC输出
void lidar_test(void) {
    printf("================================\r\n");
    // 遍历输出 0-359 度的雷达数据
    for(int i = 0; i < 360; i++)
    {
        // 格式对齐：@后面跟3位角度，5位距离
        printf("@%3d %5d \r\n", i, lidar_data[i]);
    }
    printf("================================\r\n");
}


// 极坐标转换函数测试
void tf2d_test_polar(void)
{
    point2d_t p;

    p = tf_lidar_polar_to_xy(0, 1000);
    printf("0: x=%.1f y=%.1f\r\n", p.x, p.y);

    p = tf_lidar_polar_to_xy(90, 1000);
    printf("90: x=%.1f y=%.1f\r\n", p.x, p.y);

    p = tf_lidar_polar_to_xy(180, 1000);
    printf("180: x=%.1f y=%.1f\r\n", p.x, p.y);

    p = tf_lidar_polar_to_xy(270, 1000);
    printf("270: x=%.1f y=%.1f\r\n", p.x, p.y);
}

// 车身坐标系测试
void tf2d_test_base(void)
{
    point2d_t p;
    point2d_t out;
    tf_transform2d_t tf;

    printf("===== TF2D TEST =====\r\n");


    // 1.纯平移
    p.x = 100.0f;
    p.y = 200.0f;

    tf_transform_set(&tf, 20.0f, 50.0f, 0.0f);

    out = tf_transform_point(p, &tf);

    printf("平移预期=(120,250), 真实值=(%.1f,%.1f)\r\n",
           out.x, out.y);


    // 2.纯旋转
    p.x = 100.0f;
    p.y = 0.0f;

    tf_transform_set(&tf, 0.0f, 0.0f, 90.0f);

    out = tf_transform_point(p, &tf);

    printf("旋转预期=(0,100), 真实值=(%.1f,%.1f)\r\n",
           out.x, out.y);


    //3.旋转+平移
    p.x = 100.0f;
    p.y = 0.0f;

    tf_transform_set(&tf, 20.0f, 50.0f, 90.0f);

    out = tf_transform_point(p, &tf);

    printf("变换预期=(20,150), 真实值=(%.1f,%.1f)\r\n",
           out.x, out.y);


    printf("=====================\r\n");
}

// 地图坐标系测试
void tf2d_test_map(void)
{
    point2d_t p_base = {0.0f, 1000.0f};
    point2d_t p_map;

    tf_transform2d_t tf_map_base;


    tf_transform_set(&tf_map_base,500.0f, 500.0f,-0.0f);

    p_map = tf_transform_point(p_base,&tf_map_base);

    printf("heading 0   expected=(500,1500), actual=(%.1f,%.1f)\r\n",p_map.x, p_map.y);


    tf_transform_set(&tf_map_base,500.0f, 500.0f,-90.0f);

    p_map = tf_transform_point(p_base,&tf_map_base);

    printf("heading 90  expected=(1500,500), actual=(%.1f,%.1f)\r\n",
           p_map.x, p_map.y);


    tf_transform_set(&tf_map_base,500.0f, 500.0f,-180.0f);

    p_map = tf_transform_point(p_base,&tf_map_base);

    printf("heading 180 expected=(500,-500), actual=(%.1f,%.1f)\r\n",
           p_map.x, p_map.y);


    tf_transform_set(&tf_map_base,500.0f, 500.0f,-270.0f);

    p_map = tf_transform_point(p_base,&tf_map_base);

    printf("heading 270 expected=(-500,500), actual=(%.1f,%.1f)\r\n",
           p_map.x, p_map.y);
}

// 2D坐标变换测试函数，定时打印车身姿态和地图坐标系下的点坐标
void tf2d_test(void) {
    static uint32_t last_print = 0;
    const float car_x = 500.0f;
    const float car_y = 500.0f;
    static const uint16_t test_angles[4] = {0, 90, 180, 270};

    /* 每次打印的四个点共用同一时刻的车身姿态 */
    const float yaw = my_yaw;
    tf_transform2d_t tf_map_base;

    /* 500 ms 到期前直接返回，不让串口打印占用主循环 */
    if (HAL_GetTick() - last_print < 500) {
        return;
    }
    last_print = HAL_GetTick();

    /* T_map_base：把 base 坐标中的点变换到 map 坐标 */
    tf_transform_set(&tf_map_base, car_x, car_y, yaw);

    for (uint8_t i = 0; i < 4; i++) {
        const uint16_t angle = test_angles[i];
        const uint16_t distance = lidar_data[angle];

        /* distance == 0 表示该角度当前没有有效测距 */
        if (distance == 0) {
            printf("a=%u d=0 INVALID\r\n", (unsigned int)angle);
            continue;
        }

        /* polar -> lidar -> base -> map */
        const point2d_t p_lidar = tf_lidar_polar_to_xy(angle, distance);
        const point2d_t p_base = tf_transform_point(p_lidar, &tf_base);
        const point2d_t p_map = tf_transform_point(p_base, &tf_map_base);

        printf("a=%u d=%u lidar=(%.1f,%.1f) yaw=%.1f map=(%.1f,%.1f)\r\n",
               (unsigned int)angle,
               (unsigned int)distance,
               p_lidar.x,
               p_lidar.y,
               yaw,
               p_map.x,
               p_map.y);
    }
}

// 点云测试函数，定时打印点云摘要和前8个有效点
void point_cloud_test(void) {

    static uint32_t last_cloud_print = 0;

    uint32_t now = HAL_GetTick();


    /* 每500ms打印一次摘要和前8个有效点 */
    if (now - last_cloud_print >= 500)
    {
        last_cloud_print = now;

        printf("点云 更新次数=%lu valid=%u "
               "位姿pose=(%.1f,%.1f,%.1f) age=%lums\r\n",
               (unsigned long)cloud.update_flag,
               (unsigned int)cloud.valid_count,
               cloud.car_x,
               cloud.car_y,
               cloud.car_angle,
               (unsigned long)(now - cloud.update_tick));

        uint8_t printed = 0;

        for (uint16_t angle = 0;
             angle < 360 && printed < 8;
             angle++)
        {
            lidar_point_t *point = &cloud.points[angle];

            if (point->valid == 0)
            {
                continue;
            }

            printf("angle=%u distance=%.0f map=(%.1f,%.1f)\r\n",
                   (unsigned int)angle,
                   point->distance,
                   point->map_point.x,
                   point->map_point.y);

            printed++;
        }
    }
}
// 障碍物聚类测试函数，定时打印聚类摘要和前几个候选物体
void obstacle_cluster_test(void)
{
    static uint32_t last_cluster_print = 0;
    uint32_t now = HAL_GetTick();

    // 每500ms打印一次，避免大量串口输出阻塞主循环
    if (now - last_cluster_print < 500U)
        return;

    last_cluster_print = now;

    printf("CLUSTER update=%lu cloud=%lu count=%u age=%lums\r\n",
           (unsigned long)obstacle_result.update_flag,
           (unsigned long)obstacle_result.cloud_flag,
           (unsigned int)obstacle_result.cluster_count,
           (unsigned long)(now - obstacle_result.update_tick));

    for (uint16_t i = 0;
         i < obstacle_result.cluster_count && i < 8U;
         i++)
    {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];

        printf("C%u angle=%u~%u points=%u center=(%.1f,%.1f) "
               "size=(%.1f,%.1f) near=%.0f\r\n",
               (unsigned int)i,
               (unsigned int)cluster->start_angle,
               (unsigned int)cluster->end_angle,
               (unsigned int)cluster->point_count,
               cluster->center_point.x,
               cluster->center_point.y,
               cluster->width,
               cluster->height,
               cluster->nearest_distance);
    }
}
