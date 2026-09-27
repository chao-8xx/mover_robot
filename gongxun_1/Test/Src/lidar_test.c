#include <sys/stat.h>

#include "headfile.h"


// // LiDAR 测试函数，按键1开始扫描，按键1长按停止扫描，扫描结果通过虚拟串口CDC输出
// void lidar_test(void) {
//     printf("================================\r\n");
//     // 遍历输出 0-359 度的雷达数据
//     for(int i = 0; i < 360; i++)
//     {
//         // 格式对齐：@后面跟3位角度，5位距离
//         printf("@%3d %5d \r\n", i, lidar_data[i]);
//     }
//     printf("================================\r\n");
// }

// 获取指定角度的距离值   无效点 则返回 0
static uint16_t lidar_test_get_distance(const lidar_scan_t *scan, uint16_t angle) {
    if (scan == NULL || angle >= 360)
        return 0;

    if (scan->valid[angle] == 0)
        return 0;

    return scan->distance[angle];
}

// LiDAR 测试函数，按键1开始扫描，按键1长按停止扫描，扫描结果通过虚拟串口CDC输出
void lidar_test(void) {
    static uint8_t test_start = 0;      // test 测试开始标志位
    static uint32_t last_scan_num = 0;  // 上一次扫描完成的编号

    static uint32_t all_scan_num = 0;   // 全部扫描完成的数量
    static uint32_t all_skip_num = 0;   // 全部扫描丢失的数量

    static uint8_t window_cnt = 0;      // 当前扫描完成的数量
    static uint32_t window_sum = 0;     // 窗口扫瞄的总数
    static uint32_t window_min = 0;     // 窗口的扫描最少值
    static uint32_t window_max = 0;     // 窗口的扫描最多值

    static uint32_t raw_sum = 0;        // 原始扫描的总数
    static uint16_t raw_min = 0;        // 原始扫描的最少值
    static uint16_t raw_max = 0;        // 原始扫描的最多值

    static uint32_t valid_sum = 0;
    static uint16_t valid_min = 0;
    static uint16_t valid_max = 0;

    const lidar_scan_t *scan;

    uint32_t period;
    uint32_t skipped_scan = 0;

    uint16_t distance_0;
    uint16_t distance_90;
    uint16_t distance_180;
    uint16_t distance_270;

    if (test_start == 0) {
        test_start = 1;
        printf("LiDAR 测试开始\r\n");
    }

    lidar_process();

    scan = lidar_get_scan();

    if (scan == NULL || scan->scan_num == 0)
        return;

    if (scan->scan_num == last_scan_num)
        return;

    // 检查测试函数是否漏掉了中间发布的完整圈
    if (last_scan_num != 0 && scan->scan_num > last_scan_num + 1) {

        skipped_scan = scan->scan_num - last_scan_num - 1;

        all_skip_num += skipped_scan;
    }

    last_scan_num = scan->scan_num;


    // 无符号减法可以正确处理HAL_GetTick回绕
    period = scan->end_time - scan->start_time;

    distance_0 = lidar_test_get_distance(scan, 0);

    distance_90 = lidar_test_get_distance(scan, 90);

    distance_180 = lidar_test_get_distance(scan, 180);

    distance_270 = lidar_test_get_distance(scan, 270);


    // 初始化当前统计窗口的最小值和最大值
    if (window_sum == 0) {
        window_min = period;
        window_max = period;

        raw_min = scan->raw_count;
        raw_max = scan->raw_count;

        valid_min = scan->valid_count;
        valid_max = scan->valid_count;
    }
    else {
        if (period < window_min) {
            window_min = period;
        }

        if (period > window_max) {
            window_max = period;
        }

        if (scan->raw_count < raw_min) {
            raw_min = scan->raw_count;
        }

        if (scan->raw_count > raw_max) {
            raw_max = scan->raw_count;
        }

        if (scan->valid_count < valid_min) {
            valid_min = scan->valid_count;
        }

        if (scan->valid_count > valid_max) {
            valid_max = scan->valid_count;
        }
    }


    window_sum += period;
    raw_sum += scan->raw_count;
    valid_sum += scan->valid_count;

    window_cnt++;
    all_scan_num++;


    // 每个完整圈只输出一行
    printf("SCAN n=%lu dt=%lums raw=%u valid=%u skip=%lu "
           "d0=%u d90=%u d180=%u d270=%u\r\n",
           (unsigned long)scan->scan_num,
           (unsigned long)period,
           (unsigned int)scan->raw_count,
           (unsigned int)scan->valid_count,
           (unsigned long)skipped_scan,
           (unsigned int)distance_0,
           (unsigned int)distance_90,
           (unsigned int)distance_180,
           (unsigned int)distance_270);


    // 每20圈输出一次最小值、平均值、最大值
    if (window_cnt >= 20) {

        printf("AAAA total=%lu\r\n"        // 扫描的总点数
               "dt=%lu/%lu/%lu\r\n"        // 扫描的最小值/平均值/最大值
               "raw=%u/%lu/%u\r\n"         // 扫描的原始点数最小值/平均值/最大值
               "valid=%u/%lu/%u\r\n"       // 扫描的有效点数最小值/平均值/最大值
               "skip_total=%lu\r\n"        // 扫描丢失的总点数
               "xor_ok=%lu\r\n"            // XOR校验成功计数
               "xor_err=%lu\r\n",          // XOR校验失败计数
               (unsigned long)all_scan_num,

               (unsigned long)window_min,
               (unsigned long)(window_sum / window_cnt),
               (unsigned long)window_max,

               (unsigned int)raw_min,
               (unsigned long)(raw_sum / window_cnt),
               (unsigned int)raw_max,

               (unsigned int)valid_min,
               (unsigned long)(valid_sum / window_cnt),
               (unsigned int)valid_max,

               (unsigned long)all_skip_num,
               (unsigned long)lidar_get_xor_ok_cnt(),
               (unsigned long)lidar_get_xor_err_cnt());


        // 开始下一个统计窗口
        window_cnt = 0;

        window_sum = 0;
        window_min = 0;
        window_max = 0;

        raw_sum = 0;
        raw_min = 0;
        raw_max = 0;

        valid_sum = 0;
        valid_min = 0;
        valid_max = 0;
    }
}

#define WHEEL_TEST_TYPE_NEAR 1   // 近距离点数量大于60个时 认为该角度被车轮遮挡
#define WHEEL_TEST_TYPE_BLIND 2  // 有效点数量小于等于20个时 认为该角度被车轮遮挡

// 车轮遮挡雷达角度统计结构体
typedef struct {
    uint16_t valid_cnt;  // 当前有效雷达点数量
    uint16_t near_cnt;   // 当前距离雷达最近的点数量

    uint16_t min_near;   // 当前距离雷达最近的点的最小值
    uint16_t max_near;   // 当前距离雷达最近的点的最大值
    uint16_t sum_near;   // 当前距离雷达最近的点的总和
} wheel_mask_t;

static wheel_mask_t wheel_mask[360];

static uint8_t get_wheel_masked(uint16_t angle, uint8_t type) {

    const wheel_mask_t *mask = &wheel_mask[angle];

    if (angle >= 360)
        return 0;

    // 近距离点数量大于60个时 认为该角度被车轮遮挡
    if (type == WHEEL_TEST_TYPE_NEAR)
        return mask->near_cnt >= 60;

    // 有效点数量小于等于20个时 认为该角度被车轮遮挡
    if (type == WHEEL_TEST_TYPE_BLIND)
        return mask->valid_cnt <= 20;

    return wheel_mask[angle].valid_cnt > 0;
}

// 输出一个连续候选角度区间
static void wheel_test_print_range(uint16_t start_angle, uint16_t end_angle, uint16_t type) {
    uint16_t angle;
    uint16_t angle_cnt = 0;

    uint32_t valid_total = 0;
    uint32_t near_total = 0;
    uint32_t near_sum = 0;

    uint16_t near_min = 0;
    uint16_t near_max = 0;
    angle = start_angle;

    // 遍历指定角度范围内的雷达点
    while (1) {
        const wheel_mask_t *mask = &wheel_mask[angle];
        angle_cnt++;
        valid_total += mask->valid_cnt;
        near_total += mask->near_cnt;
        near_sum += mask->sum_near;

        // 更新最小值和最大值
        if (mask->near_cnt != 0) {
            if (mask->min_near < near_min)  near_min = mask->min_near;
            if (mask->max_near < near_max)  near_max = mask->max_near;
        }
        if (angle == end_angle)
            break;
        angle++;
        if (angle >= 360)   angle %= 360;
    }
    //
    if (type == WHEEL_TEST_TYPE_NEAR) {
        uint32_t near_avg = near_total > 0 ? near_sum / near_total : 0;

        printf("NEAR range = %u~%u, angle_cnt=%u,\r\n "
               "valid_total=%lu, near_total=%lu,\r\n "
               "near_min=%u, near_max=%u, near_avg=%lu\r\n",
               start_angle, end_angle, angle_cnt,   // 起始角度 结束角度 角度数量
               (unsigned long)valid_total,          // 有效点总数
               (unsigned long)near_total,           // 近距离点总数
               near_min, near_max,                  // 近距离点最小值 最大值
               (unsigned long)near_avg);            // 近距离点平均值
    }
    else {
        uint32_t sample_avg = angle_cnt > 0 ? valid_total / angle_cnt : 0;

        printf("BLIND range=%u~%u angles=%u "
              "valid=%lu/%lu\r\n",
              (unsigned int)start_angle,        // 起始角度
              (unsigned int)end_angle,          // 结束角度
              (unsigned int)angle_cnt,          // 角度数量
              (unsigned long)valid_total,       // 有效点总数
              (unsigned long)sample_avg);       // 平均每个角度的有效点数
    }
}

//
static void wheel_test_print_group(uint8_t type) {
    uint16_t break_angle = 0;   // 当前连续区间的结束角度
    uint16_t start_angle = 0;   // 当前连续区间的起始角度
    uint16_t end_angle = 0;     // 当前连续区间的结束角度

    // uint8_t found_break;        // 当前角度是否是连续区间的结束角度
    uint8_t group_active = 0;   // 当前是否在一个连续区间内
    uint8_t group_count = 0;    // 连续区间数量

    // 先寻找一个非阻挡区当遍历起点
    for (uint16_t angle = 0; angle < 360; angle++) {

        // if (!get_wheel_masked) {
        //     break_angle = angle;
        //     found_break = 1;
        //     break;
        // }
    }

    // 360度全部属于候选区域
    // if (!found_break) {
    //     wheel_test_print_range(0, 359, type);
    //     return;
    // }

    // 从非候选角度的下一个角度开始循环一整圈
    for (uint16_t step = 1; step <= 360U; step++)
    {
        uint16_t angle = (uint16_t)((break_angle + step) % 360U);

        if (get_wheel_masked(angle, type))
        {
            if (!group_active)
            {
                start_angle = angle;
                group_active = 1;
            }

            end_angle = angle;
        }
        else if (group_active)
        {
            wheel_test_print_range(start_angle,
                                   end_angle,
                                   type);

            group_active = 0;
            group_count++;
        }
    }

    if (group_count == 0)
    {
        if (type == WHEEL_TEST_TYPE_NEAR)
        {
            printf("NEAR none\r\n");
        }
        else
        {
            printf("BLIND none\r\n");
        }
    }
}
// 四车轮遮挡雷达角度测试
void wheel_mask_test(void) {
    static uint8_t test_started = 0;
    static uint8_t test_finished = 0;

    static uint16_t scan_count = 0;
    static uint32_t last_scan_num = 0;

    const lidar_scan_t *scan;

    // 必须持续处理雷达DMA数据
    lidar_process();

    // 测试已经结束，仍处理雷达，但不再重复统计和打印
    if (test_finished)
        return;

    if (!test_started)
    {
        test_started = 1;

        memset(wheel_mask, 0, sizeof(wheel_mask));

        printf("===== WHEEL MASK TEST START =====\r\n");
        printf("scan=%u near_distance<=%umm\r\n",
               (unsigned int)100, (unsigned int)400);
    }

    scan = lidar_get_scan();

    if (scan == NULL || scan->scan_num == 0)
        return;

    // 同一个完整圈只统计一次
    if (scan->scan_num == last_scan_num)
        return;

    last_scan_num = scan->scan_num;

    // 统计当前完整圈的360度原始数据
    for (uint16_t angle = 0; angle < 360U; angle++)
    {
        wheel_mask_t *stat =
            &wheel_mask[angle];

        uint16_t distance;

        if (scan->valid[angle] == 0)
            continue;

        distance = scan->distance[angle];

        stat->valid_cnt++;

        // 记录固定近距离回波
        if (distance <= 400)
        {
            stat->near_cnt++;
            stat->sum_near += distance;

            if (stat->near_cnt == 1)
            {
                stat->min_near = distance;
                stat->max_near = distance;
            }
            else
            {
                if (distance < stat->min_near)
                    stat->min_near = distance;

                if (distance > stat->max_near)
                    stat->max_near = distance;
            }
        }
    }

    scan_count++;

    // 每20圈提示一次进度
    if ((scan_count % 20U) == 0U)
    {
        printf("WHEEL collect=%u/%u\r\n",
               (unsigned int)scan_count,
               (unsigned int)100);
    }

    if (scan_count < 100)
        return;

    printf("===== 固定近距离 =====\r\n");

    // 输出稳定近距离区域
    wheel_test_print_group(WHEEL_TEST_TYPE_NEAR);

    printf("===== 固定盲区 =====\r\n");

    // 输出长期无有效数据区域
    wheel_test_print_group(WHEEL_TEST_TYPE_BLIND);

    printf("===== 车轮遮挡测试结束 =====\r\n");
    printf("移动或旋转小车 然后复位并再次测试。.\r\n");

    test_finished = 1;
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
