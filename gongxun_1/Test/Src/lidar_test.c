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
    //
    // static uint32_t last_debug_tick = 0;
    // uint32_t now = HAL_GetTick();
    //
    // // 每秒输出一次调试信息
    // if (now - last_debug_tick >= 1000) {
    //     last_debug_tick = now;
    //     printf("LIDAR left=%lu ok=%lu err=%lu drop=%lu\r\n",
    //            (unsigned long)__HAL_DMA_GET_COUNTER(huart4.hdmarx),
    //            (unsigned long)lidar_get_xor_ok_cnt(),
    //            (unsigned long)lidar_get_xor_err_cnt(),
    //            (unsigned long)lidar_get_scan_drop_cnt());
    // }

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

    if (angle >= 360)
        return 0;

    const wheel_mask_t *mask = &wheel_mask[angle];

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

    uint16_t near_min = 9999;
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
            if (mask->max_near > near_max)  near_max = mask->max_near;
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

    uint8_t found_break = 0;    // 当前角度是否是连续区间的结束角度
    uint8_t group_active = 0;   // 当前是否在一个连续区间内
    uint8_t group_count = 0;    // 连续区间数量

    // 先寻找一个非阻挡区当遍历起点
    for (uint16_t angle = 0; angle < 360; angle++) {

        if (!get_wheel_masked(angle, type)) {
            break_angle = angle;
            found_break = 1;
            break;
        }
    }

    // 360度全部属于候选区域
    if (!found_break) {
        wheel_test_print_range(0, 359, type);
        return;
    }

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


//////////////////////////////////// 雷达安装标定测试 ////////////////////////////////////
// 车体坐标沿用tf2d.c：原点选车体中心，+X向右，+Y向前，单位mm。
// 全程固定小车。先移走挡板采背景，再把同一块50~100mm宽的平面挡板
// 依次放在前、右、左，挡板要竖直且覆盖雷达扫描高度；每次仅放一块。
// 下表是挡板朝向雷达的表面中心坐标，不是从雷达外壳量的距离，也不是圆杆圆心。
// 挡板表面应分别垂直于车体前后/左右轴。先按卷尺实测修改下表，再编译测试。
// 本测试直接读取未屏蔽的整圈数据，不使用聚类结果，不改写tf_base。
#define INSTALL_TEST_SCAN_COUNT       100U     // 每组采100圈，与车轮测试一致
#define INSTALL_TEST_MIN_HIT           60U     // 至少60圈识别到同一块挡板
#define INSTALL_TEST_MIN_DISTANCE     450U     // 排除大部分车轮/底盘近距离回波
#define INSTALL_TEST_MAX_DISTANCE    2000U     // 标定目标应放在这个范围内
#define INSTALL_TEST_CHANGE_MM        100U     // 挡板要比原背景至少近100mm
#define INSTALL_TEST_MAX_SIZE_MM     200.0f    // 多个分散目标或宽大背景，不拿来标定
#define INSTALL_TEST_MAX_STD_MM       25.0f    // 单个目标跨圈位置波动限值
#define INSTALL_TEST_MAX_ERROR_MM     25.0f    // 三点拟合与假设检查的误差限值
#define INSTALL_TEST_SIGN_MARGIN_MM    5.0f    // 两种方向误差太接近时不下结论

static const point2d_t install_test_reference[3] = {
    {   0.0f, 800.0f },  // 前方挡板表面中心，相对于车体原点
    { 800.0f,   0.0f },  // 右侧挡板表面中心
    {-800.0f,   0.0f },  // 左侧挡板表面中心
};

// 背景角度统计，保存每个角度的命中次数和距离范围
typedef struct {
    uint16_t count;
    uint16_t min_distance;
    uint16_t max_distance;
} install_background_t;

static install_background_t install_background[360];
static point2d_t install_test_measured[3];  // 当前极坐标函数输出的三组平均位置

// 背景中稳定可见的角度：只接受明显变近的回波；长期无回波的角度接受新目标。
// 时有时无或距离波动大的背景不可靠，不能据此判断挡板。
static uint8_t install_test_changed(uint16_t angle, uint16_t distance) {
    const install_background_t *background = &install_background[angle];

    if (background->count <= 20U) return 1;
    if (background->count < INSTALL_TEST_MIN_HIT ||
        background->max_distance - background->min_distance > 50) return 0;

    return (uint32_t)distance + INSTALL_TEST_CHANGE_MM <= background->min_distance;
}

// 一个完整圈提取新增挡板。用XY平均处理跨0度目标，不能直接平均359度和0度。
static uint8_t install_test_get_point(const lidar_scan_t *scan, point2d_t *point) {
    point2d_t sum = {0.0f, 0.0f};
    float min_x = 0.0f, max_x = 0.0f;
    float min_y = 0.0f, max_y = 0.0f;
    uint16_t count = 0;

    for (uint16_t angle = 0; angle < 360U; angle++) {
        uint16_t distance = scan->distance[angle];
        point2d_t now;

        if (!scan->valid[angle] || distance < INSTALL_TEST_MIN_DISTANCE ||
            distance > INSTALL_TEST_MAX_DISTANCE || !install_test_changed(angle, distance))
            continue;

        now = tf_lidar_polar_to_xy(angle, distance);
        if (count == 0) {
            min_x = max_x = now.x;
            min_y = max_y = now.y;
        } else {
            if (now.x < min_x) min_x = now.x;
            if (now.x > max_x) max_x = now.x;
            if (now.y < min_y) min_y = now.y;
            if (now.y > max_y) max_y = now.y;
        }
        sum.x += now.x;
        sum.y += now.y;
        count++;
    }

    // 一圈允许只有一个角度命中窄挡板；靠100圈重复性确认，不套用聚类>=3点。
    if (count == 0 || max_x - min_x > INSTALL_TEST_MAX_SIZE_MM ||
        max_y - min_y > INSTALL_TEST_MAX_SIZE_MM) return 0;

    point->x = sum.x / (float)count;
    point->y = sum.y / (float)count;
    return 1;
}

// 计算三点变换后的均方根误差。sign=1沿用当前转换，sign=-1先镜像X再旋转。
static float install_test_error(int8_t sign, const tf_transform2d_t *tf) {
    float sum = 0.0f;

    for (uint8_t i = 0; i < 3U; i++) {
        point2d_t source = install_test_measured[i];
        point2d_t result;
        float dx, dy;

        source.x *= (float)sign;
        result = tf_transform_point(source, tf);
        dx = result.x - install_test_reference[i].x;
        dy = result.y - install_test_reference[i].y;
        sum += dx * dx + dy * dy;
    }
    return sqrtf(sum / 3.0f);
}

// 已知三点求旋转和平移。正反两个角度方向分别拟合，避免把倒装镜像误当成90度旋转。
static uint8_t install_test_fit(int8_t sign, tf_transform2d_t *tf) {
    point2d_t source_center = {0.0f, 0.0f};
    point2d_t target_center = {0.0f, 0.0f};
    float dot = 0.0f, cross = 0.0f;
    float source_area, target_area;
    float yaw;
    point2d_t rotated;

    // 三点共线时不能可靠区分镜像方向，拒绝给出安装结果。
    source_area = (install_test_measured[1].x - install_test_measured[0].x) *
                  (install_test_measured[2].y - install_test_measured[0].y) -
                  (install_test_measured[1].y - install_test_measured[0].y) *
                  (install_test_measured[2].x - install_test_measured[0].x);
    target_area = (install_test_reference[1].x - install_test_reference[0].x) *
                  (install_test_reference[2].y - install_test_reference[0].y) -
                  (install_test_reference[1].y - install_test_reference[0].y) *
                  (install_test_reference[2].x - install_test_reference[0].x);
    if (!isfinite(source_area) || !isfinite(target_area) ||
        fabsf(source_area) < 10000.0f || fabsf(target_area) < 10000.0f) return 0;

    for (uint8_t i = 0; i < 3U; i++) {
        source_center.x += (float)sign * install_test_measured[i].x / 3.0f;
        source_center.y += install_test_measured[i].y / 3.0f;
        target_center.x += install_test_reference[i].x / 3.0f;
        target_center.y += install_test_reference[i].y / 3.0f;
    }

    for (uint8_t i = 0; i < 3U; i++) {
        float sx = (float)sign * install_test_measured[i].x - source_center.x;
        float sy = install_test_measured[i].y - source_center.y;
        float tx = install_test_reference[i].x - target_center.x;
        float ty = install_test_reference[i].y - target_center.y;

        dot += sx * tx + sy * ty;
        cross += sx * ty - sy * tx;
    }
    if (!isfinite(dot) || !isfinite(cross))
        return 0;

    // 错误镜像模型可能恰好没有唯一最优角，此时取0度计算其误差，不影响另一模型。
    yaw = fabsf(dot) + fabsf(cross) < 1.0f ? 0.0f : atan2f(cross, dot) * 180.0f / TF_PI;
    tf_transform_set(tf, 0.0f, 0.0f, yaw);
    rotated = tf_transform_point(source_center, tf);
    tf_transform_set(tf, target_center.x - rotated.x, target_center.y - rotated.y, yaw);
    return 1;
}

// 只输出测量结论，不自动把尚未实车复核的参数写进正式坐标变换。
static void install_test_print_result(void) {
    tf_transform2d_t normal_tf, mirror_tf, hypothesis;
    const tf_transform2d_t *best;
    float normal_error, mirror_error, error;
    int8_t sign;

    if (!install_test_fit(1, &normal_tf) || !install_test_fit(-1, &mirror_tf)) {
        printf("INSTALL FAIL: 三点共线或数据无效，请重新摆放并复位测试\r\n");
        return;
    }

    normal_error = install_test_error(1, &normal_tf);
    mirror_error = install_test_error(-1, &mirror_tf);
    printf("INSTALL rms_current_sign=%.1f rms_mirror_sign=%.1f mm\r\n",
           normal_error, mirror_error);
    if (!isfinite(normal_error) || !isfinite(mirror_error) ||
        fabsf(normal_error - mirror_error) < INSTALL_TEST_SIGN_MARGIN_MM ||
        fminf(normal_error, mirror_error) > INSTALL_TEST_MAX_ERROR_MM) {
        printf("INSTALL FAIL: 方向不明确或拟合误差过大，检查目标坐标/背景/车是否移动\r\n");
        return;
    }

    sign = normal_error < mirror_error ? 1 : -1;
    best = sign == 1 ? &normal_tf : &mirror_tf;
    printf("INSTALL RESULT sign=%d tx=%.1f ty=%.1f yaw=%.2f rms=%.1f\r\n",
           (int)sign, best->tx, best->ty, best->yaw_deg,
           sign == 1 ? normal_error : mirror_error);
    printf("sign=1: 沿用当前-sin；sign=-1: 当前转换还需X镜像，不能只改安装角\r\n");
    for (uint8_t i = 0; i < 3U; i++) {
        point2d_t source = install_test_measured[i];
        point2d_t result;
        source.x *= (float)sign;
        result = tf_transform_point(source, best);
        printf("INSTALL CHECK point=%u base=(%.1f,%.1f) expected=(%.1f,%.1f)mm\r\n",
               (unsigned int)(i + 1), result.x, result.y,
               install_test_reference[i].x, install_test_reference[i].y);
    }

    error = install_test_error(1, &tf_base);
    printf("INSTALL current_tf_base rms=%.1f mm\r\n", error);
    // 检验用户提出的零偏移、+90度假设，同时保留-90度和0度作对照。
    for (int16_t yaw = -90; yaw <= 90; yaw += 90) {
        tf_transform_set(&hypothesis, 0.0f, 0.0f, (float)yaw);
        error = install_test_error(sign, &hypothesis);
        printf("INSTALL hypothesis sign=%d tx=0 ty=0 yaw=%d rms=%.1f %s\r\n",
               (int)sign, (int)yaw, error,
               error <= INSTALL_TEST_MAX_ERROR_MM ? "PASS" : "FAIL");
    }
    printf("结果是lidar->base安装参数，不是base->map的my_yaw换算；重复测试后再采用\r\n");
}

static void install_test_prompt(uint8_t stage) {
    static const char *names[3] = {"前方", "右侧", "左侧"};

    if (stage == 0) {
        printf("INSTALL 背景：移走挡板，人离开扫描平面，短按KEY1采100圈\r\n");
    } else {
        printf("INSTALL %s：挡板表面中心放在车体坐标(%.1f,%.1f)mm，短按KEY1采100圈\r\n",
               names[stage - 1], install_test_reference[stage - 1].x,
               install_test_reference[stage - 1].y);
    }
}

// 雷达安装标定任务，与wheel_mask_test一样轮询DMA、按圈号去重、分组统计。
void lidar_install_test(void) {
    static uint8_t test_started = 0, test_finished = 0;
    static uint8_t stage = 0, collecting = 0, skip_first_scan = 0;
    static uint16_t scan_count = 0, hit_count = 0;
    static uint32_t last_scan_num = 0, skip_count = 0, last_key_tick = 0;
    static uint32_t xor_ok_start = 0, xor_err_start = 0;
    static point2d_t sum = {0.0f, 0.0f};
    static float square_sum = 0.0f;
    const lidar_scan_t *scan;
    point2d_t point;
    uint8_t pressed;
    uint32_t tick = HAL_GetTick();

    lidar_process();  // 等待摆放挡板和结束后也持续接收，避免DMA堵住
    // 当前timer.c没有调用key_tick，测试在主循环每5ms调用一次，不改定时器。
    // 这个独立测试不要同时调用key_handle，否则短按事件可能先被其他任务取走。
    if (tick - last_key_tick >= 5U) {
        last_key_tick = tick;
        key_tick();
    }
    pressed = key_getshortpress();  // 采集过程中按下的键丢弃，不带到下一组
    if (test_finished) return;

    if (!test_started) {
        test_started = 1;
        memset(install_background, 0, sizeof(install_background));
        memset(install_test_measured, 0, sizeof(install_test_measured));
        printf("===== LIDAR INSTALL TEST START =====\r\n");
        printf("车体原点=自选车体中心，X右Y前；固定小车，全程只移动挡板\r\n");
        install_test_prompt(stage);
        return;
    }

    scan = lidar_get_scan();
    if (!collecting) {
        if (pressed != 1U) return;
        collecting = 1;
        scan_count = hit_count = 0;
        skip_count = 0;
        sum.x = sum.y = square_sum = 0.0f;
        last_scan_num = scan == NULL ? 0 : scan->scan_num;
        skip_first_scan = 1;  // 丢掉摆放过程中跨越的那一圈，下一圈才是完整静止圈
        xor_ok_start = lidar_get_xor_ok_cnt();
        xor_err_start = lidar_get_xor_err_cnt();
        printf("INSTALL collect start stage=%u\r\n", (unsigned int)stage);
        return;
    }

    if (scan == NULL || scan->scan_num == 0 || scan->scan_num == last_scan_num) return;
    if (last_scan_num != 0 && scan->scan_num > last_scan_num + 1)
        skip_count += scan->scan_num - last_scan_num - 1;
    last_scan_num = scan->scan_num;
    if (skip_first_scan) {
        skip_first_scan = 0;
        return;
    }

    if (stage == 0) {
        for (uint16_t angle = 0; angle < 360U; angle++) {
            install_background_t *background = &install_background[angle];
            uint16_t distance = scan->distance[angle];
            if (!scan->valid[angle] || distance == 0) continue;
            if (background->count == 0) {
                background->min_distance = background->max_distance = distance;
            } else {
                if (distance < background->min_distance) background->min_distance = distance;
                if (distance > background->max_distance) background->max_distance = distance;
            }
            background->count++;
        }
    } else if (install_test_get_point(scan, &point)) {
        sum.x += point.x;
        sum.y += point.y;
        square_sum += point.x * point.x + point.y * point.y;
        hit_count++;
    }
    scan_count++;
    if (scan_count % 20U == 0U)
        printf("INSTALL stage=%u collect=%u/%u hit=%u\r\n", (unsigned int)stage,
               (unsigned int)scan_count, (unsigned int)INSTALL_TEST_SCAN_COUNT,
               (unsigned int)hit_count);
    if (scan_count < INSTALL_TEST_SCAN_COUNT) return;

    collecting = 0;
    printf("INSTALL skip=%lu xor_ok=%lu xor_err=%lu\r\n", (unsigned long)skip_count,
           (unsigned long)(lidar_get_xor_ok_cnt() - xor_ok_start),
           (unsigned long)(lidar_get_xor_err_cnt() - xor_err_start));
    if (stage != 0) {
        float deviation;
        float angle;
        if (hit_count < INSTALL_TEST_MIN_HIT) {
            printf("INSTALL FAIL: 挡板命中不足，检查背景距离/扫描高度/人员遮挡，重采本组\r\n");
            install_test_prompt(stage);
            return;
        }
        point.x = sum.x / (float)hit_count;
        point.y = sum.y / (float)hit_count;
        deviation = sqrtf(fmaxf(0.0f, square_sum / (float)hit_count -
                                        point.x * point.x - point.y * point.y));
        angle = atan2f(-point.x, point.y) * 180.0f / TF_PI;
        if (angle < 0.0f) angle += 360.0f;
        printf("INSTALL POINT stage=%u raw_angle=%.2f raw_xy=(%.1f,%.1f) std=%.1fmm\r\n",
               (unsigned int)stage, angle, point.x, point.y, deviation);
        if (!isfinite(deviation) || deviation > INSTALL_TEST_MAX_STD_MM) {
            printf("INSTALL FAIL: 目标波动过大，请固定挡板后重采本组\r\n");
            install_test_prompt(stage);
            return;
        }
        install_test_measured[stage - 1] = point;
    }

    stage++;
    if (stage <= 3U) {
        install_test_prompt(stage);
    } else {
        install_test_print_result();
        test_finished = 1;
        printf("===== LIDAR INSTALL TEST END，复位可重新测量 =====\r\n");
    }
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

    // 每500ms打印一次 避免大量串口输出阻塞主循环
    if (now - last_cluster_print < 500)
        return;

    last_cluster_print = now;

    // 聚类摘要：update=聚类结果更新序号；cloud=本结果使用的点云更新序号
    // count=当前全部有效聚类数量，不一定等于真实物体数量，也不是固定跟踪目标数量
    // age=当前时刻减去聚类结果更新时间，表示这份结果已经过去多少ms
    printf("CLUSTER update=%lu cloud=%lu count=%u age=%lums\r\n",
           (unsigned long)obstacle_result.update_flag,
           (unsigned long)obstacle_result.cloud_flag,
           (unsigned int)obstacle_result.cluster_count,
           (unsigned long)(now - obstacle_result.update_tick));

    for (uint16_t i = 0; i < obstacle_result.cluster_count && i < 8; i++)
    {
        const lidar_cluster_t *cluster = &obstacle_result.clusters[i];  // 只读访问第i组聚类

        // C0、C1等是本次结果的数组下标 下一圈重新分组后，
        // angle是原始雷达角度范围 如307~57表示跨过359度/0度，不是角度顺序错误
        // center是聚类里全部有效点map坐标的平均值，不是物体真实几何中心
        // size是map坐标下包围框沿X、Y的尺寸 height不是物体竖直高度
        // 桌面测试的位姿和安装变换都为0时，map数值等同于局部坐标
        printf("C%u angle=%u~%u points=%u center=(%.1f,%.1f) "
               "size=(%.1f,%.1f) near=%.0f\r\n",
               (unsigned int)i,                          // C编号 从0开始 只表示当前数组顺序  不保证仍对应同一个物体
               (unsigned int)cluster->start_angle,        // 本组第一个点的原始雷达角度
               (unsigned int)cluster->end_angle,          // 本组最后一个点的原始雷达角度
               (unsigned int)cluster->point_count,        // 本组有效点数 按整数角度保留后的点数
               cluster->center_point.x,                  // 全部点的平均X坐标 mm
               cluster->center_point.y,                  // 全部点的平均Y坐标 mm
               cluster->width,                           // X方向跨度 max_x-min_x，单位mm
               cluster->height,                          // Y方向跨度 max_y-min_y，单位mm
               cluster->nearest_distance);               // 本组最近有效点到雷达的原始测距 mm
    }
}
