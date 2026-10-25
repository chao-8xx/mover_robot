//
// Created by mechreuo on 2026/8/28.
//

#include "headfile.h"

// 2D坐标系说明
//             +Y
//              ↑ 前 0度
//              |
//       -X ←   +   → +X 270度
//              |
//              ↓ 后
//


static float sin_table[360];  // 正弦查找表
static float cos_table[360];  // 余弦查找表

tf_transform2d_t tf_lidar; // 雷达坐标系
tf_transform2d_t tf_base;  // 车身坐标系
tf_transform2d_t tf_map;   // 地图坐标系


// 坐标变换初始化
void tf_init(void) {
    angle_table_init();
    // tf_base_init(&tf_base, 3.5f, -6.0f, 1.2f);  // 测试
    tf_transform_set(&tf_base, 0.0f, 0.0f, 0.0f);  // 初始化车身坐标系
    // tf_transform_set(&tf_map, cloud.car_x, cloud.car_y, cloud.car_angle);   // 初始化地图坐标系  需要在地图坐标系实时更新时调用
}

// 初始化正余弦查找表
void angle_table_init(void) {

    // 开机计算一次 后续使用直接查表 避免频繁调用 sinf/cosf 函数 提高效率
    for (uint16_t angle = 0; angle < 360; angle++) {
        float rad = angle * TF_PI / 180.0f;  // 将角度转换为弧度

        sin_table[angle] = sinf(rad);
        cos_table[angle] = cosf(rad);
    }
}

// 将极坐标转换为雷达坐标
point2d_t tf_lidar_polar_to_xy(uint16_t angle, uint16_t distance) {
    point2d_t point;

    // 角度范围检查，确保在0~359度之间
    angle = angle % 360;

    // 使用查找表计算坐标
    point.x = -(float)distance * sin_table[angle];
    point.y = (float)distance * cos_table[angle];

    return point;
}

// 设置 前坐标系 -> 后坐标系 的固定坐标变换参数
// 注: x,y为前坐标系原点在后坐标系中的位置 即为前坐标系在后坐标系中的安装偏移
//      yaw_deg为前坐标系相对于后坐标系的偏航角 即为前坐标系相对于后坐标系的安装角
void tf_transform_set(tf_transform2d_t *tf, float tx, float ty, float yaw_deg) {
    // 将角度转换为弧度
    float rad = yaw_deg * TF_PI / 180.0f;

    // 存储变换参数
    tf->tx = tx;
    tf->ty = ty;
    tf->yaw_deg = yaw_deg;

    tf->sin_yaw = sinf(rad);
    tf->cos_yaw = cosf(rad);
}

// 变换一个点  齐次矩阵展开式 节省运算量
point2d_t tf_transform_point(point2d_t point,const tf_transform2d_t *tf) {
    point2d_t out;

    out.x = tf->cos_yaw * point.x - tf->sin_yaw * point.y + tf->tx;
    out.y = tf->sin_yaw * point.x + tf->cos_yaw *point.y + tf->ty;

    return out;
}


// TF_Transform2D tf2d_create(float x, float y, float theta) {
//     TF_Transform2D transform;
//     transform.x = x;
//     transform.y = y;
//     transform.theta = theta;
//     return transform;
// }