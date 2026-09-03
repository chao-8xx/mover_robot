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

    point2d_t p_base = {0.0f, 1000.0f};
    point2d_t p_map;
    tf_transform2d_t tf_map_base;

    tf_transform_set(&tf_map_base, car_x, car_y, my_yaw);
    p_map = tf_transform_point(p_base, &tf_map_base);

    if (HAL_GetTick() - last_print >= 1000) {
        last_print = HAL_GetTick();
        printf("TF alive\r\n");
        printf("车身姿态: (%.1f, %.1f), 角度: %.1f \r\n", car_x, car_y, my_yaw);
        printf("p_map = (%.1f, %.1f)\r\n", p_map.x, p_map.y);
    }
}

