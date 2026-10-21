//
// Created by mechreuo on 2026/8/28.
//

#ifndef GONGXUN_1_TF2D_H
#define GONGXUN_1_TF2D_H

#define TF_PI       3.14159265358979323846f

// 单个点的结构体组成
// 最基础的单个点 只存x与y坐标
typedef struct {
    float x;      // x坐标
    float y;      // y坐标
} point2d_t;

// 坐标系变换结构体
typedef struct tf_transform2d
{
    float tx;      // x坐标
    float ty;      // y坐标

    float yaw_deg; // 角度

    float sin_yaw; // 角度的正弦值
    float cos_yaw; // 角度的余弦值
} tf_transform2d_t;

extern tf_transform2d_t tf_lidar; // 雷达坐标系
extern tf_transform2d_t tf_base;  // 车身坐标系
extern tf_transform2d_t tf_map;   // 地图坐标系

void tf_init(void);
void angle_table_init(void);
point2d_t tf_lidar_polar_to_xy(uint16_t angle, uint16_t distance);
void tf_transform_set(tf_transform2d_t *tf, float x, float ty, float yaw_deg);
point2d_t tf_transform_point(point2d_t point,const tf_transform2d_t *tf);


#endif //GONGXUN_1_TF2D_H
