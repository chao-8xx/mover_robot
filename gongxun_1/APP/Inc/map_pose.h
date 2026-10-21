#ifndef GONGXUN_1_MAP_POSE_H
#define GONGXUN_1_MAP_POSE_H

// 位姿参数  超时时间待定位模块接入后按实际周期调整
#define MAP_POSE_TIMEOUT_MS          500       // 位姿超时时间 ms
#define MAP_POSE_DEFAULT_YAW_OFFSET  (-90.0f)  // 地图位姿 yaw 默认偏移量  my_yaw=0朝东  地图变换yaw=0朝北

// 地图位姿
typedef struct
{
    float x_mm;         // 车体原点在map_track地图中的实际X坐标 mm
    float y_mm;         // 车体原点在map_track地图中的实际Y坐标 mm
    float yaw_deg;      // 地图变换角 北0、东-90、西90、南-180  范围[-180,180)    上北下南 左西右东
    uint32_t tick;      // 本次定位测量时间 与 HAL_GetTick 使用相同时间基准
    uint8_t valid;      // 位姿是否有效
} map_pose_t;

///////////////////////////////////// 地图位姿接口 ////////////////////////////////////////
// 以下接口在主循环调用 不在中断中同时读写
// 位姿初始化  开机默认无有效位姿
void map_pose_init(void);

// 设置角度偏移  地图变换角 = imu_yaw_deg + offset_deg
// 修改偏移后旧位姿失效  需要重新更新
uint8_t map_pose_yaw_offset_set(float offset_deg);
// 读取当前角度偏移  供导航把地图方向转换为现有IMU目标角
float map_pose_yaw_offset_get(void);

// 更新位姿  X/Y传实际定位结果  imu_yaw_deg传现有my_yaw
// valid要求定位和角度均有效  数据无效或坐标超出场地时清除旧位姿
// tick传本次测量时间  不能用旧X/Y反复刷新时间  不根据运动指令推算X/Y
uint8_t map_pose_update(float x_mm, float y_mm, float imu_yaw_deg,
                        uint32_t tick, uint8_t valid);

// 获取位姿  now传当前HAL_GetTick
// 无效、超时或时间异常返回0  并清空输出
uint8_t map_pose_get(uint32_t now, map_pose_t *pose);

#endif //GONGXUN_1_MAP_POSE_H
