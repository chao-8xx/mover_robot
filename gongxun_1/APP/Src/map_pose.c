#include "headfile.h"

// 地图位姿变量
static map_pose_t map_pose;                                 // 定位模块更新的实际位姿
static float map_yaw_offset = MAP_POSE_DEFAULT_YAW_OFFSET;  // IMU角度到地图变换角的偏移

///////////////////////////////////// 角度换算 ////////////////////////////////////////
// my_yaw允许累计多圈  地图变换角统一到[-180,180)
static float map_pose_angle_wrap(float angle) {

    // fmodf 负责浮点数的取余
    angle = fmodf(angle, 360.0f);

    // 归一化
    if (angle >= 180.0f)
        angle -= 360.0f;
    if (angle < -180.0f)
        angle += 360.0f;

    return angle;
}

///////////////////////////////////// 地图位姿接口 ////////////////////////////////////////
// 位姿初始化
void map_pose_init(void) {

    // 清零旧位姿  避免本次数据无效时继续使用上次结果
    memset(&map_pose, 0, sizeof(map_pose));
    map_yaw_offset = MAP_POSE_DEFAULT_YAW_OFFSET;
}

// 设置角度偏移  清除旧位姿
uint8_t map_pose_yaw_offset_set(float offset_deg) {

    memset(&map_pose, 0, sizeof(map_pose));
    if (!isfinite(offset_deg))
        return 0;

    map_yaw_offset = map_pose_angle_wrap(offset_deg);
    return 1;
}

// 读取当前角度偏移
float map_pose_yaw_offset_get(void) {
    return map_yaw_offset;
}

// 更新实际位姿
uint8_t map_pose_update(float x_mm, float y_mm, float imu_yaw_deg, uint32_t tick, uint8_t valid) {
    // 清除旧位姿
    memset(&map_pose, 0, sizeof(map_pose));

    // 检查输入参数的有效性  位置和角度必须是有限数值 且位置在地图范围内
    if (!valid || !isfinite(x_mm) || !isfinite(y_mm) || !isfinite(imu_yaw_deg) ||
        x_mm < 0.0f || x_mm >= MAP_WIDTH || y_mm < 0.0f || y_mm >= MAP_HEIGHT)
        return 0;

    // 保存定位结果  并换算为地图变换角
    map_pose.x_mm = x_mm;
    map_pose.y_mm = y_mm;
    map_pose.yaw_deg = map_pose_angle_wrap(map_pose_angle_wrap(imu_yaw_deg) + map_yaw_offset);
    map_pose.tick = tick;
    map_pose.valid = 1;
    return 1;
}

// 获取有效位姿
uint8_t map_pose_get(uint32_t now, map_pose_t *pose) {
    if (pose == NULL)
        return 0;

    memset(pose, 0, sizeof(*pose));

    // 无符号差值处理毫秒计数回绕  同时拒绝超时或未来时间
    if (!map_pose.valid || (uint32_t)(now - map_pose.tick) > MAP_POSE_TIMEOUT_MS)
        return 0;

    *pose = map_pose;
    return 1;
}
