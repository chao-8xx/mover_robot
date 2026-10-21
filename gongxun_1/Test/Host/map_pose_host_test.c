#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "map_pose.h"

static unsigned checks;
static unsigned failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL line %d: %s\n", __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static int near(float a, float b)
{
    return fabsf(a - b) < 0.001f;
}

int main(void)
{
    map_pose_t pose;
    const float imu_angles[] = {0.0f, 90.0f, 180.0f, -90.0f, 720.0f};
    const float map_angles[] = {-90.0f, 0.0f, 90.0f, -180.0f, -90.0f};

    map_pose_init();
    CHECK(!map_pose_get(0, &pose));
    CHECK(pose.valid == 0);
    CHECK(!map_pose_get(0, NULL));

    // 对应现有控制：my_yaw=0向东；地图变换yaw=0向北。
    for (unsigned i = 0; i < sizeof(imu_angles) / sizeof(imu_angles[0]); i++)
    {
        CHECK(map_pose_update(1200.0f, 2125.0f, imu_angles[i], 100, 1));
        CHECK(map_pose_get(100, &pose));
        CHECK(near(pose.x_mm, 1200.0f) && near(pose.y_mm, 2125.0f));
        CHECK(near(pose.yaw_deg, map_angles[i]));
        CHECK(pose.tick == 100 && pose.valid == 1);
    }

    // 发布一次位姿不能永久有效，也不能接受未来时间。
    CHECK(map_pose_get(100 + MAP_POSE_TIMEOUT_MS, &pose));
    CHECK(!map_pose_get(101 + MAP_POSE_TIMEOUT_MS, &pose));
    CHECK(pose.valid == 0 && pose.x_mm == 0.0f);
    CHECK(!map_pose_get(99, &pose));
    CHECK(map_pose_update(0.0f, 0.0f, 0.0f, UINT32_MAX - 20U, 1));
    CHECK(map_pose_get(30, &pose));
    CHECK(!map_pose_get(MAP_POSE_TIMEOUT_MS, &pose));

    // 坏数据必须立即撤销旧位姿，不能沿用上次成功结果。
    CHECK(!map_pose_update(NAN, 100.0f, 0.0f, 200, 1));
    CHECK(!map_pose_get(200, &pose));
    CHECK(!map_pose_update(100.0f, INFINITY, 0.0f, 200, 1));
    CHECK(!map_pose_update(100.0f, 100.0f, NAN, 200, 1));
    CHECK(!map_pose_update(-1.0f, 100.0f, 0.0f, 200, 1));
    CHECK(!map_pose_update(2400.0f, 100.0f, 0.0f, 200, 1));
    CHECK(!map_pose_update(100.0f, 2400.0f, 0.0f, 200, 1));
    CHECK(map_pose_update(100.0f, 100.0f, 0.0f, 200, 1));
    CHECK(!map_pose_update(100.0f, 100.0f, 0.0f, 201, 0));
    CHECK(!map_pose_get(201, &pose));

    // 启动方向改变时集中修改角度偏移；旧快照随之失效。
    CHECK(map_pose_update(100.0f, 100.0f, 0.0f, 300, 1));
    CHECK(map_pose_yaw_offset_set(0.0f));
    CHECK(!map_pose_get(300, &pose));
    CHECK(map_pose_update(100.0f, 100.0f, 450.0f, 301, 1));
    CHECK(map_pose_get(301, &pose) && near(pose.yaw_deg, 90.0f));
    CHECK(!map_pose_yaw_offset_set(INFINITY));
    CHECK(!map_pose_get(301, &pose));
    map_pose_init();
    CHECK(map_pose_update(100.0f, 100.0f, 0.0f, 400, 1));
    CHECK(map_pose_get(400, &pose) && near(pose.yaw_deg, -90.0f));

    printf("map pose: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
