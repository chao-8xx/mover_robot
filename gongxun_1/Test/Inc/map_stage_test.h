#ifndef GONGXUN_1_MAP_STAGE_TEST_H
#define GONGXUN_1_MAP_STAGE_TEST_H

#include <stdint.h>

// 手动从 main 调用，串口打印地图各阶段的检查结果
uint8_t map_stage_test(void);

// 真实雷达桌面联调 主循环持续调用，使用固定测试位姿，只观察R8中间的一小块区域
void map_lidar_test(void);

// 真实位姿静止建图 主循环持续调用，短按KEY1确认右下启停区2实际起点，不触发发车
void map_pose_test(void);

#endif // GONGXUN_1_MAP_STAGE_TEST_H
