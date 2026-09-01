//
// Created by mechreuo on 2026/6/4.
//

#ifndef GONGXUN_1_TEST_H
#define GONGXUN_1_TEST_H

#include "headfile.h"

void cdc_test(void);
void openmv_qr_test(void);
void maix_blob_test(void);
void lidar_test(void);
void hwt_test(void);

// 底盘绕场一周测试
void chassis_run_test(void);
// 提示装置测试代码 5ms定时中断
void remind_test(void);
// bfs路径规划测试（无实物，仅串口打印路径）
void bfs_test(void);
// 用BFS算法让车子到达目标坐标处
void bfs_track_test(void);
// BFS 多点测试：左下角 -> (0,0) -> (4,2)朝南 -> (2,4)朝西 -> (0,2)朝北 -> (0,0) -> 左下角
// 10ms 中断直接调用；障碍由 HMI 的 BB 指令设置，路径自动避开
void bfs_multi_track_test(void);

#endif //GONGXUN_1_TEST_H
