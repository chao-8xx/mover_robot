#ifndef GONGXUN_1_CHASSIS_TEST_H
#define GONGXUN_1_CHASSIS_TEST_H

// 底盘绕场一周测试
void chassis_run_test(void);
// 用BFS算法让车子到达目标坐标处
void bfs_track_test(void);
// BFS 多点测试：左下角 -> (0,0) -> (4,2)朝南 -> (2,4)朝西 -> (0,2)朝北 -> (0,0) -> 左下角
// 10ms 中断直接调用；障碍由 HMI 的 BB 指令设置，路径自动避开
void bfs_multi_track_test(void);

#endif //GONGXUN_1_CHASSIS_TEST_H