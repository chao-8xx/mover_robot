#ifndef GONGXUN_1_CHASSIS_TEST_H
#define GONGXUN_1_CHASSIS_TEST_H

// 底盘绕场一周测试
void chassis_run_test(void);

// 用BFS算法让车子到达目标坐标处
void bfs_track_test(void);
// BFS 多点测试：左下角 -> (0,0) -> (4,2)朝南 -> (2,4)朝西 -> (0,2)朝北 -> (0,0) -> 左下角
// 10ms 中断直接调用；障碍由 HMI 的 BB 指令设置，路径自动避开
void bfs_multi_track_test(void);

// 小车麦轮跟踪锁定物料测试
void car_follow_material_test(void);
// 小车麦轮和伸缩跟踪锁定物料测试
void car_stretch_follow_material_test(void);

// 抓取颜色指令 主循环调用
void catch_color_send(void);
// 物料识别跟踪并抓取测试
void car_follow_catch_material_test(void);
// 只识别并抓取三个物料放到车上，然后结束
void car_catch_three_material_test(void);

// 放置圆环指令 主循环调用
void put_circle_send(void);
// 物料放置测试
void car_follow_put_material_test(void);

#endif //GONGXUN_1_CHASSIS_TEST_H
