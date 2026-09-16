#ifndef GONGXUN_1_SYSTEM_H
#define GONGXUN_1_SYSTEM_H

// 整车系统初始化
void system_init(void);
// 整车系统控制
void system_control(void);

// ============ 全局变量声明 ============ //
// 车辆启停、复位标志位
extern bool car_run;        // 发车总开关
extern bool car_reset;      // 车辆复位标志
// 位置状态
extern float stretch_pos;   // 伸缩装置当前伸缩位置cm

#endif //GONGXUN_1_SYSTEM_H
