#ifndef GONGXUN_1_SYSTEM_H
#define GONGXUN_1_SYSTEM_H
#include "headfile.h"

// 整车系统初始化
void system_init(void);
// 整车系统控制
void system_control(void);

// ============ 全局变量声明 ============ //
extern bool car_run;   // 发车总开关
extern bool car_reset; // 车辆复位标志

#endif //GONGXUN_1_SYSTEM_H
