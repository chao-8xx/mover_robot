#ifndef GONGXUN_1_SYSTEM_H
#define GONGXUN_1_SYSTEM_H
#include "headfile.h"

// 整车系统初始化
void system_init(void);
// 整车系统控制
void system_control(void);

// 是否具备发车条件
bool run_is_ok(void);

// ============ 全局变量声明 ============ //
// 车辆启停、复位标志位
extern bool car_run;            // 发车总开关
extern bool car_reset;          // 车辆复位标志
// 发车位置
extern uint8_t start_location;  // 0：启停区2  1：启停区1

#endif //GONGXUN_1_SYSTEM_H
