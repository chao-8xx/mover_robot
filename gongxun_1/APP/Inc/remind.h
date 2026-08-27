#ifndef GONGXUN_1_REMIND_H
#define GONGXUN_1_REMIND_H
#include "headfile.h"

typedef enum {
    RED_REMIND,     // 闪红光提醒
    GREEN_REMIND,   // 闪绿光提醒
    BLUE_REMIND,    // 闪蓝光提醒
    YELLOW_REMIND,  // 闪黄光提醒
    PINK_REMIND,    // 闪红光提醒
    WHITE_REMIND,   // 闪白光提醒
    NO_REMIND       // 无提醒
} Remind_State;

extern Remind_State remindState;   // 提示装置状态

// 提示装置初始化  包含ws2812、板载LED、板载蜂鸣器
void remind_init(void);
// 提示装置控制（非阻塞）
void remind_control(void);

#endif //GONGXUN_1_REMIND_H
