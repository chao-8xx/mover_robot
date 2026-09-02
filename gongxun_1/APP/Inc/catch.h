#ifndef GONGXUN_1_CATCH_H
#define GONGXUN_1_CATCH_H

// 夹爪当前模式：夹住/松开
typedef enum {
    CATCH,  // 夹住
    LOOSE   // 松开
} Claw_State;

// 转盘当前模式：一号位/二号位/三号位
typedef enum {
    LOCATION_1,  // 一号位
    LOCATION_2,  // 二号位
    LOCATION_3   // 三号位
} Turntable_Pos;

// 升降装置当前高度
typedef enum {
    PUT_HEIGHT,     // 放置高度
    PICK_HEIGHT,    // 升起高度（默认高度）
    CATCH_HEIGHT    // 抓取高度
} Lift_Height;

// 伸缩装置当前伸出距离
typedef enum {
    PUT_DIST,       // 放置距离
    SEARCH_DIST,    // 原料区搜索距离
    ORIGIN_DIST     // 原点距离
} Stretch_Dist;

#include "headfile.h"

// 抓取装置位置复位
void catch_init(void);

// 抓取装置夹爪伸缩 正：伸出 负：缩回
void catch_stretch(int16_t speed);

// 抓取装置夹爪伸缩一定距离 单位：cm
// 正：伸出 负：缩回  电机顺时针伸出（pulse为负）
void catch_stretch_pos(float delta_cm);

// 抓取装置夹爪升降 单位：cm
void catch_lift(float delta_cm);

// 夹爪舵机控制角度
void claw_ctrl(float angle);

// 云台旋转角度
void holder_rotate(float angle);

// 转盘旋转角度
void turntable_rotate(float angle);

// 启动一次非阻塞等待（舵机等无反馈动作计时，单位 ms）
void claw_wait_ms(uint16_t ms);

// 舵机动作等待是否还在进行
uint8_t claw_is_busy(void);

// 升降是否还在执行
uint8_t lift_is_busy(void);

// 1ms 周期调用：升降非阻塞计时，到时间后清除 busy
void catch_tick(void);

#endif //GONGXUN_1_CATCH_H
