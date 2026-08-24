#ifndef GONGXUN_1_CATCH_H
#define GONGXUN_1_CATCH_H
#include "headfile.h"

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

// 抓取装置当前模式：夹住/松开
typedef enum {
    Free,         // 空闲：等待Maix抓取指令
    Search,       // 搜寻：获取物料坐标，等待抓取
    Calibration,  // 校准：根据物料校准车身姿态和位置
    Catch,        // 抓取：从开始抓取到抓着物料上升
    Put           // 放置：旋转到转盘并放置到转盘处
} Catch_Mode;

extern Catch_Mode catch_mode;   // 抓取机构状态机（其它文件也能读写）
extern Claw_State claw_state;   // 夹爪当前状态
extern Turntable_Pos turn_pos;  // 转盘当前位置

// 抓取装置位置复位
void catch_init(void);

// 抓取装置夹爪伸缩 正：伸出 负：缩回
void catch_stretch(int16_t speed);

// 抓取装置夹爪升降 单位：cm
void catch_lift(float delta_cm);

// 云台旋转角度
void holder_rotate(float angle);

// 转盘旋转角度
void turntable_rotate(float angle);

// 夹爪舵机控制角度
void claw_ctrl(float angle);

// 启动一次非阻塞等待（舵机等无反馈动作计时，单位 ms）
void catch_wait_servo(uint16_t ms);

// 舵机动作等待是否还在进行
uint8_t catch_servo_waiting(void);

// 升降是否还在执行
uint8_t catch_is_busy(void);

// 1ms 周期调用：升降非阻塞计时，到时间后清除 busy
void catch_tick(void);

#endif //GONGXUN_1_CATCH_H
