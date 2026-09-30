#ifndef GONGXUN_1_CATCH_H
#define GONGXUN_1_CATCH_H

// 夹爪舵机角度定义
#define CLAW_CATCH  3.0f     // 夹爪夹取角度
#define CLAW_LOOSE  80.0f    // 夹爪松开角度

// 转盘舵机角度定义
#define ONE_LOCATION    10.0f   // 一号物料存放角度
#define TWO_LOCATION    130.0f  // 二号物料存放角度
#define THREE_LOCATION  250.0f  // 三号物料存放角度
#define SAFE_LOCATION   100.0f  // 安全位置，不超尺寸

// 云台舵机角度定义
#define HOLDER_PUT      244.0f   // 云台放置物料角度
#define HOLDER_CATCH    90.0f    // 云台抓取中间物料角度
#define HOLDER_RIGHT    50.0f    // 云台抓取右侧物料角度
#define HOLDER_LEFT     130.0f   // 云台抓取左侧物料角度

// 升降装置目标高度
#define LIFT_NORMAL     14.5f    // 默认高度
#define LIFT_PUT        13.0f    // 放置高度 放入转盘
#define LIFT_CATCH      7.0f     // 从原料区抓取高度
#define LIFT_PUT_ONE    (-0.5f)  // 一次放置 放在圆环
#define LIFT_PUT_TWO    5.0f     // 二次放置 叠在一次放置处

// 伸缩装置目标位置
#define STRETCH_NORMAL   0.0f   // 默认位置
#define STRETCH_CATCH    3.5f   // 抓取位置 从原料区
#define STRETCH_PUT      3.0f   // 放置位置 放入转盘
#define STRETCH_PUT_MID  4.0f   // 放置中心圆环位置 放入圆环
#define STRETCH_PUT_SIDE 5.2f   // 放置中心圆环位置 放入圆环

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

// 抓取装置位置复位
void catch_init(void);

// 抓取装置夹爪伸缩 正：伸出 负：缩回
void catch_stretch(int16_t speed);

// 抓取装置夹爪伸缩一定距离 齿轮齿条
// 控制编码器绝对位置
void catch_stretch_ang(float angle);
// 抓取装置夹爪伸缩一定距离 单位：cm
// 正：伸出 负：缩回  电机顺时针伸出（pulse为负）
void catch_stretch_dist(float delta_cm);
// 抓取装置到达固定距离
void catch_stretch_to_pos(float dist);

// 抓取装置夹爪升降 单位：cm
void catch_lift(float delta_cm);
// 抓取装置夹爪升降至固定高度
void catch_lift_to_height(float height);

// 夹爪舵机控制角度
void claw_ctrl(float angle);
// 云台旋转角度
void holder_rotate(float angle);
// 转盘旋转角度
void turntable_rotate(float angle);

// 启动一次非阻塞等待（舵机等无反馈动作计时，单位 ms）
void servo_wait_ms(uint16_t ms);

// 舵机动作等待是否还在进行
uint8_t servo_is_busy(void);

// 升降是否还在执行
uint8_t lift_is_busy(void);

// 伸缩是否还在执行
uint8_t stretch_is_busy(void);

// 1ms 周期调用：升降非阻塞计时，到时间后清除 busy
void catch_tick(void);

#endif //GONGXUN_1_CATCH_H
