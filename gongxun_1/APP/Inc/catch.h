#ifndef GONGXUN_1_CATCH_H
#define GONGXUN_1_CATCH_H

// 伸缩运行的最大距离
#define STRETCH_MAX_CM   16.0f    // 伸缩最大允许伸出距离

// 夹爪舵机角度定义
#define CLAW_CATCH  19.0f     // 夹爪夹取角度
#define CLAW_LOOSE  90.0f    // 夹爪松开角度

// 转盘舵机角度定义
#define ONE_LOCATION    10.0f   // 一号物料存放角度
#define TWO_LOCATION    130.0f  // 二号物料存放角度
#define THREE_LOCATION  250.0f  // 三号物料存放角度
#define SAFE_LOCATION   86.0f   // 安全位置，不超尺寸

// 云台舵机角度定义
#define HOLDER_SAFE     245.0f   // 云台安全角度
#define HOLDER_PUT      248.0f   // 云台放置物料角度
#define HOLDER_CATCH    98.0f    // 云台抓取中间物料角度
#define HOLDER_RIGHT    58.0f    // 云台抓取右侧物料角度
#define HOLDER_LEFT     142.0f   // 云台抓取左侧物料角度

// 升降装置目标高度
#define LIFT_ZERO_CM    17.0f   // 编码器清零时对应的物理高度，固定不动
#define LIFT_NORMAL     16.7f   // 默认高度
#define LIFT_PUT        14.3f   // 放置高度 放入转盘
#define LIFT_CATCH      9.9f    // 从原料区抓取高度
#define LIFT_PUT_ONE    (1.2f)  // 一次放置 放在圆环
#define LIFT_PUT_TWO    6.0f    // 二次放置 叠在一次放置处

// 伸缩装置目标位置
#define STRETCH_NORMAL    0.0f   // 默认位置
#define STRETCH_CATCH     12.5f  // 抓取位置 从原料区
#define STRETCH_PUT       0.6f   // 放置位置 放入转盘
#define STRETCH_PUT_MID   4.5f   // 放置中心圆环位置
#define STRETCH_PUT_LEFT  9.8f   // 放置左边圆环位置
#define STRETCH_PUT_RIGHT 10.2f  // 放置右边圆环位置
#define STRETCH_SAFE      8.0f   // 安全长度

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

// ======== 初始化 ======== //
// 抓取装置位置复位
void catch_init(void);

// ======== 齿轮齿条 ======== //
// 抓取装置夹爪伸缩 正：伸出 负：缩回
void catch_stretch(int16_t speed);
// 抓取装置夹爪伸缩停止运动
void catch_stretch_stop(void);
// 抓取装置夹爪伸缩状态赋值
void catch_stretch_send(int16_t speed);
// 抓取装置到达固定距离 绝对 齿轮齿条
void catch_stretch_distance(float distance);
// 获取当前伸缩距离 编码器值转化
uint8_t catch_stretch_get_dist(float *dist_cm);
// 获取伸缩电机编码器
void get_stretch_distance(void);

// ======== 伸缩丝杆 ======== //
// 抓取装置夹爪伸缩一定距离 单位：cm
// 正：伸出 负：缩回  电机顺时针伸出（pulse为负）
void catch_stretch_dist(float delta_cm);
// 抓取装置到达固定距离 相对 丝杆
void catch_stretch_to_pos(float dist);

// ======== 升降丝杆 ======== //
// 抓取装置夹爪升降 单位：cm
void catch_lift(float delta_cm);
// 抓取装置夹爪升降至固定高度
void catch_lift_to_height(float height);
// 获取升降电机编码器
void get_lift_distance(void);

// ======== 舵机 ======== //
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

// ======== 非阻塞计时 ======== //
// 1ms 周期调用：升降非阻塞计时，到时间后清除 busy
void catch_tick(void);

#endif //GONGXUN_1_CATCH_H
