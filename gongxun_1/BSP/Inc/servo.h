#ifndef GONGXUN_1_SERVO_H
#define GONGXUN_1_SERVO_H

#define SERVO_COUNT 3

// ---- 舵机编号与引脚/定时器通道映射 ---- //
typedef enum {
    Servo_1 = 0,    // 1号：云台舵机，PE9  -> TIM1_CH1
    Servo_2 = 1,    // 2号：转盘舵机，PE11 -> TIM1_CH2
    Servo_3 = 2     // 3号：夹爪舵机，PE13 -> TIM1_CH3
} Servo_Num;

// ---- 函数声明 ---- //
// // 180°位置舵机 // //
// 舵机初始化：把 TIM1 三个通道配置为 50Hz PWM 并启动
void servo_init(void);

// 直接设置 PWM 脉宽，pulse_us 范围 500 ~ 2500
void servo_set_pulse(Servo_Num servo_num, uint16_t pulse_us);

// 设置舵机角度，angle 范围 0.0 ~ 360.0（0° 对应 500us，360° 对应 2500us）
void servo_set_angle(Servo_Num servo_num, float angle);

// // 360°连续旋转舵机 // //

// 连续旋转舵机速度控制，speed 范围 -100 ~ +100
// 0 表示停止，负值一个方向转，正值另一个方向转，绝对值越大转速越快
// 注意：普通位置舵机（如 MG996R）没有这个功能，调用后只会停到对应脉宽位置
void servo_set_speed(Servo_Num servo_num, int16_t speed);

// 360° 连续旋转舵机相对角度转动（非阻塞，开环计时）
// deg > 0 正转，deg < 0 反转；speed 1 ~ 100 决定转速（速度可写死）
// 注意：360° 舵机无位置反馈，只能转「相对角度」，不能定位「绝对角度」，且会漂移
void servo_rotate_deg(Servo_Num servo_num, float deg, uint8_t speed);

// 主循环里周期调用，转动时间到自动停止（配合 servo_rotate_deg 使用）
void servo_rotate_tick(void);

#endif //GONGXUN_1_SERVO_H
