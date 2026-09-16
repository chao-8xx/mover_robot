#include "headfile.h"

// TIM1 配置：168MHz / (167+1) = 1MHz 计数时钟，1 个计数 = 1us
// ARR = 19999 -> 周期 20ms，即 50Hz，标准舵机控制信号
#define SERVO_PERIOD_US     20000
#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_MID_PULSE_US  1500
#define SERVO_MIN_ANGLE     0.0f
#define SERVO_MAX_ANGLE     270.0f

// 舵机编号对应的 TIM1 通道
static const uint32_t servo_channel[SERVO_COUNT] = {
    TIM_CHANNEL_1,   // Servo_1 云台舵机 PE9
    TIM_CHANNEL_2,   // Servo_2 转盘舵机 PE11
    TIM_CHANNEL_3    // Servo_3 夹爪舵机 PE13
};

// 舵机初始化：CubeMX 已配置 PWM Generation CH1/2/3，
// 这里只需设置初始中位脉宽并启动 PWM 输出
void servo_init(void)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        __HAL_TIM_SET_COMPARE(&htim1, servo_channel[i], SERVO_MID_PULSE_US);
        HAL_TIM_PWM_Start(&htim1, servo_channel[i]);
    }
}

// ================ 180°/270° 位置舵机：脉宽控制角度 ================

// 直接设置 PWM 脉宽（us），自动限幅到 500 ~ 2500
void servo_set_pulse(Servo_Num servo_num, uint16_t pulse_us)
{
    if ((int)servo_num < 0 || (int)servo_num >= SERVO_COUNT) {
        return;
    }

    if (pulse_us < SERVO_MIN_PULSE_US) {
        pulse_us = SERVO_MIN_PULSE_US;
    }
    else if (pulse_us > SERVO_MAX_PULSE_US) {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    __HAL_TIM_SET_COMPARE(&htim1, servo_channel[servo_num], pulse_us);
}

// 设置舵机角度：0° -> 500us，360° -> 2500us
void servo_set_angle(Servo_Num servo_num, float angle)
{
    uint16_t pulse_us;

    if (angle < SERVO_MIN_ANGLE) {
        angle = SERVO_MIN_ANGLE;
    }
    else if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE;
    }

    // 线性映射角度到脉宽
    pulse_us = (uint16_t)(SERVO_MIN_PULSE_US +
                          (angle / SERVO_MAX_ANGLE) *
                          (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));

    servo_set_pulse(servo_num, pulse_us);
}

// ================ 360° 连续旋转舵机：相对角度转动（开环计时） ================
#define SERVO_MAX_SPEED        100       // 舵机最大速度
#define SERVO_MAX_DEG_PER_SEC  380.0f    // speed=100 时的角速度，单位 °/s，需标定！

// 单个转动任务状态（volatile：会在定时器中断里被读写）
static volatile struct {
    uint8_t   busy;        // 1 = 正在转动
    Servo_Num num;         // 哪个舵机
    int16_t   dir;         // +1 / -1
    uint32_t  deadline;    // 停止时刻（HAL_GetTick 单位 ms）
} s_servo_rot = {0};

// 连续旋转舵机速度控制：
// 1500us = 停止，1000us / 2000us = 两个方向的最大转速
// 普通位置舵机调用这个接口只会停到对应脉宽的位置，不会持续旋转
void servo_set_speed(Servo_Num servo_num, int16_t speed)
{
    uint16_t pulse_us;

    if ((int)servo_num < 0 || (int)servo_num >= SERVO_COUNT) {
        return;
    }

    if (speed < -SERVO_MAX_SPEED) {
        speed = -SERVO_MAX_SPEED;
    }
    else if (speed > SERVO_MAX_SPEED) {
        speed = SERVO_MAX_SPEED;
    }

    // ±100 对应 ±500us，落在 1000us ~ 2000us 区间
    pulse_us = (uint16_t)(SERVO_MID_PULSE_US + speed * 5);
    servo_set_pulse(servo_num, pulse_us);
}

// 启动一次相对角度转动：deg > 0 正转(逆)，deg < 0 反转(顺)
// speed：1 ~ 100，越大越快（速度写死，固定传一个值即可）
void servo_rotate_deg(Servo_Num servo_num, float deg, uint8_t speed)
{
    if ((int)servo_num < 0 || (int)servo_num >= SERVO_COUNT) {
        return;
    }
    if (s_servo_rot.busy) {                 // 上一次还没转完，忽略新指令
        return;
    }

    if (speed == 0) speed = SERVO_MAX_SPEED;
    if (speed > SERVO_MAX_SPEED) speed = SERVO_MAX_SPEED;

    int16_t dir = (deg >= 0.0f) ? 1 : -1;
    float   d   = (deg >= 0.0f) ? deg : -deg;

    // 实际角速度(°/ms) = (speed/100) * SERVO_MAX_DEG_PER_SEC / 1000
    float deg_per_ms = ((float)speed / 100.0f) * SERVO_MAX_DEG_PER_SEC / 1000.0f;
    uint32_t ms = (uint32_t)(d / deg_per_ms);

    servo_set_speed(servo_num, dir * (int16_t)speed);

    // 先写完整状态，最后再置 busy=1 作为「就绪」标志，避免中断读到半初始化的数据
    s_servo_rot.num      = servo_num;
    s_servo_rot.dir      = dir;
    s_servo_rot.deadline = HAL_GetTick() + ms;
    s_servo_rot.busy     = 1;
}

// 在主循环（或定时器中断）里周期调用，转动时间到就自动停（输出 1500us 中位）
void servo_rotate_tick(void)
{
    if (!s_servo_rot.busy) {
        return;
    }
    if ((int32_t)(HAL_GetTick() - s_servo_rot.deadline) >= 0) {
        servo_set_speed(s_servo_rot.num, 0);
        s_servo_rot.busy = 0;
    }
}
