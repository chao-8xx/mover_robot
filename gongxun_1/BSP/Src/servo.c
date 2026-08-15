#include "servo.h"

// TIM1 配置：168MHz / (167+1) = 1MHz 计数时钟，1 个计数 = 1us
// ARR = 19999 -> 周期 20ms，即 50Hz，标准舵机控制信号
#define SERVO_PERIOD_US     20000
#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_MID_PULSE_US  1500
#define SERVO_MIN_ANGLE     0.0f
#define SERVO_MAX_ANGLE     360.0f

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

// 连续旋转舵机速度控制：
// 1500us = 停止，1000us / 2000us = 两个方向的最大转速
// 普通位置舵机调用这个接口只会停到对应脉宽的位置，不会持续旋转
void servo_set_speed(Servo_Num servo_num, int16_t speed)
{
    uint16_t pulse_us;

    if ((int)servo_num < 0 || (int)servo_num >= SERVO_COUNT) {
        return;
    }

    if (speed < -100) {
        speed = -100;
    }
    else if (speed > 100) {
        speed = 100;
    }

    // ±100 对应 ±500us，落在 1000us ~ 2000us 区间
    pulse_us = (uint16_t)(SERVO_MID_PULSE_US + speed * 5);
    servo_set_pulse(servo_num, pulse_us);
}
