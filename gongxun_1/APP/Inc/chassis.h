#ifndef GONGXUN_1_CHASSIS_H
#define GONGXUN_1_CHASSIS_H

// ---- 结构体和枚举定义 ---- //
// 底盘运动目标：x/y 平移速度 + omega 旋转角速度
typedef struct {
    int16_t x;    			// X轴
    int16_t y;   			// Y轴
    int16_t omega; 			// 旋转角速度（偏航PID输出）
} Chassis_Data;

// 底盘当前模式：空闲 / 旋转 / 移动
typedef enum {
    CHASSIS_IDLE,
    CHASSIS_ROTATE,
    CHASSIS_MOVE
} Chassis_Mode;

// ---- 函数声明 ---- //

// 初始化底盘（含偏航 PID）
void chassis_init(void);

// 原地旋转（速度模式，omega 为旋转角速度）
void car_rotate(int16_t omega);

// 停车
void car_stop(void);

// 一次调用：设置小车要转到的目标角度（°）
// 实际闭环由 car_rotate_ctrl() 在 10ms 周期里驱动
void car_rotate_angle(float target_angle);

// 10ms 周期调用（放 TIM2）：旋转闭环控制，到位后复位偏航角
void car_rotate_ctrl(void);

// 小车旋转目标角度（不复位角度版）
void car_rotate_yaw(float target_yaw);

// 一次调用：小车沿 X/Y 轴走固定距离（cm），位置模式
// x_cm > 0 向右，x_cm < 0 向左；y_cm > 0 向前，y_cm < 0 向后
void car_run_distance(int16_t speed, float x_cm, float y_cm);

// 1ms 周期调用（可放 1ms 定时器）：仅做移动非阻塞计时
void car_run_tick(void);

// 是否还在执行旋转/移动
uint8_t car_is_busy(void);

// 获取底盘状态
Chassis_Mode car_get_state(void);

// ---- 结构体和枚举声明 ---- //
extern Chassis_Data my_chassis;

// ---- 变量声明 ---- //

#endif //GONGXUN_1_CHASSIS_H

/*
  电机编号规定：

           前
      [1]-----[2]
  左  |          |  右
      [3]-----[4]
           后

  电机地址：Motor_1 ~ Motor_4 对应驱动器地址 1 ~ 4

  speed > 0 时，电机逆时针转
*/
