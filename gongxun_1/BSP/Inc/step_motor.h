#ifndef GONGXUN_1_STEP_MOTOR_H
#define GONGXUN_1_STEP_MOTOR_H

// ---- 电机数量与默认速度、加速度 ---- //
#define MAX_SPEED              3500
#define STEP_ANGLE_DIV         16.0f
#define STEP_MOTOR_COUNT       6
#define STEP_MOTOR_CHASSIS_ACC 110   // 底盘加速度
#define STEP_MOTOR_CATCH_ACC   250   // 夹取装置加速度

// ---- 定义结构体和枚举 ---- //
typedef enum {
    Motor_1 = 0,    // 一号电机
    Motor_2 = 1,    // 二号电机
    Motor_3 = 2,    // 三号电机
    Motor_4 = 3,    // 四号电机
    Motor_5 = 4,    // 五号电机
    Motor_6 = 5     // 六号电机
} Motor_Num;

// ---- 函数声明 ---- //
// 闭环步进电机初始化
void step_motor_init();
// 电机编码器清零
void motor_encoder_clear(Motor_Num motor_num);
// 电机定速控制
void motor_set_speed(Motor_Num motor_num,int16_t speed);
// 电机定速定位置控制（pulse 为相对脉冲数，正负代表方向）
void motor_set_speedPos(Motor_Num motor_num,int16_t speed,int32_t pulse);
// 电机定速定位置控制 绝对（非阻塞，可在主循环或定时器中断中调用）
void motor_set_speed_Pos(Motor_Num motor_num,int16_t speed,int32_t pos);

void motor_pos_set(Motor_Num motor_num, int32_t pos);

// ---- 变量声明 ---- //
extern int16_t motor1_speed;
extern int16_t motor2_speed;
extern int16_t motor3_speed;
extern int16_t motor4_speed;

#endif //GONGXUN_1_STEP_MOTOR_H

/*
  电机编号规定：

           前
      [1]-----[2]
  左  |          |  右
      [3]-----[4]
           后

      [5] 丝杆电机
      [6] 直线模组

  电机地址：Motor_1 ~ Motor_6 对应驱动器地址 1 ~ 6
*/
