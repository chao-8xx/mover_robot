#ifndef GONGXUN_1_CHASSIS_H
#define GONGXUN_1_CHASSIS_H

#define YAW_THRESH   0.3f   // 转到目标角的限制范围

// ---- 结构体和枚举定义 ---- //
// 底盘运动目标：x/y/omega 是参与四轮合成的指令分量，与电机 RPM 指令同尺度。
// omega 是旋转控制量，未按车体几何换算为物理角速度(rad/s)；车体 +x 向右、+y 向前。
typedef struct {
    int16_t x;    			// X轴
    int16_t y;   			// Y轴
    int16_t omega; 			// 旋转指令分量（偏航PID输出）
} Chassis_Data;

// 底盘当前模式：空闲 / 旋转 / 移动
typedef enum {
    CHASSIS_IDLE,
    CHASSIS_ROTATE,
    CHASSIS_MOVE
} Chassis_Mode;

// ---------------- 函数声明 ---------------- //

// ======== 初始化 ======== //
// 初始化底盘（含偏航 PID）
void chassis_init(void);

// ======== 底盘移动旋转控制 ======== //
// 停车
void car_stop(void);
// 速度模式：小车平移和旋转
void car_speed_move (int16_t x_speed, int16_t y_speed, int16_t omega_speed);
// 原地旋转（速度模式，omega 为电机指令尺度的旋转分量）
void car_rotate(int16_t omega);
// 一次调用：设置小车要转到的目标角度（°）
// 设置后仍需调用者周期执行 car_rotate_ctrl()；当前正式 timer.c 没有自动调用该函数
void car_rotate_angle(float target_angle);
// 需按10ms节拍调用的旋转闭环接口，到位后复位偏航角；与下方不归零接口区分
void car_rotate_ctrl(void);
// 小车旋转目标角度（不复位角度版）；比赛 rotate_state 分支每10ms持续调用，非只设置一次
void car_rotate_yaw(float target_yaw);
// 一次调用：小车沿车体 X/Y 轴走固定距离（cm），speed为驱动器速度指令RPM，位置模式
// busy结束可能来自估算计时；不能单独作为实际到位反馈
// x_cm > 0 向右，x_cm < 0 向左；y_cm > 0 向前，y_cm < 0 向后
void car_run_distance(int16_t speed, float x_cm, float y_cm);
// 底盘四个电机回到编码器零点位置
void car_back_zero(void);

// ======== 底盘状态 ======== //
// 是否还在执行旋转/移动
uint8_t car_is_busy(void);
// 获取底盘状态
Chassis_Mode car_get_state(void);

// ======== 全局位姿解析 ======== //
// 暂停底盘航迹推算（跟随模式下调用）
// 暂停期间位置未知  恢复前必须重新设置已知实际起点
void chassis_odom_hold(void);
// 车辆静止在已知位置时设置实际X/Y(cm)  重新建立轮子基准  不发送移动命令
uint8_t chassis_odom_origin_set(float x_cm, float y_cm);
// 主循环调用：轮流给四个电机发读位置请求
void chassis_odom_poll(void);
// 主循环调用：把完整位置反馈取走，换算成累计cm并记录接收时间
void chassis_odom_fetch(void);
// 主循环调用：四轮新反馈齐全才积分并发布地图位姿
void chassis_odom_update(void);
// 静止清零编码器后重建轮子基准  保留已确认的全局X/Y
void chassis_odom_reset(void);
// 底盘全局位姿实时解算tick  只放主循环  不在中断中调用
// 当前正式 main.c 未周期调用；恢复前还需在可信实际位置建立起点，不能仅打开odom_on
void chassis_odom_tick(void);

// ======== 非阻塞计时发送 ======== //
// 1ms 周期调用：推进分时发送和移动计时；位置模式四轮装填后另发广播同步启动
void car_run_tick(void);

// ---------------- 结构体和枚举声明 ---------------- //
extern Chassis_Data my_chassis;

// ---------------- 变量声明 ---------------- //

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
