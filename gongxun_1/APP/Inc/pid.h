#ifndef GONGXUN_1_PID_H
#define GONGXUN_1_PID_H

// PID结构体
typedef struct
{
    float kp,ki,kd;                         //三个系数
    float error,lastError,lastlastError;    //误差、上次误差、上上次误差
    float integral,maxIntegral;             //积分、积分限幅
    float output,maxOutput,minOutput;       //输出、输出限幅
} PID;


// 增量式PID
void Incremental_PID_Init(PID *pid, float p, float i, float d, float minOutput, float maxOutput);
void Incremental_PID_Cal(PID *pid, float set_value, float get_value);

// 位置式PID
void Positional_PID_Init(PID *pid, float p, float i, float d, float maxI,float minOutput, float maxOutput);
void Positional_PID_Cal(PID *pid,float set_value, float get_value);

// 清除PID环的任何时刻误差和积分
void PID_Reset(PID *pid);

extern PID yaw_pid;     // 偏航角PID，用于转向
extern PID follow_pid;	// 车子定位校准控制pid，用于麦轮和物料y坐标闭环
extern PID x_pid;		// 物料x坐标PID，用于麦轮和物料x坐标闭环
extern PID y_pid;		// 物料y坐标PID，用于直线模组伸缩于物料y坐标闭环

#endif //GONGXUN_1_PID_H
