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

#include "headfile.h"

// 增量式PID
void Incremental_PID_Init(PID *pid, float p, float i, float d, float minOutput, float maxOutput);
void Incremental_PID_Cal(PID *pid, float set_value, float get_value);

// 位置式PID
void Positional_PID_Init(PID *pid, float p, float i, float d, float maxI,float minOutput, float maxOutput);
void Positional_PID_Cal(PID *pid,float set_value, float get_value);

// 清除PID环的任何时刻误差和积分
void PID_Reset(PID *pid);

extern PID yaw_pid;

#endif //GONGXUN_1_PID_H
