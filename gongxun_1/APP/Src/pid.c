#include "headfile.h"

// 增量式PID参数的初始化
void Incremental_PID_Init(PID *pid, float p, float i, float d, float minOutput, float maxOutput)
{
		pid->kp = p;
		pid->ki = i;
		pid->kd = d;
		pid->maxOutput = maxOutput;
		pid->minOutput = minOutput;
}

// 增量式PID控制器
void Incremental_PID_Cal(PID *pid, float set_value,float get_value)
{
		pid->error = set_value - get_value;      									        // 计算偏差
		pid->output += pid->kp*(pid->error - pid->lastError) + pid->ki*pid->error + \
					pid->kd*(pid->error - 2*pid->lastError + pid->lastlastError);			// 增量式PI控制器
		pid->lastlastError = pid->lastError;    											// 保存上上次误差
		pid->lastError = pid->error;	           											// 保存上一次偏差

		if (pid->output > pid->maxOutput)
			pid->output = pid->maxOutput;													// 输出限幅
		else if (pid->output < pid->minOutput)
			pid->output = pid->minOutput;													//输出限幅
}


// 位置式PID参数的初始化
void Positional_PID_Init (PID *pid, float p, float i, float d, float maxI,float minOutput, float maxOutput)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOutput;
	pid->minOutput = minOutput;
}

// 位置式PID控制器。
// Ki/Kd 已按调用节拍使用，公式没有显式 dt；改变周期需重新核对整定。
// 当前输出按 +/-maxOutput 对称限幅，minOutput 虽被初始化但未用于本函数，保留原行为。
void Positional_PID_Cal(PID *pid,float set_value, float get_value)
{
	float dout,pout;

    // 更新数据
    pid->lastError = pid->error; 				// 将旧error存起来
    pid->error = set_value - get_value; 		// 计算新error

    // 计算微分
    dout = (pid->error - pid->lastError) * pid->kd;
    // 计算比例
    pout = pid->error * pid->kp;
    // 计算积分
    pid->integral += pid->error * pid->ki;

    // 积分限幅
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    else if (pid->integral < -pid->maxIntegral)
        pid->integral = -pid->maxIntegral;
    // 计算输出
    pid->output = pout + dout + pid->integral;
    // 输出限幅
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
}

// 清除PID环的任何时刻误差和积分
void PID_Reset(PID *pid)
{
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->lastlastError = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

// 清除已有PID环的任何时刻误差和积分
void PID_Clear(void)
{
	PID_Reset(&yaw_pid);    // 清除误差和积分
	PID_Reset(&x_pid);
	PID_Reset(&y_pid);
	PID_Reset(&follow_pid);
}

PID yaw_pid;	// 偏航角PID，用于转向
PID x_pid;		// 物料x坐标PID，用于麦轮和物料x坐标闭环
PID y_pid;		// 物料y坐标PID，用于麦轮和物料y坐标闭环
PID follow_pid;	// 物料y坐标PID，用于齿轮齿条和物料y坐标闭环
