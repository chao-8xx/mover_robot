#ifndef __EMM_V5_H
#define __EMM_V5_H


/**********************************************************
*** Emm_V5.0 闭环步进电机驱动器
*** 编写作者：ZHANGDATOU
*** 技术支持：张大头闭环伺服
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN专栏：https://blog.csdn.net/zhangdatou666
*** qq交流群：262438510
**********************************************************/

#define ABS(x) ((x) > 0 ? (x) : -(x))

#define EMM_V5_TX_QUEUE_SIZE 32
#define EMM_V5_TX_FRAME_MAX  20

typedef enum {
	S_VER   = 0,  /* 获取固件版本和对应的硬件版本 */
	S_RL    = 1,  /* 获取读取负载率 */
	S_PID   = 2,  /* 获取PID参数 */
	S_VBUS  = 3,  /* 获取母线电压 */
	S_CPHA  = 5,  /* 获取电流相位 */
	S_ENCL  = 7,  /* 获取编码器自校准后的编码器值 */
	S_TPOS  = 8,  /* 获取电机的目标位置角度 */
	S_VEL   = 9,  /* 获取电机实时转速 */
	S_CPOS  = 10, /* 获取电机实时位置角度 */
	S_PERR  = 11, /* 获取电机位置偏差角度 */
	S_FLAG  = 13, /* 获取使能/到位/正反转状态标志位 */
	S_Conf  = 14, /* 获取控制参数 */
	S_State = 15, /* 获取系统状态参数 */
	S_ORG   = 16, /* 获取当前回零/回零失败状态标志位 */
} SysParams_t;

/**********************************************************
*** 注意：每个函数的具体参数说明请阅读对应函数注释说明
**********************************************************/
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr); // 将当前位置清零
void Emm_V5_Reset_Clog_Pro(uint8_t addr); // 解除堵转保护
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s); // 读取参数
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode); // 在线修改控制/闭环控制模式
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF); // 使能控制
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF); // 速度模式控制
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF); // 位置模式控制
void Emm_V5_Stop_Now(uint8_t addr, bool snF); // 电机立即停止运动
void Emm_V5_Synchronous_motion(uint8_t addr); // 触发同步启动运动
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF); // 设置当前圈数零位位置
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF); // 触发回零操作
void Emm_V5_Origin_Interrupt(uint8_t addr); // 强制中断并退出回零

void Emm_V5_tx_callback(UART_HandleTypeDef *huart);	// 发送中断回调

// 复位串口2发送队列（串口错误后调用，避免队列状态卡死）
void Emm_V5_Tx_Reset(void);

// 返回串口2发送队列里还没发完的帧数（0 = 队列空闲）
uint8_t Emm_V5_Tx_Count(void);

#endif
