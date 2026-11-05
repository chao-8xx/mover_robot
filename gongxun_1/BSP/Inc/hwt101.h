#ifndef GONGXUN_1_HWT101_H
#define GONGXUN_1_HWT101_H

#define PI 3.14159265358979323846f

// HWT101 解析结果（角度单位 °，角速度单位 °/s，加速度单位 g）
typedef struct {
    float roll;         // 横滚角
    float pitch;        // 俯仰角
    float yaw;          // 偏航角
    float wx;           // X 轴角速度
    float wy;           // Y 轴角速度
    float wz;           // Z 轴角速度（偏航角速度）
    float ax;           // X 轴加速度
    float ay;           // Y 轴加速度
    float az;           // Z 轴加速度
    float yaw_accel;    // 偏航角加速度（由 wz 差分得到，单位 °/s²）
    uint32_t last_update_ms;  // 最近一次收到有效帧的时间
    uint8_t valid;            // 是否已经收到过有效数据
    uint32_t angle_tick;      // 最近一次有效角度帧的时间  角速度帧不能刷新
    uint32_t yaw_epoch;       // 初始化/归零/接收错误次数  用于使旧地图位姿失效
    uint8_t angle_valid;
} HWT101_Data_t;

extern HWT101_Data_t hwt101;

// 陀螺仪 HWT101 UART+DMA层
void hwt101_init(UART_HandleTypeDef *huart);    // 初始化
void hwt101_rx_callback(UART_HandleTypeDef *huart, uint16_t size); // 陀螺仪接收回调
void hwt101_error_callback(UART_HandleTypeDef *huart);  // 陀螺仪错误回调
void hwt101_rx_handle(void);

// 陀螺仪 HWT101 数据解析层
void hwt101_byte_handle(uint8_t byte);

// 陀螺仪 HWT101 实际读取的值
float hwt101_get_yaw(void);         // 获取偏航角（°）
float hwt101_get_yaw_rate(void);    // 获取偏航角速度（°/s）
float hwt101_get_yaw_accel(void);   // 获取偏航角加速度（°/s²）
uint8_t hwt101_data_valid(void);    // 是否已经收到过有效陀螺仪数据
uint8_t hwt101_yaw_valid(void);     // 角度帧未超时且不在归零过程中

// 陀螺仪 HWT101 非阻塞复位
void hwt101_yaw_reset(void);        // 复位偏航角（非阻塞，只置标志；实际指令由 hwt101_tick 在 1ms 中断里发送）
void hwt101_yaw_reset_request(void);// 中断安全版复位请求：可在任意中断里调用，不会阻塞

void hwt101_tick(void);             // 1ms 定时中断里调用，驱动“解锁→归零→保存”非阻塞状态机
void my_hwt101_init(void);          // HWT101初始化（串口3） 初始化加偏航角复位

// 获取陀螺仪数据
void imu_get(void);

// 全局变量
extern volatile uint8_t hwt101_rx_flag;
extern volatile uint8_t hwt101_error_flag;

extern float yaw_rate;  // 全局偏航角角速度
extern float now_yaw;   // 全局偏航角角度
extern float my_yaw;    // 解算后的偏航角

#endif //GONGXUN_1_HWT101_H
