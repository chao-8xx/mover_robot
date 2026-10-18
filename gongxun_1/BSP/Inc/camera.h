#ifndef GONGXUN_1_CAMERA_H
#define GONGXUN_1_CAMERA_H

// cam 通信协议
// 协议: AA 55 CMD LEN DATA SUM
// Maix DATA数据: FRAME_ID OK dxL dxH dyL dyH
#define HEAD1    0xAA           // 帧头
#define HEAD2    0x55
#define CMD_MV   0xB1           // 命令位 让openmv 扫码
#define CMD_QR   0xBB           // 命令位 接收openmv 二维码数据
#define MAIX_CMD_NONE  0xB0
#define MAIX_CMD_BLOB  0xB2     // 物料识别
#define MAIX_CMD_RING  0xB3     // 圆环识别
#define MAIX_CMD_ONUP  0xB4     // 码垛检测

// 颜色指令码
#define MAIX_COLOR_NONE       0xC0
#define MAIX_COLOR_RED        0xC1
#define MAIX_COLOR_YELLOW     0xC2
#define MAIX_COLOR_BLUE       0xC3
#define MAIX_COLOR_GREEN      0xC4
#define MAIX_COLOR_BLACK      0xC5
#define MAIX_COLOR_LIGHTBLUE  0xC6

// 圆环数字指令码
#define MAIX_RING_NONE 0xD0
#define MAIX_RING_1    0xD1
#define MAIX_RING_2    0xD2
#define MAIX_RING_3    0xD3

// 缓冲区及数据帧字节定长
#define CAM_LEN         256   // cam 接收缓冲区长度
#define CAM_DATA_LEN    32    // cam 协议层最大帧长
#define QR_DATA_LEN     15    // openmv 扫码数据帧长度
#define MAIX_DATA_LEN   6     // maix 数据帧长

// maix 视觉任务窗口滤波参数
#define MAIX_WINDOW_SIZE       5
#define MAIX_VALID_MIN         3
#define MAIX_STABLE_DX_SPAN    5
#define MAIX_STABLE_DY_SPAN    5
#define MAIX_TIMEOUT_MS        300

// cam 状态机状态枚举
typedef enum {
    cam_wait_aa = 0,
    cam_wait_55,
    cam_wait_cmd,
    cam_wait_len,
    cam_wait_data,
    cam_wait_sum
} cam_state_t;

// Maix单帧原始结果缓存，用于窗口判定
typedef struct
{
    uint8_t ok;
    int16_t dx;
    int16_t dy;
} maix_filter_t;

// Maix 单帧原始定长通信协议帧
typedef struct
{
    uint8_t cmd;        // 命令码
    uint8_t target;     // 目标任务  F407 当前下发的颜色/数字目标
    uint8_t frame_id;   // Maix 单字节视觉帧ID
    uint8_t ok;         // 当前单帧识别结果  B2:0无目标/1发现/2对准
    int16_t dx;         // X偏差值
    int16_t dy;         // Y偏差值
    uint32_t tick;      // F407 接收此原始帧的时间
    uint8_t new_data;   // 是否有新数据
} maix_vision_t;

// F407根据最近5个不重复FRAME_ID计算的控制层结果
// valid：窗口内至少3帧有效目标（B2的1/2均有效） 允许单帧晃动
// stable：当前帧有效  且窗口内至少3个候选坐标满足原跨度条件
// B2另要求当前OK=2  稳定候选仅取OK=2  B3/B4候选仍取OK=1
typedef struct
{
    uint8_t cmd;
    uint8_t target;
    uint8_t frame_id;    // 最近参与窗口计算的单帧ID
    uint8_t ok;          // 最新帧原始OK  超时/停止后为0  不等同窗口valid
    uint8_t valid;       // 窗口内目标有效
    uint8_t stable;      // 当前任务满足视觉稳定条件
    uint8_t valid_count; // 窗口内有效目标帧数 B2的OK=1/2都计数
    uint8_t sample_count;
    int16_t dx;          // stable时取最佳三帧中值，否则取全部有效帧中值
    int16_t dy;
    uint32_t tick;       // 最近一个不重复原始帧的F407接收时间
    uint8_t new_data;
} maix_result_t;

// OpenMV固定15字节任务码及其在F407上的接收时间
typedef struct
{
    char qr_data[QR_DATA_LEN + 1];
    uint8_t qr_ok;
    uint32_t qr_tick;
} qr_result_t;

//////////////////////////////////// 摄像头共用USART1接收层 //////////////////////////////
void cam_init(UART_HandleTypeDef *huart);
void cam_rx_callback(UART_HandleTypeDef *huart, uint16_t Size);
void cam_error_callback(UART_HandleTypeDef *huart);
void cam_empty_rx(void);

///////////////////////////////////// 通信协议解析 ////////////////////////////////////////
int16_t read_i16_le(const uint8_t *p);
uint8_t check_sum(uint8_t cmd, uint8_t len, const uint8_t *p);
void cam_byte_handle(uint8_t byte);
void cam_rx_handle(void);
void cam_frame_handle(uint8_t cmd, const uint8_t *data, uint8_t len);
void openmv_frame_handle(const uint8_t *data, uint8_t len);
void maix_frame_handle(uint8_t cmd, const uint8_t *data, uint8_t len);

///////////////////////////////////// maix稳定化处理 ////////////////////////////////////////
void maix_filter_reset(void);
void maix_filter_handle(const maix_vision_t *raw);

///////////////////////////////////// OpenMV扫码任务 //////////////////////////////
void openmv_send_start_scan(void);
void openmv_send_stop_scan(void);
void openmv_qr_task(void);
uint8_t openmv_get_qr_result(qr_result_t *out);

//////////////////////////////////// maix 视觉任务 ///////////////////////////////////////////
void maix_send_frame(uint8_t cmd, const uint8_t *data, uint8_t len);
void maix_stop(void);
void maix_start_blob(uint8_t color);
void maix_start_ring(uint8_t number);
void maix_start_onup(uint8_t color);
void maix_result_task(void);
uint8_t maix_get_result(maix_vision_t *out);
uint8_t maix_get_filtered_result(maix_result_t *out);

extern maix_vision_t maix_vision;
extern maix_result_t maix_result;
extern qr_result_t   qr_result;
extern volatile uint8_t cam_error_flag;
extern volatile uint8_t cam_rx_flag;   // 接收完成标志位

#endif //GONGXUN_1_CAMERA_H
