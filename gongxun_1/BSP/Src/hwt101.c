#include "headfile.h"

/**
  * @file    hwt101.c
  * @brief   HWT101 六轴姿态传感器（维特智能）串口驱动
  * @note    使用 USART3（PD8=TX，PD9=RX），115200 8N1
  *
  * 用法示例：
  *   uart_init() 内部会调用 hwt101_init(&huart3)，不需要手动再初始化；
  *   主循环里直接读：
  *     float yaw = hwt101_get_yaw();       // 偏航角，单位 °
  *     float wz  = hwt101_get_yaw_rate();  // 偏航角速度，单位 °/s
  *     float a   = hwt101_get_yaw_accel(); // 偏航角加速度，单位 °/s²
  *   需要把当前方向设为 0 时调用 hwt101_yaw_reset();
  *
  * 数据帧格式（WitMotion 标准协议）：
  *   [0x55] [类型] [D0L] [D0H] [D1L] [D1H] [D2L] [D2H] [D3L] [D3H] [SUM]
  *   SUM = 前 10 字节累加和的低 8 位
  */

// WitMotion 标准协议帧结构：
// [0x55] [类型] [D0L] [D0H] [D1L] [D1H] [D2L] [D2H] [D3L] [D3H] [SUM]
// SUM = 前 10 字节累加和的低 8 位
#define HWT101_FRAME_LEN    11     // 陀螺仪发送的定长数据帧长
#define HWT101_HEAD         0x55   // 帧头
#define HWT101_TYPE_ACC     0x51   // 加速度
#define HWT101_TYPE_GYRO    0x52   // 角速度
#define HWT101_TYPE_ANGLE   0x53   // 角度

// 量程换算：int16 满量程 32767 对应以下物理量
#define HWT101_ACC_RANGE    16.0f    // ±16g
#define HWT101_GYRO_RANGE   2000.0f  // ±2000°/s
#define HWT101_ANGLE_RANGE  180.0f   // ±180°

#define HWT101_DMA_BUF_SIZE 256  // DMA 环形接收缓冲区

// 复位偏航角命令（官方示例）：
// 解锁 FF AA 69 88 B5 -> 延时200ms
// Z角度归零 FF AA 76 00 00 -> 延时500ms
// 保存 FF AA 00 00 00
static const uint8_t hwt101_unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
static const uint8_t hwt101_zero_yaw_cmd[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
static const uint8_t hwt101_save_cmd[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};

HWT101_Data_t hwt101 = {0};

static UART_HandleTypeDef *hwt101_uart = NULL;

// DMA 环形接收缓冲变量
static uint8_t hwt101_dma_buf[HWT101_DMA_BUF_SIZE];// DMA 环形接收缓冲区
static uint16_t hwt101_rd = 0;                     // 读指针: 下一个待解析字节的位置
static uint8_t hwt101_buf[HWT101_FRAME_LEN];       // 当前正在拼装的一帧数据
static uint8_t hwt101_idx = 0;                     // 已收字节下标
volatile uint8_t hwt101_rx_flag = 0;               // 接收标志位
volatile uint8_t hwt101_error_flag = 0;            // 错误标志位

// 接收状态机：
// 0 = 等待 0x55 帧头
// 1 = 等待类型字节（0x51/0x52/0x53）
// 2 = 正在收剩余 9 个字节
static uint8_t hwt101_state = 0;

// 上一帧 wz 和时间，用于差分计算偏航角加速度
static float hwt101_prev_wz = 0.0f;
static uint32_t hwt101_prev_ms = 0;
static uint8_t hwt101_wz_flag = 0; // 必须先有真实上一帧 才能计算差分

// 全局变量
float yaw_rate = 0.0f;          // 全局偏航角角速度
float now_yaw = 0.0f;           // 本次偏航角角度
float last_yaw = 0.0f;          // 上次偏航角角度
float my_yaw = 0.0f;            // 解算后求得的偏航角角度
int32_t yaw_change_flag = 0;    // 角度变换标志

// 陀螺仪初始化
void hwt101_init(UART_HandleTypeDef *huart)
{
    hwt101_uart = huart;
    hwt101_rd = 0;
    hwt101_rx_flag = 0;
    hwt101_error_flag = 0;

    // 通信协议状态初始化
    hwt101_state = 0;
    hwt101_idx = 0;

    // 数据状态
    hwt101.valid = 0;
    hwt101.angle_valid = 0;
    hwt101.yaw_epoch++;

    // 角速度差分
    hwt101_prev_wz = 0.0f;
    hwt101_prev_ms = HAL_GetTick();
    hwt101_wz_flag = 0;

    // 开启陀螺仪的DMA空闲中断  现只存在空闲中断IDLE 和 IC 传输完成中断
    if (hwt101_uart != NULL && hwt101_uart->hdmarx != NULL) {
        if (HAL_UARTEx_ReceiveToIdle_DMA(hwt101_uart, hwt101_dma_buf, sizeof(hwt101_dma_buf)) == HAL_OK)
            __HAL_DMA_DISABLE_IT(hwt101_uart->hdmarx, DMA_IT_HT);
        else hwt101_error_flag = 1;
    }
}

// 陀螺仪接收处理函数
void hwt101_rx_handle(void)
{
    uint16_t wr;

    if (hwt101_uart == NULL || hwt101_uart->hdmarx == NULL)
        return;

    // 串口/DMA 错误回调函数
    if (hwt101_error_flag) {
        hwt101_error_flag = 0;
        HAL_UART_DMAStop(hwt101_uart); // 停止DMA

        // DMA 重启后丢弃旧缓冲区内容 并从下一帧头重新同步协议
        hwt101_rd = 0;
        hwt101_state = 0;
        hwt101_idx = 0;
        hwt101.valid = 0; // 下一有效帧到来前 不再沿用断线前的数据有效标志
        hwt101.angle_valid = 0;
        hwt101_wz_flag = 0;
        hwt101.yaw_accel = 0.0f;

        // 重新开启DMA空闲中断 禁止HC中断
        if (HAL_UARTEx_ReceiveToIdle_DMA(hwt101_uart, hwt101_dma_buf, sizeof(hwt101_dma_buf)) == HAL_OK) {
            __HAL_DMA_DISABLE_IT(hwt101_uart->hdmarx, DMA_IT_HT);
            printf("[HWT101 RX] DMA错误后重启成功\r\n");
        } else {
            hwt101_error_flag = 1; // 下次主循环继续重试 不反复打印塞满日志
        }
        return;
    }

    // 正常数据读取
    wr = HWT101_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(hwt101_uart->hdmarx);  // DMA写指针
    if (wr >= HWT101_DMA_BUF_SIZE)
        wr = 0;

    while (hwt101_rd != wr) {
        uint8_t byte = hwt101_dma_buf[hwt101_rd];

        // 解析协议
        hwt101_byte_handle(byte);

        hwt101_rd++;
        if (hwt101_rd >= HWT101_DMA_BUF_SIZE)
            hwt101_rd = 0;
    }
}

// 陀螺仪接收回调
void hwt101_rx_callback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance != USART3)
        return;

    (void)size;

    // 接收中断只置位 通知主循环过来处理接收到的新字节
    HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(huart);
    if (event == HAL_UART_RXEVENT_IDLE) {
        hwt101_rx_flag = 1;
    }
}

// 陀螺仪错误回调
void hwt101_error_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) {
        return;
    }

    // 中断标记本次角度失效  DMA恢复仍交给接收处理函数
    hwt101.angle_valid = 0;
    hwt101.yaw_epoch++;
    hwt101_rx_flag = 1;
    hwt101_error_flag = 1;
}

// 读取小端序 int16：低字节在前，高字节在后
static int16_t hwt101_get_i16(uint8_t offset)
{
    return (int16_t)(hwt101_buf[offset] | (uint16_t)(hwt101_buf[offset + 1] << 8));
}

// 校验并解析一帧：先验校验和，再按类型把原始 int16 换算成物理量
static void hwt101_parse_frame(void)
{
    uint8_t sum = 0;
    uint32_t now_ms;
    uint32_t dt_ms;

    // 校验和：前 10 字节累加，结果的低 8 位必须等于最后一字节
    for (uint8_t i = 0; i < HWT101_FRAME_LEN - 1; i++) {
        sum += hwt101_buf[i];
    }
    if (sum != hwt101_buf[HWT101_FRAME_LEN - 1]) {
        return;
    }

    // 类型字节决定这一帧是什么数据
    switch (hwt101_buf[1])
    {
        case HWT101_TYPE_ACC:   // 0x51：三轴加速度
            hwt101.ax = (float)hwt101_get_i16(2) / 32768.0f * HWT101_ACC_RANGE;
            hwt101.ay = (float)hwt101_get_i16(4) / 32768.0f * HWT101_ACC_RANGE;
            hwt101.az = (float)hwt101_get_i16(6) / 32768.0f * HWT101_ACC_RANGE;
            break;

        case HWT101_TYPE_GYRO:  // 0x52：三轴角速度
            hwt101.wx = (float)hwt101_get_i16(2) / 32768.0f * HWT101_GYRO_RANGE;
            hwt101.wy = (float)hwt101_get_i16(4) / 32768.0f * HWT101_GYRO_RANGE;
            hwt101.wz = (float)hwt101_get_i16(6) / 32768.0f * HWT101_GYRO_RANGE;

            // HWT101 不直接输出角加速度，这里用相邻两帧 wz 差分近似：
            // 角加速度(°/s²) = Δwz(°/s) / Δt(s)
            now_ms = HAL_GetTick();
            dt_ms = now_ms - hwt101_prev_ms;
            if (hwt101_wz_flag && dt_ms > 0 && dt_ms < 1000)  // 两帧都存在且间隔合理才计算 避免异常值
            {
                hwt101.yaw_accel = (hwt101.wz - hwt101_prev_wz) * 1000.0f / (float)dt_ms;
            }
            else
            {
                hwt101.yaw_accel = 0.0f;
            }
            hwt101_prev_wz = hwt101.wz;
            hwt101_prev_ms = now_ms;
            hwt101_wz_flag = 1;
            break;

        case HWT101_TYPE_ANGLE: // 0x53：姿态角
            hwt101.roll  = (float)hwt101_get_i16(2) / 32768.0f * HWT101_ANGLE_RANGE;
            hwt101.pitch = (float)hwt101_get_i16(4) / 32768.0f * HWT101_ANGLE_RANGE;
            hwt101.yaw   = (float)hwt101_get_i16(6) / 32768.0f * HWT101_ANGLE_RANGE;
            hwt101.angle_tick = HAL_GetTick();
            hwt101.angle_valid = 1;
            break;

        default:
            return;
    }

    // 只要收到过任意一种有效帧，就认为数据链路正常
    hwt101.last_update_ms = HAL_GetTick();
    hwt101.valid = 1;
}

// 单字节状态机：每个中断字节喂进来一次
// 流程：找 0x55 帧头 -> 认类型 -> 收满 11 字节 -> 解析 -> 重新等帧头
void hwt101_byte_handle(uint8_t byte)
{
    if (hwt101_state == 0) {     // 等帧头
        if (byte == HWT101_HEAD) {
            hwt101_buf[0] = byte;   // 帧头入缓冲
            hwt101_state = 1;       // 下一步认类型
        }
        return;
    }

    if (hwt101_state == 1) {     // 等类型字节
        // 类型不在 0x51/0x52/0x53 里，说明之前 0x55 是噪声，重新等帧头
        if (byte != HWT101_TYPE_ACC &&
            byte != HWT101_TYPE_GYRO &&
            byte != HWT101_TYPE_ANGLE)
        {
            hwt101_state = 0;
            return;
        }
        hwt101_buf[1] = byte;   // 类型入缓冲
        hwt101_idx = 2;         // 帧头+类型已经占了 2 个字节
        hwt101_state = 2;       // 开始收剩余数据
        return;
    }

    // 收剩余数据字节，直到凑满一帧
    hwt101_buf[hwt101_idx++] = byte;
    if (hwt101_idx >= HWT101_FRAME_LEN) {
        hwt101_parse_frame();   // 校验 + 解析
        hwt101_state = 0;       // 回到等帧头，准备下一帧
    }
}


float hwt101_get_yaw(void)
{
    return hwt101.yaw;    // 单位 °
}

float hwt101_get_yaw_rate(void)
{
    return hwt101.wz;     // 单位 °/s
}

float hwt101_get_yaw_accel(void)
{
    return hwt101.yaw_accel;   // 单位 °/s²
}

uint8_t hwt101_data_valid(void)
{
    return hwt101.valid;       // 1 = 已经收到过有效数据
}

// 非阻塞复位状态机：解锁 -> 200ms -> Z角度归零 -> 500ms -> 保存
typedef enum {
    HWT101_RST_IDLE,
    HWT101_RST_UNLOCK,
    HWT101_RST_WAIT_UNLOCK,
    HWT101_RST_ZERO,
    HWT101_RST_WAIT_ZERO,
    HWT101_RST_SAVE
} Hwt101_RstState_t;

static volatile uint8_t hwt101_rst_request = 0;
static volatile uint8_t hwt101_rst_state = HWT101_RST_IDLE;
static uint32_t hwt101_rst_tick = 0;

// 地图只使用最近100ms内的角度帧  归零前后的朝向基准不能混用
uint8_t hwt101_yaw_valid(void)
{
    return hwt101.valid && hwt101.angle_valid && !hwt101_error_flag &&
           !hwt101_rst_request && hwt101_rst_state == HWT101_RST_IDLE &&
           (uint32_t)(HAL_GetTick() - hwt101.angle_tick) <= 100;
}

// 中断安全版复位请求：只置标志，不阻塞、不等待
void hwt101_yaw_reset_request(void)
{
    hwt101.angle_valid = 0;
    hwt101.yaw_epoch++;
    hwt101_rst_request = 1;
}

// 复位偏航角（非阻塞，等价于 hwt101_yaw_reset_request）
void hwt101_yaw_reset(void)
{
    hwt101_yaw_reset_request();
}

// 1ms 定时中断里调用，驱动非阻塞复位流程
void hwt101_tick(void)
{
    if (hwt101_uart == NULL) {
        return;
    }

    // 有复位请求且当前空闲时，启动一次复位流程
    if (hwt101_rst_request && hwt101_rst_state == HWT101_RST_IDLE) {
        hwt101_rst_request = 0;
        hwt101_rst_state = HWT101_RST_UNLOCK;
    }

    switch (hwt101_rst_state) {
        case HWT101_RST_UNLOCK:   // 发送解锁
            // 注意：不能用 HAL_UART_GetState()，RX DMA 运行时它会返回 BUSY_RX；
            // 发送只关心 gState，和 RX 的 RxState 是分开的
            if (hwt101_uart->gState == HAL_UART_STATE_READY &&
                HAL_UART_Transmit_IT(hwt101_uart, (uint8_t *)hwt101_unlock_cmd, 5) == HAL_OK) {
                hwt101_rst_tick = HAL_GetTick();
                hwt101_rst_state = HWT101_RST_WAIT_UNLOCK;
            }
            break;

        case HWT101_RST_WAIT_UNLOCK:   // 等 200ms
            if (HAL_GetTick() - hwt101_rst_tick >= 200) {
                hwt101_rst_state = HWT101_RST_ZERO;
            }
            break;

        case HWT101_RST_ZERO:   // 发送 Z 角度归零
            if (hwt101_uart->gState == HAL_UART_STATE_READY &&
                HAL_UART_Transmit_IT(hwt101_uart, (uint8_t *)hwt101_zero_yaw_cmd, 5) == HAL_OK) {
                hwt101_rst_tick = HAL_GetTick();
                hwt101_rst_state = HWT101_RST_WAIT_ZERO;
            }
            break;

        case HWT101_RST_WAIT_ZERO:   // 等 500ms
            if (HAL_GetTick() - hwt101_rst_tick >= 500) {
                hwt101_rst_state = HWT101_RST_SAVE;
            }
            break;

        case HWT101_RST_SAVE:   // 发送保存
            if (hwt101_uart->gState == HAL_UART_STATE_READY &&
                HAL_UART_Transmit_IT(hwt101_uart, (uint8_t *)hwt101_save_cmd, 5) == HAL_OK) {
                hwt101_rst_state = HWT101_RST_IDLE;
                hwt101.angle_valid = 0;  // 等归零结束后的新角度帧
                hwt101_prev_wz = 0.0f;
                hwt101_wz_flag = 0;
                hwt101.yaw_accel = 0.0f;
            }
            break;

        default:
            hwt101_rst_state = HWT101_RST_IDLE;
            break;
    }
}

static float wz_bias = 0.0f;      // 静止角速度偏置 °/s
static uint32_t bias_t0 = 0;      // 补偿计时起点

// 测静止漂移率：车必须停稳，采 1 秒 wz 求平均
void hwt101_drift_calib(void)
{
    float sum = 0.0f;
    uint32_t n = 0;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 1000) {
        hwt101_rx_flag = 0;
        hwt101_rx_handle();        // 解析新帧，刷新 wz
        sum += hwt101_get_yaw_rate();
        n++;
        HAL_Delay(2);
    }
    wz_bias = (n > 0) ? (sum / n) : 0.0f;
    bias_t0 = HAL_GetTick();

    printf("wz_bias=%.4f deg/s\r\n", wz_bias);   // ← 就打印在这里
}

// HWT101初始化（串口3） 初始化加偏航角复位
void my_hwt101_init(void)
{
    hwt101_init(&huart3);

    // 等待陀螺仪输出第一帧有效数据，最多等 2 秒
    uint32_t wait_start = HAL_GetTick();
    while (!hwt101_data_valid() && (HAL_GetTick() - wait_start < 2000)) {
        // 中断只通知 等待期间也要解析DMA里的字节 否则valid一直不会变
        hwt101_rx_flag = 0;
        hwt101_rx_handle();
    }
    // 只有收到有效数据后再复位偏航角，复位指令才有意义
    if (hwt101_data_valid()) {
        hwt101_yaw_reset_request();   // 非阻塞，1ms 中断里自动执行完整复位流程
    }
    HAL_Delay(100);

    hwt101_drift_calib();   // 车静止，测漂移率，顺便打印
}

// 获取陀螺仪数据
void imu_get(void)
{
    yaw_rate = hwt101_get_yaw_rate();         // 偏航角角速度
    now_yaw = hwt101_get_yaw();   // 偏航角
    // now_yaw = hwt101_get_yaw() - hwt101_yaw_offset;   // 减软件零点偏移
    if (now_yaw - last_yaw > 180.0f) {
        yaw_change_flag --;
    }
    else if (now_yaw - last_yaw < -180.0f) {
        yaw_change_flag ++;
    }
    my_yaw = now_yaw + (float)yaw_change_flag * 360.0f ;
    last_yaw = now_yaw ;
}

