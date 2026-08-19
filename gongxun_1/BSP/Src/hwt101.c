#include "hwt101.h"

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


// 复位偏航角命令（官方示例）：
// 解锁 FF AA 69 88 B5 -> 延时200ms
// Z角度归零 FF AA 76 00 00 -> 延时500ms
// 保存 FF AA 00 00 00
static const uint8_t hwt101_unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
static const uint8_t hwt101_zero_yaw_cmd[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
static const uint8_t hwt101_save_cmd[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};

HWT101_Data_t hwt101 = {0};


static UART_HandleTypeDef *hwt101_uart = NULL;

static uint8_t hwt101_buf[HWT101_FRAME_LEN];    // 陀螺仪接收缓冲区
static uint8_t hwt101_idx = 0;  // 已收字节下标

// 接收状态机：
// 0 = 等待 0x55 帧头
// 1 = 等待类型字节（0x51/0x52/0x53）
// 2 = 正在收剩余 9 个字节
static uint8_t hwt101_state = 0;

// 上一帧 wz 和时间，用于差分计算偏航角加速度
static float hwt101_prev_wz = 0.0f;
static uint32_t hwt101_prev_ms = 0;


// 陀螺仪初始化
void hwt101_init(UART_HandleTypeDef *huart)
{
    hwt101_uart = huart;

    // 通信协议状态初始化
    hwt101_state = 0;
    hwt101_idx = 0;

    // 数据状态
    hwt101.valid = 0;

    // 角速度差分
    hwt101_prev_wz = 0.0f;
    hwt101_prev_ms = HAL_GetTick();

    // 开启陀螺仪的DMA空闲中断
    HAL_UARTEx_ReceiveToIdle_DMA(hwt101_uart, hwt101_buf, sizeof(hwt101_buf));
    __HAL_DMA_DISABLE_IT(hwt101_uart->hdmarx,DMA_IT_HT);      // 禁止半传输中断
}

// 陀螺仪接收回调
void hwt101_rx_callback(UART_HandleTypeDef *huart, uint16_t size)
{
    hwt101_uart = huart;

    if (size > HWT101_FRAME_LEN) {
        size = HWT101_FRAME_LEN;
    }

    // 状态机逐字节匹配接收
    for (uint16_t i = 0; i < size; i++) {
        hwt101_byte_handle(hwt101_buf[i]);
    }

    // 重新开启陀螺仪的DMA空闲中断
    HAL_UARTEx_ReceiveToIdle_DMA(hwt101_uart, hwt101_buf, sizeof(hwt101_buf));
    __HAL_DMA_DISABLE_IT(hwt101_uart->hdmarx,DMA_IT_HT);      // 禁止半传输中断
}

// 陀螺仪错误回调
void hwt101_error_callback(UART_HandleTypeDef *huart)
{
    if (huart != hwt101_uart) {
        return;
    }

    // 通信协议复位
    hwt101_state = 0;
    hwt101_idx = 0;

    // 重新开启陀螺仪的DMA空闲中断
    HAL_UARTEx_ReceiveToIdle_DMA(hwt101_uart, hwt101_buf, sizeof(hwt101_buf));
    __HAL_DMA_DISABLE_IT(hwt101_uart->hdmarx,DMA_IT_HT);      // 禁止半传输中断
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
            if (dt_ms > 0 && dt_ms < 1000)  // 间隔合理才计算，避免异常值
            {
                hwt101.yaw_accel = (hwt101.wz - hwt101_prev_wz) * 1000.0f / (float)dt_ms;
            }
            else
            {
                hwt101.yaw_accel = 0.0f;
            }
            hwt101_prev_wz = hwt101.wz;
            hwt101_prev_ms = now_ms;
            break;

        case HWT101_TYPE_ANGLE: // 0x53：姿态角
            hwt101.roll  = (float)hwt101_get_i16(2) / 32768.0f * HWT101_ANGLE_RANGE;
            hwt101.pitch = (float)hwt101_get_i16(4) / 32768.0f * HWT101_ANGLE_RANGE;
            hwt101.yaw   = (float)hwt101_get_i16(6) / 32768.0f * HWT101_ANGLE_RANGE;
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

// 中断安全版复位请求：只置标志，不阻塞、不等待
void hwt101_yaw_reset_request(void)
{
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
            if (hwt101_uart->gState == HAL_UART_STATE_READY) {
                HAL_UART_Transmit_IT(hwt101_uart, (uint8_t *)hwt101_unlock_cmd, 5);
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
            if (hwt101_uart->gState == HAL_UART_STATE_READY) {
                HAL_UART_Transmit_IT(hwt101_uart, (uint8_t *)hwt101_zero_yaw_cmd, 5);
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
            if (hwt101_uart->gState == HAL_UART_STATE_READY) {
                HAL_UART_Transmit_IT(hwt101_uart, (uint8_t *)hwt101_save_cmd, 5);
                hwt101_rst_state = HWT101_RST_IDLE;
                hwt101_prev_wz = 0.0f;
                hwt101.yaw_accel = 0.0f;
            }
            break;

        default:
            hwt101_rst_state = HWT101_RST_IDLE;
            break;
    }
}

// HWT101初始化（串口3） 初始化加偏航角复位
void my_hwt101_init(void)
{
    hwt101_init(&huart3);

    // 等待陀螺仪输出第一帧有效数据，最多等 2 秒
    uint32_t wait_start = HAL_GetTick();
    while (!hwt101_data_valid() && (HAL_GetTick() - wait_start < 2000)) {

    }
    // 只有收到有效数据后再复位偏航角，复位指令才有意义
    if (hwt101_data_valid()) {
        hwt101_yaw_reset_request();   // 非阻塞，1ms 中断里自动执行完整复位流程
    }
}
