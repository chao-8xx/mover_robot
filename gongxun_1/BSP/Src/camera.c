//
// Created by mechreuo on 2026/6/3.
//

#include "headfile.h"

// openmv 通信协议
#define HEAD1    0xAA
#define HEAD2    0x55
#define CMD_MV   0xB1    // 命令位 让openmv 扫码
#define CMD_QR   0xBB    // 命令位 接收openmv 二维码数据
#define MAX_LEN  128     // 最大数据长度

// maix 定长通信协议      # 协议: AA 55 CMD LEN DATA SUM    DATA数据: TARGET OK dxL dxH dyL dyH
#define MAIX_CMD_NONE  0xB0
#define MAIX_CMD_BLOB  0xB2     // 物料识别
#define MAIX_CMD_RING  0xB3     // 圆环识别
#define MAIX_CMD_ONUP  0xB4     // 码垛检测
#define MAIX_DATA_LEN  6
#define MAIX_FRAME_LEN (2 + 1 + 1 + MAIX_DATA_LEN + 1)  // maix接收定长数据


static UART_HandleTypeDef *mv_uart = NULL;
static UART_HandleTypeDef *maix_uart = NULL;

// mv 通信全局变量
uint8_t  mv_rxb;                 // openmv 中断收到的单字节
uint8_t  maix_rxb;               // maix 中断收到的单字节
uint8_t  state = 0;              // 0:等HEAD1, 1:等HEAD2, 2:等命令, 3:等长度, 4:等数据, 5:等校验
uint8_t  cmd, len, idx, sum;     // 当前帧的命令、长度、数据下标、校验和
uint8_t  data_buf[MAX_LEN];      // 存放收到的数据
uint8_t  data_len;               // 有效数据长度
uint8_t  frame[6];               // 发送帧缓冲区
uint8_t  qr_ok = 0;              // 接收到二维码数据的标志位

// maix 通信全局变量
uint8_t maix_rx_buf0[MAIX_FRAME_LEN];   // maix 接收双缓冲区
uint8_t maix_rx_buf1[MAIX_FRAME_LEN];
uint8_t *maix_rx_work = maix_rx_buf0;   // 当前正在接收的缓冲区
uint8_t *maix_rx_ready = NULL;          // 已经接收完成，等待解析的缓冲区
uint8_t maix_rx_use_buf0 = 1;           // 当前使用缓冲区标志
uint8_t maix_rx_idx = 0;                // 当前接收字节索引
uint8_t maix_tx_buf[6];               // maix 发送帧缓冲区

maix_vision_t maix_vision = {0};

///////////////////////////////////// 通信协议解析 /////////////////////////////////////
// 计算校验和
uint8_t check_sum(uint8_t cmd, uint8_t len, const uint8_t *p)
{
    uint16_t sum = cmd + len;
    for (uint8_t i = 0; i < len; i++)
        sum += p[i];
    return (uint8_t)sum;
}

// openmv 解析字节  不定长(变长数据帧)
void openmv_byte_handle(uint8_t byte)
{
    switch (state) {
        case 0: // 帧头1 0xAA
            if (byte == HEAD1) state = 1;
            break;

        case 1: // 帧头2 0x55
            state = (byte == HEAD2) ? 2 : 0;
            break;

        case 2: // 命令 cmd
            cmd = byte;
            state = 3;
            break;

        case 3: // 长度 len
            len = byte;
            idx = 0;
            if (len > MAX_LEN) {
                state = 0;   // 长度不合法，重头开始
            } else if (len == 0) {
                state = 5;   // 没有数据，直接等校验
            } else {
                state = 4;   // 开始收数据
            }
            break;

        case 4: // 数据 data_buf
            data_buf[idx++] = byte;
            if (idx >= len)
                state = 5;   // 收完数据，等校验
            break;

        case 5: // 校验和 sum
            sum = byte;
            if (sum == check_sum(cmd, len, data_buf)) {
                // 校验通过
                if (cmd == CMD_QR) {
                    data_len = len;
                    data_buf[data_len] = '\0';   // 变成字符串
                    qr_ok = 1;                   // 标记数据就绪
                }
            }
            state = 0;  // 无论对错，都回到等待帧头1
            break;

        default:
            state = 0;
            break;
    }
}

// maix 接收字节处理
void maix_byte_handle(uint8_t byte)
{
    if (maix_rx_idx == 0) {     // 帧头1
        if (byte != HEAD1) return;
        maix_rx_work[maix_rx_idx++] = byte;
        return;
    }

    if (maix_rx_idx == 1) {     // 帧头2
        if (byte == HEAD2) {
            maix_rx_work[maix_rx_idx++] = byte;
        } else {
            maix_rx_idx = 0;
        }
        return;
    }

    maix_rx_work[maix_rx_idx++] = byte;     // 接收命令码、长度、数据和校验和

    if (maix_rx_idx == 3 && !is_maix_cmd(maix_rx_work[2])) {    // 命令码CMD
        maix_rx_idx = 0;
        return;
    }

    if (maix_rx_idx == 4 && maix_rx_work[3] != MAIX_DATA_LEN) { // 长度LEN
        maix_rx_idx = 0;
        return;
    }

    // 收到完整帧 交换双缓冲区 解析字节内容
    if (maix_rx_idx >= MAIX_FRAME_LEN) {
        maix_swap_rx_buffer();
        maix_parse_ready_frame();
    }
}

// 判断是否是maix的命令码
uint8_t is_maix_cmd(uint8_t cmd)
{
    return (cmd == MAIX_CMD_BLOB || cmd == MAIX_CMD_RING || cmd == MAIX_CMD_ONUP);
}

// 读取小端16位整数
int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// 交换接收的双缓冲区
void maix_swap_rx_buffer(void)
{
    maix_rx_ready = maix_rx_work;
    maix_rx_use_buf0 = !maix_rx_use_buf0;
    maix_rx_work = maix_rx_use_buf0 ? maix_rx_buf0 : maix_rx_buf1;
    maix_rx_idx = 0;
}

// 解析已经接收完成的maix帧
void maix_parse_ready_frame(void)
{
    uint8_t *p = maix_rx_ready;

    if (p == NULL) return;
    maix_rx_ready = NULL;

    // 校验帧头、命令码、长度和校验和
    if (p[0] != HEAD1 || p[1] != HEAD2) return;
    if (!is_maix_cmd(p[2]) || p[3] != MAIX_DATA_LEN) return;
    if (p[MAIX_FRAME_LEN - 1] != check_sum(p[2], p[3], &p[4])) return;

    // 接收解析 DATA数据
    maix_vision.cmd = p[2];
    maix_vision.target = p[4];
    maix_vision.ok = p[5];
    maix_vision.dx = read_i16_le(&p[6]);
    maix_vision.dy = read_i16_le(&p[8]);
    maix_vision.tick = HAL_GetTick();
    maix_vision.new_data = 1;
}

/////////////////////////////////////////// openmv扫码任务 ///////////////////////////////////////////
// mv初始化
void openmv_uart_init(UART_HandleTypeDef *huart)
{
    mv_uart = huart;
    state = 0;       // 复位串口解析状态
    qr_ok = 0;
    // openmv 扫码使用简单的串口中断 每次只收一个字节就够
    HAL_UART_Receive_IT(mv_uart, &mv_rxb, 1);
}

// mv串口接收中断回调   USART1
void openmv_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    openmv_byte_handle(mv_rxb);                       // 把刚收到的字节丢去解析
    HAL_UART_Receive_IT(mv_uart, &mv_rxb, 1);  // 重新开启接收
}

// mv串口错误回调
void openmv_uart_error_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    state = 0;                                  // 出错就复位状态机
    HAL_UART_Receive_IT(mv_uart, &mv_rxb, 1);  // 继续收
}

// 通信 openmv 开始扫码
void openmv_send_start_scan(void)
{
    uint8_t dat = 1;
    frame[0] = HEAD1;
    frame[1] = HEAD2;
    frame[2] = CMD_MV;
    frame[3] = 1;               // 长度 = 1
    frame[4] = dat;             // 数据 = 1
    frame[5] = check_sum(CMD_MV, 1, &dat);  // 校验

    qr_ok = 0;  // 清除旧结果
    HAL_StatusTypeDef status =
        HAL_UART_Transmit(mv_uart, (uint8_t *)frame, 6, 100);
    printf("OpenMV start status=%d: AA 55 01 01 01 %02X\r\n",
           status, frame[5]);
}

// 通信 openmv 停止扫码
void openmv_send_stop_scan(void)
{
    uint8_t dat = 0;
    frame[0] = HEAD1;
    frame[1] = HEAD2;
    frame[2] = CMD_MV;
    frame[3] = 1;
    frame[4] = dat;
    frame[5] = check_sum(CMD_MV, 1, &dat);

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(mv_uart, (uint8_t *)frame, 6, 100);
    printf("OpenMV stop status=%d: AA 55 01 01 00 %02X\r\n",
           status, frame[5]);
}

// openmv 扫码任务
void openmv_qr_task(void)
{
    if (!qr_ok) return;

    qr_ok = 0;   // 重复标志位
    led_on();    // 亮灯提示收到码
    printf("OpenMV 扫码结果: %s\r\n", data_buf);
}

//////////////////////////////////// maix 视觉任务 ///////////////////////////////////////////
// maix 串口初始化
void maix_uart_init(UART_HandleTypeDef *huart)
{
    maix_uart = huart;
    maix_rx_idx = 0;
    maix_rx_work = maix_rx_buf0;    // 使用缓冲区0接收
    maix_rx_ready = NULL;
    maix_rx_use_buf0 = 1;           // 当前使用缓冲区0
    memset(&maix_vision, 0, sizeof(maix_vision));   // 内存清空

    HAL_UART_Receive_IT(maix_uart, &maix_rxb, 1);
}

// maix 串口接收中断回调    USART3
void maix_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    maix_byte_handle(maix_rxb);
    HAL_UART_Receive_IT(maix_uart, &maix_rxb, 1);
    // HAL_UART_Transmit_IT(maix_uart, maix_tx_buf, 5 + len);
}

// maix 串口错误回调  
void maix_uart_error_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    maix_rx_idx = 0;
    HAL_UART_Receive_IT(maix_uart, &maix_rxb, 1);
}

// maix 发送帧数据
void maix_send_frame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    if (maix_uart == NULL) {
        printf("Maix TX failed: UART not initialized\r\n");
        return;
    }
    if (len > 1) {
        printf("Maix TX failed: invalid len=%u\r\n", len);
        return;            // 此处的len只能为0/1  为0时 停止 为1时 启动maix
    }

    maix_tx_buf[0] = HEAD1;
    maix_tx_buf[1] = HEAD2;
    maix_tx_buf[2] = cmd;
    maix_tx_buf[3] = len;
    if (len == 1) {
        maix_tx_buf[4] = data[0];
    }
    maix_tx_buf[4 + len] = check_sum(cmd, len, data);

    // 测试命令只有 5~6 字节，使用阻塞发送可以直接确认发送结果，
    // 也避免 HAL_UART_Transmit_IT 因上一次发送未完成而返回 BUSY。
    HAL_StatusTypeDef status =
        HAL_UART_Transmit(maix_uart, maix_tx_buf, 5 + len, 100);

    printf("Maix TX status=%d:", status);
    for (uint8_t i = 0; i < 5 + len; i++) {
        printf(" %02X", maix_tx_buf[i]);
    }
    printf("\r\n");
}

// maix 停止视觉任务
void maix_stop(void)
{
    maix_send_frame(MAIX_CMD_NONE, NULL, 0);
}

// maix 启动物料识别任务
void maix_start_blob(uint8_t color)
{
    maix_send_frame(MAIX_CMD_BLOB, &color, 1);
}

// maix 启动圆环识别任务
void maix_start_ring(uint8_t number)
{
    maix_send_frame(MAIX_CMD_RING, &number, 1);
}

// maix 启动码垛检测任务
void maix_start_onup(uint8_t color)
{
    maix_send_frame(MAIX_CMD_ONUP, &color, 1);
}

// maix 获取视觉结构体结果
uint8_t maix_get_result(maix_vision_t *out)
{
    if (!maix_vision.new_data) return 0;

    __disable_irq();        // 在获取maix结构体时关闭中断 防止视觉数据被新传递的数据覆盖
    *out = maix_vision;
    maix_vision.new_data = 0;
    __enable_irq();         // 重新开启中断

    return 1;
}
