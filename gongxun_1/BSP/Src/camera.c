//
// Created by mechreuo on 2026/6/3.
//

#include "headfile.h"

// 通信协议定义
#define HEAD1    0xAA
#define HEAD2    0x55
#define CMD_MV   0x01    // 命令位 让openmv 扫码
#define CMD_QR   0x18    // 命令位 接收openmv 二维码数据
#define MAX_LEN  128     // 最大数据长度

static UART_HandleTypeDef *mv_uart = NULL;

// 串口通信全局变量
uint8_t  mv_rxb;                 // openmv 中断收到的单字节
uint8_t  state = 0;              // 0:等HEAD1, 1:等HEAD2, 2:等命令, 3:等长度, 4:等数据, 5:等校验
uint8_t  cmd, len, idx, sum;     // 当前帧的命令、长度、数据下标、校验和
uint8_t  data_buf[MAX_LEN];      // 存放收到的数据
uint8_t  data_len;               // 有效数据长度
uint8_t  frame[6];               // 发送帧缓冲区
uint8_t  qr_ok = 0;              // 接收到二维码数据的标志位

///////////////////////////////////// 通信协议解析 /////////////////////////////////////
// 计算校验和
uint8_t check_sum(uint8_t cmd, uint8_t len, const uint8_t *p)
{
    uint16_t sum = cmd + len;
    for (uint8_t i = 0; i < len; i++)
        sum += p[i];
    return (uint8_t)sum;
}

// 解析字节
void byte_handle(uint8_t byte)
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

/////////////////////////////////////////// openmv扫码任务 ///////////////////////////////////////////
// 初始化
void openmv_uart_init(UART_HandleTypeDef *huart)
{
    state = 0;       // 复位串口解析状态
    qr_ok = 0;
    // openmv 扫码使用简单的串口中断 每次只收一个字节就够
    HAL_UART_Receive_IT(mv_uart, &mv_rxb, 1);
}

// 串口接收中断回调
void openmv_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    byte_handle(mv_rxb);                       // 把刚收到的字节丢去解析
    HAL_UART_Receive_IT(mv_uart, &mv_rxb, 1);  // 重新开启接收
}

// 串口错误回调
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
    HAL_UART_Transmit_IT(mv_uart, (uint8_t *)frame, 6);
    printf("OpenMV 开始扫描\r\n");
}

// openmv 扫码任务
void openmv_qr_task(void)
{
    if (!qr_ok) return;

    qr_ok = 0;   // 重复标志位
    led_on();    // 亮灯提示收到码
    printf("OpenMV 扫码结果: %s\r\n", data_buf);
}