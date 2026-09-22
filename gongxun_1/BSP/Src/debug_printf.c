//
// Created by mechreuo on 2026/9/17.
//

#include "headfile.h"

static UART_HandleTypeDef *debug_uart = NULL;   // 使用的调试串口句柄

static uint8_t debug_tx_buffer[DEBUG_TX_BUF_SIZE]; // 打印发送缓冲区
static volatile uint16_t debug_wr = 0;          // 写指针
static volatile uint16_t debug_rd = 0;          // 读指针
static volatile uint16_t debug_dma_len = 0;     // 当前这一批 DMA 正在发送的字节数
static volatile uint8_t debug_dma_busy = 0;     // DMA 正在发送标志位
static volatile uint32_t debug_err_bytes = 0;   // 记录未成功发送的字节数


//////////////////////////////////// 内部辅助函数 ////////////////////////////////////
// 获取缓冲区中已使用的字节数
static uint16_t get_used_space(void) {
    if (debug_wr >= debug_rd) {
        return (uint16_t)debug_wr - debug_rd;
    }
    return DEBUG_TX_BUF_SIZE - (debug_rd - debug_wr);
}

// 获取缓冲区中可用的字节数
static uint16_t get_free_space(void) {
    return DEBUG_TX_BUF_SIZE - get_used_space() - 1; // 留一个字节作为缓冲区满的标志
}

// 尝试启动 DMA 发送   DMA空闲且缓冲区有数据时
static void debug_try_send(void) {
    uint16_t send_len;
    HAL_StatusTypeDef status;

    if (debug_uart == NULL)
        return;

    if (debug_dma_busy != 0)
        return;

    if (debug_rd == debug_wr)
        return;

    // DMA 单次只能读取一段连续内存
    // 第一种情况: 如果写指针在读指针之后 说明数据是连续的 可以直接发送
    // 第二种情况: 如果写指针在读指针之前 说明数据是环绕的 需要先发送从读指针到缓冲区末尾的数据
    if (debug_wr > debug_rd) {
        send_len = (uint16_t)(debug_wr - debug_rd);
    } else {
        send_len = (uint16_t)(DEBUG_TX_BUF_SIZE - debug_rd);
    }
    debug_dma_len = send_len;
    debug_dma_busy = 1;

    status = HAL_UART_Transmit_DMA(debug_uart, &debug_tx_buffer[debug_rd], send_len);

    // DMA 发送失败
    if (status != HAL_OK) {
        debug_dma_busy = 0;
        debug_dma_len = 0;
    }
}



//////////////////////////////////// 对外调用函数 ////////////////////////////////////
// 初始化调试串口
void debug_uart_init(UART_HandleTypeDef *huart) {
    uint32_t primask;

    // 保存当前中断状态   禁止中断
    primask = __get_PRIMASK();
    __disable_irq();

    debug_uart = huart;

    debug_wr = 0;
    debug_rd = 0;

    debug_dma_len = 0;
    debug_dma_busy = 0;
    debug_err_bytes = 0;

    // 恢复中断状态
    __set_PRIMASK(primask);
}

// 调试打印函数
uint16_t debug_printf_write(const uint8_t *data, uint16_t len) {
    uint32_t primask;

    uint16_t free_len;      // 缓冲区剩余空间
    uint16_t write_len;     // 实际写入的字节数

    uint16_t first_len;     // 第一次写入的字节数
    uint16_t second_len;    // 第二次写入的字节数

    uint16_t next_wr;       // 下一个写指针位置

    if (data == NULL || len == 0 || debug_uart == NULL) {
        return 0;
    }

    // 保存当前中断状态   禁止中断
    primask = __get_PRIMASK();
    __disable_irq();

    free_len = get_free_space();

    // 空间不足时整条丢弃，避免串口上出现残缺日志
    if (len > free_len) {
        debug_err_bytes += len;
        __set_PRIMASK(primask);
        return 0;
    }

    write_len = len;

    // 计算第一次写入的字节数
    first_len = write_len;

    // 第一段写入
    if (first_len > (uint16_t)(DEBUG_TX_BUF_SIZE - debug_wr)) {
        first_len = (uint16_t)(DEBUG_TX_BUF_SIZE - debug_wr);
    }

    // 第二段写入
    second_len = (uint16_t)write_len - first_len;

    // 写入数据到缓冲区
    memcpy(&debug_tx_buffer[debug_wr], data, first_len);
    if (second_len > 0) {
        memcpy(&debug_tx_buffer[0], &data[first_len], second_len);
    }

    // 更新写指针
    next_wr = (uint16_t)(debug_wr + write_len) % DEBUG_TX_BUF_SIZE;
    debug_wr = next_wr;

    // 尝试启动 DMA 发送
    debug_try_send();

    // 恢复中断状态
    __set_PRIMASK(primask);

    return write_len;
}

// printf DMA 发送完成回调函数
void debug_tx_callback(UART_HandleTypeDef *huart) {
    uint32_t primask;
    uint16_t next_rd;

    if (huart == NULL || debug_uart == NULL || huart != debug_uart)
        return;

    // 保存当前中断状态   禁止中断
    primask = __get_PRIMASK();
    __disable_irq();

    // DMA 发送完成
    if (debug_dma_busy != 0 && debug_dma_len > 0) {

        // 更新读指针
        next_rd = (uint16_t)(debug_rd + debug_dma_len) % DEBUG_TX_BUF_SIZE;
        debug_rd = next_rd;
    }
    debug_dma_busy = 0;
    debug_dma_len = 0;

    // 尝试发送下一批数据
    debug_try_send();

    // 恢复中断状态
    __set_PRIMASK(primask);
}

// printf DMA 发送错误回调函数
void debug_error_callback(UART_HandleTypeDef *huart) {
    uint32_t primask;
    uint16_t remain_len;

    if (huart == NULL || debug_uart == NULL || huart != debug_uart)
        return;

    // 保存当前中断状态   禁止中断
    primask = __get_PRIMASK();
    __disable_irq();

    // 记录错误的字节数
    remain_len = get_used_space();
    debug_err_bytes += remain_len;

    debug_rd = debug_wr;

    // DMA 发送失败
    debug_dma_busy = 0;
    debug_dma_len = 0;

    // 恢复中断状态
    __set_PRIMASK(primask);
}

// 获取 DMA 发送错误的字节数
uint32_t debug_get_err_bytes(void) {
    uint32_t primask;
    uint32_t err_bytes;

    primask = __get_PRIMASK();
    __disable_irq();

    err_bytes = debug_err_bytes;

    __set_PRIMASK(primask);

    return err_bytes;
}
