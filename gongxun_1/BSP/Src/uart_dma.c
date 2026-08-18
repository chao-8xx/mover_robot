#include "headfile.h"


// UART 串口设备初始化
void uart_init(void)
{
    openmv_uart_init(&huart1);
    // Maix 后续改到串口1，目前串口3给 HWT101 陀螺仪用
    // maix_uart_init(&huart3);
    hwt101_init(&huart3);
    lidar_init(&huart4);
}

// 串口打印十进制数字（定宽，末尾自动补回车换行）
// 例：UART_PrintNumber(&huart6, -123, 5); -> "-00123\r\n"
void UART_PrintNumber(UART_HandleTypeDef *UART_X, int16_t num, uint8_t len)
{
    char buf[10];
    uint8_t idx = 0;
    uint8_t i;
    uint32_t power = 1;
    int32_t value = num;    // 用 32 位处理，避免 INT16_MIN 取负时溢出

    if (UART_X == NULL || len == 0) {
        return;
    }

    // 负数先输出负号，再按正数处理
    if (value < 0) {
        buf[idx++] = '-';
        value = -value;
    }

    // power = 10^(len-1)，用于从高位到低位取每一位
    for (i = 0; i < len - 1; i++) {
        power *= 10;
    }

    for (i = 0; i < len; i++) {
        buf[idx++] = (char)('0' + (value / power) % 10);
        power /= 10;
    }

    // 追加回车换行，串口助手按行显示更清晰
    buf[idx++] = '\r';
    buf[idx++] = '\n';

    HAL_UART_Transmit(UART_X, (uint8_t *)buf, idx, 50);
}

// UART 接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // OpenMV 用 USART1
    if (huart->Instance == USART1)
    {
        openmv_uart_rx_callback(huart);
    }
    // HWT101 陀螺仪用 USART3，已改为 DMA + IDLE，
    // 数据在 HAL_UARTEx_RxEventCallback 里处理
    // // LiDAR 用 UART4
    // else if (huart->Instance == UART4)
    // {
    //     AX_LIDAR_RxCallback(huart);
    // }
}

// UART DMA 接收事件回调（USART3 用 IDLE 事件收一段数据）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        hwt101_dma_rx_event(Size);
    }
}

// UART 错误回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        openmv_uart_error_callback(huart);
    }
    else if (huart->Instance == USART3)
    {
        hwt101_uart_error_callback(huart);
    }
    // // LiDAR的错误处理
    // if (huart->Instance == UART4)
    // {
    //     // openmv_uart_error_callback(huart);
    // }
}






// // 重定向 printf 到串口6
// int fputc(int ch, FILE *f)
// {
//     (void)f;
//     uint8_t c = (uint8_t)ch;
//     HAL_UART_Transmit(&huart6, &c, 1, 10);
//     return ch;
// }

// // 使用虚拟串口CDC 重定向 printf
// 已在 主函数 main.c 中实现 _write 函数，这里不需要再定义 fputc 了
// int fputc(int ch, FILE *f)
// {
//     (void)f;
//     uint8_t c = (uint8_t)ch;
//
//     // 通过 USB CDC 发送
//     for (int i = 0; i < 5; i++) {
//         if (CDC_Transmit_FS(&c, 1) == USBD_OK)
//             break;
//     }
//     return ch;
// }
