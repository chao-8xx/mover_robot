#include "headfile.h"


// UART 串口设备初始化
void uart_init(void)
{
    openmv_uart_init(&huart1);
    maix_uart_init(&huart3);
    lidar_init(&huart4);
}

// UART 接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // OpenMV 用 USART1
    if (huart->Instance == USART1)
    {
        openmv_uart_rx_callback(huart);
    }
    // Maix 用 USART3
    else if (huart->Instance == USART3)
    {
        maix_uart_rx_callback(huart);
    }
    // // LiDAR 用 UART4
    // else if (huart->Instance == UART4)
    // {
    //     AX_LIDAR_RxCallback(huart);
    // }
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
        maix_uart_error_callback(huart);
    }
    // // LiDAR的错误处理
    // if (huart->Instance == UART4)
    // {
    //     // openmv_uart_error_callback(huart);
    // }
}




// // 重定向 printf 到串口2
// int fputc(int ch, FILE *f)
// {
//     (void)f;
//     uint8_t c = (uint8_t)ch;
//     HAL_UART_Transmit(&huart2, &c, 1, 10);
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