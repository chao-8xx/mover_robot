#include "headfile.h"


// UART 串口设备初始化
void uart_init(void)
{
    cam_init(&huart1);          // OpenMV maixcam 摄像头
    my_hwt101_init();           // HWT101 陀螺仪
    lidar_init(&huart4);        // 雷达
    hmi_init(&huart5);          // 串口屏
    debug_uart_init(&huart6);   // 串口调试
}

// 普通中断接收完成
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        lidar_rx_callback(huart);
    }
    // if (huart->Instance == USART1) {
    //     openmv_uart_rx_callback(huart);
    // }

}

// DMA空闲接收事件
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // OpenMV 与 Maix 共用 USART1
    if (huart->Instance == USART1)
    {
        cam_rx_callback(huart,Size);
    }
    else if (huart->Instance == USART3)
    {
        hwt101_rx_callback(huart,Size);
    }
    else if (huart->Instance == UART5)
    {
        hmi_rx_callback(huart, Size);
    }
}

// UART 发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        Emm_V5_tx_callback(huart);
    }
    else if (huart->Instance == USART6)
    {
        debug_tx_callback(huart);
    }
}

// UART 错误回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        cam_error_callback(huart);
    }
    else if (huart->Instance == USART2)
    {
        Emm_V5_Tx_Reset();
    }
    else if (huart->Instance == USART3)
    {
        hwt101_error_callback(huart);
    }
    else if (huart->Instance == USART2)
    {
        Emm_V5_Tx_Reset();
    }
    else if (huart->Instance == UART4)
    {
        lidar_error_callback(huart); // HAL已经停止RX DMA，交给主循环重新开启
    }
    else if (huart->Instance == USART6)
    {
        debug_error_callback(huart);
    }
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

    // buf[10] 需要预留符号位和回车换行，防止 len 过大越界
    if (len > (uint8_t)((value < 0) ? 7 : 8)) {
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

    // USART6 统一经过调试日志队列，避免阻塞发送与 DMA 争用
    if (UART_X->Instance == USART6) {
        debug_printf_write((const uint8_t *)buf, idx);
    } else {
        HAL_UART_Transmit(UART_X, (uint8_t *)buf, idx, 50);
    }
}


// 重定向 printf 到 串口6  可以无线串口 调参
int _write(int file,char *ptr,int len)
{
    (void) file;

    if ((ptr == NULL) || (len <= 0))
        return 0;

    debug_printf_write((const uint8_t *)ptr, (uint16_t)len);

    return len;
}


// // 重定向 printf 到 虚拟串口 CDC
// extern USBD_HandleTypeDef hUsbDeviceFS;
//
// int _write(int file, char *ptr, int len)
// {
//   (void)file;
//
//   if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
//     return len;
//   }
//
//   uint32_t start = HAL_GetTick();
//
//   while (CDC_Transmit_FS((uint8_t *)ptr, len) != USBD_OK) {
//     if (HAL_GetTick() - start > 20) {
//       break;
//     }
//   }
//
//   return len;
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
