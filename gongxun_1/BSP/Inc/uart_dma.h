//
// Created by mechreuo on 2026/6/2.
//

#ifndef GONGXUN_1_UART_DMA_H
#define GONGXUN_1_UART_DMA_H

void uart_init(void);

// 串口打印十进制数字（定宽，末尾自动补回车换行）
// 例：UART_PrintNumber(&huart6, -123, 5); -> "-00123\r\n"
void UART_PrintNumber(UART_HandleTypeDef *UART_X, int16_t num, uint8_t len);

// int fputc(int ch, FILE *f);

#endif //GONGXUN_1_UART_DMA_H
