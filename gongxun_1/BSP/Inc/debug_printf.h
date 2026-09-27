//
// Created by mechreuo on 2026/9/17.
//

#ifndef GONGXUN_1_DEBUG_PRINTF_H
#define GONGXUN_1_DEBUG_PRINTF_H



#define DEBUG_TX_BUF_SIZE 8192      // 调试串口发送缓冲区大小


void debug_uart_init(UART_HandleTypeDef *huart);
uint16_t debug_printf_write(const uint8_t *data, uint16_t len);

void debug_tx_callback(UART_HandleTypeDef *huart);
void debug_error_callback(UART_HandleTypeDef *huart);

uint32_t debug_get_err_bytes(void);


#endif //GONGXUN_1_DEBUG_PRINTF_H
