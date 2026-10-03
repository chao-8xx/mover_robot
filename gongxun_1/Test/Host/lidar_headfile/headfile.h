#ifndef HOST_LIDAR_HEADFILE_H
#define HOST_LIDAR_HEADFILE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2 } HAL_StatusTypeDef;
typedef struct { uint16_t remaining; uint8_t pending_tc; } DMA_HandleTypeDef;
typedef struct {
    void *Instance;
    DMA_HandleTypeDef *hdmarx;
    uint32_t RxState;
} UART_HandleTypeDef;

#define USART1 ((void *)1)
#define USART2 ((void *)2)
#define USART3 ((void *)3)
#define UART4  ((void *)4)
#define UART5  ((void *)5)
#define USART6 ((void *)6)
#define __HAL_DMA_GET_COUNTER(hdma) ((hdma)->remaining)
#define __HAL_DMA_GET_TC_FLAG_INDEX(hdma) (1U)
#define __HAL_DMA_GET_FLAG(hdma, flag) ((hdma)->pending_tc)

uint32_t host_get_primask(void);
void host_disable_irq(void);
void host_set_primask(uint32_t value);
#define __get_PRIMASK() host_get_primask()
#define __disable_irq() host_disable_irq()
#define __set_PRIMASK(value) host_set_primask(value)

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);

extern UART_HandleTypeDef huart1, huart2, huart3, huart4, huart5, huart6;
void cam_init(UART_HandleTypeDef *);
void my_hwt101_init(void);
void hmi_init(UART_HandleTypeDef *);
void debug_uart_init(UART_HandleTypeDef *);
void cam_rx_callback(UART_HandleTypeDef *, uint16_t);
void hwt101_rx_callback(UART_HandleTypeDef *, uint16_t);
void hmi_rx_callback(UART_HandleTypeDef *, uint16_t);
void hmi_error_callback(UART_HandleTypeDef *);
void Emm_V5_tx_callback(UART_HandleTypeDef *);
void debug_tx_callback(UART_HandleTypeDef *);
void cam_error_callback(UART_HandleTypeDef *);
void Emm_V5_Tx_Reset(void);
void hwt101_error_callback(UART_HandleTypeDef *);
void debug_error_callback(UART_HandleTypeDef *);
void debug_printf_write(const uint8_t *, uint16_t);

#include "../../../BSP/Inc/lidar.h"

#endif
