/* 仅桌面测试使用：替代HAL与硬件接口，实际编译BSP/Src/camera.c。 */
#ifndef CAMERA_HOST_HEADFILE_H
#define CAMERA_HOST_HEADFILE_H
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
typedef struct { uint16_t remaining; } DMA_HandleTypeDef;
typedef struct { void *Instance; DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { HAL_UART_RXEVENT_IDLE } HAL_UART_RxEventTypeTypeDef;
#define USART1 ((void *)1)
#define DMA_IT_HT 1
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
#define __HAL_DMA_DISABLE_IT(dma, it) ((void)(dma), (void)(it))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len, uint32_t timeout);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart);
void led_on(void);
#include "camera.h"
#endif
