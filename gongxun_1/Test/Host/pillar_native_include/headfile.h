#ifndef PILLAR_NATIVE_HOST_HEADFILE_H
#define PILLAR_NATIVE_HOST_HEADFILE_H
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { uint16_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
#include "lidar.h"
#include "map_tf2d.h"
#include "map_obs.h"
#endif
