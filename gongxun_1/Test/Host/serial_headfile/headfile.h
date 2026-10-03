#ifndef HOST_SERIAL_HEADFILE_H
#define HOST_SERIAL_HEADFILE_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef enum { HAL_UART_RXEVENT_IDLE, HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_HT } HAL_UART_RxEventTypeTypeDef;
typedef struct { uint16_t remaining; } DMA_HandleTypeDef;
typedef struct {
    void *Instance;
    DMA_HandleTypeDef *hdmarx;
    uint32_t gState;
} UART_HandleTypeDef;
#define USART1 ((void *)1)
#define USART3 ((void *)3)
#define UART5 ((void *)5)
#define GPIOC ((void *)6)
#define GPIO_PIN_0 1U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
void HAL_GPIO_WritePin(void *, uint16_t, uint32_t);
void hmi_solve(UART_HandleTypeDef *, uint16_t);
void hmi_rx_reset(void);
typedef struct { uint8_t cur_x, cur_y, task_location; } HostTaskState;
extern HostTaskState task_state;
extern bool car_run, car_reset;
extern uint8_t start_location;
#define PARKING_1 1U
#define PARKING_2 2U
#define BFS_MAP_SIZE 5U
void HAL_GPIO_TogglePin(void *, uint16_t);
void bfs_obstacle_clear(void);
void bfs_set_obstacle(uint8_t, uint8_t, uint8_t);
uint8_t Emm_V5_Tx_Count(void);
void catch_lift(float);
void catch_stretch_pos(float);
#define HAL_UART_STATE_READY 0U
#define DMA_IT_HT 1U
#define __HAL_DMA_GET_COUNTER(h) ((h)->remaining)
#define __HAL_DMA_DISABLE_IT(h, i) ((void)(h), (void)(i))
uint32_t HAL_GetTick(void);
uint32_t host_primask_get(void);
void host_primask_set(uint32_t value);
#define __get_PRIMASK() host_primask_get()
#define __disable_irq() host_primask_set(1)
#define __set_PRIMASK(v) host_primask_set(v)
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *);
void led_on(void);
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
#include "camera.h"
#include "hwt101.h"
#include "debug_printf.h"
#include "hmi.h"
#include "hmi_app.h"

#endif
