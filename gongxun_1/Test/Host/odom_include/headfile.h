#ifndef ODOM_HOST_HEADFILE_H
#define ODOM_HOST_HEADFILE_H

// 桌面测试复用实际反馈、里程计和位姿源码  只替代HAL外设
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>

typedef struct { uint16_t remaining; } DMA_HandleTypeDef;
typedef struct { void *Instance; DMA_HandleTypeDef *hdmarx; uint8_t gState; } UART_HandleTypeDef;
typedef int HAL_StatusTypeDef;
typedef int HAL_UART_RxEventTypeTypeDef;
#define USART2 ((void *)2)
#define USART3 ((void *)3)
#define HAL_OK 0
#define HAL_UART_STATE_READY 0
#define HAL_UART_RXEVENT_IDLE 0
#define DMA_IT_HT 0
#define GPIOC ((void *)12)
#define GPIO_PIN_1 2
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
int HAL_GPIO_ReadPin(void *port, uint16_t pin);
#define __HAL_DMA_DISABLE_IT(dma, it) ((void)(dma), (void)(it))
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart);
extern UART_HandleTypeDef huart2, huart3;

#include "Emm_V5.h"
#include "step_motor.h"
#include "hwt101.h"
#include "pid.h"
#include "chassis.h"
#include "map_track.h"
#include "map_pose.h"
#include "key_led.h"
#include "lidar.h"
#include "map_tf2d.h"
#include "map_obs.h"
#include "map_data.h"
#include "map_update.h"
#include "map_plan.h"
#include "map_lidar.h"
#include "map_nav.h"
#include "map_stage_test.h"
uint8_t map_obstacle_pose_task(void);
uint8_t obstacle_find_task(float x, float y, float yaw);
void openmv_qr_test(void);
void maix_blob_test(void);
extern bool car_run;
typedef struct { uint8_t odom_on; float x_cm, y_cm; } Task_State;
extern Task_State task_state;
#endif
