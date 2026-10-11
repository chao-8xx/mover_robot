/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfile.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  timer_init();       // 定时器初始化

  // my_hwt101_init();   // 陀螺仪
  uart_init();
  tf_init();
  // obstacle_map_init();

  HAL_Delay(5000);

  // my_hmi_init();      // 串口屏
  // HAL_Delay(10);      // 等待一会儿
  // step_motor_init();  // 闭环步进电机
  // HAL_Delay(100);     // 等待一会儿
  // chassis_init();     // 底盘装置
  // catch_init();       // 夹取装置
  // HAL_Delay(100);     // 等待一会儿
  // bfs_init();
  // bfs_base_obstacle();
  // remind_init();
  // task_init();

  // lidar_start();

  openmv_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    // 雷达安装标定测试：按串口提示放置挡板，短按KEY1采集下一组
    // lidar_install_test();
    // wheel_mask_test();  // 四车轮遮挡测试，需要时切换回来

    // lidar_test();

    // printf("111\r\n");

    // static uint32_t last_cloud_update = 0;
    //
    // lidar_process();  // 激光雷达处理
    //
    // // tf2d_test();
    //
    // // if (cam_rx_flag || cam_error_flag) {
    // //   cam_error_flag = 0;
    // //   cam_rx_flag = 0;  // 先清除标志位
    // //   cam_rx_handle();
    // // }
    // // maix_result_task();  // 维护5帧VALID/STABLE 并处理视觉数据超时
    //
    // if(hwt101_rx_flag || hwt101_error_flag) {
    //   hwt101_error_flag = 0;
    //   hwt101_rx_flag = 0;  // 先清除标志位
    //   hwt101_rx_handle();
    // }
    // uint32_t now = HAL_GetTick();
    //
    // if (now - last_cloud_update >= 500) {
    //   last_cloud_update = now;
    //   point_cloud_update(500.0f, 500.0f, my_yaw, now);
    //   obstacle_cluster_update(&cloud, now);
    // }
    //

    // imu_task();
    //
    // if (obstacle_find_task(500.0f, 500.0f, my_yaw)) {
    //   point_cloud_test();
    //   obstacle_cluster_test();
    // }




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 重定向 printf 到 串口6  可以无线串口 调参
// int _write(int file,char *ptr,int len)
// {
//   HAL_UART_Transmit(&huart6,(uint8_t *)ptr,len,100);
//
//   return len;
// }

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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
