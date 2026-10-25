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
#include "lidar_pillar_test.h"
#include "lidar_map_desk_test.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#ifndef LIDAR_MAP_DESK_TEST_ENABLE
#define LIDAR_MAP_DESK_TEST_ENABLE 1 // 固定测试位姿的裸雷达地图联调
#endif
#ifndef LIDAR_PILLAR_TEST_ENABLE
#define LIDAR_PILLAR_TEST_ENABLE 0 // 原始点/360格对照；与上面的地图测试互斥
#endif
#if LIDAR_MAP_DESK_TEST_ENABLE && LIDAR_PILLAR_TEST_ENABLE
#error "只能启用一个独立雷达测试入口"
#endif

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

//   // 系统整体初始化
// #if LIDAR_MAP_DESK_TEST_ENABLE
//   lidar_map_desk_test_init(); // 只初始化调试串口、雷达和测试地图
// #elif LIDAR_PILLAR_TEST_ENABLE
//   lidar_pillar_test_init();  // 独立小柱对照，不启动整车设备或运动任务
// #else
  system_init();
// #endif

  // maix 任务一测试
  // maix_b2_test();

  // 预设数据的地图自检
  // map_stage_test();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    cam_task();               // 解析摄像头数据
    mv_task();                // OpenMV扫码处理任务
    send_conduct_task();      // 给maix发数据处理任务
    hmi_get_car_state_task(); // 非阻塞式发送小车当前状态至串口屏任务



    // 雷达安装标定测试：按串口提示放置挡板，短按KEY1采集下一组
    // lidar_install_test();
    // wheel_mask_test();  // 四车轮遮挡测试，需要时切换回来

    // // 雷达测试 完成
    // lidar_test();


    // // 遮挡测试 完成
    // wheel_mask_test();

    // 坐标标定测试   暂缓 后续按实际尺寸复测
    // lidar_install_test();

    // 坐标变换测试 完成
    // // tf2d_test();

    // 点云 聚类静止测试    暂用雷达原点、零朝向作为局部测试坐标 不代表小车实际全局位姿
    // if (obstacle_find_task(0.0f, 0.0f, 0.0f)) {
    //   point_cloud_test();        // 打印点云摘要
    //   obstacle_cluster_test();   // 打印候选物体的角度、位置和尺寸
    // }

//
// #if LIDAR_MAP_DESK_TEST_ENABLE
//     lidar_map_desk_test(); // 固定位姿静止建图；KEY1开始或清空重测，不发送运动指令
// #elif LIDAR_PILLAR_TEST_ENABLE
//     // 独立对照复用全局点云，与正式位姿地图互斥。
//     lidar_pillar_test();
// #else
//     // // 雷达正面障碍物测试 测距离  1300mm
//     // lidar_pillar_test();
//
//     // 真实位姿静止建图：更新道路并打印位姿、扫描和道路状态 不发送运动指令
//     chassis_odom_tick();       // 实际轮子反馈 -> cm里程计 -> mm地图位姿
//     map_pose_test();
// #endif

    // 固定测试位姿的R8封路测试wm
    // map_lidar_test();


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
