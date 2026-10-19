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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "ltdc.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "disp.h"
#include "mic.h"
#include <string.h>
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
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_LTDC_Init();
  /* USER CODE BEGIN 2 */
  /* 显示初始化放在这里（MX_LTDC_Init 之后、启动调度器之前）：
     帧缓冲在内部 RAM 的 L8（0x20020000），一开机就把图层和调色板改好，
     避免 CubeMX 默认图层（RGB565 @ 0x20010400）先扫出几帧花屏。 */
  disp_init();
  /* 采样也在这里启动：不依赖 StartDefaultTask 是否被 LwIP 卡住。
     HAL_ADC_Start_DMA / HAL_TIM_Base_Start 都是纯寄存器操作，调度器起来之前调用安全。 */
  mic_init();

  /* ---- PI0（Arduino D5）= TIM5_CH4 的 1Hz 脉冲怎么处理 -------------------
     模块 4P 座的 "D5" 就是 PI0。把模块的 DO 插上去，我们的 1Hz PWM 会反灌
     模块的比较器，导致 LED1 以 1Hz 规律闪烁（跟声音无关）。

     这里默认把 PI0 释放成浮空输入（等价于"把 DO 悬空"），模块的 DO 就自由了。
     ⚠ 代价：第一步验收用的 1Hz 整秒脉冲不再输出。做 ±10µs 示波器测试时，
        把下面的宏改成 1 重新编译即可恢复（脉冲仍由 TIM5_CH4 硬件生成）。 */
#define LT_PULSE_ON_PI0   1
#if (LT_PULSE_ON_PI0 == 0)
  {
    GPIO_InitTypeDef g;
    (void)memset(&g, 0, sizeof(g));
    (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOI, &g);
  }
#endif
  {
    /* 中断优先级：在代码里设，绕开 CubeMX 对 NVIC 下拉框的限制。

       CubeMX 使能 FreeRTOS 后会把 NVIC 可选范围压到
       LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(=5) 及以上，但 TIM5 必须是 3：
       lt_now_us()/lt_now_cnt() 在"整秒标志已置起但 TIM5 中断尚未执行"时会重读，
       这要求 TIM5 能抢占调用它的中断（ETH=5、DMA2_Stream0=4），否则永远等下去。
       放在这里覆盖 tim.c / dma.c 里的生成值，重新生成代码也不会丢。 */
    HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 4, 0);
  }
  {
    static const char lt_boot[] = "\r\n[LT] boot ok: USART1 ready\r\n";
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)lt_boot, sizeof(lt_boot) - 1U, 200U);
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* TIM5 每秒溢出一次：这就是本地时间基准的整秒边界，
     同时也是整秒脉冲（PI0）上升沿的产生时刻 */
  if (htim->Instance == TIM5)
  {
    lt_inc_second();
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  {
    static const char lt_err[] = "\r\n[LT] Error_Handler reached\r\n";
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)lt_err, sizeof(lt_err) - 1U, 100U);
  }
  __disable_irq();
  /* 串口可能看不到，用 LD1(PI1) 快闪表示"某个外设初始化失败"。
     关中断后 HAL_Delay 不能用，用空循环延时。 */
  {
    GPIO_InitTypeDef g;
    (void)memset(&g, 0, sizeof(g));
    __HAL_RCC_GPIOI_CLK_ENABLE();
    g.Pin = GPIO_PIN_1;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOI, &g);
    for (;;)
    {
      volatile uint32_t d;
      HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_1);
      for (d = 0u; d < 3000000u; d++) { }
    }
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
