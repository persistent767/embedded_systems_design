/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* ---- 本地时间基准 ---------------------------------------------------------
   TIM5 以固定频率连续计数，每秒溢出一次：
     溢出次数 lt_sec          = 本地"秒"
     当前计数值 TIM5->CNT     = 秒内偏移
   整秒脉冲由 TIM5_CH4 的 PWM 硬件产生，上升沿正好落在计数器归零（溢出）那一刻，
   因此脉冲沿与本地时间基准天生对齐，中间没有任何软件介入。

   计数率不写死：启动时从 ARR 寄存器推导，所以你改 CubeMX 里的
   Prescaler / Counter Period 之后，这里不需要跟着改。
   -------------------------------------------------------------------------- */

static volatile uint32_t lt_sec = 0U;       /* TIM5 溢出次数，即本地秒计数 */
static uint32_t lt_cnt_per_sec = 1000000U;  /* 每秒计数个数 = ARR + 1 */
static uint32_t lt_cnt_per_us  = 1U;        /* 每微秒计数个数 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1124;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 562;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
/* TIM5 init function */
void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 108000000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 54000000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* 从寄存器推导计数率，避免代码与 CubeMX 配置脱节 */
  lt_cnt_per_sec = (uint32_t)(htim5.Init.Period + 1U);
  lt_cnt_per_us  = lt_cnt_per_sec / 1000000U;
  if (lt_cnt_per_us == 0U)
  {
    lt_cnt_per_us = 1U;   /* 计数率低于 1MHz 时的兜底 */
  }

  /* 先启动 PWM 输出（同时会启动计数器），再使能更新中断。
     顺序不能反过来：否则计数器先跑起来，第一秒可能输出不完整的脉冲。 */
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* HAL 在初始化时会用 UG 位产生一次更新事件（用来立即装载预分频值），
     那会把更新标志置位。若不清掉，使能中断的瞬间就会立刻进一次中断，
     让秒计数凭空多出 1。 */
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);

  if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    /* TIM5 interrupt Init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspPostInit 0 */

  /* USER CODE END TIM5_MspPostInit 0 */

    __HAL_RCC_GPIOI_CLK_ENABLE();
    /**TIM5 GPIO Configuration
    PI0     ------> TIM5_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM5_MspPostInit 1 */

  /* USER CODE END TIM5_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /* TIM5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* 整秒计时：TIM5 每次溢出（每秒一次）由 main.c 里的
   HAL_TIM_PeriodElapsedCallback 调用本函数。

   之所以不在这里直接定义 HAL_TIM_PeriodElapsedCallback，是因为 main.c
   里已经有一个同名回调，它负责 TIM6（HAL 时基）的毫秒计数，
   两个定义会冲突。 */
void lt_inc_second(void)
{
  lt_sec++;
}

/* 读取本地时间，单位微秒。

   计数值每秒归零，所以时间 = 秒计数 * 1000000 + 当前计数值换算的微秒。

   两个竞争条件要处理：读取过程中恰好跨过整秒边界时，秒计数可能还是旧值
   （溢出中断尚未执行，用 SR 里的更新标志判断），或者刚被中断加过
   （用秒计数前后比较判断）。任一情况发生就重读。

   本函数会在以太网中断里被调用，所以 TIM5 的中断优先级必须高于调用它的
   中断，否则重读会一直等下去。 */
uint64_t lt_now_us(void)
{
  uint32_t sec;
  uint32_t cnt;

  do
  {
    sec = lt_sec;
    cnt = TIM5->CNT;
  } while ((sec != lt_sec) || ((TIM5->SR & TIM_SR_UIF) != 0U));

  return ((uint64_t)sec * 1000000ULL) + (uint64_t)(cnt / lt_cnt_per_us);
}

/* 读取本地时间，单位 "TIM5 计数"（1 计数 = 9.26ns），不截断到微秒。

   麦克风的采样网格来自 TIM2（同样 108MHz），每个采样点正好 1125 个计数，
   所以"样本序号 -> 时间"是整数乘法；用微秒会丢掉这部分精度（见设计文档 03 §3.1/§3.3）。

   与 lt_now_us() 同样的竞争条件处理：本函数会在 DMA 中断里被调用，
   所以调用它的中断优先级号必须大于 TIM5 的 3，让 TIM5 能抢占它，
   否则下面的重读会一直等下去。 */
uint64_t lt_now_cnt(void)
{
  uint32_t sec;
  uint32_t cnt;

  do
  {
    sec = lt_sec;
    cnt = TIM5->CNT;
  } while ((sec != lt_sec) || ((TIM5->SR & TIM_SR_UIF) != 0U));

  return ((uint64_t)sec * (uint64_t)lt_cnt_per_sec) + (uint64_t)cnt;
}

uint32_t lt_counts_per_sec(void)
{
  return lt_cnt_per_sec;
}

uint32_t lt_counts_per_us(void)
{
  return lt_cnt_per_us;
}

/* 把本地时钟拨 delta_us 微秒（正数 = 往回拨，负数 = 往前拨）。

   直接改写 TIM5 的计数值，并同步秒计数——因为整秒脉冲就是计数器归零那一刻
   由硬件产生的，只有动 CNT 才能真正移动脉冲的位置，改软件变量没有用。

   必须屏蔽中断：读 CNT 和写 CNT 之间如果刚好跨过整秒边界，秒计数和计数值
   就会失去对应关系。

   TIM5 中断优先级是 3，高于 FreeRTOS 的临界区阈值（5），所以不能用
   taskENTER_CRITICAL（它挡不住优先级 3 的中断），只能用 __disable_irq。
   好在临界区里只有几十条指令，对系统的影响可以忽略。 */
void lt_adjust_local_time(int32_t delta_us)
{
  int64_t per_sec = (int64_t)lt_cnt_per_sec;
  int64_t per_us  = (int64_t)lt_cnt_per_us;
  int64_t total;

  __disable_irq();

  /* 若这一秒的溢出还没被处理，先把它算进去并清掉标志，免得之后重复加一 */
  if ((TIM5->SR & TIM_SR_UIF) != 0U)
  {
    TIM5->SR = ~TIM_SR_UIF;
    lt_sec++;
  }

  {
    /* 分成两句读：同一表达式里读两个 volatile 时求值顺序未定义（IAR Pa082） */
    uint32_t sec_now = lt_sec;
    uint32_t cnt_now = TIM5->CNT;
    total = ((int64_t)sec_now * per_sec) + (int64_t)cnt_now;
  }
  total -= ((int64_t)delta_us * per_us);

  if (total < 0)
  {
    total = 0;
  }

  lt_sec    = (uint32_t)(total / per_sec);
  TIM5->CNT = (uint32_t)(total % per_sec);

  __enable_irq();
}

/* ---- 以太网收包时间戳 ----------------------------------------------------
   收包时间戳必须在以太网中断里打。报文从到达网口到被应用程序取走，中间要
   经过以太网中断、ethernetif 线程、tcpip 线程、socket 好几层，延迟是毫秒
   级而且抖动很大；只有中断里的那一刻才贴近硬件。

   本函数由 stm32f7xx_it.c 的 ETH_IRQHandler 在中断入口处调用，那里有用户
   代码区，重新生成代码不会被覆盖。

   千万不能用 ethernetif.c 里的 HAL_ETH_RxLinkCallback 代替：那个回调是由
   HAL_ETH_ReadData() 调用的，而后者运行在 ethernetif 线程里、不在中断里，
   实测时间戳会晚上百微秒而且抖动很大，两块板的偏差还不一样。

   注意：它会调用 lt_now_us()，而后者在更新标志置位时要等 TIM5 中断处理完。
   TIM5 中断优先级（3）高于以太网中断（5），可以抢占，所以不会死锁。
   -------------------------------------------------------------------------- */
/* 用 32 位存微秒低位：协议里所有时间戳都只用到低 32 位（一次交换在毫秒内
   完成，32 位无符号减法本身就是回绕安全的），而且 32 位读取是原子的。 */
volatile uint32_t lt_rx_stamp = 0U;        /* 最近一次收包的硬件时刻 */
volatile uint32_t lt_rx_frames = 0U;       /* 累计收到的帧数 */

void lt_rx_tick(void)
{
  lt_rx_stamp = (uint32_t)lt_now_us();
  lt_rx_frames++;
}

/* USER CODE END 1 */

