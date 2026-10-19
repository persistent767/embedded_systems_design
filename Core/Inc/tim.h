/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim5;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM2_Init(void);
void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* 本地时间基准（定义在 tim.c） */
uint64_t lt_now_us(void);          /* 当前本地时间，单位微秒 */
uint64_t lt_now_cnt(void);         /* 当前本地时间，单位 TIM5 计数（9.26ns/格） */
uint32_t lt_counts_per_sec(void);  /* 每秒计数个数 */
uint32_t lt_counts_per_us(void);   /* 每微秒计数个数 */
void     lt_inc_second(void);      /* TIM5 整秒溢出时调用 */
void     lt_adjust_local_time(int32_t delta_us);   /* 把本地时钟拨 delta_us 微秒 */

/* 以太网收包时间戳（定义在 tim.c，由 ethernetif.c 的中断回调调用） */
extern volatile uint32_t lt_rx_stamp;      /* 最近一次收包的硬件时刻（微秒低位） */
extern volatile uint32_t lt_rx_frames;     /* 累计收到的帧数 */
void     lt_rx_tick(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

