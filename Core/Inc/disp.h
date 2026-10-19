/**
  ******************************************************************************
  * @file    disp.h
  * @brief   LCD 实时显示麦克风采样电压（LTDC L8 帧缓冲 + 简单绘图）
  ******************************************************************************
  */
#ifndef __DISP_H__
#define __DISP_H__

#include <stdint.h>

#define DISP_W        480u
#define DISP_H        272u

/* 帧缓冲地址：内部 SRAM1 的空闲区（不依赖 SDRAM）。
   L8：480x272x1 = 130560 字节 ≈ 127.5KB。
   实测链接器数据结束于 0x2001'7200，此处到 RAM 末尾还有约 220KB，安全。 */
#define DISP_FB_ADDR  0x20020000u

/* ADC 满量程参考电压（mV），用来把 12 位码换算成电压 */
#define DISP_VREF_MV  3300u
#define DISP_ADC_MAX  4095u

void disp_init(void);            /* 配置图层(L8)、装调色板、清屏（任务里调） */
void disp_task(void *argument);  /* 20Hz 刷新：波形 + 电压数字 */

#endif /* __DISP_H__ */
