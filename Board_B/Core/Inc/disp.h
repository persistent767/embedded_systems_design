/**
  ******************************************************************************
  * @file    disp.h
  * @brief   LCD 显示到达时刻、方向、最近 10 次声程差及平均值、dj
  ******************************************************************************
  */
#ifndef __DISP_H__
#define __DISP_H__

#include <stdint.h>

#define DISP_W        480u
#define DISP_H        272u

/* 帧缓冲地址：内部 SRAM1 的 0x20020000~0x2003FDFF（不依赖 SDRAM）。
   L8：480x272x1 = 130560 字节 ≈ 127.5KB；链接器普通 RAM 在它之前截止。 */
#define DISP_FB_ADDR  0x20020000u

void disp_init(void);            /* 配置图层(L8)、装调色板、清屏（任务里调） */
void disp_task(void *argument);  /* 轮询快照，仅在显示数据变化时局部重画 */

/* freertos.c 提供：首次正式宣告收敛时的上电后用时（含 10s 保持期）。 */
uint8_t lt_conv_display_snap(uint32_t *announce_ms);

uint8_t lt_air_display_snap(int32_t *temperature10, uint32_t *speed1000);
/* Selected sound speed in 0.001 m/s, or 0 when AUTO has no valid DHT11 sample. */
uint32_t disp_measurement_speed_x1000(void);
#endif /* __DISP_H__ */
