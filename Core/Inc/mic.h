/**
  ******************************************************************************
  * @file    mic.h
  * @brief   声音到达时刻检测（第二步）：麦克风采样与到达时刻输出
  ******************************************************************************
  */
#ifndef __MIC_H__
#define __MIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- 采样参数 -------------------------------------------------------------- */
#define MIC_FS_HZ        96000u      /* 采样率：TIM2 = 108MHz/1125 */
#define MIC_CNT_PER_SMP  1125u       /* 每个采样点 = 1125 个 TIM 计数，整数、无舍入 */
#define MIC_HALF_SAMPLES 1024u       /* 半个缓冲（10.667ms） */
#define MIC_BUF_SAMPLES  (MIC_HALF_SAMPLES * 2u)

/* 缓冲地址：SRAM2 尾部 4KB（见 mic.c 里的说明）。
   SRAM1 起点 0x20010000 千万别用——本项目普通数据已经用到 0x2001'2f06。 */
#define MIC_BUF_ADDR     0x2004D000u

/* ---- 接口 ------------------------------------------------------------------ */
void     mic_init(void);            /* 启动 TIM2 触发 + ADC3 + DMA 循环（任务里调） */
void     mic_task(void *argument);  /* FreeRTOS 任务：搬副本、算包络、报到达时刻 */
void     mic_isr_half(uint8_t half);/* 由 DMA2_Stream0 中断调用，只记一个时间戳 */
uint8_t  mic_quiet_ok(void);        /* 伺服闸门：最近 50ms 内没有事件时返回 1 */

/* 诊断（V1 阶段用） */
int32_t  mic_last_dc(void);
int32_t  mic_last_min(void);
int32_t  mic_last_max(void);
int32_t  mic_last_sample(void);     /* 最近一个采样点（码） */
uint32_t mic_seq_count(void);       /* 已完成的半缓冲数（每 10.67ms +1） */
uint8_t  mic_t_history(uint32_t *out10);  /* 最近若干次到达时刻（µs，旧→新）；返回个数 */

/* ---- 到达时刻结果（本板时间基准） ------------------------------------------ */
extern volatile uint32_t mic_event_t_us;   /* 事件时刻，微秒低 32 位 */
extern volatile uint32_t mic_event_dt_us;  /* 与上一次事件的间隔，0 = 首次 */
extern volatile uint32_t mic_event_seq;    /* 事件计数 */
extern volatile uint32_t mic_event_end_us; /* 最近一次事件的结束时刻（µs） */
extern volatile uint32_t mic_event_dur_us; /* 最近一次事件的持续时间（µs） */
extern volatile uint32_t mic_last_pk;      /* 最近事件的包络峰值（码），质量指标雏形 */
extern volatile uint32_t mic_last_pk_iq;   /* 最近事件的 1 kHz I/Q 峰值（门控质量） */
extern volatile uint32_t mic_last_nf;      /* 当前噪声底（码） */
extern volatile uint32_t mic_last_trig;    /* 当前触发门限（码），现场调参用 */

/* 链路诊断（显示在屏幕上，无需串口） */
extern volatile uint8_t  mic_st_adc;       /* HAL_ADC_Start_DMA 的返回值 */
extern volatile uint8_t  mic_st_tim;       /* HAL_TIM_Base_Start 的返回值 */
extern volatile uint32_t mic_task_loops;   /* micTask 主循环计数 */
extern volatile uint32_t mic_adc_probe;    /* 软件触发单次转换的读数（0xFFFFFFFF = 超时） */
extern volatile uint32_t mic_dt_jitter_us; /* 最近 10 次事件间隔的极差（验收指标） */
extern volatile uint8_t  mic_dt_count;     /* 已统计的间隔个数（0..10） */
extern volatile uint32_t mic_overruns;     /* 任务没赶上、被 DMA 覆盖的半缓冲数 */

#ifdef __cplusplus
}
#endif

#endif /* __MIC_H__ */
