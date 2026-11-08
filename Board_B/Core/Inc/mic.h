/**
  ******************************************************************************
  * @file    mic.h
  * @brief   声音到达时刻检测（第二步）：麦克风采样与到达时刻输出
  *          V6.0.2：完整 chirp 匹配合格的事件才发布结果
  ******************************************************************************
  */
#ifndef __MIC_H__
#define __MIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MIC_EVENT_NONE  0u
#define MIC_EVENT_CHIRP 1u
#define MIC_EVENT_CLAP  2u

extern volatile uint8_t mic_event_kind;
extern volatile uint8_t mic_tdoa_kind;

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
void     mic_analysis_task(void *argument);
void     mic_task(void *argument);  /* FreeRTOS 任务：搬副本、算包络、chirp 匹配、报到达时刻 */
void     mic_isr_half(uint8_t half);/* 由 DMA2_Stream0 中断调用，只记一个时间戳 */
uint8_t  mic_quiet_ok(void);        /* 伺服闸门：最近 50ms 内没有事件时返回 1 */

/* 诊断（串口 [MIC] 行用） */
int32_t  mic_last_min(void);
int32_t  mic_last_max(void);

/* ---- 到达时刻结果（本板时间基准） ------------------------------------------ */
extern volatile uint32_t mic_event_t_us;   /* 最近一次发布事件的时刻，微秒低 32 位 */
extern volatile uint32_t mic_event_seq;    /* chirp 匹配合格并发布的事件数 */
extern volatile uint32_t mic_last_pk;      /* 最近一次发布事件的包络峰值（码） */
extern volatile uint32_t mic_last_corr_q;  /* 最近一次发布事件的相关系数 ×1000 */
extern volatile uint32_t mic_event_pos256; /* 匹配峰在 24 kHz 窗口中的 Q8 位置 */
extern volatile int32_t mic_event_phase_start_us; /* 采声触发时的累计拨钟量 */
extern volatile int32_t mic_event_phase_end_us;   /* 匹配完成时的累计拨钟量 */
extern volatile uint32_t mic_match_best_q;     /* 最近一次匹配尝试峰值 ×1000 */
extern volatile uint32_t mic_match_short_cnt;  /* 窗口不足 50 ms 的次数 */
extern volatile uint32_t mic_match_reject_cnt; /* 相关不足门限的次数 */
extern volatile uint32_t mic_weak_chirp_cnt, mic_peer_attempt_cnt, mic_peer_gap_cnt;
extern volatile uint32_t mic_match_cost_us;    /* 最近一次匹配计算耗时 */
extern volatile uint32_t mic_last_nf;      /* 当前噪声底（码，与触发门限同源） */
extern volatile uint32_t mic_last_trig;    /* 当前触发门限（码），现场调参用 */
extern volatile uint32_t mic_tpl_learn_cnt;/* 兼容旧 [TPL] 诊断；V6.0.2 恒为 0 */
extern volatile uint32_t mic_tpl_learn_pk; /* 兼容旧 [TPL] 诊断；V6.0.2 恒为 0 */

/* ---- 跨板 TDOA（两块板跑同一份固件，靠报文交换各自的到达时刻） -------------- */
/* 验收量：最近 10 次配对里，相邻两次 |ΔTDOA| 的最大值，要 < 100 µs。
   板间固定差靠标定消掉，这里看的是"稳不稳"。 */
#define MIC_TDOA_GOOD_US  100u

extern volatile int32_t  mic_tdoa_us;      /* 最近一次配对的 TDOA（本板 − 对方）*/
extern volatile uint32_t mic_tdoa_dj_us;   /* 相邻 |ΔTDOA| 最大值（验收量）*/
extern volatile uint32_t mic_tdoa_pairs;   /* 配对成功次数（累计） */
extern volatile uint32_t mic_tdoa_rxn;     /* 收到对方报文条数（累计） */

/* 触发链路诊断（串口 [MIC] 行）：tg/ev 定位"触发+chirp 匹配"，rx/pr 定位"配对" */
extern volatile uint32_t mic_trig_cnt;     /* 包络连续确认通过次数 */
extern volatile uint32_t mic_env_mx_live;  /* 上一半缓冲包络最大值（实时电平） */
extern volatile uint32_t mic_td_rej_far;   /* 配对差值超物理上限被否的次数 */
extern volatile uint32_t mic_td_rej_jump;  /* 配对相对稳定位置跳变、未进入历史的次数 */
extern volatile uint32_t mic_tdoa_nomatch; /* 收报文但本板窗口内无事件可配的次数 */
extern volatile uint8_t  mic_tpl_valid;    /* V6.0.2 固定 1（chirp 参考就绪） */

uint8_t  mic_tdoa_tx_snap(uint32_t *seq, uint32_t *t_us, uint8_t *kind);
/* 屏幕用的一致快照：同一次临界区读取最近配对及最近 10 次差值。 */
#define MIC_DISTANCE_UNAVAILABLE UINT32_MAX
uint8_t  mic_tdoa_display_snap(uint32_t *local_us, uint32_t *peer_us,
                               int32_t *dt, uint32_t *distance_mm,
                              uint32_t *history_mm, uint8_t cap,
                               uint8_t *count, uint32_t *dj_us, uint8_t *kind,
                               uint32_t *sd_count, double *variance_mm2);
void     mic_tdoa_rx(uint32_t seq, uint32_t t_us, uint32_t kind);

typedef union
{
  struct { uint16_t rq, gate, rms10, pp10; uint8_t replay; } chirp;
  struct { uint16_t peak, noise, rise_us, width_us, clipped; uint8_t early_percent; } clap;
} mic_signal_quality_t;

typedef struct
{
  uint32_t pair_seq, local_seq, peer_seq, sd_count;
  uint32_t distance_mm;
  int32_t tdoa_us;  /* Calibrated local-minus-peer time difference. */
  mic_signal_quality_t signal;
  uint8_t kind, new_distance;
} mic_pair_quality_t;

uint8_t  mic_tdoa_quality_pop(mic_pair_quality_t *result);
extern volatile uint32_t mic_tdoa_quality_dropped;

/* 链路诊断 */
extern volatile uint32_t mic_overruns;     /* 任务没赶上、被 DMA 覆盖的半缓冲数 */

#ifdef __cplusplus
}
#endif

#endif /* __MIC_H__ */
