/**
  ******************************************************************************
  * @file    mic.c
  * @brief   声音到达时刻检测（V6.0.2：完整 chirp 匹配滤波）
  *
  *  数据流：AO -> PA0(ADC3_IN0) --TIM2 96kHz 触发--> DMA2_Stream0 -> mic_buf
  *          半缓冲中断里记一次 TIM5 计数（时间戳），任务里算包络找起音，
  *          用完整 50 ms chirp 参考做归一化互相关，输出"到达时刻 =
  *          本板时间基准上的微秒值"。
  *
  *  V6.0.2 的判决口径：
  *   · 标准音频是 50 ms 扫频 chirp（约 0.5~3.5 kHz），不是单频音；
  *   · 宽带包络只负责触发和事件窗口，最终时刻由整段 chirp 匹配滤波给出；
  *   · 采集窗口 84 ms（含 32 ms 预触发），完整覆盖 50 ms chirp；
  *   · 96 kHz 原始样本 4:1 抽取到 24 kHz 后做相关，兼顾精度和任务时限；
  *   · 不再学模板，第一声 chirp 就可以出结果；
  *   · 108MHz / 96kHz = 1125，每个采样点正好 1125 个 TIM5 计数，换算无舍入；
  *   · TIM2 与 TIM5 同挂 APB1、同源，采样网格相对时间基准永不漂。
  ******************************************************************************
  */
#include "mic.h"
#include "mic_clap.h"
#include "disp.h"
#include "main.h"
#include "tim.h"      /* lt_now_cnt() */
#include "adc.h"      /* hadc3 */
#include "usart.h"    /* huart1 */
#include "mic_chirp_ref.h"
#include "cmsis_os.h"
#include "lwip/opt.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

extern int32_t lt_phase_snapshot_us(void);
extern int32_t lt_acoustic_phase_snapshot_us(void);
extern uint8_t lt_mic_residual_at(uint32_t event_us, int32_t event_phase,
                                int32_t *offset, uint32_t *age, uint32_t *used);
static uint32_t mic_pend_subcnt;
static volatile int32_t mic_epoch_phase_us[2];
volatile uint32_t mic_event_raw_t_us;
volatile int32_t mic_event_sync_offset_us;
volatile uint32_t mic_event_sync_age_ms;
volatile uint8_t mic_event_sync_valid;
volatile uint32_t mic_event_sync_samples;

/* ---- 采样缓冲 --------------------------------------------------------------
   必须放在 DMA 能访问的区域。STM32F7 的 DMA 够不着 DTCM（0x2000'0000~0x2000'FFFF），
   而 IAR 默认会把 readwrite 数据从 0x2000'0000 开始放置。本项目固定占用：

     · LCD 帧缓冲：0x2002'0000~0x2003'FDFF，链接器普通 RAM 截止于 0x2001'FFFF；
     · 音频事件缓冲：0x2004'0000~0x2004'3EFF；
     · 以太网 DMA 描述符：0x2004'C000~0x2004'C13F。

   DMA 缓冲取尾部 4KB（0x2004'd000~0x2004'dfff）：既避开普通数据，也不与
   以太网描述符重叠；SRAM2 本来就被 ST 用作 DMA 区，DMA 访问没有问题。

   ！改地址后的唯一判据：编译完查 .map，确认 mic_buf 落在预期地址且没有重叠。
   写法用 __no_init + 绝对地址（与 ST 的 ethernetif.c 同一套做法），
   地址必须与 mic.h 里的 MIC_BUF_ADDR 一致。
   -------------------------------------------------------------------------- */
#pragma location=0x2004D000
__no_init volatile uint16_t mic_buf[MIC_BUF_SAMPLES];

/* ---- 中断里写的量：越少越好 ------------------------------------------------ */
static volatile uint64_t mic_epoch_cnt[2];  /* [0]=索引0的计数时刻 [1]=索引1024的 */
static volatile uint32_t mic_seq  = 0u;     /* 每完成半个缓冲 +1 */
static volatile uint8_t  mic_done = 0u;     /* 刚完成的是第几半：0 / 1 */

/* ---- 任务侧状态 ------------------------------------------------------------ */
static uint32_t mic_seen_seq = 0u;
static uint16_t mic_work[MIC_HALF_SAMPLES]; /* 副本：处理时 DMA 正在写另一半 */
static int32_t  mic_env[MIC_HALF_SAMPLES];  /* 包络，单位"码" */

static volatile uint32_t mic_last_evt_ms = 0u; /* 麦克风和同步任务共享 */
static uint8_t  mic_armed    = 1u;          /* 1 = 可以接收下一个"起音" */
static uint8_t  mic_edge_ok  = 1u;          /* V5：1 = 包络已回落过门限，才允许下一次起音 */
static uint32_t mic_quiet_ms = 0u;          /* 本次"安静"开始计时的时刻 */
static int32_t  mic_nfar     = 2;           /* 环境包络的慢速均值（触发门限基准） */
static int32_t  mic_npk      = 4;           /* 环境包络的慢速峰值（触发门限基准） */

/* ---- 宽带包络：48 点梳状滑动平均（只负责触发和事件窗口） -------------------
   1 kHz 整流后的主要纹波在 2 kHz。96 kHz 下 48 点窗口在 2 kHz 处为零点，
   因此能得到平滑的起音沿；它的窗口短（0.5 ms）。V6.0.2 起包络不再参与
   到达时刻判决，只用来触发事件和判断声音结束，最终时刻由完整 chirp
   匹配滤波给出。
   -------------------------------------------------------------------------- */
#define MIC_ENV_AVG_LEN   48u

static int32_t  mic_dc_q8 = (2048 << 8);
static uint16_t mic_abs_hist[MIC_ENV_AVG_LEN];
static uint32_t mic_abs_sum = 0u;
static uint8_t  mic_abs_idx = 0u;

/* ---- 预触发历史 + 确认触发 ------------------------------------------------
   低门限在弱信号下更容易触发，但单点越限也可能来自噪声。这里要求连续
   MIC_TRIG_CONFIRM 个包络点都超过门限才承认起音，并保留触发前 32 ms
   的包络和原始样本。远端低频起音较弱时可能晚触发，预触发要覆盖它。
   -------------------------------------------------------------------------- */
#define MIC_PRE_SAMPLES   3072u  /* 32 ms @ 96 kHz；覆盖弱信号晚触发 */
#define MIC_TRIG_CONFIRM  64u    /* 0.667 ms 连续确认。包络是 48 点滑动平均，
                                    单个尖峰也可能令包络连续越限约 48 点；
                                    原来的 16 点不能排除这种静默误触发。
                                     chirp 持续 50 ms，确认延迟仍由 32 ms
                                    预触发历史覆盖。 */
#define MIC_HIST_LEN      4096u  /* 42.67 ms，覆盖预触发 + 确认段 */

static int16_t  mic_hist[MIC_HIST_LEN];
static uint16_t mic_hist_idx = 0u;
static uint32_t mic_hist_fill = 0u;
static int32_t  mic_prev_mx = 0;          /* 上一个半缓冲的包络最大值 */
static int32_t  mic_prev_mean = 0;        /* 上一个半缓冲的平均包络 */
static uint32_t mic_pend_half_id = 0xFFFFFFFFu;

/* ---- 事件窗口：独立 84 ms 缓冲 -------------------------------------------
   标准音频是 50 ms chirp，原来的 20 ms 窗口只能截到前 40%，后面的扫频段
   全部丢掉；模板又只有 2 ms，频率一变就对不上。远端预触发扩到 32 ms 后，
   采集窗口取 84 ms，保证完整 50 ms chirp 仍有至少 2 ms 尾部余量。 */
#define MIC_PEND_LEN   8064u                 /* 84 ms @ 96 kHz */
static uint32_t mic_pend_pk = 0u;           /* 只保留包络峰值，省去整段包络数组 */
static uint16_t mic_pend_n   = 0u;           /* 已收集点数 */
static uint32_t mic_pend_us  = 0u;           /* 触发窗口起点的粗时刻（µs） */
/* Lock the selected speed at the candidate trigger. Zero means AUTO is waiting. */
static uint32_t mic_pend_speed_x1000;
static int32_t mic_pend_phase_start_us = 0;   /* 触发时的同步拨钟快照 */
static uint32_t mic_pend_ms  = 0u;           /* 触发时刻（ms），用于 2s 超时 */
static volatile uint8_t mic_active = 0u;     /* 1 = 正在跟踪一次声音 */

/* ---- V6.0.2：完整 50 ms chirp 归一化匹配滤波 --------------------------------
   参考波形取自标准音频 chirp.wav 的 0.500~0.550 s，48 kHz 源，2:1 抽取到
   24 kHz，共 2400 点（见 mic_chirp_ref.h；实际数组是 24 kHz 的 1200 点）。
   板子 ADC 是 96 kHz，采集后 4:1 抽取到 24 kHz，再做：

       r(o) = Σ x[o+i]·ref[i] / sqrt(Σ x[o+i]² · Σ ref[i]²)

   x 在抽取时先扣掉预触发段的直流；ref 在生成时已经去均值。这样与幅度、
   ADC 直流和手机音量无关，相关峰的位置就是 chirp 起点。

   为什么不用模板：
     · 50 ms 扫频覆盖约 0.5~3.5 kHz，2 ms 模板只是其中一小段；
     · 模板会随两板 AGC、反射和触发位置漂移，甚至两板学到不同片段；
     · 完整 chirp 的时宽带宽积约 150，相关峰尖锐且没有 1 ms 周期模糊。

   能量窗口用滚动更新，避免每个 lag 再对 1200 点求一次平方和；串口 [MIC]
   的 rq/ct 用于验证弱信号相关值和计算耗时。 */
#define MIC_CHIRP_DECIM    4u                              /* 96 kHz -> 24 kHz */
#define MIC_CHIRP_X_N      (MIC_PEND_LEN / MIC_CHIRP_DECIM) /* 最多 2016 点 */
#define MIC_CHIRP_MIN_R    0.22f
#define MIC_CHIRP_PEER_MIN_R 0.13f
/* Max raw TOA difference for the 2.5 m test setup. */
#define MIC_TDOA_VALID_US 7353u
#define MIC_PEER_MARGIN_US 100u  /* Clock residual and 24 kHz sample rounding. */
#define MIC_PEER_SEARCH_US (MIC_TDOA_VALID_US + MIC_PEER_MARGIN_US)
#define MIC_CHIRP_ANCHOR_Q 500u
#define MIC_PEER_HISTORY_LOW_BLOCKS 32u
#define MIC_PEER_HISTORY_HIGH_BLOCKS 15u
#define MIC_PEER_HISTORY_BLOCKS (MIC_PEER_HISTORY_LOW_BLOCKS + MIC_PEER_HISTORY_HIGH_BLOCKS)
#define MIC_PEER_BLOCK_SAMPLES (MIC_HALF_SAMPLES / MIC_CHIRP_DECIM)
#define MIC_PEER_WINDOW_SAMPLES (MIC_CHIRP_REF_N + (2u * MIC_PEER_SEARCH_US * 24u + 999u) / 1000u + 1u)
#define MIC_PEER_WINDOW_END_US (MIC_PEER_SEARCH_US + 50000u)
#define MIC_ANALYSIS_READY 1u

typedef enum {
  MIC_CAPTURE_IDLE,
  MIC_CAPTURE_RECORDING,
  MIC_CAPTURE_QUEUED
} mic_capture_state_t;

typedef struct {
  uint32_t seq, time_us, speed_x1000;
  int32_t phase_us;
  uint16_t subcnt;
} mic_peer_block_t;

static volatile mic_capture_state_t mic_capture_state;
static osThreadId_t mic_analysis_thread;
static volatile uint8_t mic_pend_from_peer;
static uint32_t mic_pend_peer_seq, mic_pend_peer_us;
static uint32_t mic_peer_attempted_seq, mic_peer_attempted_us;
static uint8_t mic_peer_attempted;

static mic_peer_block_t mic_peer_blocks[MIC_PEER_HISTORY_BLOCKS];
static uint8_t mic_peer_next, mic_peer_count;
/* About 501 ms of history, split around the fixed LwIP heap.
   The extra space after MEM_SIZE leaves room for the heap's end header. */
#if (0x20044000u + MIC_PEER_HISTORY_LOW_BLOCKS * MIC_PEER_BLOCK_SAMPLES * 2u > LWIP_RAM_HEAP_POINTER) || \
    (LWIP_RAM_HEAP_POINTER + MEM_SIZE + 0x100u > 0x2004A200u) || \
    (0x2004A200u + MIC_PEER_HISTORY_HIGH_BLOCKS * MIC_PEER_BLOCK_SAMPLES * 2u > 0x2004C000u)
#error "Microphone history overlaps the LwIP heap or ETH DMA descriptors"
#endif
#pragma location=0x20044000
__no_init uint16_t mic_peer_samples_low[MIC_PEER_HISTORY_LOW_BLOCKS][MIC_PEER_BLOCK_SAMPLES];
#pragma location=0x2004A200
__no_init uint16_t mic_peer_samples_high[MIC_PEER_HISTORY_HIGH_BLOCKS][MIC_PEER_BLOCK_SAMPLES];

static uint16_t *mic_peer_sample_block(uint32_t slot)
{
  if (slot < MIC_PEER_HISTORY_LOW_BLOCKS) { return mic_peer_samples_low[slot]; }
  return mic_peer_samples_high[slot - MIC_PEER_HISTORY_LOW_BLOCKS];
}

static float mic_chirp_x[MIC_CHIRP_X_N];   /* 抽取后的接收信号，已去直流 */
static float mic_chirp_energy[MIC_CHIRP_X_N - MIC_CHIRP_REF_N + 1u];
static float mic_chirp_ref_ss = 0.0f;      /* Σ ref²，归一化用 */

/* 以下三个变量保留给屏幕/串口兼容；V6.0.2 不再学模板，固定 valid=1。 */
volatile uint8_t  mic_tpl_valid = 1u;
volatile uint32_t mic_tpl_learn_cnt = 0u;
volatile uint32_t mic_tpl_learn_pk  = 0u;

static int16_t  mic_rawring[MIC_HIST_LEN];   /* 原始样本的预触发历史，与 mic_hist 逐点同步压入 */
/* LCD 帧缓冲占 0x20020000~0x2003FDFF；此数组放在其后、以太网描述符之前。
   触发时先写预触发段，之后持续写到 mic_pend_n；匹配只读已写入的部分。 */
#pragma location=0x20040000
__no_init int16_t mic_pend_raw[MIC_PEND_LEN];

static int32_t mic_min_code = 0, mic_max_code = 0;

volatile uint32_t mic_event_t_us  = 0u;
volatile uint32_t mic_event_seq   = 0u;   /* 只统计"chirp 匹配合格并发布"的事件 */
volatile uint32_t mic_last_pk     = 0u;
volatile uint32_t mic_last_corr_q = 0u;   /* 最近一次发布事件的相关系数 ×1000 */
volatile uint32_t mic_event_pos256 = 0u;
volatile int32_t mic_event_phase_start_us = 0;
volatile int32_t mic_event_phase_end_us = 0;
volatile uint32_t mic_match_best_q = 0u;  /* 最近一次匹配尝试的峰值 ×1000，失败也保留 */
volatile uint32_t mic_match_short_cnt = 0u;  /* 收集不足 50 ms 的次数 */
volatile uint32_t mic_match_reject_cnt = 0u; /* 相关系数未过门限的次数 */
volatile uint32_t mic_weak_chirp_cnt = 0u;
volatile uint32_t mic_peer_attempt_cnt = 0u;
volatile uint32_t mic_peer_gap_cnt = 0u;
volatile uint32_t mic_match_cost_us = 0u;   /* 最近一次匹配计算耗时 */
volatile uint32_t mic_candidate_pk = 0u;
volatile uint32_t mic_candidate_trig = 0u;
volatile uint32_t mic_last_nf     = 0u;
volatile uint32_t mic_last_trig   = 0u;
volatile uint32_t mic_overruns    = 0u;   /* 任务没赶上、被 DMA 覆盖的半缓冲数 */

/* 验收/排查用：把"触发链路"每一段的实时状态露出来（串口 [MIC] 行）。
   卡点一眼就能定位：mx 上不去 = 声音没到麦克风；mx 上去了 tg 不动 = 包络
   门限没连续确认满；tg 动了 ev 不动 = chirp 相关不合格；ev 动了
   rx 不动 = TOA 报文没收到；rx 动了 pr 不动 = 配对窗口内没有本板事件。 */
volatile uint32_t mic_trig_cnt     = 0u;   /* 包络"连续确认"通过的次数（进状态机） */
volatile uint32_t mic_env_mx_live  = 0u;   /* 上一半缓冲的包络最大值（实时电平） */
volatile uint32_t mic_td_rej_far   = 0u;   /* 配对差值超物理上限被否的次数 */
volatile uint32_t mic_tdoa_nomatch = 0u;   /* 收报文时本板窗口内没有可配的事件 */

/* ---- 中断回调：由 stm32f7xx_it.c 的 DMA2_Stream0_IRQHandler 调用 -------------
   TIM2 每 1125 计数触发一个 ADC 样本，但 DMA 中断可被其它中断延迟。
   因此不能把中断执行时刻当成半缓冲起点；用 TIM2 相位和 DMA NDTR
   推回下一半首个样本的硬件触发时刻。half=0 -> [1]，half=1 -> [0]。
   只改变声到达时刻的采样锚点，不改变 TIM5 或同步伺服。 */
void mic_isr_half(uint8_t half)
{
  uint32_t attempt;
  uint32_t phase0 = 0u, phase1 = 0u, ndtr0 = 0u, ndtr1 = 0u;
  uint32_t samples_after = 0u;
  uint32_t current_triggered = 0u;
  uint64_t now_cnt = 0u;
  uint8_t valid = 0u;
  int32_t clock_phase0=0, clock_phase1=0;

  /* ADC 采样 144 周期加转换约 6 us。周期前段本次触发尚未完成 DMA，
     后段已经完成；仅在转换边界附近短暂重试，避免长期占用高优先级中断。 */
  for (attempt = 0u; attempt < 1000u; attempt++)
  {
    clock_phase0=lt_acoustic_phase_snapshot_us();
    phase0 = TIM2->CNT;
    if ((phase0 >= 550u) && (phase0 < 700u)) { continue; }
    ndtr0 = DMA2_Stream0->NDTR;
    now_cnt = lt_now_cnt();
    phase1 = TIM2->CNT;
    clock_phase1=lt_acoustic_phase_snapshot_us();
    ndtr1 = DMA2_Stream0->NDTR;
    if ((clock_phase0 == clock_phase1) && (phase1 >= phase0) && (ndtr0 == ndtr1) &&
        (((phase0 < 550u) && (phase1 < 550u)) ||
         ((phase0 >= 700u) && (phase1 >= 700u))))
    {
      current_triggered = (phase1 < 550u) ? 1u : 0u;
      valid = 1u;
      break;
    }
  }

  if (valid != 0u)
  {
    if (half == 0u)
    {
      if ((ndtr0 <= MIC_HALF_SAMPLES) && (ndtr0 != 0u))
      {
        samples_after = MIC_HALF_SAMPLES - ndtr0;
      }
      else { valid = 0u; }
    }
    else if ((ndtr0 > MIC_HALF_SAMPLES) && (ndtr0 <= MIC_BUF_SAMPLES))
    {
      samples_after = MIC_BUF_SAMPLES - ndtr0;
    }
    else if (ndtr0 == 0u) { samples_after = 0u; }
    else { valid = 0u; }
  }

  if (valid != 0u)
  {
    /* 前段的 ADC 还没搬入 DMA，NDTR 少计了已触发的当前样本。 */
    samples_after += current_triggered;
    int64_t delta = ((int64_t)1 - (int64_t)samples_after) * MIC_CNT_PER_SMP
                  - (int64_t)((phase0 + phase1) / 2u);
    int64_t epoch = (int64_t)now_cnt + delta;
    mic_epoch_cnt[half ^ 1u] = (epoch >= 0) ? (uint64_t)epoch : 0u;
  }
  else
  {
    /* 异常 DMA 状态下沿用原时间戳，避免制造一个整半缓冲的错位。 */
    do {
      clock_phase0=lt_acoustic_phase_snapshot_us();
      now_cnt=lt_now_cnt();
      clock_phase1=lt_acoustic_phase_snapshot_us();
    } while (clock_phase0!=clock_phase1);
    mic_epoch_cnt[half ^ 1u] = now_cnt;
  }
  mic_epoch_phase_us[half ^ 1u] = clock_phase1;
  mic_done = half;
  mic_seq++;
}

uint8_t mic_quiet_ok(void)
{
  /* 到达时刻尚未算出时禁止拨钟；算完后不必等待事件状态机的静默/超时。 */
  if ((mic_capture_state != MIC_CAPTURE_IDLE) && (mic_pend_from_peer == 0u)) { return 0u; }
  return ((HAL_GetTick() - mic_last_evt_ms) > 50u) ? 1u : 0u;
}

int32_t mic_last_min(void) { return mic_min_code; }
int32_t mic_last_max(void) { return mic_max_code; }

/* ---- V6.0.2：完整 chirp 匹配滤波 -------------------------------------------- */

/* 初始化参考能量。参考数组来自 mic_chirp_ref.h（24 kHz、1200 点、已去均值）。 */
static void mic_chirp_ref_init(void)
{
  uint32_t i;
  float    ss = 0.0f;

  for (i = 0u; i < MIC_CHIRP_REF_N; i++)
  {
    float v = (float)mic_chirp_ref[i];

    ss += v * v;
  }

  mic_chirp_ref_ss  = ss;
  mic_tpl_valid     = (ss > 0.0f) ? 1u : 0u;
  mic_tpl_learn_cnt = 0u;
  mic_tpl_learn_pk  = 0u;
}

/* 96 kHz 原始采集 4:1 抽取到 24 kHz，并扣掉预触发段的直流。
   预触发段是触发点前 32 ms，正常情况下大部分是静音；即使远端起音晚了、
   这一段混入一点 chirp，chirp 本身是零均值的，直流估计仍然可用。
   返回抽取后的点数；不够一个参考长度就返回 0。 */
static uint32_t mic_chirp_decimate(void)
{
  uint32_t i, j, n, pre;
  int64_t  sum = 0;
  int32_t  dc;
  int32_t  s;

  n = (uint32_t)mic_pend_n / MIC_CHIRP_DECIM;
  if (n < MIC_CHIRP_REF_N) { return 0u; }

  pre = ((uint32_t)mic_pend_n > MIC_PRE_SAMPLES)
      ? MIC_PRE_SAMPLES : (uint32_t)mic_pend_n;

  for (i = 0u; i < pre; i++)
  {
    sum += (int32_t)mic_pend_raw[i];
  }
  dc = (int32_t)((sum + ((int64_t)pre / 2)) / (int64_t)pre);

  for (i = 0u; i < n; i++)
  {
    s = 0;
    for (j = 0u; j < MIC_CHIRP_DECIM; j++)
    {
      s += (int32_t)mic_pend_raw[(i * MIC_CHIRP_DECIM) + j];
    }
    mic_chirp_x[i] = ((float)s / (float)MIC_CHIRP_DECIM) - (float)dc;
  }

  return n;
}

/* 各 lag 的接收能量只需一次 1200 点求和，后续每移动一格进一格、出一格。 */
static void mic_chirp_energy_prepare(uint32_t max_lag)
{
  uint32_t i, o;
  float    energy = 0.0f;

  for (i = 0u; i < MIC_CHIRP_REF_N; i++)
  {
    float v = mic_chirp_x[i];
    energy += v * v;
  }
  mic_chirp_energy[0] = energy;
  for (o = 1u; o <= max_lag; o++)
  {
    float leaving = mic_chirp_x[o - 1u];
    float entering = mic_chirp_x[o + MIC_CHIRP_REF_N - 1u];
    energy += (entering * entering) - (leaving * leaving);
    if (energy < 0.0f) { energy = 0.0f; }
    mic_chirp_energy[o] = energy;
  }
}

/* 相关系数平方乘参考能量：dot² / Ex。比较峰值无需每个 lag 开方。 */
static float mic_chirp_score_at(uint32_t o)
{
  uint32_t i;
  float dot = 0.0f;
  float ex = mic_chirp_energy[o];

  if (ex <= 1.0e-6f) { return 0.0f; }
  for (i = 0u; i < MIC_CHIRP_REF_N; i++)
  {
    float xv = mic_chirp_x[o + i];

    dot += xv * (float)mic_chirp_ref[i];
  }
  return (dot * dot) / ex;
}

static uint32_t mic_chirp_peak_pos256(uint32_t best, uint32_t max_lag, float r1)
{
  float r0, r2, den, xoff, pos;
  r0 = (best > 0u)      ? sqrtf(mic_chirp_score_at(best - 1u) / mic_chirp_ref_ss) : r1;
  r2 = (best < max_lag) ? sqrtf(mic_chirp_score_at(best + 1u) / mic_chirp_ref_ss) : r1;

  den  = (r0 - (2.0f * r1)) + r2;
  xoff = 0.0f;
  if (den < -1.0e-6f)
  {
    xoff = (0.5f * (r0 - r2)) / den;
    if (xoff >  0.5f) { xoff =  0.5f; }
    if (xoff < -0.5f) { xoff = -0.5f; }
  }

  pos = (float)best + xoff;
  if (pos < 0.0f) { pos = 0.0f; }
  return (uint32_t)((pos * 256.0f) + 0.5f);

}

/* 在整段 84 ms 采集里搜完整 chirp。成功返回 1，并给出：
     pos256_out：chirp 起点相对采集窗口起点的位置，Q8（24 kHz 样本）
     r_out     ：峰值归一化相关系数
   相关峰用三点抛物线插值到亚样本；找不到足够强的峰就不发布。 */
static uint8_t mic_chirp_match(uint32_t *pos256_out, float *r_out, uint32_t n,
                               float threshold)
{
  uint32_t max_lag, o, best = 0u, first = 0u, lo, hi;
  uint8_t  have_first = 0u;
  float    score, best_score = -1.0f, min_score;

  *pos256_out = 0u;
  *r_out      = 0.0f;

  if (n < MIC_CHIRP_REF_N) { return 0u; }
  if (mic_chirp_ref_ss <= 0.0f) { return 0u; }

  max_lag = n - MIC_CHIRP_REF_N;
  mic_chirp_energy_prepare(max_lag);
  min_score = threshold * threshold * mic_chirp_ref_ss;

  /* 粗搜步长 2：24 kHz 下 chirp 相关主瓣宽约 4 个样本，步长 2 不会
     跳过主瓣；找到粗峰后再逐点细化 ±2。计算量约为全逐点搜索的一半，
     给 216 MHz 的 F746 留出半缓冲余量。 */
  for (o = 0u; o <= max_lag; o += 2u)
  {
    /* 取相关值的绝对值：两块板的麦克风/模拟前端极性可能相反，
       固定参考波形不能用“只找正峰”的写法。极性只影响符号，
       不影响到达时刻和 TDOA。 */
    score = mic_chirp_score_at(o);
    if ((have_first == 0u) && (score >= min_score))
    {
      first = o;
      have_first = 1u;
    }
    if (score > best_score)
    {
      best_score = score;
      best   = o;
    }
  }

  /* 到达时刻取最早的合格 chirp，避免后续较强反射抢走全局最高峰。
     首次越限可能落在主峰前约 5 个样本的旁瓣，细搜 ±8 点以回到主峰。 */
  if (have_first != 0u)
  {
    best = first;
    best_score = mic_chirp_score_at(best);
    lo = (best > 8u) ? (best - 8u) : 0u;
    hi = best + 8u;
  }
  else
  {
    lo = (best > 2u) ? (best - 2u) : 0u;
    hi = best + 2u;
  }
  if (hi > max_lag) { hi = max_lag; }
  for (o = lo; o <= hi; o++)
  {
    score = mic_chirp_score_at(o);
    if (score > best_score)
    {
      best_score = score;
      best   = o;
    }
  }

  *r_out = sqrtf(best_score / mic_chirp_ref_ss);
  if (*r_out < threshold) { return 0u; }

  *pos256_out = mic_chirp_peak_pos256(best, max_lag, *r_out);

  return 1u;
}

/* ---- 跨板 TDOA：两块板互相报到达时刻，在这里配成一对 ------------------------
    两块板跑同一份固件：每发布一个分类合格的事件，就把（事件序号, 到达时刻, 类型）
   用一条新报文告诉对方；收到报文的板子在"本板最近 8 次事件"里找窗口内最接近
   的一次，配成一对，TDOA = 本板时刻 − 对方时刻，屏幕上的 dj 就是验收量 ——
   最近 10 次配对里相邻两次 |ΔTDOA| 的最大值。

   窗口取 ±50 ms：手机 5 s 才响一声，窗口里只可能有一次事件，配不错；两边相差
   超过窗口（例如有一板根本没听见）就干脆不配，宁可少一次也不凑数。

   扫频与拍手只在同类型的事件环内配对，分别保存结果和统计。

   这里只做减法、比较和统计，不碰时间基准本身（那是第一步同步的职责）。
   两板已经同步到 ≤10 µs，两个本地时刻可以直接相减。
   -------------------------------------------------------------------------- */
#define MIC_TDOA_RING    8u        /* 记住本板最近 8 次事件 */
#define MIC_TDOA_STAT    10u       /* 验收量的统计窗口 */
#define MIC_TDOA_DIAG_US 50000u  /* Count rejected nearby events in far. */
#define MIC_TDOA_STABLE_US 100u  /* 固定声源的单次跳变限值，约 3.4 cm @343 m/s */
#define MIC_TDOA_NEW_US     60u  /* 新位置连续候选的相互容差，约 2.1 cm */
#define MIC_TDOA_NEW_COUNT   3u  /* 连续 3 次才切换位置，单次反射不污染历史 */

/* 202.7 cm calibration with the source outside each end of the baseline.
   A-side mean: 201.94 cm at 21.7 C; B-side mean: 201.55 cm at 21.4 C.
   Remove the signed local bias, then subtract the range bias from the magnitude. */
#define MIC_CHIRP_LOCAL_BIAS_US (3)
#define MIC_CHIRP_RANGE_BIAS_US (-9)
#define MIC_CHIRP_LOCAL_SIGN (1)

static mic_chirp_cal_t mic_chirp_cal = {
  MIC_CHIRP_LOCAL_SIGN * MIC_CHIRP_LOCAL_BIAS_US, MIC_CHIRP_RANGE_BIAS_US
};

#define MIC_CLAP_TDOA_STABLE_US 800u
#define MIC_CLAP_TDOA_NEW_US    400u
#define MIC_CLAP_DIRECTION_MIN_US 400  /* Ignore near-zero sign flips for fast switching. */

/* Keep matching, position filtering and saved distances separate by type. */
typedef struct
{
  uint32_t local_seq[MIC_TDOA_RING], local_us[MIC_TDOA_RING];
  uint32_t local_speed[MIC_TDOA_RING];
  mic_signal_quality_t local_quality[MIC_TDOA_RING];
  uint8_t local_count, local_next;
  uint32_t anchor_us;
  uint8_t anchor_valid;
  uint32_t peer_seq, peer_us;
  uint8_t peer_pending;
  uint32_t used_peer_seq, used_local_seq;
  uint32_t times_us[MIC_TDOA_STAT], distances_mm[MIC_TDOA_STAT];
  uint8_t history_count, history_next;
  uint8_t position_count;  /* Recent pairs used for the current position reference. */
  uint32_t time_count, distance_count;
  double time_mean, time_m2, distance_mean, distance_m2;
  int32_t new_center_us;
  uint8_t new_count;
  uint32_t last_local_us, last_peer_us, last_pair_seq, dj_us;
  int32_t last_diff_us;
} mic_td_state_t;

static mic_td_state_t mic_td_states[2];

#define MIC_QUALITY_QUEUE_SIZE 8u
static mic_pair_quality_t mic_quality_queue[MIC_QUALITY_QUEUE_SIZE];
static uint8_t mic_quality_head, mic_quality_count;
volatile uint32_t mic_tdoa_quality_dropped;

uint8_t mic_tdoa_quality_pop(mic_pair_quality_t *result)
{
  uint8_t ready;

  taskENTER_CRITICAL();
  ready = (uint8_t)(mic_quality_count != 0u);
  if (ready != 0u)
  {
    *result = mic_quality_queue[mic_quality_head];
    mic_quality_head = (uint8_t)((mic_quality_head + 1u) % MIC_QUALITY_QUEUE_SIZE);
    mic_quality_count--;
  }
  taskEXIT_CRITICAL();
  return ready;
}
static uint8_t mic_td_active_kind;
static uint32_t mic_td_display_from_seq;
static uint8_t mic_have_clap;
static uint32_t mic_last_clap_ms;
volatile uint8_t mic_event_kind, mic_tdoa_kind;

static mic_td_state_t *mic_td_state(uint8_t kind)
{
  switch (kind)
  {
    case MIC_EVENT_CHIRP: return &mic_td_states[0];
    case MIC_EVENT_CLAP:  return &mic_td_states[1];
    default:              return NULL;
  }
}

volatile int32_t  mic_tdoa_us    = 0;         /* 最近一次配对成功的 TDOA */
volatile uint32_t mic_tdoa_dj_us = 0u;        /* 最近 10 次相邻 |ΔTDOA| 最大值 */
volatile uint32_t mic_tdoa_pairs = 0u;        /* 配对成功次数（累计） */
volatile uint32_t mic_tdoa_rxn   = 0u;        /* 收到对方报文条数（累计） */
volatile uint32_t mic_td_rej_jump = 0u;       /* 与稳定位置不符而丢弃的配对 */

void mic_chirp_cal_get(mic_chirp_cal_t *cal)
{
  taskENTER_CRITICAL();
  *cal = mic_chirp_cal;
  taskEXIT_CRITICAL();
}

void mic_chirp_cal_apply(const mic_chirp_cal_t *cal)
{
  taskENTER_CRITICAL();
  if ((cal->ab_bias_us != mic_chirp_cal.ab_bias_us) ||
      (cal->range_bias_us != mic_chirp_cal.range_bias_us))
  {
    mic_td_state_t *chirp = mic_td_state(MIC_EVENT_CHIRP);
    uint32_t used_peer_seq = chirp->used_peer_seq;

    mic_chirp_cal = *cal;
    /* Start a fresh CHIRP history and SD under the new calibration. */
    (void)memset(chirp, 0, sizeof(*chirp));
    chirp->used_peer_seq = used_peer_seq;
  }
  taskEXIT_CRITICAL();
}

/* Wrapped subtraction gives the signed local-minus-peer difference. */
static int32_t mic_td_corrected_us(uint32_t tl, uint32_t tp, uint8_t kind)
{
  int32_t dt_raw = (int32_t)(tl - tp);
  int32_t range_bias;
  if (kind != MIC_EVENT_CHIRP) { return dt_raw; }

  dt_raw -= MIC_CHIRP_LOCAL_SIGN * mic_chirp_cal.ab_bias_us;
  range_bias = mic_chirp_cal.range_bias_us;
  if (dt_raw > 0)
  {
    return (dt_raw > range_bias) ? dt_raw - range_bias : 0;
  }
  if (dt_raw < 0)
  {
    return (dt_raw < -range_bias) ? dt_raw + range_bias : 0;
  }
  return 0;
}

static void mic_td_commit(mic_td_state_t *state, uint8_t slot, uint32_t pseq,
                          uint32_t tp, int32_t dt_us, uint8_t kind, uint8_t new_distance)
{
  uint32_t lseq = state->local_seq[slot];
  uint32_t tl = state->local_us[slot];
  uint32_t speed_x1000 = state->local_speed[slot];
  uint32_t i, n, mx = 0u;
  mic_pair_quality_t *quality;
  uint32_t distance_mm = MIC_DISTANCE_UNAVAILABLE;

  if (new_distance != 0u)
  {
    state->distance_count = 0u;
    state->distance_mean = 0.0;
    state->distance_m2 = 0.0;
  }

  mic_tdoa_us      = dt_us;
  state->last_local_us  = tl;
  state->last_peer_us  = tp;
  state->used_local_seq     = lseq;
  state->used_peer_seq      = pseq;
  state->last_pair_seq = lseq;
  state->last_diff_us = dt_us;
  if (state->time_count != 0xFFFFFFFFu)
  {
    double x = (dt_us < 0) ? -(double)dt_us : (double)dt_us;
    double delta;
    state->time_count++;
    delta = x - state->time_mean;
    state->time_mean += delta / (double)state->time_count;
    state->time_m2 += delta * (x - state->time_mean);
  }

  if (speed_x1000 != 0u)
  {
    uint32_t magnitude_us = (uint32_t)((dt_us < 0) ? -dt_us : dt_us);
    uint64_t scaled = (uint64_t)magnitude_us * speed_x1000;
    distance_mm = (uint32_t)((scaled + 500000u) / 1000000u);
    if (state->distance_count != UINT32_MAX)
    {
      double exact_mm = (double)scaled / 1000000.0;
      double delta = exact_mm - state->distance_mean;
      state->distance_count++;
      state->distance_mean += delta / (double)state->distance_count;
      state->distance_m2 += delta * (exact_mm - state->distance_mean);
    }
  }

  state->distances_mm[state->history_next] = distance_mm;
  state->times_us[state->history_next] = (uint32_t)dt_us;
  state->history_next = (uint8_t)((state->history_next + 1u) % MIC_TDOA_STAT);
  if (state->history_count < MIC_TDOA_STAT) { state->history_count++; }
  if (state->position_count < 5u) { state->position_count++; }

  /* 按时间顺序排好，相邻两次作差，取最大值（含跨过环尾的那一对） */
  n = state->history_count;
  for (i = 1u; i < n; i++)
  {
    uint8_t  a = (uint8_t)((state->history_next + MIC_TDOA_STAT + i - 1u - n) % MIC_TDOA_STAT);
    uint8_t  b = (uint8_t)((state->history_next + MIC_TDOA_STAT + i - n) % MIC_TDOA_STAT);
    /* 差值是有符号的（本板−对板），环里存的是补码：先转回 int64 再作差。
       否则跨过 0 的相邻两条会被当成"相差 2^32"，现场 dj 显示
       4294967197（= 2^32−99）就是这个 bug。 */
    int64_t  av = (int64_t)(int32_t)state->times_us[a];
    int64_t  bv = (int64_t)(int32_t)state->times_us[b];
    uint32_t d  = (uint32_t)((av > bv) ? (av - bv) : (bv - av));

    if (d > mx) { mx = d; }
  }
  state->dj_us = mx;
  mic_tdoa_dj_us = mx;
  mic_tdoa_kind = kind;
  mic_tdoa_pairs++;

  if (mic_quality_count == MIC_QUALITY_QUEUE_SIZE)
  {
    mic_quality_head = (uint8_t)((mic_quality_head + 1u) % MIC_QUALITY_QUEUE_SIZE);
    mic_quality_count--;
    mic_tdoa_quality_dropped++;
  }
  quality = &mic_quality_queue[(mic_quality_head + mic_quality_count) % MIC_QUALITY_QUEUE_SIZE];
  quality->pair_seq = mic_tdoa_pairs;
  quality->local_seq = lseq;
  quality->peer_seq = pseq;
  quality->sd_count = state->distance_count;
  quality->distance_mm = distance_mm;
  quality->tdoa_us = dt_us;
  quality->signal = state->local_quality[slot];
  quality->kind = kind;
  quality->new_distance = new_distance;
  mic_quality_count++;
}

/* 两个 32 位微秒时刻的绝对差。先转成有符号再取绝对值，跨过 32 位
   回绕点时仍然是真实的时间差，不会被当成 40 多亿微秒。 */
static uint32_t mic_td_absdiff(uint32_t a, uint32_t b)
{
  int64_t d = (int64_t)(int32_t)(a - b);

  return (uint32_t)((d >= 0) ? d : -d);
}

/* 最近最多 5 个已接受值的中位数作基准；跳变连续出现才视作声源换了位置。
   只筛除明显错峰/反射，不把亚厘米级波动伪装成 2 mm 精度。 */
typedef enum
{
  MIC_PAIR_REJECTED,
  MIC_PAIR_ACCEPTED,
  MIC_PAIR_NEW_DISTANCE
} mic_pair_decision_t;

static mic_pair_decision_t mic_td_stable(int32_t dt_us, uint8_t kind)
{
  mic_td_state_t *state = mic_td_state(kind);
  uint32_t stable_us = (kind == MIC_EVENT_CLAP) ? MIC_CLAP_TDOA_STABLE_US : MIC_TDOA_STABLE_US;
  uint32_t new_us = (kind == MIC_EVENT_CLAP) ? MIC_CLAP_TDOA_NEW_US : MIC_TDOA_NEW_US;
  /* Chirp has a tighter time estimate than a clap. Reuse its existing
     stability limit as the minimum magnitude for a clear side change. */
  int32_t direction_min_us = (kind == MIC_EVENT_CLAP) ?
                             MIC_CLAP_DIRECTION_MIN_US : (int32_t)MIC_TDOA_STABLE_US;
  int32_t v[5], center, diff;
  uint32_t magnitude_us, center_us;
  mic_pair_decision_t new_position;
  uint8_t i, j, n = state->position_count;

  if (n == 0u) { return MIC_PAIR_ACCEPTED; }
  for (i = 0u; i < n; i++)
  {
    uint8_t k = (uint8_t)((state->history_next + MIC_TDOA_STAT - 1u - i) % MIC_TDOA_STAT);
    v[i] = (int32_t)state->times_us[k];
  }
  for (i = 1u; i < n; i++)
  {
    int32_t x = v[i];
    j = i;
    while ((j > 0u) && (v[j - 1u] > x))
    {
      v[j] = v[j - 1u];
      j--;
    }
    v[j] = x;
  }
  center = v[n / 2u];
  /* The displayed distance uses the magnitude. A reversal at the same
     path difference starts a new direction reference but keeps its SD. */
  magnitude_us = (uint32_t)((dt_us < 0) ? -dt_us : dt_us);
  center_us = (uint32_t)((center < 0) ? -center : center);
  new_position = (mic_td_absdiff(magnitude_us, center_us) > stable_us) ?
                 MIC_PAIR_NEW_DISTANCE : MIC_PAIR_ACCEPTED;
  if (((center >= direction_min_us) && (dt_us <= -direction_min_us)) ||
      ((center <= -direction_min_us) && (dt_us >= direction_min_us)))
  {
    /* Opposite arrival order establishes a new side on the first valid pair.
       Keep displayed measurements; the next reference starts at this pair. */
    state->new_count = 0u;
    state->position_count = 0u;
    return new_position;
  }

  diff = dt_us - center;
  if (diff < 0) { diff = -diff; }
  if ((uint32_t)diff <= stable_us)
  {
    state->new_count = 0u;
    return MIC_PAIR_ACCEPTED;
  }

  diff = dt_us - state->new_center_us;
  if (diff < 0) { diff = -diff; }
  if ((state->new_count == 0u) || ((uint32_t)diff > new_us))
  {
    state->new_center_us = dt_us;
    state->new_count = 1u;
  }
  else
  {
    state->new_center_us = (state->new_center_us * (int32_t)state->new_count + dt_us) /
                           (int32_t)(state->new_count + 1u);
    state->new_count++;
  }
  if (state->new_count < MIC_TDOA_NEW_COUNT)
  {
    mic_td_rej_jump++;
    return MIC_PAIR_REJECTED;
  }

  state->new_count = 0u;
  state->position_count = 0u;
  state->history_count = 0u;
  state->history_next = 0u;
  state->dj_us = 0u;
  return new_position;
}

/* 试配一对：窗口内 + 差值不超过物理上限，才承认 */
static void mic_td_try(uint8_t slot, uint32_t pseq, uint32_t tp, uint8_t kind)
{
  mic_td_state_t *state = mic_td_state(kind);
  uint32_t tl = state->local_us[slot];
  uint32_t d = mic_td_absdiff(tl, tp);
  int32_t dt_us;
  mic_pair_decision_t decision;

  if (d > MIC_TDOA_VALID_US)
  {
    if (d <= MIC_TDOA_DIAG_US) { mic_td_rej_far++; }
    return;
  }

  dt_us = mic_td_corrected_us(tl, tp, kind);
  decision = mic_td_stable(dt_us, kind);
  if (decision == MIC_PAIR_REJECTED)
  {
    /* 被拒绝的一对也记为已处理，避免重复 UDP 报文把一次坏值算作多次新位置。 */
    state->used_local_seq = state->local_seq[slot];
    state->used_peer_seq = pseq;
    return;
  }

  mic_td_commit(state, slot, pseq, tp, dt_us, kind, (uint8_t)(decision == MIC_PAIR_NEW_DISTANCE));
}

/* Publish a classified event in its own matching ring. */
static void mic_td_local(uint32_t seq, uint32_t t_us, uint32_t speed_x1000, uint8_t kind,
                         uint8_t anchor, const mic_signal_quality_t *quality)
{
  mic_td_state_t *state = mic_td_state(kind);
  uint8_t slot;

  if (state == NULL) { return; }
  taskENTER_CRITICAL();
  slot = state->local_next;
  if (mic_td_active_kind != kind)
  {
    mic_td_display_from_seq = seq;
    mic_td_active_kind = kind;
  }

  if (anchor != 0u)
  {
    state->anchor_us = t_us;
    state->anchor_valid = 1u;
  }
  state->local_seq[slot] = seq;
  state->local_us [slot] = t_us;
  state->local_speed[slot] = speed_x1000;
  state->local_quality[slot] = *quality;
  state->local_next = (uint8_t)((slot + 1u) % MIC_TDOA_RING);
  if (state->local_count < MIC_TDOA_RING) { state->local_count++; }

  /* 对方的报文先到了：这一拍把另一半补上 */
  if ((state->peer_pending != 0u) && (state->peer_seq != state->used_peer_seq))
  {
    mic_td_try(slot, state->peer_seq, state->peer_us, kind);
    /* An out-of-range peak must leave the peer report available for replay. */
    if (state->peer_seq == state->used_peer_seq) { state->peer_pending = 0u; }
  }

  taskEXIT_CRITICAL();
}

/* 取本板最近一次已发布事件的 (序号, 时刻)，给发送任务用。
   返回 0 = 还没有发布过分类合格的事件。 */
uint8_t mic_tdoa_tx_snap(uint32_t *seq, uint32_t *t_us, uint8_t *kind)
{
  uint8_t ok = 0u;
  const mic_td_state_t *state;

  taskENTER_CRITICAL();
  *kind = mic_td_active_kind;
  state = mic_td_state(*kind);
  if ((state != NULL) && (state->local_count != 0u))
  {
    uint8_t j = (uint8_t)((state->local_next + MIC_TDOA_RING - 1u) % MIC_TDOA_RING);

    *seq  = state->local_seq[j];
    *t_us = state->local_us[j];
    ok = 1u;
  }
  taskEXIT_CRITICAL();

  return ok;
}

/* 一次临界区取得屏幕使用的全部配对数据，避免新配对恰好发生在两次读取之间。 */
double mic_tdoa_all_sample_variance_us2(uint32_t *count)
{
  double variance;
  const mic_td_state_t *state;
  taskENTER_CRITICAL();
  state = mic_td_state(mic_td_active_kind);
  *count = (state != NULL) ? state->time_count : 0u;
  variance = (*count >= 2u) ?
             state->time_m2 / (double)(state->time_count - 1u) : 0.0;
  taskEXIT_CRITICAL();
  return variance;
}

uint8_t mic_tdoa_display_snap(uint32_t *local_us, uint32_t *peer_us,
                              int32_t *dt, uint32_t *distance_mm,
                              uint32_t *history_mm, uint8_t cap,
                              uint8_t *count, uint32_t *dj_us, uint8_t *kind,
                              uint32_t *sd_count, double *variance_mm2)
{
  uint8_t i, n, paired;
  const mic_td_state_t *state;

  taskENTER_CRITICAL();
  *kind = mic_td_active_kind;
  state = mic_td_state(*kind);
  /* The distance and SD come from the same kind and visible pair. */
  paired = (uint8_t)((state != NULL) && (state->history_count != 0u) &&
                    ((int32_t)(state->last_pair_seq - mic_td_display_from_seq) >= 0));
  *sd_count = 0u;
  *variance_mm2 = 0.0;
  if (paired != 0u)
  {
    n = state->history_count;
    if (n > MIC_TDOA_STAT) { n = MIC_TDOA_STAT; }
    if (n > cap) { n = cap; }
    *local_us = state->last_local_us;
    *peer_us = state->last_peer_us;
    *dt = state->last_diff_us;
    *distance_mm = state->distances_mm[(state->history_next + MIC_TDOA_STAT - 1u) % MIC_TDOA_STAT];
    *dj_us = state->dj_us;
    *sd_count = state->distance_count;
    if (*sd_count >= 2u)
    {
      *variance_mm2 = state->distance_m2 / (double)(*sd_count - 1u);
    }
    for (i = 0u; i < n; i++)
    {
      uint8_t k = (uint8_t)((state->history_next + MIC_TDOA_STAT + i - n) % MIC_TDOA_STAT);
      history_mm[i] = state->distances_mm[k];
    }
  }
  else
  {
    n = 0u;
    *dj_us = 0u;
  }
  *count = n;
  taskEXIT_CRITICAL();

  return paired;
}

/* A remote TOA can only match a local event of the same type. */
void mic_tdoa_rx(uint32_t seq, uint32_t t_us, uint32_t kind)
{
  uint8_t  i, best = 0xFFu, far = 0u;
  uint32_t bd = 0xFFFFFFFFu;
  mic_td_state_t *state;

  if ((kind != MIC_EVENT_CHIRP) && (kind != MIC_EVENT_CLAP)) { return; }
  state = mic_td_state((uint8_t)kind);
  taskENTER_CRITICAL();
  mic_tdoa_rxn++;

  if (seq != state->used_peer_seq)          /* 同一条报文只认一次 */
  {
    /* 在本板最近 8 次已发布事件里挑窗口内最接近的一次。 */
    for (i = 0u; i < state->local_count; i++)
    {
      uint8_t  j  = (uint8_t)((state->local_next + MIC_TDOA_RING - 1u - i) % MIC_TDOA_RING);
      uint32_t tl = state->local_us[j];
      uint32_t d  = mic_td_absdiff(tl, t_us);

      if (state->local_seq[j] == state->used_local_seq) { continue; }
      if (d > MIC_TDOA_VALID_US)
      {
        if (d <= MIC_TDOA_DIAG_US) { far = 1u; }
        continue;
      }
      if (d < bd) { bd = d; best = j; }
    }

    if (best != 0xFFu)
    {
      mic_td_try(best, seq, t_us, (uint8_t)kind);
      state->peer_pending = 0u;      /* 这条报文已经处理过，不再留给下一拍 */
    }
    else
    {
      /* 本板还没有对得上的一拍：先存着，等本板下一个事件来配 */
      mic_td_rej_far += far;
      mic_tdoa_nomatch++;    /* 诊断：收到报文但本板窗口内没有可配的事件 */
      state->peer_seq = seq;
      state->peer_us  = t_us;
      state->peer_pending = 1u;
    }
  }

  taskEXIT_CRITICAL();
}

/* Resolve overlap only when both detectors pass. A short impulse can have
   a high correlation at one lag, but lacks energy across the 50 ms sweep. */
static uint8_t mic_chirp_has_coverage(uint32_t pos256, uint32_t noise)
{
  const uint32_t segment_len = MIC_CHIRP_REF_N / 5u;
  uint32_t start = pos256 / 256u;
  uint32_t segment, i, covered = 0u;
  float energy[5] = {0.0f};
  float peak = 0.0f;
  float floor = 3.0f * (float)noise * (float)noise * (float)segment_len;

  if (start + MIC_CHIRP_REF_N > (uint32_t)mic_pend_n / MIC_CHIRP_DECIM) { return 0u; }
  for (segment = 0u; segment < 5u; ++segment)
  {
    for (i = 0u; i < segment_len; ++i)
    {
      float x = mic_chirp_x[start + segment * segment_len + i];
      energy[segment] += x * x;
    }
    if (energy[segment] > peak) { peak = energy[segment]; }
  }
  if (floor < peak / 16.0f) { floor = peak / 16.0f; }
  for (segment = 0u; segment < 5u; ++segment)
  {
    if (energy[segment] > floor) { ++covered; }
  }
  return (uint8_t)(covered >= 4u);
}

typedef struct {
  uint32_t raw_us, time_us, pos256, speed_x1000, peak, correlation_q;
  uint32_t sync_age_ms, sync_samples;
  int32_t phase_start_us, phase_end_us, sync_offset_us;
  uint8_t sync_valid;
} mic_toa_candidate_t;

static void mic_prepare_toa(mic_toa_candidate_t *event, uint32_t pos256, uint32_t peak)
{
  /* pos256 is Q8 on the 24 kHz grid. Keep the captured fractional time. */
  event->raw_us = mic_pend_us
       + (uint32_t)((((uint64_t)mic_pend_subcnt * 256u) +
                     ((uint64_t)pos256 * (MIC_CHIRP_DECIM * MIC_CNT_PER_SMP)) +
                     (108u * 128u)) / (108u * 256u));
  event->pos256 = pos256;
  event->speed_x1000 = mic_pend_speed_x1000;
  event->peak = peak;
  event->correlation_q = mic_match_best_q;
  event->phase_start_us = mic_pend_phase_start_us;
  event->phase_end_us = lt_acoustic_phase_snapshot_us();
  event->sync_valid = lt_mic_residual_at(event->raw_us, event->phase_start_us,
                     &event->sync_offset_us, &event->sync_age_ms, &event->sync_samples);
  event->time_us = event->raw_us - (uint32_t)event->sync_offset_us;
}

static void mic_publish_toa(const mic_toa_candidate_t *event, uint8_t kind,
                            const mic_signal_quality_t *quality)
{
  uint8_t anchor = (uint8_t)((kind == MIC_EVENT_CHIRP) && (mic_pend_from_peer == 0u) &&
                             (event->correlation_q >= MIC_CHIRP_ANCHOR_Q));

  mic_event_raw_t_us = event->raw_us;
  mic_event_sync_offset_us = event->sync_offset_us;
  mic_event_sync_age_ms = event->sync_age_ms;
  mic_event_sync_valid = event->sync_valid;
  mic_event_sync_samples = event->sync_samples;
  mic_event_t_us = event->time_us;
  mic_event_pos256 = event->pos256;
  mic_event_phase_start_us = event->phase_start_us;
  mic_event_phase_end_us = event->phase_end_us;
  mic_event_kind = kind;
  mic_event_seq++;
  mic_td_local(mic_event_seq, event->time_us, event->speed_x1000, kind, anchor, quality);
  mic_last_pk = event->peak;
  mic_last_corr_q = event->correlation_q;
}

/* Require matching sweep content in multiple parts of the template.
   The sign can be either polarity, but should agree across the sweep. */
static uint8_t mic_peer_sweep_consistent(uint32_t pos256)
{
  const uint32_t width = MIC_CHIRP_REF_N / 5u;
  uint32_t i, part, positive = 0u, negative = 0u;
  uint32_t start = (pos256 + 128u) / 256u;

  for (part = 0u; part < 5u; ++part)
  {
    float dot = 0.0f, energy = 0.0f, ref_energy = 0.0f;
    for (i = part * width; i < (part + 1u) * width; ++i)
    {
      float x = mic_chirp_x[start + i];
      float ref = (float)mic_chirp_ref[i];
      dot += x * ref;
      energy += x * x;
      ref_energy += ref * ref;
    }
    if ((energy > 0.0f) && ((dot * dot) >= 0.0025f * energy * ref_energy))
    {
      if (dot > 0.0f) { ++positive; }
      else { ++negative; }
    }
  }
  return (uint8_t)((positive >= 3u) || (negative >= 3u));
}

/* Sweep only physically admissible lags. A peak that fails the sweep
   consistency check must not hide a weaker coherent peak. */
static uint8_t mic_chirp_match_peer(int32_t start_diff_us, uint32_t *pos256, float *r_out)
{
  const uint32_t max_lag = MIC_PEER_WINDOW_SAMPLES - MIC_CHIRP_REF_N;
  const float min_score = MIC_CHIRP_PEER_MIN_R * MIC_CHIRP_PEER_MIN_R * mic_chirp_ref_ss;
  float left = -1.0f, score, right, best_score = -1.0f, max_score = 0.0f;
  uint32_t lag, best = 0u;

  *r_out = 0.0f;
  if (mic_chirp_ref_ss <= 0.0f) { return 0u; }
  mic_chirp_energy_prepare(max_lag);
  score = mic_chirp_score_at(0u);
  for (lag = 0u; lag <= max_lag; ++lag)
  {
    int32_t diff_us = start_diff_us + (int32_t)((lag * 1000u + 12u) / 24u);
    right = (lag < max_lag) ? mic_chirp_score_at(lag + 1u) : -1.0f;
    if ((diff_us >= -(int32_t)MIC_TDOA_VALID_US) && (diff_us <= (int32_t)MIC_TDOA_VALID_US))
    {
      if (score > max_score) { max_score = score; }
      if ((score >= min_score) && (score > best_score) && (score > left) &&
          (score >= right) && (mic_peer_sweep_consistent(lag * 256u) != 0u))
      {
        best = lag;
        best_score = score;
      }
    }
    left = score;
    score = right;
  }
  if (best_score < 0.0f)
  {
    *r_out = sqrtf(max_score / mic_chirp_ref_ss);
    return 0u;
  }
  *r_out = sqrtf(best_score / mic_chirp_ref_ss);
  *pos256 = mic_chirp_peak_pos256(best, max_lag, *r_out);
  return 1u;
}

/* Amplitude metrics use the 50 ms window selected by the correlator.
   Both regular and replay captures are measured on the same 24 kHz grid. */
static void mic_chirp_quality(mic_signal_quality_t *quality, uint32_t pos256, uint8_t replay)
{
  uint32_t i, start = pos256 / 256u;
  float low = mic_chirp_x[start], high = low;
  float gate = (replay != 0u) ? MIC_CHIRP_PEER_MIN_R : MIC_CHIRP_MIN_R;

  for (i = start + 1u; i < start + MIC_CHIRP_REF_N; ++i)
  {
    float sample = mic_chirp_x[i];
    if (sample < low) { low = sample; }
    if (sample > high) { high = sample; }
  }
  quality->chirp.rq = (uint16_t)mic_match_best_q;
  quality->chirp.gate = (uint16_t)(gate * 1000.0f + 0.5f);
  quality->chirp.rms10 = (uint16_t)(sqrtf(mic_chirp_energy[start] / (float)MIC_CHIRP_REF_N) * 10.0f + 0.5f);
  quality->chirp.pp10 = (uint16_t)((high - low) * 10.0f + 0.5f);
  quality->chirp.replay = replay;
}

static void mic_finalize_peer_chirp(void)
{
  uint32_t i, pos256;
  uint32_t sum = 0u;
  float rr, mean;
  uint8_t matched;
  uint64_t t_calc = lt_now_cnt();
  mic_toa_candidate_t event;
  mic_signal_quality_t quality = {0};
  const mic_td_state_t *state;

  for (i = 0u; i < MIC_PEER_WINDOW_SAMPLES; ++i)
    sum += (uint16_t)mic_pend_raw[i];
  mean = (float)sum / (float)MIC_PEER_WINDOW_SAMPLES;
  for (i = 0u; i < MIC_PEER_WINDOW_SAMPLES; ++i)
    mic_chirp_x[i] = ((float)mic_pend_raw[i] - mean) / (float)MIC_CHIRP_DECIM;

  mic_prepare_toa(&event, 0u, mic_pend_pk);
  matched = mic_chirp_match_peer((int32_t)(event.time_us - mic_pend_peer_us), &pos256, &rr);
  mic_match_cost_us = (uint32_t)((lt_now_cnt() - t_calc) / 108u);
  mic_match_best_q = (uint32_t)((rr * 1000.0f) + 0.5f);
  if (matched == 0u) { return; }

  mic_prepare_toa(&event, pos256, mic_pend_pk);
  if (mic_td_absdiff(event.time_us, mic_pend_peer_us) > MIC_TDOA_VALID_US) { return; }
  mic_chirp_quality(&quality, pos256, 1u);

  taskENTER_CRITICAL();
  state = mic_td_state(MIC_EVENT_CHIRP);
  if ((state->peer_pending != 0u) && (state->peer_seq == mic_pend_peer_seq) &&
      (state->peer_us == mic_pend_peer_us) && (state->used_peer_seq != mic_pend_peer_seq))
  {
    mic_weak_chirp_cnt++;
    mic_publish_toa(&event, MIC_EVENT_CHIRP, &quality);
  }
  taskEXIT_CRITICAL();
}

static void mic_finalize(void)
{
  uint32_t pk = mic_pend_pk;
  uint32_t pos256 = 0u;
  uint64_t t_calc;
  float    rr = 0.0f;
  uint8_t  kind = MIC_EVENT_NONE;
  uint8_t  chirp_ok, clap_ok;
  mic_clap_result_t clap;
  mic_toa_candidate_t event;
  mic_signal_quality_t quality = {0};

  mic_candidate_pk = pk;

  if (mic_pend_n < (uint16_t)(MIC_CHIRP_REF_N * MIC_CHIRP_DECIM))
  {
    /* 还不到 50 ms，不可能覆盖完整 chirp；不发布。 */
    mic_match_best_q = 0u;
    mic_match_short_cnt++;
    return;
  }

  t_calc = lt_now_cnt();
  chirp_ok = mic_chirp_match(&pos256, &rr, mic_chirp_decimate(), MIC_CHIRP_MIN_R);
  clap_ok = mic_clap_analyze(mic_pend_raw, mic_pend_n, &clap);
  if (chirp_ok == 0u) { mic_match_reject_cnt++; }

  if ((chirp_ok != 0u) &&
      ((clap_ok == 0u) || (mic_chirp_has_coverage(pos256, clap.noise) != 0u)))
  {
    kind = MIC_EVENT_CHIRP;
  }
  else if ((clap_ok != 0u) &&
           ((mic_have_clap == 0u) ||
            ((uint32_t)(mic_pend_ms - mic_last_clap_ms) >= MIC_CLAP_DEDUP_US / 1000u)))
  {
    kind = MIC_EVENT_CLAP;
    pos256 = clap.pos256;
    mic_have_clap = 1u;
    mic_last_clap_ms = mic_pend_ms;
  }
  mic_match_cost_us = (uint32_t)((lt_now_cnt() - t_calc) / 108u);
  if (rr < 0.0f) { rr = 0.0f; }
  if (rr > 1.0f) { rr = 1.0f; }
  mic_match_best_q = (uint32_t)((rr * 1000.0f) + 0.5f);
  if (kind != MIC_EVENT_NONE)
  {
    if (kind == MIC_EVENT_CHIRP)
    {
      mic_chirp_quality(&quality, pos256, 0u);
    }
    else
    {
      quality.clap.peak = (uint16_t)clap.peak;
      quality.clap.noise = (uint16_t)clap.noise;
      quality.clap.rise_us = (uint16_t)clap.rise_us;
      quality.clap.width_us = (uint16_t)clap.width_us;
      quality.clap.early_percent = (uint8_t)clap.early_percent;
      quality.clap.clipped = (uint16_t)clap.clipped;
    }
    mic_prepare_toa(&event, pos256, pk);
    mic_publish_toa(&event, kind, &quality);
  }
}

void mic_analysis_task(void *argument)
{
  (void)argument;
  for (;;)
  {
    uint32_t flags = osThreadFlagsWait(MIC_ANALYSIS_READY, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) == 0u)
    {
      if (mic_pend_from_peer != 0u) { mic_finalize_peer_chirp(); }
      else { mic_finalize(); }
      mic_capture_state = MIC_CAPTURE_IDLE;
    }
  }
}

static void mic_queue_analysis(void)
{
  mic_capture_state = MIC_CAPTURE_QUEUED;
  if ((osThreadFlagsSet(mic_analysis_thread, MIC_ANALYSIS_READY) & osFlagsError) != 0u)
  {
    mic_capture_state = MIC_CAPTURE_IDLE;
  }
}

static void mic_record_peer_half(uint64_t t0, uint32_t seq, int32_t phase)
{
  uint32_t i, j;
  uint16_t *samples = mic_peer_sample_block(mic_peer_next);
  mic_peer_block_t *block = &mic_peer_blocks[mic_peer_next];

  for (i = 0u; i < MIC_PEER_BLOCK_SAMPLES; ++i)
  {
    uint16_t sum = 0u;
    for (j = 0u; j < MIC_CHIRP_DECIM; ++j)
      sum += mic_work[i * MIC_CHIRP_DECIM + j];
    samples[i] = sum;
  }
  block->seq = seq;
  block->time_us = (uint32_t)(t0 / 108u);
  block->subcnt = (uint16_t)(t0 % 108u);
  block->phase_us = phase;
  block->speed_x1000 = disp_measurement_speed_x1000();
  mic_peer_next = (uint8_t)((mic_peer_next + 1u) % MIC_PEER_HISTORY_BLOCKS);
  if (mic_peer_count < MIC_PEER_HISTORY_BLOCKS) { ++mic_peer_count; }
}

static uint8_t mic_copy_peer_window(uint32_t target_us, int32_t phase)
{
  const uint32_t ticks_per_sample = MIC_CNT_PER_SMP * MIC_CHIRP_DECIM;
  uint32_t slot = (mic_peer_next + MIC_PEER_HISTORY_BLOCKS - mic_peer_count) % MIC_PEER_HISTORY_BLOCKS;
  uint32_t i;

  for (i = 0u; i < mic_peer_count; ++i)
  {
    const mic_peer_block_t *block = &mic_peer_blocks[slot];
    int32_t dt = (int32_t)(target_us + (uint32_t)phase - block->time_us - (uint32_t)block->phase_us);
    int64_t offset_ticks = (int64_t)dt * 108 - block->subcnt;

    if ((offset_ticks >= 0) && (offset_ticks < (int64_t)MIC_PEER_BLOCK_SAMPLES * ticks_per_sample))
    {
      uint32_t start = (uint32_t)(offset_ticks / ticks_per_sample);
      uint32_t offset = start, done = 0u, sequence = block->seq;
      uint32_t remaining_blocks = mic_peer_count - i;
      uint32_t start_ticks = block->subcnt + start * ticks_per_sample;
      uint16_t min_code = UINT16_MAX, max_code = 0u;

      mic_pend_us = block->time_us + start_ticks / 108u;
      mic_pend_subcnt = start_ticks % 108u;
      mic_pend_phase_start_us = block->phase_us;
      mic_pend_speed_x1000 = block->speed_x1000;
      while ((done < MIC_PEER_WINDOW_SAMPLES) && (remaining_blocks != 0u))
      {
        const uint16_t *samples;
        if (mic_peer_blocks[slot].seq != sequence) { return 0u; }
        samples = mic_peer_sample_block(slot);
        for (; (offset < MIC_PEER_BLOCK_SAMPLES) && (done < MIC_PEER_WINDOW_SAMPLES); ++offset)
        {
          uint16_t value = samples[offset];
          mic_pend_raw[done++] = (int16_t)value;
          if (value < min_code) { min_code = value; }
          if (value > max_code) { max_code = value; }
        }
        offset = 0u;
        slot = (slot + 1u) % MIC_PEER_HISTORY_BLOCKS;
        ++sequence;
        --remaining_blocks;
      }
      mic_pend_pk = (uint32_t)(max_code - min_code) / (2u * MIC_CHIRP_DECIM);
      return (uint8_t)(done == MIC_PEER_WINDOW_SAMPLES);
    }
    slot = (slot + 1u) % MIC_PEER_HISTORY_BLOCKS;
  }
  return 0u;
}

static void mic_try_peer_capture(void)
{
  const mic_peer_block_t *newest;
  const mic_td_state_t *state, *clap;
  uint32_t peer_seq, peer_us;
  uint8_t pending;

  if ((mic_capture_state != MIC_CAPTURE_IDLE) || (mic_peer_count == 0u)) { return; }

  taskENTER_CRITICAL();
  state = mic_td_state(MIC_EVENT_CHIRP);
  pending = (uint8_t)((state->peer_pending != 0u) && (state->peer_seq != state->used_peer_seq));
  peer_seq = state->peer_seq;
  peer_us = state->peer_us;
  clap = mic_td_state(MIC_EVENT_CLAP);
  /* A conflicting remote timestamp cannot pull a strong standalone chirp
     onto a later reflection. Protect confirmed clap pairs as well. */
  if (((state->anchor_valid != 0u) &&
       (mic_td_absdiff(state->anchor_us, peer_us) <= MIC_TDOA_DIAG_US)) ||
      ((clap->last_pair_seq != 0u) &&
       (mic_td_absdiff(clap->last_local_us, peer_us) <= MIC_TDOA_VALID_US)))
  {
    pending = 0u;
  }
  taskEXIT_CRITICAL();

  if ((pending == 0u) || ((mic_peer_attempted != 0u) &&
      (peer_seq == mic_peer_attempted_seq) && (peer_us == mic_peer_attempted_us))) { return; }

  newest = &mic_peer_blocks[(mic_peer_next + MIC_PEER_HISTORY_BLOCKS - 1u) % MIC_PEER_HISTORY_BLOCKS];
  if ((int32_t)(newest->time_us - peer_us) < (int32_t)MIC_PEER_WINDOW_END_US) { return; }

  mic_peer_attempted = 1u;
  mic_peer_attempted_seq = peer_seq;
  mic_peer_attempted_us = peer_us;
  if (mic_copy_peer_window(peer_us - MIC_PEER_SEARCH_US, newest->phase_us) == 0u)
  {
    mic_peer_gap_cnt++;
    return;
  }

  mic_peer_attempt_cnt++;
  mic_pend_peer_seq = peer_seq;
  mic_pend_peer_us = peer_us;
  mic_pend_from_peer = 1u;
  mic_queue_analysis();
}

/* ---- 一个半缓冲的处理 ------------------------------------------------------
   ① 去慢速直流，abs() 后用 48 点滑动平均得到无 2 kHz 纹波的包络；
   ② 包络连续 64 点越过门限才承认起音，并保留触发前 32 ms 历史；
   ③ 把 50 ms chirp 完整收进 84 ms 事件窗口；
   ④ 事件时刻由扫频相关峰或拍手包络的 20% 交点计算。
   -------------------------------------------------------------------------- */
static void mic_process_half(uint8_t half, uint64_t t0, uint32_t seq_id, int32_t epoch_phase)
{
  uint32_t i;
  uint32_t now;
  uint32_t trig_run = 0u;
  uint32_t clap_run = 0u;
  uint32_t trig_i   = 0u;
  uint32_t collect_from = MIC_HALF_SAMPLES;
  uint32_t hist_need = MIC_PRE_SAMPLES + MIC_TRIG_CONFIRM;
  int32_t  env, x, a, mx;
  int32_t  trig, low, clap_gate;
  int64_t  sum_env = 0;
  uint8_t  triggered = 0u;

  memcpy(mic_work, (const void *)&mic_buf[half * MIC_HALF_SAMPLES], sizeof(mic_work));
  mic_record_peer_half(t0, seq_id, epoch_phase);
  mic_min_code = 4095;
  mic_max_code = 0;

  /* 环境底用上一个半缓冲的 mx 更新；用当前 mx 会在"刚开始响"的那一半
     把门限一起抬高，反而更容易漏掉弱起音。 */
  if ((mic_active == 0u) && (mic_prev_mx < (mic_nfar * 2 + 8)))
  {
    /* 平均包络 → 环境均值 nfar，给 50% 判决做底。 */
    mic_nfar += (mic_prev_mean - mic_nfar) >> 3;
    if (mic_nfar < 2) { mic_nfar = 2; }

    /* V5.5：再慢跟踪一个"环境包络峰值"npk。噪声尖峰比均值高得多
       （现场静默 mean≈2、mx≈5~9），触发门限必须压在尖峰之上，
       否则去掉 1 kHz 门控后"噪声自触发"会回来。只统计安静半缓冲
       （上一半的 mx 没超过 nfar*2+8），有声音的半缓冲不会把它抬高。 */
    mic_npk += (mic_prev_mx - mic_npk) >> 2;
    if (mic_npk < 4)  { mic_npk = 4; }
    if (mic_npk > 64) { mic_npk = 64; }
  }

  /* V5：门限回到噪声之上（V2 的 nfar+5 / 下限 7），并把事件结束门限
     挂到触发门限上（low = trig-1）。
     现场教训（2026-10-19 晚）：V4 把 trig 降到 nfar+2（下限 4）后，静默时
     包络峰值本来就有 5，门限被压进噪声里——"噪声自触发"不断发生（屏上
     tg 自己一直涨）；而 low 又低（3），事件要满足"mx < low 持续 60 ms"才能
     结束，噪声下永远满足不了，每次都只能等 2 s 超时。于是状态机被噪声事件
     占满，真炮起音落在被占的窗口里被吞掉——屏上 mx 冲到几百、ev 不涨。
     V5 的修法：
       ① trig 抬回噪声之上：nfar+5、下限 7（静默 nfar 2~3，留 4 码余量）；
       ② low = trig-1：声音一落回门限以下 60 ms 内事件就结束，不再占 2 s；
       ③ 触发再加"回落"要求（mic_edge_ok）：包络必须先降到门限下，
          连续长音 / 门限内噪声不会反复触发。
     真炮幅度实测到 350（验收距离也有十几），远高于 7，抬门限不会漏炮；
     V3/V4 降门限本是为修"1 m 不容易触发"，现场复查那是麦克风被线缆挡住
     （右板 mx 1、ev 2）造成的，不是门限问题。 */
  /* 远距离弱信号：门限留在环境均值和峰值之上。
     不再依赖 1 kHz 门控（那等于只认 1 kHz）：任何频率的声音，
      包络超过环境峰值 2 个码即可进入连续确认；环境越吵门限越高。
     如静默时 tg 增长，需根据 [MIC] 的 mx/nf/trg 再校准。 */
  trig = mic_nfar + 4;
  if (trig < (mic_npk + 2)) { trig = mic_npk + 2; }
  if (trig < 16) { trig = 16; }  /* Minimum envelope threshold in ADC counts. */
  low = trig - 1;
  if (low < 3) { low = 3; }
  mic_last_trig = (uint32_t)trig;
  clap_gate = mic_nfar * (int32_t)MIC_CLAP_GATE_RATIO;
  if (clap_gate < trig * 2) { clap_gate = trig * 2; }
  if (clap_gate < (int32_t)MIC_CLAP_GATE_MIN) { clap_gate = MIC_CLAP_GATE_MIN; }
  now = HAL_GetTick();

  /* ① 宽带包络：去慢速直流、取 abs、48 点梳状平均。
     它只负责触发和事件窗口，最终时刻由完整 chirp 匹配滤波给出。 */
  mx = 0;
  for (i = 0u; i < MIC_HALF_SAMPLES; i++)
  {
    int32_t v = (int32_t)mic_work[i];

    if (v < mic_min_code) { mic_min_code = v; }
    if (v > mic_max_code) { mic_max_code = v; }

    /* 宽带包络：去慢速直流后取 abs，再做 48 点梳状平均。 */
    mic_dc_q8 += (((int32_t)v << 8) - mic_dc_q8) >> 10;
    x = v - (mic_dc_q8 >> 8);
    a = (x < 0) ? -x : x;
    mic_abs_sum -= (uint32_t)mic_abs_hist[mic_abs_idx];
    mic_abs_sum += (uint32_t)a;
    mic_abs_hist[mic_abs_idx] = (uint16_t)a;
    mic_abs_idx++;
    if (mic_abs_idx >= MIC_ENV_AVG_LEN) { mic_abs_idx = 0u; }
    env = (int32_t)(mic_abs_sum / MIC_ENV_AVG_LEN);

    if (env < 0)    { env = 0; }
    if (env > 4095) { env = 4095; }

    mic_env[i] = env;
    if (env > mx) { mx = env; }
    sum_env += env;

    /* 预触发历史：每处理一个样本就压入，跨半缓冲不中断。
       包络与原始样本用同一个写指针、同一个长度，两者逐点对应。 */
    mic_hist[mic_hist_idx]    = (int16_t)env;
    mic_rawring[mic_hist_idx] = (int16_t)v;
    mic_hist_idx++;
    if (mic_hist_idx >= MIC_HIST_LEN) { mic_hist_idx = 0u; }
    if (mic_hist_fill < 0xFFFFFFFFu) { mic_hist_fill++; }

    /* V5：包络一回到门限之下就重新允许起音。必须**无条件**评估——
       放进下面那个 if 里的话，mic_edge_ok 一旦清 0、条件就永远进不去，
       它也就永远回不到 1，整条触发链会死掉（现场实机踩过这个坑）。 */
    if (env <= trig) { mic_edge_ok = 1u; }

    /* ② 低门限 + 连续确认。确认窗口结束后，把"触发前 32 ms + 确认段"
       整段从环形历史里拷出来，弱信号晚触发时也尽量保留 chirp 前半段。
       V5 增加"回落"要求（mic_edge_ok）：包络必须先降到门限之下，才承认
       下一次起音——否则连续长音会每隔 2 s 被重新触发一次，时间戳取自
       声音中间（错的），还把状态机刷满。
        V6.0 不再用任何单频门控：假触发由"门限压在环境峰值之上
         （trig >= npk+2）"和 64 点连续确认一起挡，任何频率的真声都能进。 */
    if ((mic_active == 0u) && (mic_armed != 0u) && (mic_edge_ok != 0u) &&
        (mic_capture_state == MIC_CAPTURE_IDLE) && (mic_hist_fill >= hist_need) &&
        ((now - mic_last_evt_ms) >= 100u))
    {
      if (env > trig)
      {
        if (trig_run == 0u) { trig_i = i; }
        trig_run++;
        clap_run = (env >= clap_gate) ? (clap_run + 1u) : 0u;
        if ((trig_run >= MIC_TRIG_CONFIRM) || (clap_run >= MIC_CLAP_TRIGGER_CONFIRM))
        {
          uint32_t capture_n = MIC_PRE_SAMPLES + trig_run;
          uint32_t k;
          uint32_t start;
          int32_t  start_off;
          int64_t  us64;

          start = (uint32_t)((mic_hist_idx + MIC_HIST_LEN - capture_n) %
                             MIC_HIST_LEN);
          mic_pend_pk = 0u;
          for (k = 0u; k < capture_n; k++)
          {
            uint32_t e = (uint32_t)mic_hist[(start + k) % MIC_HIST_LEN];
            if (e > mic_pend_pk) { mic_pend_pk = e; }
            mic_pend_raw[k] = mic_rawring[(start + k) % MIC_HIST_LEN];
          }
          mic_pend_n = (uint16_t)capture_n;

          /* 分析窗口第一个样本 = 首个越限样本再往前 32 ms。 */
          start_off = (int32_t)trig_i - (int32_t)MIC_PRE_SAMPLES;
          /* Keep the fractional microsecond until the final TOA rounding. */
          us64 = (int64_t)t0 + ((int64_t)start_off * MIC_CNT_PER_SMP);
          mic_pend_us = (uint32_t)(us64 / 108);
          mic_pend_subcnt = (uint32_t)(us64 % 108);
          mic_pend_phase_start_us = epoch_phase;
          mic_pend_speed_x1000 = disp_measurement_speed_x1000();
          mic_candidate_trig = (uint32_t)trig;

          mic_pend_ms    = now;
          mic_pend_half_id = seq_id;
          mic_active     = 1u;
          mic_pend_from_peer = 0u;
          mic_capture_state = MIC_CAPTURE_RECORDING;
          mic_armed      = 0u;
          mic_edge_ok    = 0u;     /* V5：要等包络回落后才允许下一次起音 */
          mic_quiet_ms   = 0u;
          mic_trig_cnt++;          /* 诊断：触发链路第 2 段（串口 [MIC] 行） */
          collect_from   = i + 1u;
          triggered      = 1u;
        }
      }
      else
      {
        trig_run = 0u;
        clap_run = 0u;
      }
    }
  }

  /* ③ ACTIVE：把本半缓冲的剩余样本续进 84 ms 事件窗口。 */
  if (mic_capture_state == MIC_CAPTURE_RECORDING)
  {
    if (triggered != 0u)
    {
      for (i = collect_from; (i < MIC_HALF_SAMPLES) && (mic_pend_n < MIC_PEND_LEN); i++)
      {
        if ((uint32_t)mic_env[i] > mic_pend_pk) { mic_pend_pk = (uint32_t)mic_env[i]; }
        mic_pend_raw[mic_pend_n] = (int16_t)mic_work[i];
        mic_pend_n++;
      }
    }
    else if (mic_pend_half_id != seq_id)
    {
      for (i = 0u; (i < MIC_HALF_SAMPLES) && (mic_pend_n < MIC_PEND_LEN); i++)
      {
        if ((uint32_t)mic_env[i] > mic_pend_pk) { mic_pend_pk = (uint32_t)mic_env[i]; }
        mic_pend_raw[mic_pend_n] = (int16_t)mic_work[i];
        mic_pend_n++;
      }
    }
  }

  /* ④ 状态机：必须收满 84 ms 再做匹配，保证晚触发时 50 ms 参考仍完整，
     且相关峰后留有余量；否则在“声音刚结束”的半缓冲就精化，峰会落在右边界，
     抛物线插值缺右侧邻点，两个板各差 0.5 个 24 kHz 样本就会引入十几 µs
     的 TDOA 误差。连续 60 ms 安静只负责结束事件，不再提前精化。 */
  if (mic_active != 0u)
  {
    if ((mic_capture_state == MIC_CAPTURE_RECORDING) && (mic_pend_n >= MIC_PEND_LEN))
    {
      mic_queue_analysis();
    }

    if (mx < low)
    {
      if (mic_quiet_ms == 0u) { mic_quiet_ms = now; }
      else if ((now - mic_quiet_ms) >= 60u)
      {
        if (mic_capture_state == MIC_CAPTURE_RECORDING) { mic_queue_analysis(); }

        mic_last_evt_ms = now;
        mic_active      = 0u;
        mic_armed       = 1u;
        mic_quiet_ms    = 0u;
        mic_pend_half_id = 0xFFFFFFFFu;
      }
    }
    else
    {
      mic_quiet_ms = 0u;
    }

    /* 2s 超时兜底。 */
    if ((now - mic_pend_ms) > 2000u)
    {
      if (mic_capture_state == MIC_CAPTURE_RECORDING) { mic_queue_analysis(); }
      mic_last_evt_ms = now;
      mic_active      = 0u;
      mic_armed       = 1u;
      mic_quiet_ms    = 0u;
      mic_pend_half_id = 0xFFFFFFFFu;
    }
  }

  mic_prev_mx = mx;
  mic_prev_mean = (int32_t)(sum_env / (int64_t)MIC_HALF_SAMPLES);
  mic_env_mx_live = (uint32_t)mx;    /* 诊断：实时电平（串口 [MIC] 行） */
  mic_last_nf = (uint32_t)mic_nfar;  /* 与门限同源的噪声底 */
}

/* DMA 是双半缓冲；任务如果晚于 10.67ms 才回来，中间那一半已经被覆盖。
   这种事件的时间戳不可信，直接作废。 */
static void mic_abort_event(void)
{
  /* 丢过半缓冲后，宽带平均窗不再连续；清一次，让下一次事件从干净状态开始。 */
  mic_abs_sum = 0u;
  mic_abs_idx = 0u;
  memset(mic_abs_hist, 0, sizeof(mic_abs_hist));
  mic_hist_fill = 0u;

  if (mic_capture_state == MIC_CAPTURE_RECORDING)
  {
    mic_capture_state = MIC_CAPTURE_IDLE;
    mic_pend_n = 0u;
  }
  if (mic_active != 0u)
  {
    mic_last_evt_ms  = HAL_GetTick();
    mic_active       = 0u;
    mic_armed        = 1u;
    mic_quiet_ms     = 0u;
    mic_pend_half_id = 0xFFFFFFFFu;
  }
}

/* ---- 任务 ------------------------------------------------------------------ */
void mic_task(void *argument)
{
  mic_analysis_thread = (osThreadId_t)argument;

  for (;;)
  {
    if (mic_seq != mic_seen_seq)
    {
      uint8_t  half;
      uint32_t seq;
      uint32_t primask;
      uint64_t t0;
      int32_t epoch_phase;

      /* half、t0、seq 必须来自同一个瞬间。否则在读到 half 之后、
         读 epoch 之前刚好来一次 DMA 中断，会把"下一半的起点"当成
         当前半的起点，时间戳会突然跳约 10.7ms。 */
      primask = __get_PRIMASK();
      __disable_irq();
      half = mic_done;
      t0   = mic_epoch_cnt[half];
      epoch_phase = mic_epoch_phase_us[half];
      seq  = mic_seq;
      if (primask == 0u) { __enable_irq(); }

      if ((uint32_t)(seq - mic_seen_seq) > 1u)
      {
        mic_overruns += (seq - mic_seen_seq - 1u);
        mic_abort_event();
      }
      mic_seen_seq = seq;
      mic_process_half(half, t0, seq, epoch_phase);
    }
    mic_try_peer_capture();
    osDelay(2);
  }
}

/* 启动：先武装 DMA，再放开采样触发源，避免头几个转换落在没人收的地方。 */
void mic_init(void)
{
  HAL_StatusTypeDef st_adc;
  HAL_StatusTypeDef st_tim;
  char b[128];
  int  n;

  /* V6.0.2：参考 chirp 就绪；不再学模板，第一声即可发布。 */
  mic_chirp_ref_init();

  st_adc = HAL_ADC_Start_DMA(&hadc3, (uint32_t *)mic_buf, MIC_BUF_SAMPLES);

  /* 触发方式：不用 TIM2 的"更新事件 TRGO"，改用 TIM2_CH2 的 PWM 输出（OC2REF）。

     原因：MMS=010(Update) 时 TRGO 只有 1 个定时器时钟宽（108MHz → 9.26ns），
     而 ADCCLK 只有 27MHz（37ns/拍），ADC 的触发检测器很可能根本采不到这么窄的
     脉冲 —— 这正是"配置全对、SR 里却连 STRT 都没有"的原因。

     用 PWM 模式后 OC2REF 的高电平持续半个周期（≈5.2μs），触发一定能被识别。
     注意：只配定时器的内部通道，不给 CH2 分配引脚（PA1 是 ETH 的 REF_CLK）。 */
  TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S))
              | (((uint32_t)TIM_OCMODE_PWM1) << 8) | TIM_CCMR1_OC2PE;
  TIM2->CCR2 = 562u;                       /* 1125 / 2 */
  TIM2->CCER |= TIM_CCER_CC2E;             /* 使能 CH2 输出（内部信号） */
  TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_TRGO_OC2REF;   /* MMS = 001 */
  TIM2->EGR = TIM_EGR_UG;                  /* 立即装载 CCR2 预装载值 */

  st_tim = HAL_TIM_Base_Start(&htim2);

  /* 保险 1：外部触发源改成 TIM2_OC2（EXTSEL = 0x3）、上升沿。 */
  ADC3->CR2 = (ADC3->CR2 & ~(ADC_CR2_EXTSEL | ADC_CR2_EXTEN))
            | ADC_EXTERNALTRIGCONV_T2_CC2
            | ADC_EXTERNALTRIGCONVEDGE_RISING;

  /* 保险 2：确保 DMA 流真的使能（若 HAL 启动时流没开，数据永远不会搬） */
  if ((DMA2_Stream0->CR & DMA_SxCR_EN) == 0u)
  {
    DMA2_Stream0->CR |= DMA_SxCR_EN;
  }

  mic_seen_seq = mic_seq;

  n = snprintf(b, sizeof(b), "[MIC] start TS=DMA2 a%d t%d CR2 %08lX T2CR2 %04lX\r\n",
               (int)st_adc, (int)st_tim,
               (unsigned long)ADC3->CR2, (unsigned long)(TIM2->CR2 & 0xFFFFu));
  if (n > 0) { (void)HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)n, 80u); }
}
