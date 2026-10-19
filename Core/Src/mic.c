/**
  ******************************************************************************
  * @file    mic.c
  * @brief   声音到达时刻检测（第二步）
  *
  *  数据流：AO -> PA0(ADC3_IN0) --TIM2 96kHz 触发--> DMA2_Stream0 -> mic_buf
  *          半缓冲中断里记一次 TIM5 计数（时间戳），任务里算包络、定判决点，
  *          输出"到达时刻 = 本板时间基准上的微秒值"。
  *
  *  为什么这样搭（详见设计文档 03）：
  *   · 108MHz / 96kHz = 1125，每个采样点正好 1125 个 TIM5 计数，换算无舍入；
  *   · TIM2 与 TIM5 同挂 APB1、同源，采样网格相对时间基准永不漂；
  *   · 采样时刻由定时器硬件触发决定，软件只负责"认是第几个样本"；
  *   · 判决点用"峰值的 50%"这种与幅度无关的判据，重复性才好。
  ******************************************************************************
  */
#include "mic.h"
#include "main.h"
#include "tim.h"      /* lt_now_cnt() */
#include "adc.h"      /* hadc3 */
#include "usart.h"    /* huart1 */
#include "cmsis_os.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ---- 采样缓冲 --------------------------------------------------------------
   必须放在 DMA 能访问的区域。STM32F7 的 DMA 够不着 DTCM（0x2000'0000~0x2000'FFFF），
   而 IAR 默认就把 readwrite 数据放在那里。本项目实测（EWARM/.../LT_KeShe.map）：

     · 普通数据（含 32KB FreeRTOS 堆）已用到 0x2001'2f06，CSTACK/HEAP 到 0x2001'3508
       —— 所以 SRAM1 起点 0x2001'0000 早就被占了，绝对不能用；
     · ST 的 ethernetif.c 把以太网 DMA 描述符绝对定位在 0x2004'c000~0x2004'c140；
     · 空闲区是 0x2001'3508~0x2004'bfff 和 0x2004'c140~0x2004'ffff。

   这里取 SRAM2 尾部的 4KB（0x2004'd000~0x2004'dfff）：既避开普通数据，也不与
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

static int32_t  mic_nf = 8;                 /* 噪声底（包络域） */
static uint32_t mic_last_evt_ms = 0u;       /* 上次事件时刻，用于抑制重复触发 */
static uint8_t  mic_armed    = 1u;          /* 1 = 可以接收下一个"起音" */
static uint32_t mic_quiet_ms = 0u;          /* 本次"安静"开始计时的时刻 */
static int32_t  mic_nfar     = 2;           /* 环境包络的慢速估计（门限基准） */

/* ---- 宽带包络：48 点梳状滑动平均（负责“时间判决”） ----------------------
   1 kHz 整流后的主要纹波在 2 kHz。96 kHz 下 48 点窗口在 2 kHz 处为零点，
   因此能得到平滑的起音沿；它的窗口短（0.5 ms），起音瞬态比 1 ms I/Q 窗小，
   实测 10 cm 的极差约 91 µs。这个包络只负责找 50% 起音点。

   注意：下面 I/Q 只负责判断“这确实是一段 1 kHz 声音”，不参与时间判决。
   这样既能利用 I/Q 的抗噪能力，又不会被 I/Q 窗在起音段的相位瞬态带偏。
   -------------------------------------------------------------------------- */
#define MIC_ENV_AVG_LEN   48u
#define MIC_ENV_DELAY_Q8  (((MIC_ENV_AVG_LEN - 1u) * 128u)) /* 23.5 样本 */

static int32_t  mic_dc_q8 = (2048 << 8);
static uint16_t mic_abs_hist[MIC_ENV_AVG_LEN];
static uint32_t mic_abs_sum = 0u;
static uint8_t  mic_abs_idx = 0u;

/* ---- 1 kHz 正交解调（I/Q 匹配滤波） ---------------------------------------
   标准测试音是 1 kHz。采样率 96 kHz 时，一个周期正好 96 点。用 96 点
   矩形窗做 I/Q 解调：

       I[n] = Σ x[n-k]·cos(2πk/96)
       Q[n] = Σ x[n-k]·sin(2πk/96)

   它的频率响应在 DC、2 kHz、3 kHz … 都是零点，只保留 1 kHz 基波。
   与"整流 + 低通"相比，弱信号下不会把带外噪声整流成包络，信噪比
   高得多；幅度 mag=sqrt(I²+Q²) 与声音相位无关。

   窗长 96 点 = 1 ms，群延迟 (96-1)/2 = 47.5 点 ≈ 495 µs。
   归一化后 env 的单位仍是"1 kHz 基波幅度（ADC 码）"，所以门限
   trig=nfar+5 的物理含义不变。
   -------------------------------------------------------------------------- */
#define MIC_IQ_N          96u
#define MIC_IQ_DELAY_Q8   12160u   /* 47.5 × 256 */
#define MIC_IQ_MIN_PK     12       /* 事件合格的最小 I/Q 峰值（码） */

static int16_t  mic_iq_cos[MIC_IQ_N];
static int16_t  mic_iq_sin[MIC_IQ_N];
static uint16_t mic_iq_ring[MIC_IQ_N];
static uint8_t  mic_iq_idx = 0u;
static int64_t  mic_iq_i = 0;
static int64_t  mic_iq_q = 0;
static int32_t  mic_iq_env_last = 0;

/* ---- 预触发历史 + 确认触发 ------------------------------------------------
   低门限在弱信号下更容易触发，但单点越限也可能来自噪声。这里要求连续
   MIC_TRIG_CONFIRM 个包络点都超过门限才承认起音，并保留触发前 4 ms
   的包络。这样即使确认过程让"发现触发"晚了一点，50% 判决点仍完整落在
   分析窗口内，不会因为窗口从触发点才开始而被截掉。
   -------------------------------------------------------------------------- */
#define MIC_PRE_SAMPLES   384u   /* 4 ms @ 96 kHz */
#define MIC_TRIG_CONFIRM  24u    /* 0.25 ms 连续确认 */
#define MIC_HIST_LEN      512u   /* 5.33 ms，覆盖 4 ms 预触发 + 确认段 */
#define MIC_FIT_HALF      48u    /* 50% 点两侧各 0.5 ms 做线性拟合 */
#define MIC_PEAK_SEARCH   336u   /* 峰值只在触发后 3.5 ms 内找，避开后到的反射峰 */

static int16_t  mic_hist[MIC_HIST_LEN];
static uint16_t mic_hist_idx = 0u;
static uint32_t mic_hist_fill = 0u;
static int32_t  mic_prev_mx = 0;          /* 上一个半缓冲的包络最大值 */
static int32_t  mic_prev_mean = 0;        /* 上一个半缓冲的平均包络 */
static int32_t  mic_event_base = 2;       /* 触发时的环境底，精化时冻结 */
static uint32_t mic_pend_half_id = 0xFFFFFFFFu;
static uint32_t mic_last_sound_us = 0u;
static uint8_t  mic_dt_skip = 0u;         /* 丢失半缓冲后跳过下一次间隔 */

/* ---- 起音精化：独立 20ms 窗口 -------------------------------------------
   不能在半个缓冲（10.7ms）里找峰值——攻击段较慢时峰值会落在缓冲之外，
   截断后的"局部峰值"会让 50% 判决点随缓冲对齐位置漂移（毫秒级）。
   所以触发后把包络连续收集 20ms 到独立缓冲，再统一精化。 */
#define MIC_PEND_LEN   1920u                 /* 20ms @ 96kHz */
static int16_t  mic_pend_env[MIC_PEND_LEN];  /* 从触发点开始收集的包络 */
static uint16_t mic_pend_n   = 0u;           /* 已收集点数 */
static uint32_t mic_pend_us  = 0u;           /* 触发点的粗时刻（µs） */
static uint32_t mic_pend_ms  = 0u;           /* 触发时刻（ms），用于 2s 超时 */
static uint8_t  mic_active   = 0u;           /* 1 = 正在跟踪一次声音 */
static uint8_t  mic_arr_done = 0u;           /* 1 = 本次事件的到达时刻已算出 */
static int32_t  mic_pend_pk  = 0;            /* 本次事件精化后的峰值 */
static int32_t  mic_pend_pk_q = 0;           /* 本次事件 I/Q 峰值（门控质量） */
static uint16_t mic_iq_gate_left = 0u;       /* I/Q 质量窗剩余样本（4 ms） */

static int32_t mic_dc_code = 0, mic_min_code = 0, mic_max_code = 0;
static int32_t mic_last_code = 0;           /* 最近一个采样点，供显示用 */

volatile uint32_t mic_event_t_us  = 0u;
volatile uint32_t mic_event_dt_us = 0u;
volatile uint32_t mic_event_seq   = 0u;
volatile uint32_t mic_last_pk     = 0u;
volatile uint32_t mic_last_pk_iq  = 0u;
volatile uint32_t mic_last_nf     = 0u;
volatile uint32_t mic_last_trig   = 0u;
volatile uint8_t  mic_st_adc      = 0xEEu;   /* 0xEE = mic_init 还没执行 */
volatile uint8_t  mic_st_tim      = 0xEEu;
volatile uint32_t mic_task_loops  = 0u;
volatile uint32_t mic_adc_probe   = 0xFFFFFFFEu;   /* 0xFFFFFFFE = 还没测过 */
volatile uint32_t mic_dt_jitter_us = 0u;
volatile uint8_t  mic_dt_count     = 0u;
volatile uint32_t mic_event_end_us = 0u;   /* 最近一次事件的结束时刻（µs） */
volatile uint32_t mic_event_dur_us = 0u;   /* 最近一次事件的持续时间（µs） */
volatile uint32_t mic_overruns     = 0u;   /* 任务没赶上、被 DMA 覆盖的半缓冲数 */

/* 最近 10 次事件间隔的环形缓冲：屏幕上的 jit = 极差，就是第（2）项的验收量 */
static uint32_t mic_dt_ring[10];
static uint8_t  mic_dt_idx = 0u;
static uint8_t  mic_dt_n   = 0u;

/* 最近 10 次到达时刻本身（µs），供屏幕排队显示 */
static uint32_t mic_t_ring[10];
static uint8_t  mic_t_idx = 0u;
static uint8_t  mic_t_n   = 0u;

/* ---- 中断回调：由 stm32f7xx_it.c 的 DMA2_Stream0_IRQHandler 调用 -------------
   只做一件事：把"刚完成的那个半缓冲起点"记到 TIM5 计数上。

   约定：half=0 表示前半（索引 0..1023）刚写完，接下来 DMA 会写索引 1024，
         所以这一时刻是下一半的起点，记到 [1]；
         half=1 表示整缓冲写完，接下来从索引 0 开始，记到 [0]。
   这样任一半缓冲的起点时刻都与其样本索引对得上。                            */
void mic_isr_half(uint8_t half)
{
  mic_epoch_cnt[half ^ 1u] = lt_now_cnt();
  mic_done = half;
  mic_seq++;
}

uint8_t mic_quiet_ok(void)
{
  return ((HAL_GetTick() - mic_last_evt_ms) > 50u) ? 1u : 0u;
}

int32_t mic_last_dc(void)  { return mic_dc_code;  }
int32_t mic_last_min(void) { return mic_min_code; }
int32_t mic_last_max(void) { return mic_max_code; }
int32_t mic_last_sample(void) { return mic_last_code; }
uint32_t mic_seq_count(void)  { return mic_seq; }

/* 按"旧→新"导出最近 n 个到达时刻，返回个数。 */
uint8_t mic_t_history(uint32_t *out10)
{
  uint8_t n = mic_t_n;
  uint8_t i;

  for (i = 0u; i < n; i++)
  {
    out10[i] = mic_t_ring[(mic_t_idx + 10u - n + i) % 10u];
  }
  return n;
}

/* ---- 起音精化：50% 点多点线性拟合 -----------------------------------------
   窗口现在是"触发前 4ms + 触发后约 16ms"：
     ① 找峰值 pk；
     ② 用触发时冻结的环境底 base，算 50% 门限 th；
     ③ 从峰值往回找到最后一个低于 th 的点 c；
     ④ 在 c 两侧各 0.5ms 内对包络做最小二乘直线拟合，
        求拟合直线与 th 的交点，代替原来的两点插值。

   单点插值会把残余噪声、整流纹波和 ADC 量化直接带进时间戳；多点拟合
   相当于在 1ms 窗口内做平均，只保留"起音斜率"这一个共同特征，因此
   弱信号、低门限下的重复性会明显好于两点插值。

   时间判决用的是 48 点宽带包络（0.5 ms 窗），群延迟固定 23.5 样本；
   最后从 k256 里扣掉。I/Q 只负责确认这一段确实是 1 kHz 声音。
   -------------------------------------------------------------------------- */
static void mic_finalize(void)
{
  int32_t  pk, base, span, th;
  uint32_t pk_i, c, i, pk_start, pk_end;
  uint32_t j0, j1, k;
  uint32_t fit_n;
  uint32_t k256;
  uint32_t t_us;
  float    sx, sy, sxx, sxy, denom, slope, intercept, xcross;
  uint8_t  fit_ok = 0u;

  if (mic_pend_n < 32u)          /* 太短，放弃这次 */
  {
    mic_dt_skip = 1u;
    mic_arr_done = 1u;
    return;
  }

  /* I/Q 质量门控：只有确认到足够强的 1 kHz 基波才承认事件。
     这能把 50 cm 处被宽带包络误当成起音的噪声挡掉。 */
  if (mic_pend_pk_q < MIC_IQ_MIN_PK)
  {
    mic_dt_skip = 1u;
    mic_arr_done = 1u;
    return;
  }

  /* 前 4ms 只用于让 50% 穿越点落在窗口内；峰值从触发点之后找，
     避免极弱信号时把预触发段的噪声尖峰误当成峰值。 */
  pk_start = (mic_pend_n > MIC_PRE_SAMPLES) ? MIC_PRE_SAMPLES : 0u;
  if ((pk_start + MIC_PEAK_SEARCH) < mic_pend_n)
  {
    pk_end = pk_start + MIC_PEAK_SEARCH;
  }
  else
  {
    pk_end = mic_pend_n;
  }
  pk = mic_pend_env[pk_start];
  pk_i = pk_start;
  for (i = pk_start + 1u; i < pk_end; i++)
  {
    if (mic_pend_env[i] > pk)
    {
      pk = mic_pend_env[i];
      pk_i = i;
    }
    else if ((pk_i > (pk_start + 8u)) &&
             ((mic_pend_env[i] + 5) < pk))
    {
      /* 取第一个局部峰：直达声之后若出现更深/更晚的反射峰，不再改峰值。 */
      break;
    }
  }
  base = mic_event_base;
  if (base < 0) { base = 0; }
  if (pk <= (base + 4))          /* 没有明显的峰，放弃 */
  {
    mic_dt_skip = 1u;
    mic_arr_done = 1u;
    return;
  }

  mic_pend_pk = pk;

  span = pk - base;
  th   = base + (span >> 1);

  c = pk_i;
  while ((c > 0u) && (mic_pend_env[c] > th)) { c--; }

  /* 以 c 为中心取 ±0.5ms，拟合包络上升沿。窗口不越过峰值，避免把
     峰顶和下降沿也拉进同一条直线。 */
  j0 = (c > MIC_FIT_HALF) ? (c - MIC_FIT_HALF) : 0u;
  j1 = c + MIC_FIT_HALF;
  if (j1 >= mic_pend_n) { j1 = mic_pend_n - 1u; }
  if (j1 > pk_i)        { j1 = pk_i; }

  if (j1 > (j0 + 7u))
  {
    sx = 0.0f; sy = 0.0f; sxx = 0.0f; sxy = 0.0f;
    for (k = j0; k <= j1; k++)
    {
      float xx = (float)k - (float)c;
      float yy = (float)mic_pend_env[k];

      sx  += xx;
      sy  += yy;
      sxx += xx * xx;
      sxy += xx * yy;
    }
    fit_n = j1 - j0 + 1u;
    denom = ((float)fit_n * sxx) - (sx * sx);
    if (denom > 1.0e-3f)
    {
      slope     = (((float)fit_n * sxy) - (sx * sy)) / denom;
      intercept = (sy - (slope * sx)) / (float)fit_n;
      if (slope > 1.0e-3f)
      {
        xcross = ((float)th - intercept) / slope;
        if ((xcross >= -(float)MIC_FIT_HALF) &&
            (xcross <=  (float)MIC_FIT_HALF))
        {
          fit_ok = 1u;
        }
      }
    }
  }

  if (fit_ok != 0u)
  {
    float pos = (float)c + xcross;

    if (pos < 0.0f) { pos = 0.0f; }
    if (pos > (float)(mic_pend_n - 1u)) { pos = (float)(mic_pend_n - 1u); }
    k256 = (uint32_t)((pos * 256.0f) + 0.5f);
  }
  else
  {
    int32_t  e0 = mic_pend_env[c];
    int32_t  e1 = mic_pend_env[(c + 1u < mic_pend_n) ? (c + 1u) : c];
    uint32_t frac256 = 0u;

    if ((e1 > e0) && (e0 < th))
    {
      frac256 = (uint32_t)(((th - e0) * 256) / (e1 - e0));
      if (frac256 > 255u) { frac256 = 255u; }
    }
    k256 = (c << 8) + frac256;
  }

  /* 扣掉 48 点宽带包络的固定群延迟。 */
  if (k256 >= MIC_ENV_DELAY_Q8) { k256 -= MIC_ENV_DELAY_Q8; }
  else                          { k256 = 0u; }

  t_us = mic_pend_us
       + (uint32_t)(((uint64_t)k256 * MIC_CNT_PER_SMP) / (108u * 256u));

  if ((mic_event_seq == 0u) || (mic_dt_skip != 0u))
  {
    mic_event_dt_us = 0u;
    mic_dt_skip = 0u;
  }
  else
  {
    mic_event_dt_us = t_us - mic_event_t_us;
  }
  mic_event_t_us   = t_us;
  if (mic_last_sound_us < t_us) { mic_last_sound_us = t_us; }

  /* 最近 10 次到达时刻 */
  mic_t_ring[mic_t_idx] = t_us;
  mic_t_idx = (uint8_t)((mic_t_idx + 1u) % 10u);
  if (mic_t_n < 10u) { mic_t_n++; }

  /* 最近 10 个间隔的极差 = 验收指标 */
  if (mic_event_seq > 0u && mic_event_dt_us != 0u)
  {
    uint32_t d = mic_event_dt_us;
    uint32_t mn, mx2, r;

    mic_dt_ring[mic_dt_idx] = d;
    mic_dt_idx = (uint8_t)((mic_dt_idx + 1u) % 10u);
    if (mic_dt_n < 10u) { mic_dt_n++; }

    mn  = mic_dt_ring[0];
    mx2 = mic_dt_ring[0];
    for (r = 1u; r < mic_dt_n; r++)
    {
      if (mic_dt_ring[r] < mn)  { mn  = mic_dt_ring[r]; }
      if (mic_dt_ring[r] > mx2) { mx2 = mic_dt_ring[r]; }
    }
    mic_dt_jitter_us = mx2 - mn;
    mic_dt_count = mic_dt_n;
  }

  mic_event_seq++;
  mic_last_pk = (uint32_t)pk;
  mic_last_pk_iq = (uint32_t)mic_pend_pk_q;
  mic_arr_done = 1u;
}

/* ---- 一个半缓冲的处理 ------------------------------------------------------
   ① 去慢速直流，abs() 后用 48 点滑动平均得到无 2 kHz 纹波的包络；
   ② 包络连续 24 点越过门限才承认起音，并保留触发前 4 ms 历史；
   ③ 对 50% 点附近的 1 ms 上升沿做最小二乘拟合；
   ④ 事件时刻 = 窗口起点 + 拟合交点 × 1125 计数。
   -------------------------------------------------------------------------- */
static void mic_process_half(uint8_t half, uint64_t t0, uint32_t seq_id)
{
  uint32_t i;
  uint32_t now;
  uint32_t trig_run = 0u;
  uint32_t trig_i   = 0u;
  uint32_t collect_from = MIC_HALF_SAMPLES;
  uint32_t hist_need = MIC_PRE_SAMPLES + MIC_TRIG_CONFIRM;
  uint32_t c;
  int32_t  env, env_q, x, a, mx;
  int32_t  trig, low;
  int64_t  sum = 0;
  int64_t  sum_env = 0;
  uint8_t  triggered = 0u;

  memcpy(mic_work, (const void *)&mic_buf[half * MIC_HALF_SAMPLES], sizeof(mic_work));
  mic_last_code = (int32_t)mic_work[MIC_HALF_SAMPLES - 1u];
  mic_min_code = 4095;
  mic_max_code = 0;

  /* 环境底用上一个半缓冲的 mx 更新；用当前 mx 会在"刚开始响"的那一半
     把门限一起抬高，反而更容易漏掉弱起音。 */
  if ((mic_active == 0u) && (mic_prev_mx < (mic_nfar * 2 + 8)))
  {
    /* 用平均包络更新环境底，而不是用最大值：最大值会被单个噪声
       尖峰抬高，弱信号时就更容易漏触发。 */
    mic_nfar += (mic_prev_mean - mic_nfar) >> 3;
    if (mic_nfar < 2) { mic_nfar = 2; }
  }

  trig = mic_nfar + 5;
  low  = mic_nfar + 2;
  if (trig < 7) { trig = 7; }
  if (low  < 4) { low  = 4; }
  mic_last_trig = (uint32_t)trig;
  now = HAL_GetTick();

  /* ① 双前端：env（宽带 48 点）做时间判决，env_q（I/Q）只做 1 kHz 门控。 */
  mx = 0;
  for (i = 0u; i < MIC_HALF_SAMPLES; i++)
  {
    int32_t v = (int32_t)mic_work[i];

    if (v < mic_min_code) { mic_min_code = v; }
    if (v > mic_max_code) { mic_max_code = v; }
    sum += v;

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

    /* 96 点 I/Q 滑动窗：p 既是新样本的相位，也是 96 点前旧样本的相位。 */
    {
      uint32_t p = mic_iq_idx;
      int32_t  old = (int32_t)mic_iq_ring[p];
      int32_t  dv  = v - old;
      int32_t  i32;
      int32_t  q32;
      int64_t  mag2;
      float    mag;

      mic_iq_ring[p] = (uint16_t)v;
      mic_iq_i += (int64_t)dv * (int64_t)mic_iq_cos[p];
      mic_iq_q += (int64_t)dv * (int64_t)mic_iq_sin[p];
      mic_iq_idx = (uint8_t)((p + 1u) % MIC_IQ_N);

      /* I/Q 只做质量验证，不需要 96 kHz 的幅度更新；每 4 点算一次
         sqrtf（24 kHz），显著降低任务 CPU 占用。 */
      if ((i & 3u) == 0u)
      {
        /* 右移 14 位后数值仍落在浮点高精度区，避免大数平方丢失精度；
           归一化回 ADC 码幅度。 */
        i32 = (int32_t)(mic_iq_i >> 14);
        q32 = (int32_t)(mic_iq_q >> 14);
        mag2 = ((int64_t)i32 * i32) + ((int64_t)q32 * q32);
        mag  = sqrtf((float)mag2);
        env_q = (int32_t)((mag * 32768.0f) /
                          ((float)MIC_IQ_N * 32767.0f) + 0.5f);
        mic_iq_env_last = env_q;
      }
      else
      {
        env_q = mic_iq_env_last;
      }
    }
    if (env < 0)    { env = 0; }
    if (env > 4095) { env = 4095; }
    if (env_q < 0)    { env_q = 0; }
    if (env_q > 4095) { env_q = 4095; }

    if ((mic_active != 0u) && (mic_arr_done == 0u) &&
        (mic_iq_gate_left > 0u))
    {
      if (env_q > mic_pend_pk_q) { mic_pend_pk_q = env_q; }
      mic_iq_gate_left--;
    }

    mic_env[i] = env;
    if (env > mx) { mx = env; }
    sum_env += env;

    /* 预触发历史：每处理一个样本就压入，跨半缓冲不中断。 */
    mic_hist[mic_hist_idx] = (int16_t)env;
    mic_hist_idx++;
    if (mic_hist_idx >= MIC_HIST_LEN) { mic_hist_idx = 0u; }
    if (mic_hist_fill < 0xFFFFFFFFu) { mic_hist_fill++; }

    /* ② 低门限 + 连续确认。确认窗口结束后，把"触发前 4 ms + 确认段"
       整段从环形历史里拷出来，因此分析窗口不会缺掉起音前半段。 */
    if ((mic_active == 0u) && (mic_armed != 0u) &&
        (mic_hist_fill >= hist_need) &&
        ((now - mic_last_evt_ms) >= 100u))
    {
      if (env > trig)
      {
        if (trig_run == 0u) { trig_i = i; }
        trig_run++;
        if (trig_run >= MIC_TRIG_CONFIRM)
        {
          uint32_t k;
          uint32_t start;
          int32_t  start_off;
          int64_t  us64;

          start = (uint32_t)((mic_hist_idx + MIC_HIST_LEN - hist_need) %
                             MIC_HIST_LEN);
          for (k = 0u; k < hist_need; k++)
          {
            mic_pend_env[k] = mic_hist[(start + k) % MIC_HIST_LEN];
          }
          mic_pend_n = (uint16_t)hist_need;

          /* 分析窗口第一个样本 = 首个越限样本再往前 4 ms。 */
          start_off = (int32_t)trig_i - (int32_t)MIC_PRE_SAMPLES;
          us64 = (int64_t)(t0 / 108u)
               + (((int64_t)start_off * (int64_t)MIC_CNT_PER_SMP) / 108);
          mic_pend_us = (uint32_t)us64;

          mic_event_base = mic_nfar;
          mic_pend_ms    = now;
          mic_pend_pk    = 0;
          mic_pend_pk_q  = env_q;
          mic_iq_gate_left = (uint16_t)MIC_PEAK_SEARCH;
          mic_last_sound_us = 0u;
          mic_pend_half_id = seq_id;
          mic_active     = 1u;
          mic_arr_done   = 0u;
          mic_armed      = 0u;
          mic_quiet_ms   = 0u;
          collect_from   = i + 1u;
          triggered      = 1u;
        }
      }
      else
      {
        trig_run = 0u;
      }
    }
  }
  mic_dc_code = (int32_t)(sum / (int64_t)MIC_HALF_SAMPLES);

  /* ③ ACTIVE：把本半缓冲的剩余包络续进 20 ms 窗口。 */
  if ((mic_active != 0u) && (mic_arr_done == 0u))
  {
    if (triggered != 0u)
    {
      for (i = collect_from; (i < MIC_HALF_SAMPLES) && (mic_pend_n < MIC_PEND_LEN); i++)
      {
        mic_pend_env[mic_pend_n++] = (int16_t)mic_env[i];
      }
    }
    else if (mic_pend_half_id != seq_id)
    {
      for (i = 0u; (i < MIC_HALF_SAMPLES) && (mic_pend_n < MIC_PEND_LEN); i++)
      {
        mic_pend_env[mic_pend_n++] = (int16_t)mic_env[i];
      }
    }
  }

  /* ④ 状态机：窗口满或声音结束 → 精化；连续 60 ms 安静 → 结束。 */
  if (mic_active != 0u)
  {
    if ((mic_arr_done == 0u) &&
        ((mic_pend_n >= MIC_PEND_LEN) || (mx < low)))
    {
      mic_finalize();
    }

    if (mx < low)
    {
      if (mic_quiet_ms == 0u) { mic_quiet_ms = now; }
      else if ((now - mic_quiet_ms) >= 60u)
      {
        if (mic_arr_done == 0u) { mic_finalize(); }

        mic_event_end_us = mic_last_sound_us;
        if ((mic_event_end_us < mic_event_t_us) && (mic_pend_pk > 0))
        {
          mic_event_end_us = mic_event_t_us;
        }
        if (mic_pend_pk > 0)
        {
          mic_event_dur_us = mic_event_end_us - mic_event_t_us;
        }

        mic_active      = 0u;
        mic_armed       = 1u;
        mic_quiet_ms    = 0u;
        mic_last_evt_ms = now;
        mic_pend_half_id = 0xFFFFFFFFu;
      }
    }
    else
    {
      /* 记录本次事件最后一个仍高于 low 的样本，避免结束时刻被 60 ms
         安静确认拖到后面一个半缓冲。 */
      c = MIC_HALF_SAMPLES;
      while ((c > 0u) && (mic_env[c - 1u] <= low)) { c--; }
      if (c > 0u)
      {
        mic_last_sound_us = (uint32_t)(t0 / 108u)
                          + (uint32_t)(((uint64_t)(c - 1u) * MIC_CNT_PER_SMP) / 108u);
      }
      mic_quiet_ms = 0u;
    }

    /* 2s 超时兜底。 */
    if ((now - mic_pend_ms) > 2000u)
    {
      if (mic_arr_done == 0u) { mic_finalize(); }
      mic_event_end_us = mic_last_sound_us;
      if ((mic_event_end_us < mic_event_t_us) && (mic_pend_pk > 0))
      {
        mic_event_end_us = mic_event_t_us;
      }
      if (mic_pend_pk > 0)
      {
        mic_event_dur_us = mic_event_end_us - mic_event_t_us;
      }
      mic_active      = 0u;
      mic_armed       = 1u;
      mic_quiet_ms    = 0u;
      mic_last_evt_ms = now;
      mic_pend_half_id = 0xFFFFFFFFu;
    }
  }

  mic_prev_mx = mx;
  mic_prev_mean = (int32_t)(sum_env / (int64_t)MIC_HALF_SAMPLES);

  {
    int32_t lo = mic_env[0];

    for (i = 1u; i < MIC_HALF_SAMPLES; i++)
    {
      if (mic_env[i] < lo) { lo = mic_env[i]; }
    }
    if (lo < mic_nf)        { mic_nf = lo; }
    else if (mic_nf < 4000) { mic_nf++; }
    mic_last_nf = (uint32_t)mic_nf;
  }
}

/* DMA 是双半缓冲；任务如果晚于 10.67ms 才回来，中间那一半已经被覆盖。
   这种事件的时间戳不可信，直接作废，并让下一次间隔不进极差统计，避免
   一次丢失缓冲把 10 个样本的 jit 全部污染。 */
static void mic_abort_event(void)
{
  /* 丢过半缓冲后，I/Q 环和宽带平均窗都不再是连续样本；清一次，
     让下一次事件从一个干净的前端状态开始。 */
  mic_iq_i = 0;
  mic_iq_q = 0;
  mic_iq_env_last = 0;
  mic_iq_idx = 0u;
  memset(mic_iq_ring, 0, sizeof(mic_iq_ring));
  mic_abs_sum = 0u;
  mic_abs_idx = 0u;
  memset(mic_abs_hist, 0, sizeof(mic_abs_hist));
  mic_dc_q8 = (2048 << 8);

  mic_active       = 0u;
  mic_armed        = 1u;
  mic_arr_done     = 1u;
  mic_pend_n       = 0u;
  mic_iq_gate_left = 0u;
  mic_quiet_ms     = 0u;
  mic_pend_half_id = 0xFFFFFFFFu;
  mic_dt_skip      = 1u;
  mic_last_evt_ms  = HAL_GetTick();
}

/* ---- 任务 ------------------------------------------------------------------ */
void mic_task(void *argument)
{
  (void)argument;

  for (;;)
  {
    mic_task_loops++;
    if (mic_seq != mic_seen_seq)
    {
      uint8_t  half;
      uint32_t seq;
      uint32_t primask;
      uint64_t t0;

      /* half、t0、seq 必须来自同一个瞬间。否则在读到 half 之后、
         读 epoch 之前刚好来一次 DMA 中断，会把"下一半的起点"当成
         当前半的起点，时间戳会突然跳约 10.7ms。 */
      primask = __get_PRIMASK();
      __disable_irq();
      half = mic_done;
      t0   = mic_epoch_cnt[half];
      seq  = mic_seq;
      if (primask == 0u) { __enable_irq(); }

      if ((uint32_t)(seq - mic_seen_seq) > 1u)
      {
        mic_overruns += (seq - mic_seen_seq - 1u);
        if (mic_active != 0u) { mic_abort_event(); }
      }
      mic_seen_seq = seq;
      mic_process_half(half, t0, seq);
    }
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
  uint32_t k;

  /* 1 kHz @ 96 kHz = 每个周期 96 点。表只初始化一次，运行中只查表。 */
  for (k = 0u; k < MIC_IQ_N; k++)
  {
    float ph = (2.0f * 3.14159265358979323846f * (float)k) /
               (float)MIC_IQ_N;
    mic_iq_cos[k] = (int16_t)(32767.0f * cosf(ph));
    mic_iq_sin[k] = (int16_t)(32767.0f * sinf(ph));
  }
  mic_iq_idx = 0u;
  mic_iq_i   = 0;
  mic_iq_q   = 0;
  memset(mic_iq_ring, 0, sizeof(mic_iq_ring));

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

  mic_st_adc = (uint8_t)st_adc;
  mic_st_tim = (uint8_t)st_tim;
  mic_seen_seq = mic_seq;

  n = snprintf(b, sizeof(b), "[MIC] start a%d t%d CR2 %08lX T2CR2 %04lX\r\n",
               (int)st_adc, (int)st_tim,
               (unsigned long)ADC3->CR2, (unsigned long)(TIM2->CR2 & 0xFFFFu));
  if (n > 0) { (void)HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)n, 80u); }
}
