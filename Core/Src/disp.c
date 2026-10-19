/**
  ******************************************************************************
  * @file    disp.c
  * @brief   LCD 实时显示麦克风采样数据（LTDC + L8 帧缓冲 + 11x16 字库）
  *
  *  只显示数据，不画波形：
  *    大字 = 最新采样点的电压（mV，实时变化）
  *    下面依次是伏特值、直流电平、峰峰值、噪声底、事件计数、到达时刻、间隔。
  *
  *  帧缓冲用 L8（8 位调色板索引）：
  *    480x272x1 = 127.5KB，放在内部 SRAM1 的 0x20020000，不依赖 SDRAM；
  *    256 项 CLUT 的每一项都是 24 位真彩，界面观感与 RGB565 无差别。
  *    注意 L8 必须置 LTDC_LxCR 的 CLUTEN 位，否则调色板不生效 → 整屏黑。
  ******************************************************************************
  */
#include "disp.h"
#include "ltdc.h"
#include "mic.h"
#include "usart.h"
#include "cmsis_os.h"
#include "LCD_Font16.h"
#include <stdio.h>
#include <string.h>

/* ---- 调色板索引 ---- */
#define C_BG    0u
#define C_TEXT  1u
#define C_DIM   2u
#define C_GRID  3u
#define C_ACC   4u
#define C_WARN  5u
#define C_BLUE  6u
#define C_WHITE 7u

static uint8_t *const disp_fb = (uint8_t *)DISP_FB_ADDR;
static uint32_t       disp_clut[256];

static void disp_render(void);   /* 前置声明：disp_init 里也要先画一帧 */

/* ---- 基本绘图（全部写调色板索引） ------------------------------------------ */
static void disp_px(int32_t x, int32_t y, uint8_t c)
{
  if ((x < 0) || (y < 0) || (x >= (int32_t)DISP_W) || (y >= (int32_t)DISP_H)) { return; }
  disp_fb[((uint32_t)y * DISP_W) + (uint32_t)x] = c;
}

static void disp_fill(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint8_t c)
{
  int32_t x, y;

  if (x0 < 0) { x0 = 0; }
  if (y0 < 0) { y0 = 0; }
  if (x1 > (int32_t)DISP_W - 1) { x1 = (int32_t)DISP_W - 1; }
  if (y1 > (int32_t)DISP_H - 1) { y1 = (int32_t)DISP_H - 1; }
  if ((x0 > x1) || (y0 > y1)) { return; }

  for (y = y0; y <= y1; y++)
  {
    uint8_t *p = &disp_fb[((uint32_t)y * DISP_W) + (uint32_t)x0];
    for (x = x0; x <= x1; x++) { *p++ = c; }
  }
}

static void disp_clear(void)
{
  disp_fill(0, 0, (int32_t)DISP_W - 1, (int32_t)DISP_H - 1, C_BG);
}

static void disp_hline(int32_t x0, int32_t x1, int32_t y, uint8_t c)
{
  disp_fill(x0, y, x1, y, c);
}

/* ---- 11x16 字库（字模每行 2 字节，bit15 = 最左点） ------------------------- */
static int32_t disp_char(int32_t x, int32_t y, char ch, uint8_t c, int32_t scale)
{
  const uint8_t *g;
  int32_t  row, col, dx, dy;
  uint32_t bits;

  if ((ch < FONT_FIRST) || (ch > FONT_LAST)) { ch = '?'; }
  g = &FONT_DATA[(uint32_t)(ch - FONT_FIRST) * FONT_GLYPH];

  for (row = 0; row < FONT_H; row++)
  {
    bits = ((uint32_t)g[row * FONT_STRIDE] << 8) | (uint32_t)g[row * FONT_STRIDE + 1];
    for (col = 0; col < FONT_W; col++)
    {
      if (((bits >> (15 - col)) & 1u) == 0u) { continue; }
      for (dy = 0; dy < scale; dy++)
      {
        for (dx = 0; dx < scale; dx++)
        {
          disp_px(x + (col * scale) + dx, y + (row * scale) + dy, c);
        }
      }
    }
  }
  return x + (FONT_W * scale);
}

static int32_t disp_str(int32_t x, int32_t y, const char *s, uint8_t c, int32_t scale)
{
  while (*s != '\0')
  {
    x = disp_char(x, y, *s, c, scale);
    s++;
  }
  return x;
}

/* ---- 图层与调色板 ---------------------------------------------------------- */
static void disp_setup_layer(void)
{
  uint32_t ahbp = (LTDC->BPCR >> 16U) & 0x0FFFu;   /* 累积背 porch，和 HAL 算法一致 */
  uint32_t avbp = (LTDC->BPCR) & 0x0FFFu;
  uint32_t aaw  = (LTDC->AWCR >> 16U) & 0x0FFFu;
  uint32_t aah  = (LTDC->AWCR) & 0x0FFFu;

  /* 窗口铺满整屏 */
  LTDC_Layer1->WHPCR = ((aaw << 16U) | (ahbp + 1U));
  LTDC_Layer1->WVPCR = ((aah << 16U) | (avbp + 1U));

  LTDC_Layer1->PFCR  = LTDC_PIXEL_FORMAT_L8;       /* 0x7 */
  LTDC_Layer1->CACR  = 255u;
  LTDC_Layer1->BFCR  = (LTDC_BLENDING_FACTOR1_CA | LTDC_BLENDING_FACTOR2_CA);
  LTDC_Layer1->CFBAR = DISP_FB_ADDR;
  /* CFBLR: 高 16 位 = 行间距(字节)，低 16 位 = 行长(字节)+3 */
  LTDC_Layer1->CFBLR = ((DISP_W << 16U) | (DISP_W + 3U));
  LTDC_Layer1->CFBLNR = DISP_H;

  /* 直接置位：LEN + CLUTEN（L8 必需） */
  LTDC_Layer1->CR = (LTDC_LxCR_LEN | LTDC_LxCR_CLUTEN);
}

static void disp_setup_clut(void)
{
  uint32_t i;

  for (i = 0u; i < 256u; i++) { disp_clut[i] = 0u; }

  disp_clut[C_BG]   = 0x000000u;   /* 黑底 */
  disp_clut[C_TEXT] = 0xFFFFFFu;   /* 白 */
  disp_clut[C_DIM]  = 0x8A9299u;   /* 灰 */
  disp_clut[C_GRID] = 0x223038u;   /* 深青灰（分隔线） */
  disp_clut[C_ACC]  = 0x50E070u;   /* 绿 */
  disp_clut[C_WARN] = 0xFF5A4Eu;   /* 红 */
  disp_clut[C_BLUE] = 0x3070FFu;   /* 蓝 */
  disp_clut[C_WHITE]= 0xFFFFFFu;   /* 白 */

  /* 直接写 CLUT 寄存器：索引在 bit31:24，颜色 0x00RRGGBB */
  for (i = 0u; i < 256u; i++)
  {
    LTDC_Layer1->CLUTWR = ((i << 24U) | (disp_clut[i] & 0x00FFFFFFu));
  }
}

/* 读回 PLLSAI 配置，算出 LTDC 像素时钟（kHz）。板载 RK043FN48H 要约 9.6MHz。 */
static uint32_t disp_ltdc_clock_khz(void)
{
  uint32_t pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
  uint32_t n    = (RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIN) >> 6;
  uint32_t r    = (RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIR) >> 28;
  uint32_t divr = (RCC->DCKCFGR1 & RCC_DCKCFGR1_PLLSAIDIVR) >> 16;
  uint32_t div  = 2u << divr;              /* 00->/2, 01->/4, 10->/8, 11->/16 */

  if ((pllm == 0u) || (r == 0u)) { return 0u; }

  /* HSE = 25MHz：VCO 输入 = 25000kHz / PLLM */
  return ((25000u / pllm) * n) / r / div;
}

/* 时钟不对就按 ST BSP 的参数改回 9.6MHz：
   VCO = 1MHz x 192 = 192MHz → /5 = 38.4MHz → /4 = 9.6MHz。 */
static void disp_clock_fix(void)
{
  uint32_t khz = disp_ltdc_clock_khz();

  if ((khz >= 8600u) && (khz <= 10600u)) { return; }   /* 已经合适 */

  __HAL_LTDC_DISABLE(&hltdc);
  {
    RCC_PeriphCLKInitTypeDef c;
    (void)memset(&c, 0, sizeof(c));
    c.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    c.PLLSAI.PLLSAIN = 192;
    c.PLLSAI.PLLSAIR = 5;
    c.PLLSAI.PLLSAIQ = 2;
    c.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
    c.PLLSAIDivQ = 1;
    c.PLLSAIDivR = RCC_PLLSAIDIVR_4;
    (void)HAL_RCCEx_PeriphCLKConfig(&c);
  }
  __HAL_LTDC_ENABLE(&hltdc);
}

void disp_init(void)
{
  char b[128];
  int  n;

  disp_clock_fix();     /* 先保证像素时钟 ~9.6MHz */

  disp_setup_clut();
  disp_setup_layer();
  LTDC->SRCR = LTDC_SRCR_IMR;

  disp_render();        /* 立刻画一帧，避免开机空白 */

  n = snprintf(b, sizeof(b), "[DISP] clk %lu kHz  L1CR %08lX  PFCR %lX  CFBAR %08lX\r\n",
               (unsigned long)disp_ltdc_clock_khz(),
               (unsigned long)LTDC_Layer1->CR,
               (unsigned long)LTDC_Layer1->PFCR,
               (unsigned long)LTDC_Layer1->CFBAR);
  if (n > 0) { (void)HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)n, 80u); }
}

/* ---- 一帧：只画数据 -------------------------------------------------------- */
static void disp_render(void)
{
  static uint32_t frames = 0u;
  char     b[48];
  int32_t  dc  = mic_last_dc();
  int32_t  lo  = mic_last_min();
  int32_t  hi  = mic_last_max();
  int32_t  now = mic_last_sample();
  int32_t  u10, dc10, pp10;
  uint32_t th[10];
  uint8_t  nth;
  uint8_t  k;

  frames++;
  disp_clear();

  u10  = (int32_t)(((int64_t)now * DISP_VREF_MV * 10) / (int64_t)DISP_ADC_MAX);
  dc10 = (int32_t)(((int64_t)dc  * DISP_VREF_MV * 10) / (int64_t)DISP_ADC_MAX);
  pp10 = (int32_t)(((int64_t)(hi - lo) * DISP_VREF_MV * 10) / (int64_t)DISP_ADC_MAX);

  /* ---- 实时电压 ---- */
  (void)disp_str(10, 4, "MIC AO  VOLTAGE", C_DIM, 1);
  disp_hline(10, (int32_t)DISP_W - 11, 22, C_GRID);

  (void)snprintf(b, sizeof(b), "%ld mV", (long)(u10 / 10));
  (void)disp_str(10, 28, b, C_TEXT, 3);

  (void)snprintf(b, sizeof(b), "DC %ld.%01ld mV   pp %ld.%01ld mV   nf %lu",
                 (long)(dc10 / 10), (long)(dc10 % 10),
                 (long)(pp10 / 10), (long)(pp10 % 10),
                 (unsigned long)mic_last_nf);
  (void)disp_str(10, 84, b, C_DIM, 1);

  disp_hline(10, (int32_t)DISP_W - 11, 104, C_GRID);

  /* ---- 事件与到达时刻 ---- */
  (void)snprintf(b, sizeof(b), "evt %lu  seq %lu  pk %lu iq %lu",
                 (unsigned long)mic_event_seq, (unsigned long)mic_seq_count(),
                 (unsigned long)mic_last_pk, (unsigned long)mic_last_pk_iq);
  (void)disp_str(10, 112, b, C_TEXT, 1);

  (void)snprintf(b, sizeof(b), "t %lu us    dt %lu us",
                 (unsigned long)mic_event_t_us,
                 (unsigned long)mic_event_dt_us);
  (void)disp_str(10, 132, b, C_TEXT, 1);

  /* 结束时刻与持续时间（到达 = 起音，结束 = 包络回落到门限以下） */
  (void)snprintf(b, sizeof(b), "end %lu   dur %lu us",
                 (unsigned long)mic_event_end_us,
                 (unsigned long)mic_event_dur_us);
  (void)disp_str(10, 152, b, C_DIM, 1);

  /* 验收指标：最近 10 次事件间隔的极差 */
  (void)snprintf(b, sizeof(b), "jit %lu us   n %u/10",
                 (unsigned long)mic_dt_jitter_us, (unsigned)mic_dt_count);
  (void)disp_str(10, 172, b, C_ACC, 1);

  /* ---- 最近 10 次到达时刻（相对最老一次，单位 µs） ---- */
  (void)disp_str(10, 194, "LAST 10 ARRIVAL (us)", C_DIM, 1);

  nth = mic_t_history(th);
  if (nth > 0u)
  {
    uint32_t t0 = th[0];
    int32_t  x  = 10;

    for (k = 0u; k < nth; k++)
    {
      if (k == 5u) { x = 10; }
      (void)snprintf(b, sizeof(b), "%7lu ", (unsigned long)(th[k] - t0));
      x = disp_str(x, (k < 5u) ? 214 : 232, b, C_TEXT, 1);
    }
  }

  (void)snprintf(b, sizeof(b), "UI #%lu  fs96k  gap %lu",
                 (unsigned long)frames, (unsigned long)mic_overruns);
  (void)disp_str(10, 252, b, C_DIM, 1);
}

void disp_task(void *argument)
{
  char b[] = "[DISP] task up, drawing data\r\n";

  (void)argument;

  osDelay(100);         /* 让开机那一帧先露一下脸 */
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)(sizeof(b) - 1U), 80u);

  for (;;)
  {
    disp_render();
    osDelay(50);          /* 20 fps */
  }
}
