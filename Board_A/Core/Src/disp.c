/**
  ******************************************************************************
  * @file    disp.c
  * @brief   480x272 声源方向与声程差仪表盘（LTDC + L8 + 11x16 字库）
  *
  * 屏幕左侧固定为主板，右侧固定为从板；M-S = 主板到达时刻减从板到达时刻。
  * 最近 10 次声程差、样本标准差和 dj 显示。
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
#include "i2c.h"
#include "cmsis_os.h"
#include "LCD_Font16.h"
#include <stdio.h>
#include <string.h>
#include <math.h>


/* 两块板共用布局；启动时 IP_ADDRESS 尚未初始化，角色在各自固件中固定。 */
#define DISP_IS_MASTER 1u

/* 用户手动选定的空气温度；声速按 331.3+0.606*T m/s，保留 0.001 m/s。 */
static const int8_t disp_temp_c[5] = {-20, 0, 20, 40, 60};
static const uint32_t disp_speed_mps_x1000[5] = {319180u, 331300u, 343420u, 355540u, 367660u};
typedef enum
{
  DISP_TEMP_CLOSED,
  DISP_TEMP_MODE,
  DISP_TEMP_PRESET
} disp_temp_menu_t;

static volatile uint8_t disp_temp_sel = 2u;
static disp_temp_menu_t disp_temp_menu = DISP_TEMP_CLOSED;
static volatile uint8_t disp_auto_temp = 1u;
static uint32_t disp_temp_snapshot(int32_t *temperature10);

#define TS_ADDR       0x70u  /* STM32F746G-DISCO FT5336, HAL 8-bit address */
#define TS_TOUCH_REG  0x02u

/* ---- 调色板索引 ---- */
#define C_BG     0u
#define C_TEXT   1u
#define C_DIM    2u
#define C_GRID   3u
#define C_ACC    4u
#define C_WARN   5u
#define C_BLUE   6u
#define C_PANEL  7u

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

  disp_clut[C_BG]    = 0x08131Du;
  disp_clut[C_TEXT]  = 0xF1F7FAu;
  disp_clut[C_DIM]   = 0x9AAEBBu;
  disp_clut[C_GRID]  = 0x304454u;
  disp_clut[C_ACC]   = 0x42DBA1u;
  disp_clut[C_WARN]  = 0xFFB459u;
  disp_clut[C_BLUE]  = 0x64ADFFu;
  disp_clut[C_PANEL] = 0x122332u;

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

/* ---- 声程差换算和显示（选定温度的声速，旧→新） --------------------------- */
static void disp_temp_button(void)
{
  char b[24];
  int32_t t = 0;
  uint32_t speed = disp_temp_snapshot(&t);

  disp_fill(12, 3, 195, 32, C_GRID);
  if (speed == 0u)
  {
    (void)snprintf(b, sizeof(b), "AUTO WAIT");
  }
  else if (disp_auto_temp != 0u)
  {
    unsigned mag = (unsigned)((t < 0) ? -t : t);
    (void)snprintf(b, sizeof(b), "AUTO %s%u.%uC", t < 0 ? "-" : "", mag / 10u, mag % 10u);
  }
  else
  {
    (void)snprintf(b, sizeof(b), "SET %+dC", (int)(t / 10));
  }
  (void)disp_str(22, 10, b, C_TEXT, 1);
}

static void disp_temp_option_rect(uint8_t i, int32_t *x0, int32_t *y0,
                                  int32_t *x1, int32_t *y1)
{
  *x0 = (i == 4u) ? 160 : ((i & 1u) != 0u ? 245 : 70);
  *x1 = (i == 4u) ? 320 : ((i & 1u) != 0u ? 410 : 235);
  *y0 = (i == 4u) ? 221 : (145 + (int32_t)(i / 2u) * 38);
  *y1 = *y0 + 31;
}

static void disp_temp_popup(void)
{
  uint8_t i;
  char b[32];

  disp_fill(45, 87, 435, 264, C_BG);
  disp_fill(51, 93, 429, 258, C_PANEL);
  (void)disp_str(70, 105, (disp_temp_menu == DISP_TEMP_MODE) ? "TEMP MODE" : "SET TEMP", C_TEXT, 1);
  disp_fill(345, 99, 414, 128, C_GRID);
  (void)disp_str(352, 106, "CLOSE", C_DIM, 1);
  if (disp_temp_menu == DISP_TEMP_MODE)
  {
    disp_fill(70, 153, 235, 206, (disp_auto_temp != 0u) ? C_ACC : C_GRID);
    disp_fill(245, 153, 410, 206, (disp_auto_temp == 0u) ? C_ACC : C_GRID);
    (void)disp_str(98, 172, "DHT11 AUTO", (disp_auto_temp != 0u) ? C_BG : C_TEXT, 1);
    (void)disp_str(284, 172, "SET TEMP", (disp_auto_temp == 0u) ? C_BG : C_TEXT, 1);
    return;
  }
  disp_fill(245, 99, 320, 128, C_GRID);
  (void)disp_str(260, 106, "BACK", C_TEXT, 1);
  for (i = 0u; i < DISP_TEMP_COUNT; i++)
  {
    int32_t x0, y0, x1, y1;
    disp_temp_option_rect(i, &x0, &y0, &x1, &y1);
    disp_fill(x0, y0, x1, y1, (i == disp_temp_sel) ? C_ACC : C_GRID);
    (void)snprintf(b, sizeof(b), "%+dC %um/s", (int)disp_temp_c[i],
                   (unsigned)((disp_speed_mps_x1000[i] + 500u) / 1000u));
    (void)disp_str(x0 + 11, y0 + 8, b,
                   (i == disp_temp_sel) ? C_BG : C_TEXT, 1);
  }
}

/* FT5336: 0x02 为触点数，0x03..0x06 为第一触点坐标。板载屏交换 X/Y。 */
static void disp_touch_poll(void)
{
  static uint8_t was_down = 0u;
  uint8_t p[5];
  uint16_t x, y;
  uint8_t i;

  if (HAL_I2C_Mem_Read(&hi2c3, TS_ADDR, TS_TOUCH_REG,
                       I2C_MEMADD_SIZE_8BIT, p, sizeof(p), 10u) != HAL_OK)
  {
    return;
  }
  if ((p[0] & 0x0Fu) != 1u)
  {
    was_down = 0u;
    return;
  }
  if (was_down != 0u) { return; }
  was_down = 1u;

  x = (uint16_t)(((uint16_t)(p[3] & 0x0Fu) << 8) | p[4]);
  y = (uint16_t)(((uint16_t)(p[1] & 0x0Fu) << 8) | p[2]);
  if ((x >= DISP_W) || (y >= DISP_H)) { return; }

  if (disp_temp_menu == DISP_TEMP_CLOSED)
  {
    if ((x >= 12u) && (x <= 195u) && (y >= 3u) && (y <= 32u))
    {
      disp_temp_menu = DISP_TEMP_MODE;
    }
    return;
  }
  if ((x >= 345u) && (x <= 414u) && (y >= 99u) && (y <= 128u))
  {
    disp_temp_menu = DISP_TEMP_CLOSED;
    return;
  }
  if (disp_temp_menu == DISP_TEMP_MODE)
  {
    if ((y >= 153u) && (y <= 206u))
    {
      if ((x >= 70u) && (x <= 235u))
      {
        lt_temp_select(DISP_TEMP_AUTO);
        disp_temp_menu = DISP_TEMP_CLOSED;
      }
      else if ((x >= 245u) && (x <= 410u))
      {
        disp_temp_menu = DISP_TEMP_PRESET;
      }
    }
    return;
  }
  if ((x >= 245u) && (x <= 320u) && (y >= 99u) && (y <= 128u))
  {
    disp_temp_menu = DISP_TEMP_MODE;
    return;
  }
  for (i = 0u; i < DISP_TEMP_COUNT; i++)
  {
    int32_t x0, y0, x1, y1;
    disp_temp_option_rect(i, &x0, &y0, &x1, &y1);
    if (((int32_t)x >= x0) && ((int32_t)x <= x1) &&
        ((int32_t)y >= y0) && ((int32_t)y <= y1))
    {
      lt_temp_select(i);
      disp_temp_menu = DISP_TEMP_CLOSED;
      break;
    }
  }
}

/* 声源指向先收到声音的一侧。屏幕左=主板，右=从板。 */
static void disp_arrow(int32_t x, int32_t y, int32_t dir)
{
  int32_t k;

  disp_fill(x - 25, y - 3, x + 25, y + 3, C_ACC);
  for (k = 0; k <= 16; k++)
  {
    int32_t px = x + dir * (24 + k);
    int32_t h = 16 - k;

    disp_fill(px, y - h, px, y + h, C_ACC);
  }
}

/* 最近 10 次声程差，五列两行，左旧右新，保留 0.1 cm 读数。 */
static void disp_history(const uint32_t *v, uint8_t n)
{
  char b[20];
  uint8_t i;

  for (i = 0u; i < n; i++)
  {
    uint32_t mm = v[i];
    int32_t x = 20 + (int32_t)(i % 5u) * 90;
    int32_t y = 229 + (int32_t)(i / 5u) * 19;
    if (mm == MIC_DISTANCE_UNAVAILABLE)
    {
      (void)disp_str(x, y, "--", C_DIM, 1);
      continue;
    }
    (void)snprintf(b, sizeof(b), "%lu.%lu", (unsigned long)(mm / 10u),
                   (unsigned long)(mm % 10u));
    (void)disp_str(x, y, b, (i == n - 1u) ? C_ACC : C_TEXT, 1);
  }
}

/* 固定元素只绘制一次。LTDC 一直扫描同一块帧缓冲，不能周期性整屏清零。 */
static void disp_draw_base(void)
{
  disp_clear();
  disp_fill(0, 0, 479, 35, C_PANEL);
  disp_fill(0, 0, 5, 35, C_ACC);
  (void)disp_str(351, 10, (DISP_IS_MASTER != 0u) ? "MASTER .10" : "SLAVE  .11", C_BLUE, 1);

  disp_fill(10, 43, 292, 143, C_PANEL);
  disp_fill(300, 43, 469, 143, C_PANEL);
  (void)disp_str(20, 51, "SOURCE DIRECTION", C_DIM, 1);
  (void)disp_str(313, 51, "PATH DIFF cm", C_DIM, 1);
  (void)disp_str(22, 88, "MASTER", C_BLUE, 1);
  (void)disp_str(209, 88, "SLAVE", C_BLUE, 1);
  disp_fill(24, 109, 49, 133, C_BLUE);
  disp_fill(251, 109, 276, 133, C_BLUE);
  disp_hline(52, 248, 121, C_GRID);

  disp_fill(10, 150, 158, 198, C_PANEL);
  disp_fill(166, 150, 314, 198, C_PANEL);
  disp_fill(322, 150, 469, 198, C_PANEL);
  (void)disp_str(20, 155, "MASTER us", C_DIM, 1);
  (void)disp_str(176, 155, "SLAVE us", C_DIM, 1);
  (void)disp_str(332, 155, "M-S us", C_DIM, 1);

  disp_fill(10, 205, 469, 268, C_PANEL);
  (void)disp_str(20, 211, "LAST 10", C_DIM, 1);
  disp_temp_button();
}

/* 只对变化的读数局部清底并重画；收敛提示可独立更新。 */
static void disp_render(void)
{
  static uint8_t drawn = 0u, last_paired = 0u, last_tpl_ok = 0u;
  static uint8_t last_nth = 0u, last_conv_ok = 0u;
  static uint32_t last_sd_count = 0u;
  static double last_variance_mm2 = 0.0;
  static uint8_t last_temp_sel = 2u, last_temp_menu = 0u;
  static uint32_t last_lus = 0u, last_pus = 0u, last_dj = 0u, last_conv_ms = 0u;
  static int32_t last_dt = 0;
  static uint32_t last_hist[10];
  static uint32_t last_speed;
  static uint8_t last_auto = 1u;
  char     b[48];
  uint32_t lus = 0u, pus = 0u;
  uint32_t conv_ms = 0u;
  uint32_t master_us, slave_us;
  int32_t  dt = 0, master_minus_slave;
  uint32_t distance_mm = 0u, hist[10];
  uint8_t  tpl_ok = mic_tpl_valid;
  uint8_t  paired, kind;
  static uint8_t last_kind;
  uint8_t  nth = 0u;
  uint8_t  conv_ok, acoustic_changed, full_redraw;
  uint32_t dj_us = 0u, sd_count;
  double variance_mm2;
  uint32_t speed = disp_measurement_speed_x1000();
  uint8_t air_changed = (uint8_t)((speed != last_speed) || (disp_auto_temp != last_auto));
  paired = mic_tdoa_display_snap(&lus, &pus, &dt, &distance_mm, hist, 10u, &nth,
                                 &dj_us, &kind, &sd_count, &variance_mm2);

  /* 没配对上时不显示任何结果，避免旧读数被当成验收数据。 */
  if (tpl_ok == 0u) { paired = 0u; }
  if (paired == 0u) { nth = 0u; }
  conv_ok = lt_conv_display_snap(&conv_ms);

  full_redraw = (uint8_t)((drawn == 0u) || (disp_temp_menu != last_temp_menu));

  acoustic_changed = (uint8_t)((full_redraw != 0u) || (paired != last_paired) || (kind != last_kind) ||
                               (tpl_ok != last_tpl_ok) || (nth != last_nth) ||
                               (lus != last_lus) || (pus != last_pus) ||
                               (dt != last_dt) || (dj_us != last_dj) ||
                               (disp_temp_sel != last_temp_sel));
  if ((air_changed != 0u) || (sd_count != last_sd_count) ||
      (variance_mm2 != last_variance_mm2))
  {
    acoustic_changed = 1u;
  }
  if ((acoustic_changed == 0u) && (nth != 0u) &&
      (memcmp(hist, last_hist, (size_t)nth * sizeof(hist[0])) != 0))
  {
    acoustic_changed = 1u;
  }

  if (full_redraw != 0u) { disp_draw_base(); }
  else if ((disp_temp_sel != last_temp_sel) || air_changed) { disp_temp_button(); }
  if ((full_redraw != 0u) || (conv_ok != last_conv_ok) || (conv_ms != last_conv_ms))
  {
    disp_fill(201, 8, 345, 27, C_PANEL);
    if (conv_ok != 0u)
    {
      (void)snprintf(b, sizeof(b), "CONF T+%lu.%02lus",
                     (unsigned long)(conv_ms / 1000u),
                     (unsigned long)((conv_ms % 1000u) / 10u));
      (void)disp_str(201, 10, b, C_ACC, 1);
    }
    else { (void)disp_str(212, 10, "SYNC WAIT", C_DIM, 1); }
  }
  if (acoustic_changed == 0u)
  {
    last_conv_ok = conv_ok;
    last_conv_ms = conv_ms;
    return;
  }

  master_us = (DISP_IS_MASTER != 0u) ? lus : pus;
  slave_us = (DISP_IS_MASTER != 0u) ? pus : lus;
  master_minus_slave = (DISP_IS_MASTER != 0u) ? dt : -dt;

  /* 上半屏动态内容：保留静态标签和两端位置。 */
  disp_fill(75, 70, 202, 89, C_PANEL);
  disp_fill(52, 105, 249, 137, C_PANEL);
  disp_hline(52, 248, 121, C_GRID);
  disp_fill(311, 49, 467, 69, C_PANEL);
  (void)disp_str(313, 51, (kind == MIC_EVENT_CLAP) ? "CLAP cm" :
                         (kind == MIC_EVENT_CHIRP) ? "CHIRP cm" : "PATH DIFF cm", C_DIM, 1);
  disp_fill(311, 76, 467, 108, C_PANEL);

  if (paired != 0u)
  {
    if (distance_mm == MIC_DISTANCE_UNAVAILABLE)
    {
      (void)disp_str(313, 77, "--", C_DIM, 2);
    }
    else
    {
      (void)snprintf(b, sizeof(b), "%lu.%lu", (unsigned long)(distance_mm / 10u),
                     (unsigned long)(distance_mm % 10u));
      (void)disp_str(313, 77, b, C_ACC, 2);
    }
    if (master_minus_slave == 0)
    {
      (void)disp_str(108, 71, "CENTER", C_ACC, 1);
      disp_fill(145, 102, 152, 127, C_ACC);
    }
    else
    {
      (void)disp_str(86, 71, (master_minus_slave < 0) ? "TO MASTER" : "TO SLAVE", C_ACC, 1);
      disp_arrow(150, 121, (master_minus_slave < 0) ? -1 : 1);
    }
  }
  else
  {
    (void)disp_str(313, 77, "--", C_DIM, 2);
    (void)disp_str(91, 71, (tpl_ok == 0u) ? "REF WAIT" : ((speed == 0u) ? "WAIT TEMP" : "WAIT PAIR"), C_WARN, 1);
  }

  if ((full_redraw != 0u) || (air_changed != 0u))
  {
    disp_fill(311, 109, 467, 124, C_PANEL);
    if (speed == 0u)
    {
      (void)disp_str(313, 109, "C -- m/s", C_DIM, 1);
    }
    else
    {
      (void)snprintf(b, sizeof(b), "C %lu.%03lum/s",
                     (unsigned long)(speed / 1000u), (unsigned long)(speed % 1000u));
      (void)disp_str(313, 109, b, C_ACC, 1);
    }
  }

  /* 三个固定位置的时间读数；差值统一定义为主板减从板。 */
  disp_fill(20, 175, 155, 195, C_PANEL);
  disp_fill(176, 175, 311, 195, C_PANEL);
  disp_fill(332, 175, 466, 195, C_PANEL);
  if (paired != 0u)
  {
    (void)snprintf(b, sizeof(b), "%lu", (unsigned long)master_us);
    (void)disp_str(20, 177, b, C_TEXT, 1);
    (void)snprintf(b, sizeof(b), "%lu", (unsigned long)slave_us);
    (void)disp_str(176, 177, b, C_TEXT, 1);
    (void)snprintf(b, sizeof(b), "%ld", (long)master_minus_slave);
    (void)disp_str(332, 177, b, C_ACC, 1);
  }
  else
  {
    (void)disp_str(20, 177, "--", C_DIM, 1);
    (void)disp_str(176, 177, "--", C_DIM, 1);
    (void)disp_str(332, 177, "--", C_DIM, 1);
  }

  /* 最近十次的声程差，左旧右新。SD 从当前距离确认时开始累计。 */
  disp_fill(99, 209, 141, 228, C_PANEL);
  disp_fill(145, 209, 337, 228, C_PANEL);
  disp_fill(343, 209, 468, 228, C_PANEL);
  disp_fill(20, 229, 459, 265, C_PANEL);
  (void)snprintf(b, sizeof(b), "%u/10", (unsigned)nth);
  (void)disp_str(99, 211, b, C_DIM, 1);
  {
    const char *sd_label = (kind == MIC_EVENT_CLAP) ? "CLAP SD" :
                           (kind == MIC_EVENT_CHIRP) ? "CHIRP SD" : "SD";

    if ((paired != 0u) && (sd_count >= 2u))
    {
      uint32_t sd = (uint32_t)(sqrt(variance_mm2) * 10.0 + 0.5);
      (void)snprintf(b, sizeof(b), "%s %lu.%02lucm", sd_label,
                     (unsigned long)(sd / 100u), (unsigned long)(sd % 100u));
    }
    else { (void)snprintf(b, sizeof(b), "%s --", sd_label); }
    (void)disp_str(145, 211, b, C_ACC, 1);
  }
  if (nth >= 2u) { (void)snprintf(b, sizeof(b), "dj %luus", (unsigned long)dj_us); }
  else           { (void)snprintf(b, sizeof(b), "dj --"); }
  (void)disp_str(343, 211, b, (nth >= 2u && dj_us >= MIC_TDOA_GOOD_US) ? C_WARN : C_ACC, 1);
  disp_history(hist, nth);

  if (disp_temp_menu != 0u) { disp_temp_popup(); }

  drawn = 1u;
  last_speed = speed;
  last_auto = disp_auto_temp;
  last_temp_sel = disp_temp_sel;
  last_temp_menu = disp_temp_menu;
  last_paired = paired;
  last_kind = kind;
  last_tpl_ok = tpl_ok;
  last_nth = nth;
  last_sd_count = sd_count;
  last_variance_mm2 = variance_mm2;
  last_lus = lus;
  last_pus = pus;
  last_dt = dt;
  last_dj = dj_us;
  last_conv_ok = conv_ok;
  last_conv_ms = conv_ms;
  if (nth != 0u) { (void)memcpy(last_hist, hist, (size_t)nth * sizeof(hist[0])); }
}

#include "lt_dht11_pb4.h"
/* Publish the sensor reading even when A is locally using a preset. */
uint32_t disp_air_tx_snap(int32_t *t)
{
  uint32_t mask = __get_PRIMASK();
  uint32_t speed = 0u;

  __disable_irq();
  if (dht_initialized && dht_valid)
  {
    int32_t product;
    int32_t delta;
    *t = dht_calibrated_temp10();
    product = 606 * *t;
    delta = (product >= 0) ? (product + 5) / 10 : -((-product + 5) / 10);
    speed = (uint32_t)(331300 + delta);
  }
  __set_PRIMASK(mask);
  return speed;
}

static uint32_t disp_temp_snapshot(int32_t *t)
{
  uint32_t mask = __get_PRIMASK();
  uint32_t speed = 0u;

  __disable_irq();
  if (disp_auto_temp != 0u)
  {
    speed = disp_air_tx_snap(t);
  }
  else
  {
    uint8_t index = disp_temp_sel;
    *t = (int32_t)disp_temp_c[index] * 10;
    speed = disp_speed_mps_x1000[index];
  }
  __set_PRIMASK(mask);
  return speed;
}

uint32_t disp_measurement_speed_x1000(void)
{
  int32_t temperature10;
  return disp_temp_snapshot(&temperature10);
}

/* Apply the shared setting as one snapshot before drawing or computing speed. */
static void disp_temp_apply_selection(void)
{
  uint32_t mask = __get_PRIMASK();
  uint8_t selection;

  __disable_irq();
  selection = lt_temp_selected();
  if (selection < DISP_TEMP_COUNT) { disp_temp_sel = selection; }
  disp_auto_temp = (uint8_t)(selection == DISP_TEMP_AUTO);
  __set_PRIMASK(mask);
}

void disp_task(void *argument)
{
  char b[] = "[DISP] task up, drawing data\r\n";
  uint8_t touch_ready = 0u;
  uint32_t retry_at = 0u;

  (void)argument;

  osDelay(100);         /* 让开机那一帧先露一下脸 */
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)(sizeof(b) - 1U), 80u);

  for (;;)
  {
    if ((touch_ready == 0u) && (HAL_GetTick() >= retry_at))
    {
      touch_ready = (HAL_I2C_IsDeviceReady(&hi2c3, TS_ADDR, 1u, 10u) == HAL_OK) ? 1u : 0u;
      retry_at = HAL_GetTick() + 2000u;
    }
    if (touch_ready != 0u) { disp_touch_poll(); }
    disp_temp_apply_selection();
    disp_render();
    dht11_draw(); dht11_poll(); dht11_report();
    osDelay(20);          /* 触摸轮询 50 Hz；数据未变时不写帧缓冲 */
  }
}
