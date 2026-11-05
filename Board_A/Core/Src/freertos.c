/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netif.h"      /* 用来查链路状态 netif_is_link_up() */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "usart.h"
#include "tim.h"
#include "mic.h"
#include "disp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 时间戳交换协议的报文，定长 28 字节。
   四个时间戳都只带低 32 位微秒：一次交换在毫秒内完成，四个时刻相距极近，
   32 位无符号减法本身就是回绕安全的，不需要 64 位。

   类型 4（TOA）是后加的"声到达时刻交换"，借同一套字段、长度仍是 28 字节：
   这样收报文处的长度校验和"哪些类型算通信建立"的判断范围都不用改。
   DREQ 的 pad[0] 第 0 位用于告知主板：从板已完成频率预标定。
   TOA 的字段含义：seq = 本板事件序号，t1 = 该事件在本地时钟上的到达时刻（µs），
   t2 = 0（V6.0 保留字段，不再区分判决路径），其余字段为 0。 */
typedef struct
{
  uint8_t  type;    /* 1=SYNC(主板->从板) 2=DREQ(从板->主板) 3=DRESP(主板->从板) */
  uint8_t  pad[3];  /* DREQ.pad[0] bit0 = 从板频率预标定完成 */
  uint32_t seq;
  uint32_t t1;      /* 主板发 SYNC 的时刻（主板时钟） */
  uint32_t t2;      /* 从板收 SYNC 的时刻（从板时钟） */
  uint32_t t3;      /* 从板发 DREQ 的时刻（从板时钟） */
  uint32_t t4;      /* 主板收 DREQ 的时刻（主板时钟） */
  int32_t  off_m;   /* 主板算出的偏移，由 DRESP 带回给从板（伺服用它）*/
  uint32_t dly_m;
} lt_msg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LT_PORT             5000U   /* 本板与对端统一使用这个 UDP 端口 */
#define LT_REPORT_PERIOD_MS 2000U   /* 串口统计打印周期 */
#define LT_SYNC_PERIOD_MS   100U    /* 主板发起一次测量的周期 */
#define LT_MAX_SAMPLES      512U    /* 回归窗口长度（样本数，约 51 秒）*/
#define LT_DELAY_MARGIN_US  20U     /* 回归的时延筛选余量，与伺服判据保持一致。
                                       原来放 300us 太松，会把被打印扰动过的样本
                                       （只高出约 95us）也放进来，它们的偏移会把
                                       速率估计带偏，进而让 ARR 修正量错一个量级。 */
#define LT_MSG_SYNC         1U
#define LT_MSG_DREQ         2U
#define LT_MSG_DRESP        3U
#define LT_MSG_TOA          4U      /* 声到达时刻交换：新加的，不参与同步伺服 */
#define LT_FLAG_FF_READY    0x01U   /* DREQ.pad[0]：从板频率预标定已结束 */
#define LT_DLY_ACCEPT_US    20U
#define LT_DLY_RECENT_MS    2000U   /* V5.8：dly 地板只统计最近 2 秒的样本。
                                       旧版取 512 个样本（约 51 秒）的最小值，只要
                                       51 秒内出现过一次快档，快档消失后门限仍停在
                                       快档，慢档样本会被全部剔除；伺服随之停摆，
                                       残余频差让示波器间距越漂越大。 */
#define LT_DLY_MODE_DIFF_US 30U     /* 地板变化超过这个量，认为换了一条时延路径 */
#define LT_CONV_LIMIT_MS    30000U  /* 收敛时限：任务书要求 30 秒 */
#define LT_CONV_BAND_US     8       /* 判据带：比任务书的 ±10us 再收紧 2us，给标定
                                       残差和示波器读数留余量。环路稳态误差在 1~2us
                                       量级，8us 的带子不会拖慢判定；原来放 30us 会
                                       误报提前，放到 10us 又等于"踩着指标宣告收敛"。 */
#define LT_CONV_HOLD_MS     10000U  /* 要连续保持这么久才算收敛（与验收"连续 10 次"
                                       对应：脉冲是 1Hz，连续 10 次测量就是 10 秒）*/
#define LT_CONV_MIN_SAMP    20U     /* 这段稳定期里至少要有这么多干净样本，
                                       防止"样本被剔光、等于没判"也算稳定 */
#define LT_CONV_GAP_MS      2000U   /* V5.7：干净样本断流超过这么久，稳定期作废。
                                       旧判据只数样本、不看样本年龄：干净样本在
                                       第 20 秒被 dly 门限全部剔掉后，st 会冻在
                                       51，再等 10 秒就误报 CONVERGED——现场那次
                                       56.79 s 就是这么来的，当时当前样本的 offm
                                       还是 -64us，根本没有收敛。 */

/* ---- ARR（频率）环路的参数（V5.6 起：先预标定、再慢积分）---------------------
   老版本只靠"相位偏差积分"把 ARR 推到正确频差上：积分在 10 Hz 上累加，而
   ARR 要等下一个 1 s 边界才生效，很容易冲过头，形成几秒~几十秒周期的摆动——
   收敛慢、稳住后也常在 ±10us 边上蹭。现在频差由 3 s 的预标定一次测准，
   积分只负责温漂和残差，增益可以小一个数量级。 */
#define LT_ARR_INT_GAIN     10      /* 残余积分增益：每 1us 偏差每拍给 ARR 加 10 计数
                                       （10 Hz 下最多约 0.9 ppm/秒），够跟温漂 */
#define LT_ARR_INT_STEP_MAX 30      /* 每拍最多 30 计数（约 0.28ppm）：脏样本拉不走 */
#define LT_ARR_INT_RANGE_US 25      /* |偏差| > 25us 时不积分：先把相位拉回来。
                                       留 25 而不是 15，是为了让"预标定万一没测准"
                                       也能靠慢积分自己救回来（残差再大，相位环路的
                                       稳态滞后也在 25us 以内，积分不会被门限锁死）*/

/* ---- V5.7 频率预标定（feed-forward）-----------------------------------------
   现场日志证明 V5.6 的 3 秒窗口不够：干净样本只有每秒 2~3 个，3 秒里常常只有
   6~10 个点，dly 换档带来的 96us 台阶和量化噪声会完全盖住 16ppm 的真实斜率
   （B 板最后一次只算出 arr=+464，理论应约 ±1700）。窗口拉长到 6 秒以上，
   再用 Theil-Sen 中值斜率代替最小二乘：台阶/野值只污染少数点对，中值不受影响。
   测量期仍严格不拨相位、不动 ARR。 */
#define LT_FF_SAMPLES       64U     /* 最多存 64 个干净样本（10 Hz 时约 6.4 s）*/
#define LT_FF_MIN_SAMP      24U     /* 少于 24 个点不结算：斜率会被台阶带偏 */
#define LT_FF_SPAN_MS       6000U   /* 至少覆盖 6 s；10 Hz 下 64 点已够 */
#define LT_FF_GIVEUP_MS     12000U  /* 等到 12 s 仍不足 24 点：放弃预标定 */
#define LT_FF_ARR_MAX       8000    /* ARR 修正上限：约 ±74 ppm（晶振差不可能更大）*/
/* MAC 发送时间戳启用后的 PPS 标定值。0 是待实测初值，不是精度结论。
   两板均烧录同一取值；稳定后量 A/B 的 PI0 上升沿，B 先到记正 S，
   再按 新CAL = 旧CAL - S 调整，并复测两个网络时延档位。 */
#define LT_OFF_CAL_US       (0)

/* 致动器验证模式。置 1 时不做闭环伺服，而是每 10 秒给本地时钟一个固定的
   +100us 相位拨动。用示波器看脉冲间距：若呈 100us 台阶的锯齿状，说明
   "拨动 -> 时钟 -> 脉冲"这条通路是通的，问题在控制器；若纹丝不动，
   说明拨动没有真正作用到脉冲上。 */
#define LT_NUDGE_TEST       0
/* 对端 IP 不写死：在 lt_start() 里按本板 IP 自动推导，两块板可烧同一份固件 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static int lt_sock = -1;
static volatile uint32_t lt_dly_from_master = 0U;
static volatile uint32_t lt_servo_skip = 0U;
volatile int32_t lt_off_clean = 0;         /* 对外只读：屏幕未配对诊断位要用（disp.c）*/
static struct sockaddr_in lt_peer;
extern uint8_t IP_ADDRESS[4];              /* 定义在 lwip.c，本板的 IPv4 地址 */
extern struct netif gnetif;                /* 定义在 lwip.c，查链路是否建立用它 */
static uint8_t  lt_link_announced = 0U;    /* "链路建立"那行只打印一次 */

/* --- 收敛计时：上电到两板时钟真正对齐所用的时间 ---
   两端都取物理时刻，不取"判定成立"的时刻：

     起点：通信建立（首次收到对方报文）。不用上电时刻，是因为两块板不是同时
       上电的，先上电的那块会把自己等待另一块板、等 PHY 自协商的时间也算进去，
       两块板报出来的数就会差一个上电时差。以"能通上话"为起点，两块板量到的
       是同一件事，而且这正是同步算法真正开始工作的时刻。
       不过上电到收敛那个数也一起打出来，方便与任务书的表述对照。
     终点：判据量的"稳定期"起点。收敛定义 = 偏差进 ±LT_CONV_BAND_US 带内
       且连续保持 LT_CONV_HOLD_MS（10 秒）不出去（期间至少 LT_CONV_MIN_SAMP 个
       干净样本）——这样才排除掉"锁上一次又被频率环路拉出去"的假收敛。
       物理上两板是在这段稳定期的起点真正收敛到 ±10us 以内的，
       所以用时取这个起点，不取"判定成立"的时刻（后者还要多等 10 秒）。 */
static volatile uint32_t lt_conv_boot_ms = 0U;   /* 本板上电时刻（近似）*/
static volatile uint32_t lt_conv_comm_ms = 0U;   /* 通信建立时刻（首次收到对方报文）*/
static volatile uint8_t  lt_conv_comm_up = 0U;   /* 1 = 通信已建立 */
volatile uint8_t  lt_conv_ok      = 0U;   /* 1 = 已收敛（对外只读，屏幕诊断位用）*/
static volatile uint32_t lt_conv_ms_comm = 0U;   /* 收敛用时：通信建立 -> 进带 */
static volatile uint32_t lt_conv_ms_pwr  = 0U;   /* 收敛用时：本板上电 -> 进带 */
static volatile uint32_t lt_conv_ms_ann_pwr = 0U;/* V5.9：本板上电 -> 宣告（含 10s 稳定期）*/
static volatile uint8_t  lt_boot_marked  = 0U;   /* lt_boot_mark() 只认第一次打点 */
static uint32_t lt_conv_t_first_ms = 0U;         /* 本段首个进带样本的时刻 */
static uint32_t lt_conv_streak     = 0U;         /* 连续在带内的干净样本数 */
static uint32_t lt_conv_last_ms    = 0U;         /* V5.7：最近一个干净样本的时刻 */
static uint8_t  lt_clean_lost      = 0U;         /* V5.7：干净样本断流提示只打一次 */
static uint8_t  lt_conv_announced  = 0U;         /* 成功提示只打印一次 */

static uint8_t  lt_is_master = 0U;   /* 1=主板（时间基准，自己的时钟不动）*/
static uint32_t lt_seq       = 0U;   /* 主板发起测量的序号 */
static uint32_t lt_cur_seq   = 0U;   /* 从板：当前正在处理的测量序号 */
static uint32_t lt_t1 = 0U;          /* 主板：发 SYNC 的时刻 */
static uint32_t lt_t2 = 0U;          /* 从板：收 SYNC 的时刻 */
static uint32_t lt_t3 = 0U;          /* 从板：发 DREQ 的时刻 */

static uint32_t lt_mes_ok  = 0U;     /* 完成的测量次数 */
static uint32_t lt_mes_bad = 0U;     /* 被丢弃的测量次数 */
static volatile int32_t  lt_cur_off_us = 0;    /* 最近一次偏移（从板 - 主板）*/
static volatile uint32_t lt_cur_dly_us = 0U;   /* 最近一次往返时延 */
static volatile int32_t  lt_slope_ppb  = 0;    /* 回归得到的频率差 */
static volatile uint32_t lt_win_cnt    = 0U;   /* 参与回归的样本数 */
static volatile int32_t  lt_min_dly_us = 0;    /* 窗口内最小往返时延 */
static uint32_t lt_arr_nominal = 0U;           /* TIM5 名义 ARR（每秒计数个数 - 1）*/
static int32_t  lt_arr_delta   = 0;            /* 当前施加的频率修正量，单位计数 */
static uint8_t  lt_ff_state    = 0U;           /* 0=等首个干净样本 1=测频差 2=闭环 */
static uint8_t  lt_ff_n        = 0U;           /* 测量期已攒样本数 */
static uint32_t lt_ff_t[LT_FF_SAMPLES];        /* 测量期样本：本地毫秒时刻 */
static uint32_t lt_ff_dly[LT_FF_SAMPLES];      /* 测量期样本：dly（µs，用来识别换档）*/
static int32_t  lt_ff_off[LT_FF_SAMPLES];      /* 测量期样本：off（µs，从板−主板）*/
static int32_t  lt_ff_slopes[2048];            /* Theil-Sen 两两点对斜率（ppb）*/
static int32_t  lt_ff_slope_ppb = 0;           /* V5.7 预标定结果：频差（ppb）*/
static int32_t  lt_ff_cnt       = 0;           /* V5.7 预标定结果：ARR 计数*/
static uint32_t lt_ff_span_ms   = 0U;          /* V5.7 预标定覆盖的时间跨度 */
static uint32_t lt_ff_used      = 0U;          /* V5.7 实际参与拟合的样本数 */
static uint32_t lt_ff_good      = 0U;          /* V5.7 残差在 25us 内的样本数 */
static int32_t  lt_ff_resid     = 0;           /* V5.7 最大残差（us，诊断）*/
static uint8_t  lt_ff_print     = 0U;          /* 1 = 有一行预标定结果等打印任务输出 */
static uint32_t lt_dly_old_us   = 0U;          /* V5.7 最近一次 dly 地板换档 */
static uint32_t lt_dly_new_us   = 0U;
static uint8_t  lt_dly_print    = 0U;          /* 1 = 有一行 dly 换档等打印任务输出 */
static uint32_t lt_dly_changes  = 0U;          /* dly 地板换档次数（诊断）*/

/* --- 诊断用：把最近一次交换的四个原始时间戳打印出来，便于两块板逐字段对照 --- */
static volatile uint32_t lt_rx_msgs = 0U;      /* 收到的报文数 */
static volatile uint32_t lt_tx_stamp_ok = 0U;  /* V5.8：在 MAC 发送前重打时间戳的次数 */
static volatile uint32_t lt_dbg_seq = 0U;
static volatile uint32_t lt_dbg_t1 = 0U, lt_dbg_t2 = 0U, lt_dbg_t3 = 0U, lt_dbg_t4 = 0U;
static volatile int32_t lt_phase_total_us = 0; /* 跨任务读取的累计拨钟量 */

/* 样本窗口：index 0 最旧，按时间先后排列。
   时刻用毫秒存，避免后面做回归时数字太大溢出。 */
static int32_t  lt_s_off[LT_MAX_SAMPLES];
static uint32_t lt_s_dly[LT_MAX_SAMPLES];
static uint32_t lt_s_t[LT_MAX_SAMPLES];
static uint32_t lt_s_cnt = 0U;

static const osThreadAttr_t lt_tx_attr = {
  .name = "ltTx",
  .stack_size = 512 * 4,
  /* 打印任务必须比接收任务、麦克风任务、TCP/IP 线程都低。
     串口打印是阻塞式轮询，一次十几毫秒；旧版把它放在 Normal(24)，和 tcpip
     线程同优先级，时间片轮转会让 DREQ/DRESP 的入队延迟抖出约 96us——现场
     dly 在 291/383us 两档之间成片跳变，时间片是首要嫌疑。V5.7 降到 Low(8)：
     打印只会被别人抢占，不会反过来拖住同步收发。 */
  .priority = (osPriority_t)osPriorityLow,
};
static const osThreadAttr_t lt_rx_attr = {
  .name = "ltRx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityAboveNormal,
};

/* 麦克风采样处理任务：栈 512 字（2KB）、优先级 Normal。
   必须低于 ltRx(AboveNormal)——串口打印是阻塞式轮询、一次几毫秒，抢了 ETH
   接收任务的优先级会把第一步压干净的 dly 又污染掉（文档 02 的 E4）。 */
static const osThreadAttr_t mic_attr = {
  .name = "micTask",
  .stack_size = 512 * 4,
  /* V5.7：降到 BelowNormal(16)。事件时刻来自 DMA 半缓冲的样本序号，不依赖
     任务被调度的那一刻，所以降优先级不会损失 TDOA 精度；反而让 tcpip(24)
     始终高于 mic/打印，DREQ/DRESP 入队不再被 1ms 时间片切成两档。 */
  .priority = (osPriority_t)osPriorityBelowNormal,
};

/* 显示任务：20fps 刷屏，优先级放在 micTask 之下——画屏只是"看"，不能影响
   采样处理与以太网接收。 */
static const osThreadAttr_t disp_attr = {
  .name = "dispTask",
  .stack_size = 512 * 4,
  /* 画屏优先级再降一档，保证 micTask 永远先于画屏运行。 */
  .priority = (osPriority_t)osPriorityLow,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void lt_log(const char *fmt, ...);
static void lt_start(void);
static void lt_tx_task(void *argument);
static void lt_rx_task(void *argument);
static void lt_push_sample(int32_t off_us, uint32_t dly_us, uint32_t t_ms);
static void lt_run_regression(void);
static void lt_servo(int32_t off_us, uint32_t dly_us);
static void lt_conv_push(int32_t off_us);
static void lt_conv_reset(void);
static void lt_conv_poll(void);
static void lt_conv_report(void);
uint32_t lt_tx_stamp_frame(uint8_t *frame, uint32_t len);
void lt_boot_mark(void);                          /* main() 上电打点 */
static volatile int32_t lt_off_from_master = 0;   /* 主板算出的偏移 */
static volatile int32_t lt_servo_in = 0;          /* 伺服实际收到的输入（诊断用）*/
static volatile int32_t lt_step_effect = 0;       /* 拨动命令实际使时钟移动了多少（应为 -step）*/

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  {
    /* 麦克风采样处理任务（mic.c）与显示任务（disp.c）。
       把创建结果和剩余堆打出来：若显示任务创建失败，屏幕会停在开机那一帧。 */
    osThreadId_t h_mic = osThreadNew(mic_task, NULL, &mic_attr);
    osThreadId_t h_dsp = osThreadNew(disp_task, NULL, &disp_attr);

    lt_log("[RTOS] mic %s  disp %s  heap %lu\r\n",
           (h_mic != NULL) ? "ok" : "FAIL",
           (h_dsp != NULL) ? "ok" : "FAIL",
           (unsigned long)xPortGetFreeHeapSize());
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* V5.9：上电时刻由 main() 在 HAL_Init() 之后立刻打点（lt_boot_mark()），
     覆盖外设/显示/采样初始化。旧版在这里（MX_LWIP_Init() 之后）才记点，
     "上电->链路/收敛"会把这段初始化整个漏掉。这里再调一次只是兜底。 */
  lt_boot_mark();
  lt_start();   /* 启动链路自检；第二步会替换成正式的通信任务 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* V5.9：main() 在 HAL_Init() 之后立刻调用，把"上电时刻"钉在最早的
   HAL tick 上（HAL_Init 里已经起了 SysTick）。只认第一次：后到的
   调用（StartDefaultTask 的兜底）不再改动基点。 */
void lt_boot_mark(void)
{
  if (lt_boot_marked == 0U)
  {
    lt_boot_marked  = 1U;
    lt_conv_boot_ms = HAL_GetTick();
  }
}

/* 屏幕显示稳定等待期结束、正式确认收敛的上电用时。 */
uint8_t lt_conv_display_snap(uint32_t *confirmed_ms)
{
  uint8_t ok = lt_conv_ok;
  *confirmed_ms = (ok != 0U) ? lt_conv_ms_ann_pwr : 0U;
  return ok;
}

/* Acoustic timestamp correction only; it does not drive the clock servo.
   Capture the phase associated with t2/t3 so a later servo step is not
   subtracted twice. Expired or noisy estimates fall back to raw TOA. */
static int32_t lt_mic_exchange_phase_us;
#define LT_MIC_FIT_N 9u
typedef struct {
  uint32_t stamp;
  int32_t phase, offset;
} lt_mic_fit_sample_t;
static lt_mic_fit_sample_t lt_mic_fit_samples[LT_MIC_FIT_N];
static uint8_t lt_mic_fit_index, lt_mic_fit_count;
extern int32_t lt_rollover_jump_us(void);
extern volatile int32_t lt_rx_acoustic_phase_us;

int32_t lt_acoustic_phase_snapshot_us(void)
{
  /* Unstep both explicit servo changes and the nominal-second/ARR gap. */
  return lt_phase_total_us - lt_rollover_jump_us();
}

static double lt_mic_median(double *v, uint32_t n)
{
  uint32_t i, j;
  for (i = 1u; i < n; i++) {
    double x = v[i]; j = i;
    while (j && v[j-1u] > x) { v[j] = v[j-1u]; j--; }
    v[j] = x;
  }
  return (n & 1u) ? v[n/2u] : (v[n/2u-1u]+v[n/2u])*0.5;
}

/* Called only by micTask. Scratch storage is static to protect its 2 KB stack.
   Fit the slave clock residual, not a series of acoustic distances. Normalize
   each sample to the event's phase before fitting so servo steps cancel. */
uint8_t lt_mic_residual_at(uint32_t event_us, int32_t event_phase,
                          int32_t *offset, uint32_t *age, uint32_t *used)
{
  static lt_mic_fit_sample_t samples[LT_MIC_FIT_N];
  static double x[LT_MIC_FIT_N], y[LT_MIC_FIT_N];
  static double slopes[LT_MIC_FIT_N*(LT_MIC_FIT_N-1u)/2u];
  static double predicted[LT_MIC_FIT_N];
  uint32_t i, j, count, n=0u, ns=0u, nearest=0xFFFFFFFFu;
  uint32_t event_unstepped = event_us + (uint32_t)event_phase;
  double slope, estimate;
  *offset=0; *age=0u; *used=0u;
  if (lt_is_master != 0u) { return 1u; }
  taskENTER_CRITICAL();
  count=lt_mic_fit_count;
  for (i=0u;i<count;i++) { samples[i]=lt_mic_fit_samples[i]; }
  taskEXIT_CRITICAL();
  for (i=0u;i<count;i++) {
    int32_t dt=(int32_t)((samples[i].stamp+(uint32_t)samples[i].phase)-event_unstepped);
    uint32_t distance;
    if (dt < -800000 || dt > 250000) { continue; }
    distance=(uint32_t)((dt<0)?-dt:dt);
    if (distance<nearest) { nearest=distance; }
    x[n]=(double)dt;
    y[n]=(double)samples[i].offset -
         (double)(int32_t)((uint32_t)event_phase-(uint32_t)samples[i].phase);
    n++;
  }
  *age=(nearest==0xFFFFFFFFu)?0xFFFFFFFFu:(nearest/1000u);
  *used=n;
  if (n<3u || nearest>200000u) { return 0u; }
  for (i=0u;i<n;i++) {
    for (j=i+1u;j<n;j++) {
      double dx=x[j]-x[i];
      if (dx>40000.0 || dx < -40000.0) { slopes[ns++]=(y[j]-y[i])/dx; }
    }
  }
  if (ns==0u) { return 0u; }
  slope=lt_mic_median(slopes,ns);
  /* After feed-forward calibration, >100 ppm suggests a corrupt local fit. */
  if (slope < -0.0001 || slope > 0.0001) { return 0u; }
  for (i=0u;i<n;i++) { predicted[i]=y[i]-slope*x[i]; }
  estimate=lt_mic_median(predicted,n);
  if (estimate < -50.0 || estimate > 50.0) { return 0u; }
  *offset=(int32_t)((estimate>=0.0)?estimate+0.5:estimate-0.5);
  return 1u;
}

int32_t lt_phase_snapshot_us(void)
{
  return lt_phase_total_us;
}

/* 串口打印：直接走 USART1，不依赖 printf 重定向 */
static void lt_log(const char *fmt, ...)
{
  char buf[192];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n > 0)
  {
    if (n > (int)(sizeof(buf) - 1U))
    {
      n = (int)(sizeof(buf) - 1U);
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 200U);
  }
}

/* ---- V5.8：MAC 发送前重打 t1 / t3 -----------------------------------------
   旧版在 lwip_sendto() 之前取时间戳，报文还要在 tcpip 线程里排队，真正上线的
   时刻可能晚几十到上百微秒；两块板、两种调度相位就形成了现场看到的
   291/383us 两档 dly 和 96us 的 off 台阶。把时间戳挪到 ethernetif 的
   low_level_output()、即将交给 MAC DMA 的那一刻，排队延迟不再进入测量。
   只改本协议 UDP 端口 5000 的 SYNC(type=1)/DREQ(type=2) 载荷；DRESP/TOA
   以及其它报文一个字节都不碰。校验和由 MAC 硬件在发送时重算，所以先改载荷
   再启动 DMA 是安全的。 */
uint32_t lt_tx_stamp_frame(uint8_t *frame, uint32_t len)
{
  uint32_t l2 = 14U;
  uint32_t ihl;
  uint32_t ulen;
  uint32_t base;
  uint32_t now;
  uint8_t *ip;
  uint8_t *udp;
  uint8_t *pay;

  if ((frame == NULL) || (len < 42U))
  {
    return 0U;
  }

  /* 兼容可选的 VLAN 标签；现场没有 VLAN，但交换机的管理口可能加上 */
  if ((frame[12] == 0x81U) && (frame[13] == 0x00U))
  {
    l2 = 18U;
    if (len < 46U)
    {
      return 0U;
    }
  }

  if ((frame[l2 - 2U] != 0x08U) || (frame[l2 - 1U] != 0x00U))
  {
    return 0U;                    /* 不是 IPv4 */
  }

  ip = frame + l2;
  if ((ip[0] >> 4) != 4U)
  {
    return 0U;
  }

  ihl = (uint32_t)(ip[0] & 0x0FU) * 4U;
  if (ihl < 20U)
  {
    return 0U;
  }
  if (((ip[6] & 0x1FU) != 0U) || (ip[7] != 0U))
  {
    return 0U;                    /* 非首片分片，UDP 头不在这里 */
  }
  if (ip[9] != 17U)
  {
    return 0U;                    /* 不是 UDP */
  }
  if (len < (l2 + ihl + 8U + sizeof(lt_msg_t)))
  {
    return 0U;
  }

  udp = ip + ihl;
  if ((((uint32_t)udp[2] << 8) | (uint32_t)udp[3]) != LT_PORT)
  {
    return 0U;                    /* 目的端口不是本协议端口 */
  }
  ulen = ((uint32_t)udp[4] << 8) | (uint32_t)udp[5];
  if (ulen != (8U + sizeof(lt_msg_t)))
  {
    return 0U;
  }

  base = l2 + ihl + 8U;
  pay  = frame + base;
  now  = (uint32_t)lt_now_us();

  if (pay[0] == LT_MSG_SYNC)
  {
    pay[8]  = (uint8_t)(now);
    pay[9]  = (uint8_t)(now >> 8);
    pay[10] = (uint8_t)(now >> 16);
    pay[11] = (uint8_t)(now >> 24);
    lt_t1   = now;
    lt_tx_stamp_ok++;
    return base + 8U;
  }
  else if (pay[0] == LT_MSG_DREQ)
  {
    pay[16] = (uint8_t)(now);
    pay[17] = (uint8_t)(now >> 8);
    pay[18] = (uint8_t)(now >> 16);
    pay[19] = (uint8_t)(now >> 24);
    lt_t3   = now;
    lt_tx_stamp_ok++;
    return base + 16U;
  }
  return 0U;
}

/* ---- 时间戳交换协议 --------------------------------------------------------
   三条定长报文，一个来回：

     主板 --SYNC {seq,t1}------> 从板      从板在中断里记下 t2
     从板 --DREQ {seq,t2,t3}---> 主板      主板在中断里记下 t4
     主板 --DRESP{seq,t4}------> 从板      从板由此凑齐 t1..t4

   两边各自算：
     偏移  off = ((t2 - t1) - (t4 - t3)) / 2     从板时钟比主板快多少
     往返  dly =  (t4 - t1) - (t3 - t2)

   off 的来历：t2-t1 是"去程时间 + 时钟偏移"，t4-t3 是"回程时间 - 时钟偏移"，
   两者相减，去程回程抵消，剩下两倍偏移。这是 NTP 与 PTP 共用的原理，协议本
   身是自己定的，与标准 PTP 无关。

   t1、t3 是发送前在任务里打的（偏早），t2、t4 是在以太网中断里打的（偏晚）。
   两条偏差方向相反，而两块板跑同一份代码、大小相近，在差值里基本抵消；剩下
   的随机抖动靠多样本回归和按时延筛选压掉。
   -------------------------------------------------------------------------- */

/* ---- V5.7：频率预标定 -------------------------------------------------------
   把测量期攒下的 (t, off) 做一次 Theil-Sen 中值斜率，斜率就是两板频率差，
   一次写进 ARR。只取 dly 同一档内的点对，换档台阶不会混进斜率。

   单位换算：斜率 ppb 是"每秒偏移多少纳秒"，1 ppm = 1000 ppb。
   1 ppm 对应 108 个 ARR 计数（ARR 加 1，一秒长 9.26 ns = 9.26 ppb），
   所以 ARR 修正量 = ppb × 108 / 1000。
   -------------------------------------------------------------------------- */
static int32_t lt_ff_median(int32_t *a, uint32_t n)
{
  uint32_t k = n / 2U;
  uint32_t l = 0U;
  uint32_t r = n - 1U;

  while (l < r)
  {
    int32_t pivot = a[(l + r) / 2U];
    uint32_t i = l;
    uint32_t j = r;

    while (i <= j)
    {
      while (a[i] < pivot) { i++; }
      while (a[j] > pivot) { j--; }
      if (i <= j)
      {
        int32_t t = a[i];
        a[i] = a[j];
        a[j] = t;
        i++;
        if (j == 0U) { break; }
        j--;
      }
    }

    if (k <= j)      { r = j; }
    else if (k >= i) { l = i; }
    else             { break; }
  }

  return a[k];
}

/* 用 Theil-Sen 中值斜率结算预标定。
   为什么不用最小二乘：dly 换档会让 off 出现约 96us 的台阶，普通最小二乘会被
   台阶拉偏一个量级（V5.6 现场只算出 +464 计数，真实频差约 ±1700）。Theil-Sen
   取所有点对斜率的中值，同档内的点对占多数时，台阶只污染少数点对，中值不动。
   时间单位是 ms，斜率换算成 ppb 后，ARR 计数 = ppb × 108 / 1000。 */
static void lt_ff_apply(void)
{
  uint32_t i, j, m = 0U;
  uint32_t n = lt_ff_n;
  int32_t  slope = 0;
  int32_t  maxres = 0;
  uint32_t good = 0U;
  int64_t  cnt;

  lt_ff_state = 2U;                 /* 不管成败都退出测量期 */
  lt_conv_reset();                  /* 写 ARR 会改变斜率：旧稳定期从这一刻作废 */
  lt_ff_used  = n;

  if (n >= 2U)
  {
    lt_ff_span_ms = lt_ff_t[n - 1U] - lt_ff_t[0];
  }
  else
  {
    lt_ff_span_ms = 0U;
  }

  if (n < 12U)
  {
    lt_ff_print = 1U;               /* 样本太少：放弃预标定，交给慢积分 */
    return;
  }

  for (i = 0U; i < n; i++)
  {
    for (j = i + 1U; j < n; j++)
    {
      int64_t dt = (int64_t)(lt_ff_t[j] - lt_ff_t[i]);
      int32_t dd = (int32_t)(lt_ff_dly[j] - lt_ff_dly[i]);
      int64_t s;

      if (dt <= 0) { continue; }

      /* 只取同一 dly 档内的点对：换档带来的 96us 台阶是共模误差，
         跨档点对的斜率会被它污染；同档内的斜率才只反映晶振频差。 */
      if (dd < 0) { dd = -dd; }
      if (dd > 20) { continue; }

      s = ((int64_t)(lt_ff_off[j] - lt_ff_off[i]) * 1000000LL) / dt;
      if (s >  200000LL) { s =  200000LL; }
      if (s < -200000LL) { s = -200000LL; }

      if (m < (uint32_t)(sizeof(lt_ff_slopes) / sizeof(lt_ff_slopes[0])))
      {
        lt_ff_slopes[m] = (int32_t)s;
        m++;
      }
    }
  }

  if (m >= 10U)
  {
    slope = lt_ff_median(lt_ff_slopes, m);

    /* 残差只作可信度诊断：用首点作基准，台阶另一侧的样本残差会接近 96us。
       这里不再用残差把 slope 清零——斜率已经只取同档点对，跨档残差大是正常
       现象；清零反而会把一次本来正确的预标定退化成慢积分。 */
    for (i = 0U; i < n; i++)
    {
      int64_t pred = (int64_t)lt_ff_off[0] +
                     ((int64_t)slope * (int64_t)(lt_ff_t[i] - lt_ff_t[0])) / 1000000LL;
      int64_t res = (int64_t)lt_ff_off[i] - pred;
      int32_t ar = (res >= 0) ? (int32_t)res : (int32_t)(-res);

      if (ar > maxres) { maxres = ar; }
      if (ar <= 25)    { good++; }
    }
  }

  lt_ff_slope_ppb = slope;
  lt_ff_good      = good;
  lt_ff_resid     = maxres;

  cnt = ((int64_t)slope * 108LL) / 1000LL;
  if (cnt >  (int64_t)LT_FF_ARR_MAX) { cnt =  (int64_t)LT_FF_ARR_MAX; }
  if (cnt < -(int64_t)LT_FF_ARR_MAX) { cnt = -(int64_t)LT_FF_ARR_MAX; }

  lt_ff_cnt    = (int32_t)cnt;
  lt_arr_delta = (int32_t)cnt;
  TIM5->ARR = (uint32_t)((int64_t)lt_arr_nominal + lt_arr_delta);
  lt_ff_print = 1U;
}

/* ---- 伺服 ------------------------------------------------------------------
   只在从板上运行。主板是全系统的时间基准，它自己的时钟一根手指都不动。

   修正分两件事，缺一不可：

     频率修正（治本）——上电先用 6 秒以上干净样本测出两板频差，一次写进
       TIM5 的 ARR（预标定，见下面 ⓪）；之后只留一个很小的积分跟温漂。ARR
       每加 1 个计数，一秒变长 9.26ns（速率慢 9.26ppb）。速率差纠掉，相位才
       不会一直往外漂，相位环路也就不必长期硬扛漂移。

     相位修正（治标）——把已经积累的偏差直接拨回去。
       只做频率不做相位，要等很久偏差才会自己回来；
       只做相位不做频率，偏差会一直漂、要反复拉。
   -------------------------------------------------------------------------- */

/* 每次测量后执行一遍。off_us = 偏差（从板−主板，已减去 LT_OFF_CAL_US）。
   即"两板脉冲实际间距"的估计值，伺服目标就是把它压到 0。 */
static void lt_servo(int32_t off_us, uint32_t dly_us)
{
  int32_t step;
  int32_t da;

  lt_servo_in = off_us;

  /* ---- ⓪ V5.6：上电先测频差（预标定）---------------------------------------
     把"频率"和"相位"分开对付：频差一次测准写进 ARR，相位环只拉剩下的几十
     微秒。收敛从"积分慢慢逼近"变成"2 s 测量 + 1~2 s 拉相位 + 10 s 稳定期"。

     测量期为什么必须"只听不动"：相位拨动会让 off 立刻跳一个台阶，ARR 改动
     会改变斜率，两者都会把中值斜率的点对带偏。只有偏差大得离谱
     （|off| > 1000 µs，相当于 300 ppm 以上，不可能是晶振差）时才粗拨一次，
     拨完重新开始测。

     粗拨和原来一样要避开麦克风事件窗口（mic_quiet_ok）：它是直接改
     TIM5->CNT 的，落在采样窗口里会把后半窗的时间戳整体带偏。 */
  if (lt_ff_state < 2U)
  {
    if (lt_ff_state == 0U)
    {
      /* 第一个干净样本：先把相位粗拨到位（这一步会跳变，所以它不进拟合）*/
      if (((off_us > 5) || (off_us < -5)) && (mic_quiet_ok() != 0u))
      {
        lt_adjust_local_time(off_us);
        lt_phase_total_us += off_us;
      }
      lt_ff_state = 1U;
      lt_ff_n     = 0U;
      return;
    }

    if ((off_us > 1000) || (off_us < -1000))
    {
      if (mic_quiet_ok() != 0u)
      {
        lt_adjust_local_time(off_us);
        lt_phase_total_us += off_us;
      }
      lt_ff_n = 0U;              /* 拨过之后斜率不连续，重测 */
      return;
    }

    lt_ff_t[lt_ff_n]   = (uint32_t)(lt_now_us() / 1000ULL);
    lt_ff_dly[lt_ff_n] = dly_us;
    lt_ff_off[lt_ff_n] = off_us;
    lt_ff_n++;

    /* V5.7 结算条件：
       1) 存满 64 点（10 Hz 约 6.4 s）；
       2) 至少 24 点且跨度达到 6 s；
       3) 等到 12 s 仍不足 24 点 —— 放弃，交给慢积分，别把 30 s 预算耗光。
       旧版 20 点/3 s 太短：干净样本每秒只有 2~3 个时，6~10 个点的斜率会被
       dly 换档的 96us 台阶完全带偏。 */
    {
      uint32_t span = (uint32_t)(lt_now_us() / 1000ULL) - lt_ff_t[0];

      if ((lt_ff_n >= LT_FF_SAMPLES) ||
          ((lt_ff_n >= LT_FF_MIN_SAMP) && (span >= LT_FF_SPAN_MS)) ||
          (span >= LT_FF_GIVEUP_MS))
      {
        lt_ff_apply();
      }
    }
    return;                      /* 测量期不碰相位、不碰 ARR */
  }

  /* ---- ① 频率修正：用相位偏差做积分，只动从板的 ARR（V5.5 及以前的做法）
     旧写法是 arr = 回归斜率 × 0.108 直接赋值。问题是回归斜率量的是"速率差
     被相位环路抵掉之后剩下的残余"——相位环路一开始工作，这个斜率就趋近 0，
     arr 也随之退回 0，速率修正永远锁不住。

     后果就是收敛快不了：频率差只能靠相位环路每 100ms 拨一次去硬扛，而相位
     环路（细跟踪增益 1/4、一拍延迟）对恒定漂移必然留下大约"4 倍每周期漂移量"
     的稳态滞后——漂移 10.5ppm 时约 4us，冷启动漂移更大时就直接顶到 10us 以上。
     表现为"锁上一次又被拉出去"，要等回归窗口攒够、漂移估计变准才慢慢稳住，
     于是收敛时间被拖到几十秒。

     改成积分环节：只要相位还有偏差，就继续把 ARR 往"抵消漂移"的方向累加，
     直到漂移真的被抵消、相位偏差自己回到 0。三道闸：
       · 只在相位基本拉回（|off| <= LT_ARR_INT_RANGE_US）之后才累加，避免
         粗捕获阶段那几百毫秒的偏差把积分冲爆（饱和）；
       · 每拍最多动 LT_ARR_INT_STEP_MAX 计数，个别脏样本过不了闸；
       · 总量限幅 ±6000 计数（约 ±55ppm），防异常值把周期改坏。

     试过但已回退的改法（记录在此，避免重复走）：把积分改成"每 10 个干净样本
     结算一次（约 1 秒，与 ARR 预装载的生效节拍对齐）、用本秒平均偏差"。离散
     仿真预测稳态摆动会从 7.6us 降到 4.2us，但**实测 30 秒内都收敛不到 ±10us**，
     比按拍累加差，故回退。结论：这套执行机构（1 秒周期 + ARR 预装载）上，
     按拍累加这组参数（增益 50、每拍 170、门限 25us）就是现场验证过能达标的点。

     总量限幅 ±2000 计数（约 ±18.5ppm）：它本来只是"防异常值把周期改坏"的兜底，
     但如果两板冷启动频差超过 18.5ppm，它就会把频率环路卡住——速率差纠不干净，
     相位环路只能长期背着它，稳态滞后顶在 10us 以上（离散仿真：频差 40ppm、
     限幅 ±2000 时 90 秒内都没进过 ±10us 带）。现场实测过把它放开到 ±6000，
     收敛时间没有变化，说明本机两板冷启动频差没超过 ±18.5ppm，于是按原设计
     保留 ±2000。真正的保护是上面那两道闸（每拍限速 + 相位拉回后才累加）。

      ---- V5.6 起：频差由上面的预标定一次测掉，这下面只留一个很慢的残余积分
      （增益 10、每拍 30、门限 25us），只跟温漂和标定残差。 */
  if ((off_us <= LT_ARR_INT_RANGE_US) && (off_us >= -LT_ARR_INT_RANGE_US) &&
      ((off_us > 1) || (off_us < -1)))
  {
    da = off_us * LT_ARR_INT_GAIN;
    if (da >  LT_ARR_INT_STEP_MAX) { da =  LT_ARR_INT_STEP_MAX; }
    if (da < -LT_ARR_INT_STEP_MAX) { da = -LT_ARR_INT_STEP_MAX; }
    lt_arr_delta += da;
  }

  if (lt_arr_delta >  LT_FF_ARR_MAX) { lt_arr_delta =  LT_FF_ARR_MAX; }
  if (lt_arr_delta < -LT_FF_ARR_MAX) { lt_arr_delta = -LT_FF_ARR_MAX; }

  TIM5->ARR = (uint32_t)((int64_t)lt_arr_nominal + lt_arr_delta);
  /* Auto-reload preload 已打开，下一个周期边界才生效 */

  /* ---- ② 相位修正 ---- */
  if ((off_us > 1000) || (off_us < -1000))
  {
    step = off_us;          /* 粗捕获：一次拨到底，几百毫秒就拉平 */
  }
  else if ((off_us > 1) || (off_us < -1))
  {
    /* 细跟踪：只拨四分之一（增益 1 配 100ms 采样和一拍延迟时是临界稳定，
       会在 0 和 ±偏值之间来回摆；1/4 才收敛）。
       整数除法会把 2~3 µs 的偏差截成 0（等于死区变成 3 µs），所以四舍五入、
       至少拨 1 µs，否则最后几微秒永远纠不掉。 */
    step = (off_us > 0) ? ((off_us + 2) / 4) : ((off_us - 2) / 4);
  }
  else
  {
    step = 0;               /* 死区：偏到 1us 以内就不再动，保持脉冲干净 */
  }

  /* 检测窗口内不拨 CNT：伺服是直接改写 TIM5->CNT 的，若这一拨正好落在麦克风的
     采样窗口里，窗口后半段的样本时间戳会整体偏掉一个拨动量（最多几百微秒）。
     频率修正（改 ARR）不受影响，它不动 CNT。稳态下死区本来就让它多数时候不拨，
     这道闸只为挡住冷启动和被串口扰动之后的那几次大拨动。 */
  if ((step != 0) && (mic_quiet_ok() == 0u))
  {
    step = 0;
  }

  if (step != 0)
  {
    uint64_t before = lt_now_us();
    lt_adjust_local_time(step);
    /* 诊断：拨动后本地时间应恰好前移 step 微秒（再加执行本身的几微秒） */
    lt_step_effect = (int32_t)((int64_t)lt_now_us() - (int64_t)before);
    lt_phase_total_us += step;
  }
}

/* ---- 判定用的 3 点中值滤波器 -----------------------------------------------
   为什么要在判定上加一道滤波：

   判据量 err = off - LT_OFF_CAL_US 本身就是"两板脉冲实际间距的估计值"，
   但它每个样本都带测量噪声，而且噪声不是对称的——时间戳打在 sendto 之前，
   从打戳到报文真正发出去的调度延迟只会让 off 偏大（单边正偏置）。dly 筛选
   放行的是"往返时延不超过 dmin+20us"的样本，也就等于允许约 +10us 的单边
   偏差混进来（延迟增加 δ，off 偏 δ/2）。

   于是一个"干净但偏了 8~10us"的偶发样本就能把 10 秒稳定期打断，重新计时。
   离散仿真（tmp\sim_conv_crit.ps1：30% 脏样本、σ=3us、4% 样本带 +12us 单边偏置、
   自由频差 10.5ppm，40 次）：
     原判据（单样本）  t_first 中值 20.8 s，最坏 78.8 s（>60 s 才宣告收敛）
     3 点中值          t_first 中值 1.6 s，最坏 8.8 s
   —— 这就是"收敛慢、30 秒不达标"的主因：环路其实早就拉平了，是判据在反复复位。

   为什么用中值而不是平均：单边偏置会把平均拉偏；中值只取序中那个样本，
   两个正常值就能把偶发偏置顶掉，而真实的持续偏差（环路真的把间距拉到 15us）
   三个值都超标，判定照样不通过——只滤偶发，不掩盖真偏差。
   -------------------------------------------------------------------------- */
static int32_t  lt_cv_win[3] = { 0, 0, 0 };   /* [0] 最旧，[2] 最新 */
static uint32_t lt_cv_n = 0U;                 /* 已填入样本数（满 3 才开始滤波）*/
static volatile int32_t lt_off_filt = 0;      /* 滤波后的判定值（串口诊断用）*/

static int32_t lt_conv_filt(int32_t v)
{
  int32_t a, b, c, t;

  lt_cv_win[0] = lt_cv_win[1];
  lt_cv_win[1] = lt_cv_win[2];
  lt_cv_win[2] = v;
  if (lt_cv_n < 3U)
  {
    lt_cv_n++;
    return v;            /* 还没攒够 3 个：先用原值，别让判定原地不动 */
  }

  a = lt_cv_win[0];
  b = lt_cv_win[1];
  c = lt_cv_win[2];
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { b = a; }
  return b;              /* 中值 */
}

/* ---- 收敛判定 --------------------------------------------------------------
   任务书要求：上电后 30 秒内，两板收敛到统一时间基准，且偏差不大于 ±10us。

   判据量是伺服误差信号的 3 点中值：
       err = med(最近 3 个干净样本的 off) - LT_OFF_CAL_US
   它与伺服的死区、拨动方向共用同一个量（也就是串口里的 sv / cf），
   判定门槛不会和环路目标打架；由标定关系「实际间距 = 读数 - 标定值」，
   它同时也是两板脉冲实际间距的估计值。滤波只压偶发的单边偏置样本，
   带宽 ±10us 没变、保持 10 秒的要求没变。

   只在"干净样本"上判定：被调度或打印扰动过的样本，其 off 可能偏出上百微秒，
   拿它判定会把已经收敛的系统误判成"未收敛"（这是 E4/E5 那类错误的翻版）。
   -------------------------------------------------------------------------- */
static void lt_conv_push(int32_t off_us)
{
  int32_t err;
  int32_t cv;
  uint32_t now = HAL_GetTick();

  /* 这一拍本身也要做断流检查：打印任务万一被拖住、lt_conv_poll 没来得及跑，
     也不能拿十几秒前的 st 继续累加。先判 gap，再刷新时刻。 */
  if ((lt_conv_last_ms != 0U) &&
      ((now - lt_conv_last_ms) > LT_CONV_GAP_MS))
  {
    lt_conv_streak     = 0U;
    lt_conv_t_first_ms = 0U;
  }
  lt_conv_last_ms = now;

  /* 收敛后仍继续更新 cf（3 点中值），只用于串口/屏幕观察；首次收敛用时
     已经冻结，不会因为这里的更新而改变。V5.8 之前这里直接 return，收敛后
     cf 停在最后一拍，现场容易误以为 off 没有变化。 */
  cv  = lt_conv_filt(off_us);
  lt_off_filt = cv;

  if (lt_conv_ok != 0U)
  {
    return;                          /* 已收敛，耗时不更新，保持首次收敛值 */
  }

  err = cv - LT_OFF_CAL_US;

  if ((err <= LT_CONV_BAND_US) && (err >= -LT_CONV_BAND_US))
  {
    if (lt_conv_streak == 0U)
    {
      /* 本段稳定期从这一拍开始：两板就是在这里真正收敛到 ±10us 以内的 */
      lt_conv_t_first_ms = now;
    }
    lt_conv_streak++;

    /* 两个条件同时满足才算真收敛：样本够多，且稳定期够长 */
    if ((lt_conv_streak >= LT_CONV_MIN_SAMP) &&
        ((now - lt_conv_t_first_ms) >= LT_CONV_HOLD_MS))
    {
      /* 冻结收敛用时：两个"用时"都算到进带时刻（10s 稳定期的起点）。
         再记一个"宣告"用时：从上电到此刻（稳定期满、即将打印
         CONVERGED 行），它才对应串口见到该行的墙上时间。 */
      lt_conv_ms_comm = lt_conv_t_first_ms - lt_conv_comm_ms;
      lt_conv_ms_pwr  = lt_conv_t_first_ms - lt_conv_boot_ms;
      lt_conv_ms_ann_pwr = now - lt_conv_boot_ms;
      lt_conv_ok = 1U;
    }
  }
  else
  {
    lt_conv_streak = 0U;             /* 只要有一次出带，稳定期重新开始 */
  }
}

/* V5.7：干净样本断流或 dly 地板换档时，正在累计的稳定期立即作废。
   旧判据只在"来新样本且出带"时清零；样本被门限全部剔掉时 st 会冻住，
   等满 10 s 就误报收敛（现场 56.79 s 那次当前 offm 仍是 -64us）。 */
static void lt_conv_reset(void)
{
  if (lt_conv_ok != 0U)
  {
    return;                      /* 已经宣告过：保留首次收敛用时，不反复刷屏 */
  }

  lt_conv_streak     = 0U;
  lt_conv_t_first_ms = 0U;
}

/* 由打印任务每 100 ms 调用一次：不依赖"下一个样本"，断流也能发现。 */
static void lt_conv_poll(void)
{
  uint32_t now = HAL_GetTick();

  if (lt_conv_last_ms == 0U)
  {
    return;
  }

  if ((now - lt_conv_last_ms) > LT_CONV_GAP_MS)
  {
    if (lt_clean_lost == 0U)
    {
      lt_clean_lost = 1U;
      lt_log("[LT] CLEAN lost: age %lu ms, conv %u (dly gate too tight?)\r\n",
             (unsigned long)(now - lt_conv_last_ms),
             (unsigned)lt_conv_ok);
    }
    if (lt_conv_ok == 0U)
    {
      lt_conv_reset();
    }
  }
  else
  {
    lt_clean_lost = 0U;
  }
}

/* 收敛成功后在串口上打一行收敛用时（单位：秒），只打一次。 */
static void lt_conv_report(void)
{
  if ((lt_conv_ok == 0U) || (lt_conv_announced != 0U))
  {
    return;   /* 还没收敛，或者已经报过一次了 */
  }

  lt_conv_announced = 1U;
  /* V5.9：三个"用时"分开写清楚——前两个是到"进带"（10s 稳定期起点），
     最后一个是到"宣告"（此刻）。对上电秒表请读最后一个。 */
  lt_log("[LT] CONVERGED / 收敛成功: 链路->进带 %lu.%02lu s | 上电->链路 %lu.%02lu s | 上电->进带 %lu.%02lu s | 上电->宣告 %lu.%02lu s\r\n",
         (unsigned long)(lt_conv_ms_comm / 1000U),
         (unsigned long)((lt_conv_ms_comm % 1000U) / 10U),
         (unsigned long)((lt_conv_comm_ms - lt_conv_boot_ms) / 1000U),
         (unsigned long)(((lt_conv_comm_ms - lt_conv_boot_ms) % 1000U) / 10U),
         (unsigned long)(lt_conv_ms_pwr / 1000U),
         (unsigned long)((lt_conv_ms_pwr % 1000U) / 10U),
         (unsigned long)(lt_conv_ms_ann_pwr / 1000U),
         (unsigned long)((lt_conv_ms_ann_pwr % 1000U) / 10U));
}

/* 把一次测量结果放进窗口，并重新算一次回归 */
static void lt_push_sample(int32_t off_us, uint32_t dly_us, uint32_t t_ms)
{
  uint32_t i;

  if (lt_s_cnt >= LT_MAX_SAMPLES)
  {
    for (i = 1U; i < LT_MAX_SAMPLES; i++)
    {
      lt_s_off[i - 1U] = lt_s_off[i];
      lt_s_dly[i - 1U] = lt_s_dly[i];
      lt_s_t[i - 1U]   = lt_s_t[i];
    }
    lt_s_cnt = LT_MAX_SAMPLES - 1U;
  }

  lt_s_off[lt_s_cnt] = off_us;
  lt_s_dly[lt_s_cnt] = dly_us;
  lt_s_t[lt_s_cnt]   = t_ms;
  lt_s_cnt++;

  lt_mes_ok++;
  lt_cur_off_us = off_us;
  lt_cur_dly_us = dly_us;

  lt_run_regression();
}

/* 对窗口内的样本做线性回归 off = a + b*t，斜率 b 就是频率差。
   b 的单位是"每毫秒偏移多少微秒"，乘 1e6 换算成 ppb。
   时刻和偏移都先减去第一个样本的值，避免大数相减丢精度。 */
static void lt_run_regression(void)
{
  uint32_t i;
  uint32_t dmin_gate;
  uint32_t t_cut;
  uint32_t first;
  uint32_t used = 0U;
  int64_t  st = 0, so = 0, stt = 0, sto = 0;
  int32_t  o0;
  uint32_t t0;

  /* V5.8：dly 地板只看最近 LT_DLY_RECENT_MS（2 秒）。
     旧版取 512 个样本（约 51 秒）的最小值：只要 51 秒内出现过一次快档，
     快档消失后门限仍停在快档，慢档样本会被 dly <= dmin+20 全部剔除。
     伺服没有输入，残余频差就会让两板脉冲越漂越远——现场"收敛后又变大"
     就是这个机理。最近 2 秒的最小值在快档消失后约 2 秒抬到当前档，干净
     样本重新进来；快档回来时又会降下去，只追这一条路径，不会 51 秒断流。 */
  t_cut = lt_s_t[lt_s_cnt - 1U];
  if (t_cut > LT_DLY_RECENT_MS)
  {
    t_cut -= LT_DLY_RECENT_MS;
  }
  else
  {
    t_cut = 0U;
  }

  first = 0U;
  while (((first + 1U) < lt_s_cnt) && (lt_s_t[first] < t_cut))
  {
    first++;
  }
  dmin_gate = lt_s_dly[first];
  for (i = first + 1U; i < lt_s_cnt; i++)
  {
    if (lt_s_dly[i] < dmin_gate)
    {
      dmin_gate = lt_s_dly[i];
    }
  }

  /* 地板变化超过 30us：时间戳换了一条路径。换档瞬间不是时钟真的跳了，
     未收敛时正在累计的稳定期必须作废，否则旧档攒的 st 会在新档里被当成
     "连续在带"。收敛后不重置首次收敛用时，但会在串口打一行，便于对照
     示波器上是否同时出现台阶。 */
  if ((lt_s_cnt > 10U) && (lt_min_dly_us > 0) &&
      (((int32_t)dmin_gate > (lt_min_dly_us + (int32_t)LT_DLY_MODE_DIFF_US)) ||
       ((int32_t)dmin_gate < (lt_min_dly_us - (int32_t)LT_DLY_MODE_DIFF_US))))
  {
    lt_dly_old_us = (uint32_t)lt_min_dly_us;
    lt_dly_new_us = dmin_gate;
    lt_dly_changes++;
    lt_dly_print = 1U;
    if (lt_conv_ok == 0U)
    {
      lt_conv_reset();
    }
  }

  lt_min_dly_us = (int32_t)dmin_gate;

  if (lt_s_cnt < 8U)
  {
    lt_win_cnt   = lt_s_cnt;
    lt_slope_ppb = 0;
    return;
  }

  /* 参考点取"最新一个样本"，不取最老的那个：上电瞬间那几百毫秒量级的大偏差
     样本还要在窗口里待够 512 拍（约 51 秒）才会被挤出去，如果拿它们当基准，
     整条序列都会跟着偏，回归出来的斜率会大到离谱（dfreq 显示成几十上百 ppm）。
     参考点取最新样本、再配上下面"偏差超过 ±1ms 就丢掉"的筛子，这些启动
     瞬态的样本就进不了回归。 */
  o0 = lt_s_off[lt_s_cnt - 1U];
  t0 = lt_s_t[lt_s_cnt - 1U];

  for (i = 0U; i < lt_s_cnt; i++)
  {
    int64_t dt;

    if (lt_s_dly[i] > (dmin_gate + LT_DELAY_MARGIN_US))
    {
      continue;   /* 被排队延迟过的样本，丢掉 */
    }

    if ((lt_s_off[i] - o0 > 1000) || (o0 - lt_s_off[i] > 1000))
    {
      continue;   /* 上电瞬态的大偏差点，不代表速率差，丢掉 */
    }

    dt   = (int64_t)(lt_s_t[i] - t0);
    st  += dt;
    so  += (int64_t)(lt_s_off[i] - o0);
    stt += dt * dt;
    sto += dt * (int64_t)(lt_s_off[i] - o0);
    used++;
  }

  lt_win_cnt = used;
  if (used < 8U)
  {
    return;
  }

  {
    int64_t num = ((int64_t)used * sto) - (st * so);
    int64_t den = ((int64_t)used * stt) - (st * st);
    if (den != 0)
    {
      lt_slope_ppb = (int32_t)((num * 1000000LL) / den);
    }
  }
}

/* 发送任务：主板每 100ms 发起一次测量；同时负责周期打印统计 */
static void lt_tx_task(void *argument)
{
  lt_msg_t m;
  uint32_t last_report = 0U;
  uint32_t air_sent_at = 0U;
  uint32_t now;
  static uint32_t lt_toa_sent = 0U;   /* 已经报给对方的事件序号 */

  (void)argument;

  for (;;)
  {
    /* 链路建立时刻只报一次：把"上电 -> 链路建立 -> 时钟收敛"三段拆开。
       如果收敛慢，先看这一行——链路很晚才建立，那时间就花在等对方上电 /
       PHY 自协商 / 交换机 STP 上，跟同步算法无关。 */
    if ((lt_link_announced == 0U) && (netif_is_link_up(&gnetif) != 0))
    {
      uint32_t t_link = HAL_GetTick() - lt_conv_boot_ms;

      lt_link_announced = 1U;
      lt_log("[LT] LINK UP at %lu.%02lu s (自本板上电)\r\n",
             (unsigned long)(t_link / 1000U),
             (unsigned long)((t_link % 1000U) / 10U));
    }

    if (lt_is_master != 0U)
    {
      lt_seq++;
      memset(&m, 0, sizeof(m));
      m.type = LT_MSG_SYNC;
      m.seq  = lt_seq;
      m.t1   = (uint32_t)lt_now_us();   /* 发 SYNC 的时刻（主板时钟）*/
      lt_t1  = m.t1;
      (void)lwip_sendto(lt_sock, &m, sizeof(m), 0U,
                        (struct sockaddr *)&lt_peer, sizeof(lt_peer));
    }

    /* ---- 声到达时刻交换（跨板 TDOA 用）：与上面那条同步链路完全独立 --------
       不参与 t1..t4、不参与伺服，只是把"本板第几次事件、什么时刻到"告诉对方，
       每判决出一次事件只发一条。
       放在这个 100 ms 的发送任务里发，mic 任务里一行网络代码都不加：那边要赶
       10.67 ms 的半缓冲，不能被 sendto 拖住。
       链路没通之前不发：ARP 还没建立时 sendto 可能阻塞，等第一次同步交换成功
       （对方地址已经热了）再开张，保险。                                       */
    if (lt_conv_comm_up != 0U)
    {
      uint32_t s_ev = 0U, t_ev = 0U;

      if ((mic_tdoa_tx_snap(&s_ev, &t_ev) != 0U) && (s_ev != lt_toa_sent))
      {
        lt_toa_sent = s_ev;
        memset(&m, 0, sizeof(m));
        m.type = LT_MSG_TOA;
        m.seq  = s_ev;      /* 本板事件序号 */
        m.t1   = t_ev;      /* 该事件在本地时钟上的到达时刻（µs）*/
        m.t2   = 0U;         /* V6.0 保留：不再区分判决路径 */
        (void)lwip_sendto(lt_sock, &m, sizeof(m), 0U,
                          (struct sockaddr *)&lt_peer, sizeof(lt_peer));
      }
    }

    /* Independent temperature/speed packet, existing message layout. */
    if ((lt_is_master != 0U) && (lt_conv_comm_up != 0U) && ((uint32_t)(HAL_GetTick()-air_sent_at)>=1000u))
    {
      int32_t t; uint32_t speed=disp_air_tx_snap(&t); air_sent_at=HAL_GetTick();
      memset(&m,0,sizeof(m)); m.type=5u; m.t1=speed; m.t2=0x41495231u; m.off_m=t;
      (void)lwip_sendto(lt_sock,&m,sizeof(m),MSG_DONTWAIT,(struct sockaddr *)&lt_peer,sizeof(lt_peer));
    }
    /* ---- 声音触发链路诊断行（只走串口） ----------------------------------
       事件 / 配对 / 溢出一有变化就补一行，配合 2 s 一次的 [MIC] 实时行，
       对着串口打一声就能看出卡在哪一段（对照 mic.c 里 mic_trig_cnt 的注释）。
       mic 任务里一行打印都不能有（那边要赶 10.67 ms 的半缓冲），所以在这里、
       在变化发生之后的第一时间打印，最多晚 100 ms。 */
    {
      static uint32_t s_dbg_ev = 0U;
      static uint32_t s_dbg_pr = 0U;
      static uint32_t s_dbg_ov = 0U;
      static uint32_t s_dbg_tpl = 0U;

      if (mic_tpl_learn_cnt != s_dbg_tpl)
      {
        s_dbg_tpl = mic_tpl_learn_cnt;
        lt_log("[TPL] learn %lu pk %lu valid %u\r\n",
               (unsigned long)mic_tpl_learn_cnt,
               (unsigned long)mic_tpl_learn_pk,
               (unsigned)mic_tpl_valid);
      }
      if (mic_event_seq != s_dbg_ev)
      {
        extern volatile uint32_t mic_event_raw_t_us, mic_event_sync_age_ms, mic_event_sync_samples;
        extern volatile int32_t mic_event_sync_offset_us;
        extern volatile uint8_t mic_event_sync_valid;
        s_dbg_ev = mic_event_seq;
        lt_log("[EVT] ev %lu t %lu pk %lu rq %lu pos %lu ph0 %ld ph1 %ld raw %lu sc %ld sa %lu sok %u sn %lu\r\n",
               (unsigned long)mic_event_seq,
               (unsigned long)mic_event_t_us,
               (unsigned long)mic_last_pk,
               (unsigned long)mic_last_corr_q,
               (unsigned long)mic_event_pos256,
               (long)mic_event_phase_start_us,
               (long)mic_event_phase_end_us,
               (unsigned long)mic_event_raw_t_us,
               (long)mic_event_sync_offset_us,
               (unsigned long)mic_event_sync_age_ms,
               (unsigned)mic_event_sync_valid,
               (unsigned long)mic_event_sync_samples);
      }
      if (mic_tdoa_pairs != s_dbg_pr)
      {
        s_dbg_pr = mic_tdoa_pairs;
        lt_log("[TDOA] pr %lu rx %lu tdoa %ld dj %lu | off %ld age %lu far %lu nomatch %lu jump %lu\r\n",
               (unsigned long)mic_tdoa_pairs,
               (unsigned long)mic_tdoa_rxn,
               (long)mic_tdoa_us,
               (unsigned long)mic_tdoa_dj_us,
               (long)lt_off_clean,
               (unsigned long)((lt_conv_last_ms != 0U) ?
                               (HAL_GetTick() - lt_conv_last_ms) : 0U),
               (unsigned long)mic_td_rej_far,
                (unsigned long)mic_tdoa_nomatch,
                (unsigned long)mic_td_rej_jump);
      }
      if (mic_overruns != s_dbg_ov)
      {
        s_dbg_ov = mic_overruns;
        lt_log("[OVR] overruns %lu\r\n", (unsigned long)mic_overruns);
      }
    }

    /* V5.7 预标定结果 / dly 换档事件：只在发生时打一行，仍放在打印任务里。
       接收任务里绝不能碰串口，否则会把紧接着的 DREQ/DRESP 拖出上百微秒。 */
    if (lt_ff_print != 0U)
    {
      lt_ff_print = 0U;
      lt_log("[LT] FF n %lu span %lu ms slope %ld ppb cnt %ld arr %ld good %lu resid %ld us\r\n",
             (unsigned long)lt_ff_used,
             (unsigned long)lt_ff_span_ms,
             (long)lt_ff_slope_ppb,
             (long)lt_ff_cnt,
             (long)lt_arr_delta,
             (unsigned long)lt_ff_good,
             (long)lt_ff_resid);
    }
    if (lt_dly_print != 0U)
    {
      lt_dly_print = 0U;
      lt_log("[LT] DLY mode %lu -> %lu us, changes %lu\r\n",
             (unsigned long)lt_dly_old_us,
             (unsigned long)lt_dly_new_us,
             (unsigned long)lt_dly_changes);
    }

    /* 收敛成功后在串口上补一行收敛用时。放在打印任务里，不塞进接收任务。 */
    lt_conv_report();

    /* 干净样本断流检查：不依赖下一个样本，断开 2 s 就把稳定期清零。 */
    lt_conv_poll();

    now = osKernelGetTickCount();
    if ((now - last_report) >= LT_REPORT_PERIOD_MS)
    {
      last_report = now;
      lt_log("[LT] SAMP dlm %lu skp %lu age %lu tx %lu | reg %lu/%lu\r\n",
             (unsigned long)lt_dly_from_master,
             (unsigned long)lt_servo_skip,
             (unsigned long)((lt_conv_last_ms != 0U) ?
                             (HAL_GetTick() - lt_conv_last_ms) : 0U),
             (unsigned long)lt_tx_stamp_ok,
             (unsigned long)TIM5->ARR,
             (unsigned long)TIM5->CCR4);
      lt_log("[LT] %s | ok %lu bad %lu | off %ld us | dly %lu us | dmin %ld us | dfreq %ld ppb | arr %ld | ph %ld | offm %ld | sv %ld | ste %ld | win %lu | cf %ld | st %lu | ff %lu\r\n",
             (lt_is_master != 0U) ? "MASTER" : "SLAVE ",
             (unsigned long)lt_mes_ok,
             (unsigned long)lt_mes_bad,
             (long)lt_off_clean,
             (unsigned long)lt_cur_dly_us,
             (long)lt_min_dly_us,
             (long)lt_slope_ppb,
             (long)lt_arr_delta,
             (long)lt_phase_total_us,
             (long)lt_off_from_master,
             (long)lt_servo_in,
             (long)lt_step_effect,
             (unsigned long)lt_win_cnt,
             (long)lt_off_filt,            /* cf：判定用的中值滤波值 */
             (unsigned long)lt_conv_streak, /* st：当前连续在带内样本数 */
             (unsigned long)lt_ff_state);   /* ff：0/1=频率预标定中 2=闭环 */
      /* [MIC] = 触发链路实时状态（每 2 s 一行）：
         tg 包络连续确认通过次数 / ev 模板合格并发布的事件数 / mx 本半缓冲包络
         峰值（打一声就跳）/ nf 噪声底 / trg 当前触发门限 / tpl 模板是否可用 /
         learn 学模板次数 / rx 收到对板 TOA 报文数 / pr 配对成功次数 /
         ov 丢半缓冲次数；rq 为上次相关峰值，sh/rej 分别是窗口过短/相关不足，
         ct 为上次匹配计算耗时。 */
       lt_log("[MIC] tg %lu ev %lu mx %lu pp %ld nf %lu trg %lu tpl %u learn %lu rx %lu pr %lu ov %lu rq %lu sh %lu rej %lu ct %lu cp %lu ctr %lu jr %lu\r\n",
             (unsigned long)mic_trig_cnt,
             (unsigned long)mic_event_seq,
             (unsigned long)mic_env_mx_live,
             (long)(mic_last_max() - mic_last_min()),
             (unsigned long)mic_last_nf,
             (unsigned long)mic_last_trig,
             (unsigned)mic_tpl_valid,
             (unsigned long)mic_tpl_learn_cnt,
             (unsigned long)mic_tdoa_rxn,
             (unsigned long)mic_tdoa_pairs,
             (unsigned long)mic_overruns,
             (unsigned long)mic_match_best_q,
             (unsigned long)mic_match_short_cnt,
             (unsigned long)mic_match_reject_cnt,
             (unsigned long)mic_match_cost_us,
             (unsigned long)mic_candidate_pk,
              (unsigned long)mic_candidate_trig,
              (unsigned long)mic_td_rej_jump);
      lt_log("[LT] RAW seq %lu t1 %lu t2 %lu t3 %lu t4 %lu | frames %lu msgs %lu | reg %lu/%lu\r\n",
             (unsigned long)lt_dbg_seq,
             (unsigned long)lt_dbg_t1,
             (unsigned long)lt_dbg_t2,
             (unsigned long)lt_dbg_t3,
             (unsigned long)lt_dbg_t4,
             (unsigned long)lt_rx_frames,
             (unsigned long)lt_rx_msgs,
             (unsigned long)TIM5->ARR,
             (unsigned long)TIM5->CCR4);
    }

    osDelay(LT_SYNC_PERIOD_MS);
  }
}

/* 接收任务：处理三种报文。收包时刻已经在以太网中断里打好。 */
static void lt_rx_task(void *argument)
{
  lt_msg_t m, r;
  struct sockaddr_in from;
  socklen_t fromlen;
  int n;
  uint32_t t_rx;
  int32_t  off;
  uint32_t dly;

  (void)argument;

  for (;;)
  {
    fromlen = sizeof(from);
    n = lwip_recvfrom(lt_sock, &m, sizeof(m), 0U,
                      (struct sockaddr *)&from, &fromlen);

    /* 中断里刚打下的收包时刻，越早读越好 */
    t_rx = lt_rx_stamp;

    if ((n != (int)sizeof(m)) || (m.type == 0U))
    {
      continue;
    }
    lt_rx_msgs++;

    /* 记下"通信建立"的时刻：第一次收到对方的有效报文。
       链路没起来之前报文根本发不出去，所以这一刻近似等于两块板都能通上话
       的时刻，两块板量到的起点是同一件事（相差不到一个同步周期）。 */
    if ((lt_conv_comm_up == 0U) &&
        (m.type >= LT_MSG_SYNC) && (m.type <= LT_MSG_DRESP))
    {
      lt_conv_comm_up = 1U;
      lt_conv_comm_ms = HAL_GetTick();
    }

    if (m.type == LT_MSG_SYNC)
    {
      /* 从板：记下 t2、t3，立刻回 DREQ */
      if (lt_is_master != 0U)
      {
        continue;
      }
      lt_cur_seq = m.seq;
      lt_t1 = m.t1;
      lt_t2 = t_rx;
      lt_t3 = (uint32_t)lt_now_us();
      lt_mic_exchange_phase_us = lt_rx_acoustic_phase_us;

      memset(&r, 0, sizeof(r));
      r.type = LT_MSG_DREQ;
      r.seq  = m.seq;
      r.pad[0] = (lt_ff_state >= 2U) ? LT_FLAG_FF_READY : 0U;
      r.t2   = lt_t2;
      r.t3   = lt_t3;
      (void)lwip_sendto(lt_sock, &r, sizeof(r), 0U,
                        (struct sockaddr *)&from, sizeof(from));
    }
    else if (m.type == LT_MSG_DREQ)
    {
      /* 主板：t1 是本地存的、t4 是刚打的时间戳，它也算一遍用于显示 */
      if (lt_is_master == 0U)
      {
        continue;
      }
      off = ((int32_t)(m.t2 - lt_t1) - (int32_t)(t_rx - m.t3)) / 2;
      dly = (t_rx - lt_t1) - (m.t3 - m.t2);
      lt_push_sample(off, dly, (uint32_t)(lt_now_us() / 1000ULL));

      /* 从板预标定至少持续 6 s；主板等它完成后再累计 10 s 稳定期。 */
      if (((m.pad[0] & LT_FLAG_FF_READY) != 0U) &&
          (dly <= ((uint32_t)lt_min_dly_us + LT_DLY_ACCEPT_US)))
      {
        lt_off_clean = off;
        lt_conv_push(off);           /* 只用干净样本判定收敛 */
      }

      lt_dbg_seq = m.seq;
      lt_dbg_t1  = lt_t1;
      lt_dbg_t2  = m.t2;
      lt_dbg_t3  = m.t3;
      lt_dbg_t4  = t_rx;

      memset(&r, 0, sizeof(r));
      r.type = LT_MSG_DRESP;
      r.seq  = m.seq;
      r.t1   = lt_t1;
      r.t4   = t_rx;
      r.dly_m = dly;
      r.off_m = off;      /* 把主板算出的偏移带给从板 */
      (void)lwip_sendto(lt_sock, &r, sizeof(r), 0U,
                        (struct sockaddr *)&from, sizeof(from));
    }
    else if (m.type == LT_MSG_DRESP)
    {
      /* 从板：现在凑齐 t1..t4 */
      if (lt_is_master != 0U)
      {
        continue;
      }
      if (m.seq != lt_cur_seq)
      {
        lt_mes_bad++;   /* 序号对不上（极少见），丢弃这次 */
        continue;
      }
      off = ((int32_t)(lt_t2 - lt_t1) - (int32_t)(m.t4 - lt_t3)) / 2;
      dly = (m.t4 - lt_t1) - (lt_t3 - lt_t2);
      lt_push_sample(off, dly, (uint32_t)(lt_now_us() / 1000ULL));

      /* 从板自己算的偏移带一个固定偏差（实测约 90us），拿它伺服会把时钟
         锁到错误的位置。偏移是同一个物理量，主板算得干净，就用主板的。 */
      lt_off_from_master = m.off_m;
      lt_dly_from_master = m.dly_m;
#if LT_NUDGE_TEST
      {
        static uint32_t s_last_ms = 0U;
        uint32_t now_ms = (uint32_t)(lt_now_us() / 1000ULL);
        if ((now_ms - s_last_ms) >= 10000U)
        {
          s_last_ms = now_ms;
          lt_adjust_local_time(100);
          lt_phase_total_us += 100;
        }
      }
#else
      if (m.dly_m <= ((uint32_t)lt_min_dly_us + LT_DLY_ACCEPT_US))
      {
        lt_off_clean = m.off_m;
        /* 预标定还没结算时不判定收敛：那段时间频差未修正，即使暂时在带内
           也不能算数，否则 CONVERGED 可能早于 ARR 写入。 */
        if (lt_ff_state >= 2U)
        {
          lt_conv_push(m.off_m);
        }
        /* Stricter quality check for TOA correction than for the servo:
           4 us extra round-trip delay can already mean 2 us offset error. */
        if ((lt_ff_state >= 2u) &&
            (m.dly_m <= (uint32_t)lt_min_dly_us + 4u) &&
            (m.off_m - LT_OFF_CAL_US >= -50) &&
            (m.off_m - LT_OFF_CAL_US <= 50))
        {
          taskENTER_CRITICAL();
          lt_mic_fit_samples[lt_mic_fit_index].stamp = lt_t2;
          lt_mic_fit_samples[lt_mic_fit_index].phase = lt_mic_exchange_phase_us;
          lt_mic_fit_samples[lt_mic_fit_index].offset = m.off_m - LT_OFF_CAL_US;
          lt_mic_fit_index=(uint8_t)((lt_mic_fit_index+1u)%LT_MIC_FIT_N);
          if (lt_mic_fit_count < LT_MIC_FIT_N) { lt_mic_fit_count++; }
          taskEXIT_CRITICAL();
        }
        lt_servo(m.off_m - LT_OFF_CAL_US, m.dly_m);
      }
      else
      {
        lt_servo_skip++;
      }
#endif

      lt_dbg_seq = m.seq;
      lt_dbg_t1  = lt_t1;
      lt_dbg_t2  = lt_t2;
      lt_dbg_t3  = lt_t3;
      lt_dbg_t4  = m.t4;
    }
    else if (m.type == LT_MSG_TOA)
    {
      /* 对方的声到达时刻：只喂给跨板 TDOA 的配对统计。
         这条分支不读写任何同步状态量（lt_off_ / lt_dly_ / lt_conv_ / lt_servo_ 开头的那些），
         所以加它不会影响第一步的时钟收敛。 */
      mic_tdoa_rx(m.seq, m.t1);
    }
    else
    {
      lt_mes_bad++;
    }
  }
}

/* 建 socket、绑定端口、起收发两个任务 */
static void lt_start(void)
{
  struct sockaddr_in localaddr;
  uint32_t peer_last;

  lt_sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
  if (lt_sock < 0)
  {
    lt_log("[LT] socket() failed (%d)\r\n", lt_sock);
    return;
  }

  memset(&localaddr, 0, sizeof(localaddr));
  localaddr.sin_family = AF_INET;
  localaddr.sin_port = htons(LT_PORT);
  localaddr.sin_addr.s_addr = htonl(0U);   /* 0.0.0.0：接受本机任意地址的报文 */

  if (lwip_bind(lt_sock, (struct sockaddr *)&localaddr, sizeof(localaddr)) < 0)
  {
    lt_log("[LT] bind(port %u) failed\r\n", (unsigned)LT_PORT);
    (void)lwip_close(lt_sock);
    lt_sock = -1;
    return;
  }

  memset(&lt_peer, 0, sizeof(lt_peer));
  lt_peer.sin_family = AF_INET;
  lt_peer.sin_port = htons(LT_PORT);
  /* 对端就是同一网段里的另一块板：本板 .10 指向 .11，本板 .11 指向 .10 */
  peer_last = (IP_ADDRESS[3] == 10U) ? 11U : 10U;
  /* 角色：IP 小的是主板，它的时钟就是全系统的时间基准，永远不调整 */
  lt_is_master = (IP_ADDRESS[3] == 10U) ? 1U : 0U;
  /* 记住名义 ARR，伺服就是在这个值上加减计数来做频率修正的 */
  lt_arr_nominal = TIM5->ARR;
  lt_peer.sin_addr.s_addr = htonl(((uint32_t)192U << 24) |
                                  ((uint32_t)168U << 16) |
                                  ((uint32_t)1U   <<  8) |
                                  peer_last);

  lt_log("\r\n[LT] sync start: local %u.%u.%u.%u:%u -> peer 192.168.1.%u:%u\r\n",
         (unsigned)IP_ADDRESS[0], (unsigned)IP_ADDRESS[1],
         (unsigned)IP_ADDRESS[2], (unsigned)IP_ADDRESS[3],
         (unsigned)LT_PORT, (unsigned)peer_last, (unsigned)LT_PORT);
  lt_log("[LT] role: %s | 发起周期 %u ms | 回归窗口 %u 样本\r\n",
         (lt_is_master != 0U) ? "MASTER (时间基准)" : "SLAVE  (跟随主板)",
         (unsigned)LT_SYNC_PERIOD_MS, (unsigned)LT_MAX_SAMPLES);
  lt_log("[LT] time base: %lu counts/s, %lu counts/us, PPS on PI0 (Arduino D5)\r\n",
         (unsigned long)lt_counts_per_sec(), (unsigned long)lt_counts_per_us());
   lt_log("[LT] offset calibration: %ld us\r\n", (long)LT_OFF_CAL_US);
  /* 一次性回读关键寄存器：用来判断 PWM 到底有没有配置成功。
     期望值 CEN=1 CC4E=1 OC4M=6 (PWM1) MODER=2 (复用) AFR=2 (AF2) */
  lt_log("[LT] TIM5 chk: CEN=%u CC4E=%u OC4M=%u CCR4=%lu ARR=%lu\r\n",
         (unsigned)((TIM5->CR1  & TIM_CR1_CEN)     ? 1U : 0U),
         (unsigned)((TIM5->CCER & TIM_CCER_CC4E)   ? 1U : 0U),
         (unsigned)((TIM5->CCMR2 >> 12) & 0x7UL),   /* OC4M 在 CCMR2 的 [14:12] */
         (unsigned long)TIM5->CCR4,
         (unsigned long)TIM5->ARR);
  lt_log("[LT] PI0 chk: MODER=%lu AFR=%lu\r\n",
         (unsigned long)((GPIOI->MODER  >>  0) & 0x3UL),
         (unsigned long)((GPIOI->AFR[0] >>  0) & 0xFUL));

  (void)osThreadNew(lt_tx_task, NULL, &lt_tx_attr);
  (void)osThreadNew(lt_rx_task, NULL, &lt_rx_attr);
}

/* USER CODE END Application */

