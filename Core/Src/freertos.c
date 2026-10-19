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
   32 位无符号减法本身就是回绕安全的，不需要 64 位。 */
typedef struct
{
  uint8_t  type;    /* 1=SYNC(主板->从板) 2=DREQ(从板->主板) 3=DRESP(主板->从板) */
  uint8_t  pad[3];
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
#define LT_DLY_ACCEPT_US    20U
#define LT_CONV_LIMIT_MS    30000U  /* 收敛时限：任务书要求 30 秒 */
#define LT_CONV_BAND_US     10      /* 判据带 = 任务书指标 ±10us（也是手工测量口径）。
                                       原来放 30us：偏差还有一二十微秒时就宣告收敛，
                                       与"收敛到 10us 以内"不是一回事，会误报提前。 */
#define LT_CONV_HOLD_MS     10000U  /* 要连续保持这么久才算收敛（与验收"连续 10 次"
                                       对应：脉冲是 1Hz，连续 10 次测量就是 10 秒）*/
#define LT_CONV_MIN_SAMP    20U     /* 这段稳定期里至少要有这么多干净样本，
                                       防止"样本被剔光、等于没判"也算稳定 */

/* ---- ARR（频率）环路的积分参数 ---------------------------------------------
   详见 lt_servo() 里的说明：频率修正要"锁得住"，否则相位环路必须长期硬扛
   速率差，稳态滞后会把两板间距顶到 10us 以上，收敛就快不了。 */
#define LT_ARR_INT_GAIN     50      /* 积分增益：每 1us 相位偏差，每拍给 ARR 加 50 计数。
                                       实测这组值能满足题目要求（30 秒内收敛到 ±10us
                                       以内），不要再往上加：ARR 每秒才生效一次，
                                       增益再大（100/150）会过冲、摆动变大 */
#define LT_ARR_INT_STEP_MAX 170     /* 每拍最多累加 170 计数（约 1.6ppm，即 16ppm/秒）：
                                       单个脏样本最多把周期拉走这么点，环路自己纠得回 */
#define LT_ARR_INT_RANGE_US 25      /* 偏差超过 ±25us 时先不碰频率：相位没拉回之前积分
                                       不参与，避免粗捕获阶段把积分冲爆（饱和） */
/* 单点标定值（微秒）：抵消两块板软件延时不对称带来的固定偏差。
   标定方法：系统跑起来后，用示波器量两个脉冲沿的间距（设为 S，从板在前取正），
   同时读串口上主板那行的 off（设为 M），则本值取 (M - S)。
   例如主板 off 显示 +45、示波器量到 26，本值填 19。 */
/* 单点标定值（微秒）：抵消协议测量与示波器真值之间的固定偏差。
   实验数据：CAL=0 时示波器间距 16.2us，CAL=+5 时 21.2us，
   即 间距 = CAL + 16.2，斜率 +1。故取 CAL = -16 可把间距推到 0。
   若实测与预期不符，按 新CAL = 旧CAL - 实测间距 修正。 */
#define LT_OFF_CAL_US       (-16)

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
static volatile int32_t lt_off_clean = 0;
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
static volatile uint8_t  lt_conv_ok      = 0U;   /* 1 = 已收敛 */
static volatile uint32_t lt_conv_ms_comm = 0U;   /* 收敛用时：通信建立 -> 对齐 */
static volatile uint32_t lt_conv_ms_pwr  = 0U;   /* 收敛用时：本板上电 -> 对齐 */
static uint32_t lt_conv_t_first_ms = 0U;         /* 本段首个进带样本的时刻 */
static uint32_t lt_conv_streak     = 0U;         /* 连续在带内的干净样本数 */
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

/* --- 诊断用：把最近一次交换的四个原始时间戳打印出来，便于两块板逐字段对照 --- */
static volatile uint32_t lt_rx_msgs = 0U;      /* 收到的报文数 */
static volatile uint32_t lt_dbg_seq = 0U;
static volatile uint32_t lt_dbg_t1 = 0U, lt_dbg_t2 = 0U, lt_dbg_t3 = 0U, lt_dbg_t4 = 0U;
static int32_t lt_phase_total_us = 0;   /* 伺服累计拨动了多少微秒（诊断用）*/

/* 样本窗口：index 0 最旧，按时间先后排列。
   时刻用毫秒存，避免后面做回归时数字太大溢出。 */
static int32_t  lt_s_off[LT_MAX_SAMPLES];
static uint32_t lt_s_dly[LT_MAX_SAMPLES];
static uint32_t lt_s_t[LT_MAX_SAMPLES];
static uint32_t lt_s_cnt = 0U;

static const osThreadAttr_t lt_tx_attr = {
  .name = "ltTx",
  .stack_size = 512 * 4,
  /* 打印任务必须比接收任务低。
     串口打印是阻塞式轮询，一次约 13ms；如果和接收任务同优先级，时间片轮转
     会在这 13ms 里把接收任务切走，而接收任务"收 SYNC -> 记 t2 -> 发 DREQ"
     是被切在中间就变成 DREQ 发送延迟增加约 95us，直接污染 dly 和 off。
     降到 Normal 后接收任务随时抢占，样本就干净了。 */
  .priority = (osPriority_t)osPriorityNormal,
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
  .priority = (osPriority_t)osPriorityNormal,
};

/* 显示任务：20fps 刷屏，优先级放在 micTask 之下——画屏只是"看"，不能影响
   采样处理与以太网接收。 */
static const osThreadAttr_t disp_attr = {
  .name = "dispTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityBelowNormal,
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
static void lt_servo(int32_t off_us);
static void lt_conv_push(int32_t off_us);
static void lt_conv_report(void);
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
  /* 记下本板上电时刻（近似）。取在 MX_LWIP_Init() 之后，比真正的上电晚
     几十毫秒——初始化本身的开销，相对 30 秒的预算可以忽略。 */
  lt_conv_boot_ms = HAL_GetTick();
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

/* 把一次测量结果放进窗口，并重新算一次回归 */
/* ---- 伺服 ------------------------------------------------------------------
   只在从板上运行。主板是全系统的时间基准，它自己的时钟一根手指都不动。

   修正分两件事，缺一不可：

     频率修正（治本）——把相位偏差"积分"成速率修正，改 TIM5 的 ARR。
       ARR 每加一个计数，一秒就变长 9.26ns，也就是速率减慢 9.26ppb。
       把速率差纠掉之后，相位才不会一直往外漂，相位环路也就不必长期
       硬扛漂移（那是收敛慢的根子，详见下面 ①）。

     相位修正（治标）——把已经积累的偏差直接拨回去。
       只做频率不做相位，要等很久偏差才会自己回来；
       只做相位不做频率，偏差会一直漂、要反复拉。
   -------------------------------------------------------------------------- */

/* 每次测量后执行一遍。off_us = 从板时钟 - 主板时钟。 */
static void lt_servo(int32_t off_us)
{
  int32_t step;
  int32_t da;

  lt_servo_in = off_us;

  /* ---- ① 频率修正：用相位偏差做积分，只动从板的 ARR --------------------------
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
     保留 ±2000。真正的保护是上面那两道闸（每拍限速 + 相位拉回后才累加）。 */
  if ((off_us <= LT_ARR_INT_RANGE_US) && (off_us >= -LT_ARR_INT_RANGE_US))
  {
    da = off_us * LT_ARR_INT_GAIN;
    if (da >  LT_ARR_INT_STEP_MAX) { da =  LT_ARR_INT_STEP_MAX; }
    if (da < -LT_ARR_INT_STEP_MAX) { da = -LT_ARR_INT_STEP_MAX; }
    lt_arr_delta += da;
  }

  if (lt_arr_delta >  2000) { lt_arr_delta =  2000; }   /* 约 ±18.5ppm，见上面的说明 */
  if (lt_arr_delta < -2000) { lt_arr_delta = -2000; }

  TIM5->ARR = (uint32_t)((int64_t)lt_arr_nominal + lt_arr_delta);
  /* Auto-reload preload 已打开，下一个周期边界才生效 */

  /* ---- ② 相位修正 ---- */
  if ((off_us > 1000) || (off_us < -1000))
  {
    step = off_us;          /* 粗捕获：一次拨到底，几百毫秒就拉平 */
  }
  else if ((off_us > 1) || (off_us < -1))
  {
    /* 细跟踪：只拨四分之一。
       增益为 1（一次拨到位）配 100ms 采样和一拍延迟时，环路是临界稳定的，
       偏移会在 0 和 ±偏值之间来回摆；降到 1/4 才能收敛。 */
    step = off_us / 4;
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

/* ---- 收敛判定 --------------------------------------------------------------
   任务书要求：上电后 30 秒内，两板收敛到统一时间基准，且偏差不大于 ±10us。

   判据量取的就是伺服自己的误差信号：
       err = off_clean - LT_OFF_CAL_US
   它与伺服的死区、拨动方向共用同一个量（也就是串口里的 sv），
   判定门槛不会和环路目标打架；由标定关系「实际间距 = 读数 - 标定值」，
   它同时也是两板脉冲实际间距的估计值。

   只在"干净样本"上判定：被调度或打印扰动过的样本，其 off 可能偏出上百微秒，
   拿它判定会把已经收敛的系统误判成"未收敛"（这是 E4/E5 那类错误的翻版）。
   -------------------------------------------------------------------------- */
static void lt_conv_push(int32_t off_us)
{
  int32_t err;

  if (lt_conv_ok != 0U)
  {
    return;                          /* 已收敛，耗时不更新，保持首次收敛值 */
  }

  err = off_us - LT_OFF_CAL_US;

  if ((err <= LT_CONV_BAND_US) && (err >= -LT_CONV_BAND_US))
  {
    if (lt_conv_streak == 0U)
    {
      /* 本段稳定期从这一拍开始：两板就是在这里真正收敛到 ±10us 以内的 */
      lt_conv_t_first_ms = HAL_GetTick();
    }
    lt_conv_streak++;

    /* 两个条件同时满足才算真收敛：样本够多，且稳定期够长 */
    if ((lt_conv_streak >= LT_CONV_MIN_SAMP) &&
        ((HAL_GetTick() - lt_conv_t_first_ms) >= LT_CONV_HOLD_MS))
    {
      lt_conv_ok = 1U;
      /* 冻结收敛用时：主数从"通信建立"算起，另一个从"本板上电"算起 */
      lt_conv_ms_comm = lt_conv_t_first_ms - lt_conv_comm_ms;
      lt_conv_ms_pwr  = lt_conv_t_first_ms - lt_conv_boot_ms;
    }
  }
  else
  {
    lt_conv_streak = 0U;             /* 只要有一次出带，稳定期重新开始 */
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
  lt_log("[LT] CONVERGED / 收敛成功: 链路->收敛 %lu.%02lu s | 上电->链路 %lu.%02lu s | 上电->收敛 %lu.%02lu s\r\n",
         (unsigned long)(lt_conv_ms_comm / 1000U),
         (unsigned long)((lt_conv_ms_comm % 1000U) / 10U),
         (unsigned long)((lt_conv_comm_ms - lt_conv_boot_ms) / 1000U),
         (unsigned long)(((lt_conv_comm_ms - lt_conv_boot_ms) % 1000U) / 10U),
         (unsigned long)(lt_conv_ms_pwr / 1000U),
         (unsigned long)((lt_conv_ms_pwr % 1000U) / 10U));
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
  uint32_t dmin;
  uint32_t used = 0U;
  int64_t  st = 0, so = 0, stt = 0, sto = 0;
  int32_t  o0;
  uint32_t t0;

  /* 先找窗口内最小的往返时延。时延最小的样本说明报文没被排队、中断来得快，
     时间戳打得最干净——按它筛选是压制抖动最有效的一招。

     注意这一步必须**从第一个样本就开始做**（不能等攒够 8 个样本）。原来放在
     样本数判断之后，等于开头 0.8 秒既没有质量筛选、伺服也不敢动手，白等近
     一秒才进入收敛过程；而 dmin 是"窗口内最小值"，脏样本只会把它抬高到真实
     地板，脏样本自己过不了 dly <= dmin+20 这一关，所以早算不会误判。 */
  dmin = lt_s_dly[0];
  for (i = 1U; i < lt_s_cnt; i++)
  {
    if (lt_s_dly[i] < dmin)
    {
      dmin = lt_s_dly[i];
    }
  }
  lt_min_dly_us = (int32_t)dmin;

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

    if (lt_s_dly[i] > (dmin + LT_DELAY_MARGIN_US))
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
  uint32_t now;

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

    /* 收敛成功后在串口上补一行收敛用时。放在打印任务里，不塞进接收任务。 */
    lt_conv_report();

    now = osKernelGetTickCount();
    if ((now - last_report) >= LT_REPORT_PERIOD_MS)
    {
      last_report = now;
      lt_log("[LT] SAMP dlm %lu skp %lu | reg %lu/%lu\r\n",
             (unsigned long)lt_dly_from_master,
             (unsigned long)lt_servo_skip,
             (unsigned long)TIM5->ARR,
             (unsigned long)TIM5->CCR4);
      lt_log("[LT] %s | ok %lu bad %lu | off %ld us | dly %lu us | dmin %ld us | dfreq %ld ppb | arr %ld | ph %ld | offm %ld | sv %ld | ste %ld | win %lu\r\n",
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
             (unsigned long)lt_win_cnt);
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

      memset(&r, 0, sizeof(r));
      r.type = LT_MSG_DREQ;
      r.seq  = m.seq;
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

      if (dly <= ((uint32_t)lt_min_dly_us + LT_DLY_ACCEPT_US))
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
        lt_conv_push(m.off_m);       /* 同上：脏样本不参与收敛判定 */
        lt_servo(m.off_m - LT_OFF_CAL_US);
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

