/* DHT11 PB4: TIM7 generates start pulse, EXTI4 records sensor edges. */
#ifndef LT_DHT11_PB4_H
#define LT_DHT11_PB4_H
#include "main.h"
static volatile uint8_t dht_phase, dht_active, dht_state, dht_bits, dht_error;
static volatile uint8_t dht_bytes[5];
static volatile uint16_t dht_edge;
static volatile uint32_t dht_rx_started;
static volatile uint8_t dht_initialized, dht_valid;
static volatile int dht_temp10;
/* Single-point calibration in tenths of a degree C; raw reading is retained. */
#define DHT11_TEMP_OFFSET10 (5)
static int dht_calibrated_temp10(void) { return dht_temp10 + DHT11_TEMP_OFFSET10; }
static unsigned dht_hum10;
static uint32_t dht_due, dht_sequence;
static void dht_release(void) { GPIOB->BSRR = GPIO_PIN_4; GPIOB->MODER &= ~(3u << 8); __DSB(); }
static void dht_stop_rx(uint8_t error) { EXTI->IMR &= ~GPIO_PIN_4; dht_error = error; dht_active = 0u; }
void TIM7_IRQHandler(void)
{
  if ((TIM7->SR & TIM_SR_UIF) == 0u) { return; }
  TIM7->DIER = 0u; TIM7->ARR = 0xFFFFu; TIM7->EGR = TIM_EGR_UG; TIM7->SR = 0u;
  if (dht_phase != 1u) { return; }
  dht_rx_started = HAL_GetTick(); dht_phase = 2u; dht_state = 0u; dht_bits = 0u;
  dht_error = 0u; dht_edge = 0u; dht_active = 1u;
  EXTI->PR = GPIO_PIN_4; EXTI->IMR |= GPIO_PIN_4; dht_release();
}
void EXTI4_IRQHandler(void)
{
  uint16_t now, width; uint8_t high, index;
  if ((EXTI->PR & GPIO_PIN_4) == 0u) { return; }
  now = (uint16_t)TIM7->CNT; high = ((GPIOB->IDR & GPIO_PIN_4) != 0u) ? 1u : 0u;
  EXTI->PR = GPIO_PIN_4; if (dht_active == 0u) { return; }
  width = (uint16_t)(now - dht_edge);
  switch (dht_state)
  {
    case 0u: if (high == 0u) { dht_edge = now; dht_state = 1u; } break;
    case 1u: if ((high == 0u) || (width < 40u) || (width > 140u)) { dht_stop_rx(4u); break; } dht_edge=now; dht_state=2u; break;
    case 2u: if ((high != 0u) || (width < 40u) || (width > 140u)) { dht_stop_rx(4u); break; } dht_edge=now; dht_state=3u; break;
    case 3u: if ((high == 0u) || (width < 20u) || (width > 100u)) { dht_stop_rx(4u); break; } dht_edge=now; dht_state=4u; break;
    case 4u:
      if ((high != 0u) || (width < 10u) || (width > 110u)) { dht_stop_rx(4u); break; }
      index=dht_bits/8u; dht_bytes[index]=(uint8_t)((dht_bytes[index]<<1)|((width>45u)?1u:0u));
      ++dht_bits; dht_edge=now; if (dht_bits==40u) { dht_stop_rx(0u); } else { dht_state=3u; } break;
    default: dht_stop_rx(4u); break;
  }
}
static void dht11_init(void)
{
  GPIO_InitTypeDef pin = {0}; uint32_t f=HAL_RCC_GetPCLK1Freq();
  uint32_t div=(RCC->CFGR & RCC_CFGR_PPRE1)>>RCC_CFGR_PPRE1_Pos;
  __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_SYSCFG_CLK_ENABLE(); __HAL_RCC_TIM7_CLK_ENABLE();
  if ((RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE)!=0u) { f=(div<=5u)?HAL_RCC_GetHCLKFreq():f*4u; }
  else if (div>=4u) { f*=2u; }
  TIM7->CR1=0u; TIM7->DIER=0u; TIM7->PSC=f/1000000u-1u; TIM7->ARR=0xFFFFu; TIM7->EGR=TIM_EGR_UG; TIM7->SR=0u;
  GPIOB->BSRR=GPIO_PIN_4; pin.Pin=GPIO_PIN_4; pin.Mode=GPIO_MODE_OUTPUT_OD; pin.Pull=GPIO_NOPULL; pin.Speed=GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB,&pin); dht_release(); EXTI->IMR &= ~GPIO_PIN_4;
  SYSCFG->EXTICR[1]=(SYSCFG->EXTICR[1]&~0xFu)|1u; EXTI->RTSR|=GPIO_PIN_4; EXTI->FTSR|=GPIO_PIN_4; EXTI->PR=GPIO_PIN_4;
  HAL_NVIC_SetPriority(EXTI4_IRQn,6u,0u); HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn); HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_SetPriority(TIM7_IRQn,6u,0u); HAL_NVIC_ClearPendingIRQ(TIM7_IRQn); HAL_NVIC_EnableIRQ(TIM7_IRQn);
  dht_due=HAL_GetTick()+2000u; dht_initialized=1u;
}
static void dht11_poll(void)
{
  uint32_t now=HAL_GetTick(); uint8_t p[5],i,sum; int temperature;
  if (!dht_initialized) { dht11_init(); return; }
  if (dht_phase==1u) { if ((uint32_t)(now-dht_due)>100u) { TIM7->DIER=0u; dht_release(); dht_stop_rx(5u); dht_phase=0u; dht_valid=0u; dht_due=now+2000u; ++dht_sequence; } return; }
  if (dht_phase==2u)
  {
    if (dht_active) { if ((uint32_t)(now-dht_rx_started)<10u) { return; } dht_stop_rx((dht_state==0u)?1u:2u); }
    dht_valid=0u; for(i=0u;i<5u;++i) { p[i]=dht_bytes[i]; }
    if ((dht_error==0u)&&(dht_bits==40u))
    {
      sum=(uint8_t)(p[0]+p[1]+p[2]+p[3]);
      if (sum!=p[4]) { dht_error=3u; }
      else if ((p[0]>100u)||(p[1]>9u)||(p[2]>80u)||((p[3]&0x7Fu)>9u)) { dht_error=6u; }
      else { temperature=(int)p[2]*10+(p[3]&0x7Fu); if(p[3]&0x80u) {temperature=-temperature;} dht_temp10=temperature; dht_hum10=(unsigned)p[0]*10u+p[1]; dht_valid=1u; }
    }
    dht_phase=0u; dht_due=now+2000u; ++dht_sequence; return;
  }
  if ((int32_t)(now-dht_due)<0) { return; }
  if (!(GPIOB->IDR & GPIO_PIN_4)) { dht_error=7u; dht_valid=0u; dht_due=now+2000u; ++dht_sequence; return; }
  EXTI->IMR &= ~GPIO_PIN_4; for(i=0u;i<5u;++i) {dht_bytes[i]=0u;} dht_phase=1u; dht_due=now;
  TIM7->CR1=0u; TIM7->DIER=0u; TIM7->ARR=19999u; TIM7->EGR=TIM_EGR_UG; TIM7->SR=0u;
  GPIOB->BSRR=(uint32_t)GPIO_PIN_4<<16; GPIOB->MODER=(GPIOB->MODER&~(3u<<8))|(1u<<8);
  TIM7->DIER=TIM_DIER_UIE; TIM7->CR1=TIM_CR1_CEN;
}
static void dht11_draw(void)
{
  char text[24]; int temp10=dht_calibrated_temp10(); unsigned mag=(unsigned)((temp10<0)?-temp10:temp10);
  static const char *const errors[]={"Temp:WAIT","Temp:NO DEV","Temp:RX ERR","Temp:CRC","Temp:TIMING","Temp:TIMER","Temp:RANGE","Temp:LOW"};
  if (disp_temp_menu != DISP_TEMP_CLOSED) {return;}
  if (disp_auto_temp == 0u)
  {
    (void)snprintf(text, sizeof(text), "Used:%+dC", (int)disp_temp_c[disp_temp_sel]);
    disp_fill(311,122,467,139,C_PANEL);
    (void)disp_str(313,123,text,C_ACC,1);
    return;
  }
  if (dht_valid) {(void)snprintf(text,sizeof(text),"Temp:%s%u.%uC",temp10<0?"-":"",mag/10u,mag%10u);}
  else {(void)snprintf(text,sizeof(text),"%s",errors[dht_error<8u?dht_error:2u]);}
  disp_fill(311,122,467,139,C_PANEL); (void)disp_str(313,123,text,dht_valid?C_ACC:C_WARN,1);
}
static void dht11_report(void)
{
  static uint32_t last=0xFFFFFFFFu; char message[150]; int n;
  if(last==dht_sequence){return;} last=dht_sequence;
  n=snprintf(message,sizeof(message),"[DHT11] PB4 valid=%u error=%u bits=%u T10=%d RH10=%u raw=%02X %02X %02X %02X %02X\r\n",(unsigned)dht_valid,(unsigned)dht_error,(unsigned)dht_bits,dht_temp10,dht_hum10,(unsigned)dht_bytes[0],(unsigned)dht_bytes[1],(unsigned)dht_bytes[2],(unsigned)dht_bytes[3],(unsigned)dht_bytes[4]);
  if((n>0)&&(n<(int)sizeof(message))){(void)HAL_UART_Transmit(&huart1,(uint8_t *)message,(uint16_t)n,20u);}
}
#endif
