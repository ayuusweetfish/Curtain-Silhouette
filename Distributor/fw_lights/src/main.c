#include <stm32g0xx_hal.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// #define RELEASE
#ifndef RELEASE
#define _release_inline
static uint8_t swv_buf[256];
static size_t swv_buf_ptr = 0;
__attribute__ ((noinline, used))
void swv_trap_line()
{
  *(volatile char *)swv_buf;
}
static inline void swv_putchar(uint8_t c)
{
  // ITM_SendChar(c);
  if (c == '\n') {
    swv_buf[swv_buf_ptr >= sizeof swv_buf ?
      (sizeof swv_buf - 1) : swv_buf_ptr] = '\0';
    swv_trap_line();
    swv_buf_ptr = 0;
  } else if (++swv_buf_ptr <= sizeof swv_buf) {
    swv_buf[swv_buf_ptr - 1] = c;
  }
}
static void swv_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) swv_putchar(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) swv_putchar('.');
    swv_putchar('\n');
  }
}
#else
#define _release_inline inline
#define swv_printf(...)
#endif

TIM_HandleTypeDef tim3;
DMA_HandleTypeDef dma1_ch1;

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // ======== Clocks ========
  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc_init.PLL.PLLM = RCC_PLLM_DIV1;  // VCO input 16 MHz (2.66 ~ 16 MHz)
  osc_init.PLL.PLLN = 8;              // VCO output 128 MHz (64 ~ 344 MHz)
  osc_init.PLL.PLLP = RCC_PLLP_DIV2;  // PLLPCLK 64 MHz
  osc_init.PLL.PLLR = RCC_PLLR_DIV2;  // PLLRCLK 64 MHz
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 64 MHz
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOF, &gpio_init);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, 1);

  // ======== DMA and timer ========
  __HAL_RCC_DMA1_CLK_ENABLE();
  dma1_ch1.Instance = DMA1_Channel1;
  dma1_ch1.Init.Request = DMA_REQUEST_TIM3_UP;
  dma1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  dma1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  dma1_ch1.Init.MemInc = DMA_MINC_ENABLE;
  dma1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  // dma1_ch1.Init.Mode = DMA_CIRCULAR;
  dma1_ch1.Init.Mode = DMA_NORMAL;
  dma1_ch1.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&dma1_ch1);
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  __HAL_RCC_TIM3_CLK_ENABLE();
  tim3 = (TIM_HandleTypeDef){
    .Instance = TIM3,
    .Init = {
      .Prescaler = 32000 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 1000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 1,
    },
  };
  HAL_TIM_Base_Init(&tim3);
/*
  HAL_TIM_Base_Start_IT(&tim3);
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
*/

  HAL_TIM_Base_Start(&tim3);
  __HAL_LINKDMA(&tim3, hdma[TIM_DMA_ID_UPDATE], dma1_ch1);
  __HAL_TIM_ENABLE_DMA(&tim3, TIM_DMA_UPDATE);

  static uint32_t w[32] = {
    1 << 17, 1 << 17, 1 << 1, 1 << 17,
    1 << 1, 1 << 17, 1 << 1, 1 << 17,
  };
  swv_printf("DMA state %d (CCR %08x / ISR %08x / GPIOF_ODR %08x) (w %08x / CNDTR %08x / CMAR %08x / CPAR %08x)\n", (int)dma1_ch1.State, DMA1_Channel1->CCR, DMA1->ISR, GPIOF->ODR, (uint32_t)w, DMA1_Channel1->CNDTR, DMA1_Channel1->CMAR, DMA1_Channel1->CPAR);
  // int a = HAL_DMA_Start_IT(&dma1_ch1, (uint32_t)w, (uint32_t)&(GPIOF->BSRR), sizeof w / 4);
  int a = HAL_DMA_Start(&dma1_ch1, (uint32_t)w, (uint32_t)&(GPIOF->BSRR), 8);
  swv_printf("start result %d\n", a);
  // Starts with:
  // DMA state 2 (CCR 00000a11 / ISR 00000000 / GPIOF_ODR 00000003) (w 20000008 / CNDTR 00000008 / CMAR 20000008 / CPAR 50001418)
  // turns to the following at first TIM3 update event:
  // DMA state 2 (CCR 00000a10 / ISR 00000009 / GPIOF_ODR 00000003) (w 20000008 / CNDTR 00000008 / CMAR 20000008 / CPAR 50001418)
  // CCR.TEIF1 is set, indicating an error: DMA cannot access GPIO in 'G0
  // Ref: https://community.st.com/t5/stm32-mcus-products/stm32g030-dma-to-gpio-bsrr-bus-error/td-p/291235
  for (int i = 0; i < 50; i++) {
    swv_printf("DMA state %d (CCR %08x / ISR %08x / GPIOF_ODR %08x) (w %08x / CNDTR %08x / CMAR %08x / CPAR %08x)\n", (int)dma1_ch1.State, DMA1_Channel1->CCR, DMA1->ISR, GPIOF->ODR, (uint32_t)w, DMA1_Channel1->CNDTR, DMA1_Channel1->CMAR, DMA1_Channel1->CPAR);
  }

  while (1) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0); HAL_Delay(499);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1); HAL_Delay(499);
    swv_printf("DMA state %d (CCR %08x / ISR %08x) (w %08x / CNDTR %08x / CMAR %08x)\n", (int)dma1_ch1.State, DMA1_Channel1->CCR, DMA1->ISR, (uint32_t)w, DMA1_Channel1->CNDTR, DMA1_Channel1->CMAR);
  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
void SVC_Handler() { while (1) { } }
void PendSV_Handler() { while (1) { } }
void WWDG_IRQHandler() { while (1) { } }
void RTC_TAMP_IRQHandler() { while (1) { } }
void FLASH_IRQHandler() { while (1) { } }
void RCC_IRQHandler() { while (1) { } }
void EXTI0_1_IRQHandler() { while (1) { } }
void EXTI2_3_IRQHandler() { while (1) { } }
void EXTI4_15_IRQHandler() { while (1) { } }
void DMA1_Channel1_IRQHandler() {
  HAL_DMA_IRQHandler(&dma1_ch1);
}
void DMA1_Channel2_3_IRQHandler() { while (1) { } }
void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler() { while (1) { } }
void ADC1_IRQHandler() { while (1) { } }
void TIM1_BRK_UP_TRG_COM_IRQHandler() { while (1) { } }
void TIM1_CC_IRQHandler() { while (1) { } }
void TIM3_IRQHandler() {
/*
  if (__HAL_TIM_GET_FLAG(&tim3, TIM_FLAG_UPDATE) &&
      __HAL_TIM_GET_IT_SOURCE(&tim3, TIM_IT_UPDATE)) {
    __HAL_TIM_CLEAR_IT(&tim3, TIM_IT_UPDATE);
    static int p = 0;
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, p ^= 1);
  }
*/
}
void TIM14_IRQHandler() { while (1) { } }
void TIM16_IRQHandler() { while (1) { } }
void TIM17_IRQHandler() { while (1) { } }
void I2C1_IRQHandler() { while (1) { } }
void I2C2_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }
void SPI2_IRQHandler() { while (1) { } }
void USART1_IRQHandler() { while (1) { } }
void USART2_IRQHandler() { while (1) { } }
