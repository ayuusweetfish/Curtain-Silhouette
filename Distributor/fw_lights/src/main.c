#include <stm32g0xx_hal.h>
#include <assert.h>
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
SPI_HandleTypeDef spi2;
DMA_HandleTypeDef dma1_ch1;

uint8_t spi_rx_buf[25 * 120] = {0};
// GRB8
uint32_t colours[8] = {
  0x114400, 0x330011, 0x0a4400, 0x050522,
  0x041100, 0x110200, 0x041100, 0x050522,
};

uint16_t out_buf[11][8] = {{ 0 }};

inline void run();

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

  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin = GPIO_PIN_0;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);

  // ======== Timer ========
  __HAL_RCC_TIM3_CLK_ENABLE();
  tim3 = (TIM_HandleTypeDef){
    .Instance = TIM3,
    .Init = {
      .Prescaler = 1 - 1,
      // .Prescaler = 10000 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 80 - 1,
      // .Period = 2000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_Base_Init(&tim3);
  HAL_TIM_Base_Start_IT(&tim3);
  // HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  // HAL_NVIC_EnableIRQ(TIM3_IRQn);

  // ======== SPI2 (PB9 CS, PB8 SCK, PB7 MOSI) ========
  gpio_init.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF1_SPI2;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Pin = GPIO_PIN_9;
  gpio_init.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  __HAL_RCC_SPI2_CLK_ENABLE();
  spi2 = (SPI_HandleTypeDef){
    .Instance = SPI2,
    .Init = (SPI_InitTypeDef){
      .Mode = SPI_MODE_SLAVE,
      .Direction = SPI_DIRECTION_2LINES,
      .DataSize = SPI_DATASIZE_8BIT,
      .CLKPolarity = SPI_POLARITY_LOW,  // CPOL = 0
      .CLKPhase = SPI_PHASE_1EDGE,      // CPHA = 0
      .NSS = SPI_NSS_HARD_INPUT,
      .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4,
      .FirstBit = SPI_FIRSTBIT_MSB,
      .TIMode = SPI_TIMODE_DISABLE,
      .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
      .NSSPMode = SPI_NSS_PULSE_DISABLE,
    },
  };
  HAL_SPI_Init(&spi2);

  // ======== DMA1 Ch1 (SPI2 Rx) ========
  __HAL_RCC_DMA1_CLK_ENABLE();
  dma1_ch1 = (DMA_HandleTypeDef){
    .Instance = DMA1_Channel1,
    .Init = (DMA_InitTypeDef){
      .Request = DMA_REQUEST_SPI2_RX,
      .Direction = DMA_PERIPH_TO_MEMORY,
      .PeriphInc = DMA_PINC_DISABLE,
      .MemInc = DMA_MINC_ENABLE,
      .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
      .MemDataAlignment = DMA_MDATAALIGN_BYTE,
      .Mode = DMA_NORMAL,
      .Priority = DMA_PRIORITY_HIGH,
    },
  };

  __HAL_LINKDMA(&spi2, hdmarx, dma1_ch1);
  HAL_DMA_Init(&dma1_ch1);
  __HAL_DMA_ENABLE_IT(&dma1_ch1, DMA_IT_TC | DMA_IT_TE);
  __HAL_DMA_ENABLE(&dma1_ch1);
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  // HAL_SPI_Receive_DMA(&spi2, spi_rx_buf, 2);
  while (0) {
    swv_printf("data %02x %02x\n", (int)spi_rx_buf[0], (int)spi_rx_buf[1]);
    HAL_Delay(400);
  }

  spi_rx_buf[0] = 15;
  spi_rx_buf[1] = 6;

  // Light [0] yellow
  out_buf[0][3] = 1;
  // Light [4] red
  out_buf[4][3] = 1;
  out_buf[4][2] = 1;
  out_buf[4][1] = 1;
  out_buf[4][0] = 1;
  // Light [10] yellow
  out_buf[10][3] = 1;

  inline uint32_t my_rand() {
    uint32_t seed = 2451023;
    seed = seed * 1103515245 + 12345;
    return seed & 0x7fffffff;
  }

  float colour_values[2] = {0, 0};

  int count = 0;
  while (1) {
    __disable_irq();
    // __set_BASEPRI(1 << 4);
    run();
    __enable_irq();
    // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0); HAL_Delay(499);
    // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1); HAL_Delay(499);
    HAL_Delay(18);

    float rate1 = 0.5f * (1 + sinf(count * (float)(0.02 * M_PI * 2)));
    float rate2 = 0.5f * (1 + sinf(count * (float)(0.02 * M_PI * 2) + (float)(M_PI * 0.5)));
    float rate3 = 0.5f * (1 + sinf(count * (float)(0.02 * M_PI * 2) + (float)(M_PI * 1)));
    float rate4 = 0.5f * (1 + sinf(count * (float)(0.02 * M_PI * 2) + (float)(M_PI * 1.5)));
    // colours[0] = ((uint8_t)(0x11 * rate1) << 16) | ((uint8_t)(0x44 * rate1) << 8) | (uint8_t)(0x00 * rate1);
    // colours[1] = ((uint8_t)(0x33 * rate2) << 16) | ((uint8_t)(0x00 * rate2) << 8) | (uint8_t)(0x11 * rate2);
    colours[6] =
      ((uint32_t)(0x11 * rate1 + 0x33 * rate2) << 16) |
      ((uint32_t)(0x44 * rate1 + 0x00 * rate2) <<  8) |
      ((uint32_t)(0x00 * rate1 + 0x11 * rate2) <<  0);
    colours[7] =
      ((uint32_t)(0x11 * rate3 + 0x33 * rate4) << 16) |
      ((uint32_t)(0x44 * rate3 + 0x00 * rate4) <<  8) |
      ((uint32_t)(0x00 * rate3 + 0x11 * rate4) <<  0);
  /*
    colour_values[0] += ((float)spi_rx_buf[0] - colour_values[0]) / 10;
    colour_values[1] += ((float)spi_rx_buf[1] - colour_values[1]) / 10;
    colours[0] = ((uint32_t)(colour_values[0] / 4) << 16) | 0;
    colours[1] = ((uint32_t)(colour_values[1] / 4) <<  8) | 0;
  */

    if (++count % 50 == 0) {
      if (count == 100) count = 0;
      HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_0);
    }
  }

  while (1) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0); HAL_Delay(499);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1); HAL_Delay(499);
    HAL_TIM_Base_Start_IT(&tim3);
  }
}

// LED used is WS2812 (instead of WS2812B; the timing requirements are different)
#pragma GCC optimize("O3")
void run()
{
  TIM3->SR = ~TIM_SR_UIF;

  for (int i = 0; i < 11; i++) {
    #define OUTPUT_BITS(_bits) do { \
      /* Output a bit vector for all 8 lights in each group at bit `j` */ \
      uint16_t bits = (_bits); \
      \
      while ((TIM3->SR & TIM_SR_UIF) == 0) { } \
      TIM3->SR = ~TIM_SR_UIF; \
      GPIOA->ODR = 0xffff; \
      \
      while (TIM3->CNT < 16) { } \
      GPIOA->ODR = bits; \
      \
      while (TIM3->CNT < 48) { } \
      GPIOA->ODR = 0x0000; \
    } while (0)

    // G
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(~out_buf[i][3]);
    OUTPUT_BITS(~out_buf[i][2]);
    OUTPUT_BITS(~out_buf[i][1]);
    OUTPUT_BITS(~out_buf[i][0]);

    // R
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(out_buf[i][3]);
    OUTPUT_BITS(out_buf[i][2]);
    OUTPUT_BITS(out_buf[i][1]);
    OUTPUT_BITS(out_buf[i][0]);

    // B
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
    OUTPUT_BITS(0);
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
  HAL_SPI_IRQHandler(&spi2);
  if (spi2.ErrorCode == 0) {
    HAL_SPI_Receive_DMA(&spi2, spi_rx_buf, 2);
  } else {
    swv_printf("SPI error %d\n", (int)spi2.ErrorCode);
    while (1) {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0); HAL_Delay(100);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1); HAL_Delay(100);
    }
  }
}
void DMA1_Channel2_3_IRQHandler() { while (1) { } }
void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler() { while (1) { } }
void ADC1_IRQHandler() { while (1) { } }
void TIM1_BRK_UP_TRG_COM_IRQHandler() { while (1) { } }
void TIM1_CC_IRQHandler() { while (1) { } }
void TIM3_IRQHandler() { while (1) { } }
void TIM14_IRQHandler() { while (1) { } }
void TIM16_IRQHandler() { while (1) { } }
void TIM17_IRQHandler() { while (1) { } }
void I2C1_IRQHandler() { while (1) { } }
void I2C2_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }
void SPI2_IRQHandler() { while (1) { } }
void USART1_IRQHandler() { while (1) { } }
void USART2_IRQHandler() { while (1) { } }
