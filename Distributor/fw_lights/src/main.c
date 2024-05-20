#include <stm32g0xx_hal.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define RELEASE
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

#define N_SUSPEND 200
uint8_t spi_rx_buf[32 * N_SUSPEND / 2] = {0};
uint16_t out_buf[N_SUSPEND][6] = {{ 0 }};
uint8_t out_buf_cd[N_SUSPEND][3] = {{ 0 }};

inline void process_lights();
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

  // ======== GPIO (Act LEDs) ========
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOF, &gpio_init);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, 1);

  // ======== GPIO (Light strips) ========
  // GPIOA
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
           GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
           GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
  HAL_GPIO_WritePin(GPIOA, gpio_init.Pin, 0);

  // GPIOB
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
           GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
           GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOB, &gpio_init);
  HAL_GPIO_WritePin(GPIOB, gpio_init.Pin, 0);

  // GPIOC
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_6 | GPIO_PIN_7,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOC, &gpio_init);
  HAL_GPIO_WritePin(GPIOC, gpio_init.Pin, 0);

  // GPIOD
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOD, &gpio_init);
  HAL_GPIO_WritePin(GPIOD, gpio_init.Pin, 0);

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

  for (int i = 0; i < 32; i++)
    for (int j = 0; j < N_SUSPEND / 2; j++)
      spi_rx_buf[i * N_SUSPEND / 2 + j] =
        ((i + j) % 5 == 0 ? 0xf : (i + j) % 5 - 1) * 0x11;
  process_lights();

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
      .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
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
  __HAL_DMA_ENABLE(&dma1_ch1);
  __HAL_DMA_ENABLE_IT(&dma1_ch1, DMA_IT_TC);
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  HAL_SPI_Receive_DMA(&spi2, spi_rx_buf, 32 * N_SUSPEND / 2);
  __HAL_DMA_DISABLE_IT(&dma1_ch1, DMA_IT_HT); // We don't need the half-transfer interrupt

  while (0) {
    swv_printf("data %02x %02x\n", (int)spi_rx_buf[0], (int)spi_rx_buf[1]);
    HAL_Delay(400);
  }

  inline uint32_t my_rand() {
    uint32_t seed = 2451023;
    seed = seed * 1103515245 + 12345;
    return seed & 0x7fffffff;
  }

  int count = 0;

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);

  uint32_t tick = HAL_GetTick();

  while (1) {
    __disable_irq();
    // __set_BASEPRI(1 << 4);  // Disable all interrupts with priority >= 1
    // asm volatile ("MSR basepri, %0" : : "r" (1 << 4) : "memory");

    run();

    __enable_irq();
    // asm volatile ("MSR basepri, %0" : : "r" (0 << 4) : "memory");

    uint32_t cur;
    while ((cur = HAL_GetTick()) - tick < 20) { }
    tick = cur;

if (0) {
    static int phase = 0;

    for (int i = 0; i < sizeof spi_rx_buf / sizeof spi_rx_buf[0]; i++)
      spi_rx_buf[i] = 0;

    if (count % 2 == 0) phase = (phase + 1) % 8;
    for (int strip = 0; strip < 8; strip++) {
      for (int i = 0; i < N_SUSPEND; i++) {
        uint8_t value;
        if (i % 8 == (strip + count / 50) % 8) value = 0xf;
        else {
          int x = (i / 2 + phase) % 8;
          value = (x >= 4 ? (7 - x) : x);
          // x = (i / 2) % 8;
          // value = (x < 4 ? 0 : 3);
        }
        spi_rx_buf[(strip * N_SUSPEND + i) / 2] |= (value << (i % 2 == 0 ? 0 : 4));
      }
    }
    process_lights();
 }

    if (++count % 50 == 0) {
      if (count == 100) count = 0;
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, count == 0 ? 0 : 1);
    }
  }

  while (1) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0); HAL_Delay(499);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1); HAL_Delay(499);
    HAL_TIM_Base_Start_IT(&tim3);
  }
}

#pragma GCC optimize("O3")
void process_lights()
{
  // Ports A, B

  for (unsigned pendu = 0; pendu < N_SUSPEND; pendu++) {
    uint8_t shift = (pendu % 2 ? 4 : 0);
    for (unsigned bit = 0; bit <= 1; bit++) {
      out_buf[pendu][0 + bit] =
        (((spi_rx_buf[( 0 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  0) | 
        (((spi_rx_buf[( 1 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  1) | 
        (((spi_rx_buf[( 2 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  2) | 
        (((spi_rx_buf[( 3 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  3) | 
        (((spi_rx_buf[( 4 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  4) | 
        (((spi_rx_buf[( 5 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  5) | 
        (((spi_rx_buf[( 6 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  6) | 
        (((spi_rx_buf[( 7 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  7) |
        (((spi_rx_buf[(22 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  8) |
        (((spi_rx_buf[(21 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  9) |
        (((spi_rx_buf[(18 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) << 10) |
        (((spi_rx_buf[(17 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) << 11) |
        (((spi_rx_buf[(16 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) << 12);
      out_buf[pendu][3 + bit] =
        (((spi_rx_buf[( 8 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  0) | 
        (((spi_rx_buf[( 9 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  1) | 
        (((spi_rx_buf[(10 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  2) | 
        (((spi_rx_buf[(28 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  3) | 
        (((spi_rx_buf[(29 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  4) | 
        (((spi_rx_buf[(30 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  5) | 
        (((spi_rx_buf[(31 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  6) | 
        (((spi_rx_buf[(11 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) << 10) |
        (((spi_rx_buf[(12 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) << 11) |
        (((spi_rx_buf[(13 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) << 12) |
        (((spi_rx_buf[(14 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) << 13) |
        (((spi_rx_buf[(15 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) << 14) |
        (((spi_rx_buf[(23 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) << 15);
    }
    out_buf[pendu][0 + 2] =
      (((spi_rx_buf[( 0 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  0)) |
      (((spi_rx_buf[( 1 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  1)) |
      (((spi_rx_buf[( 2 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  2)) |
      (((spi_rx_buf[( 3 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  3)) |
      (((spi_rx_buf[( 4 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  4)) |
      (((spi_rx_buf[( 5 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  5)) |
      (((spi_rx_buf[( 6 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  6)) |
      (((spi_rx_buf[( 7 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  7)) |
      (((spi_rx_buf[(22 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  8)) |
      (((spi_rx_buf[(21 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  9)) |
      (((spi_rx_buf[(18 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 << 10)) |
      (((spi_rx_buf[(17 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 << 11)) |
      (((spi_rx_buf[(16 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 << 12));
    out_buf[pendu][3 + 2] =
      (((spi_rx_buf[( 8 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  0)) |
      (((spi_rx_buf[( 9 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  1)) |
      (((spi_rx_buf[(10 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  2)) |
      (((spi_rx_buf[(28 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  3)) |
      (((spi_rx_buf[(29 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  4)) |
      (((spi_rx_buf[(30 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  5)) |
      (((spi_rx_buf[(31 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  6)) |
      (((spi_rx_buf[(11 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 << 10)) |
      (((spi_rx_buf[(12 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 << 11)) |
      (((spi_rx_buf[(13 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 << 12)) |
      (((spi_rx_buf[(14 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 << 13)) |
      (((spi_rx_buf[(15 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 << 14)) |
      (((spi_rx_buf[(23 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 << 15));
  }

  // Ports C, D

  for (unsigned pendu = 0; pendu < N_SUSPEND; pendu++) {
    uint8_t shift = (pendu % 2 ? 4 : 0);
    for (unsigned bit = 0; bit <= 1; bit++) {
      out_buf_cd[pendu][0 + bit] =
        (((spi_rx_buf[(20 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  6) | 
        (((spi_rx_buf[(19 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  7) |
        (((spi_rx_buf[(24 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  0) | 
        (((spi_rx_buf[(25 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  1) | 
        (((spi_rx_buf[(26 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  2) | 
        (((spi_rx_buf[(27 * N_SUSPEND + pendu) / 2] >> (bit + shift)) & 1) <<  3);
    }
    out_buf_cd[pendu][0 + 2] =
      (((spi_rx_buf[(20 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  6)) |
      (((spi_rx_buf[(19 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  7)) |
      (((spi_rx_buf[(24 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  0)) |
      (((spi_rx_buf[(25 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  1)) |
      (((spi_rx_buf[(26 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  2)) |
      (((spi_rx_buf[(27 * N_SUSPEND + pendu) / 2] >> shift) & 0xf) == 0xf ? 0 : (1 <<  3));
  }
}

// LED used is WS2812 (instead of WS2812B; the timing requirements are different)
#define OUTPUT_BITS(_portA, _portB, _bitsA, _bitsB) do { \
  /* Output a bit vector for up to 16 lights in each group at bit `j` */ \
  uint16_t bitsA = (_bitsA); \
  \
  while ((TIM3->SR & TIM_SR_UIF) == 0) { } \
  TIM3->SR = ~TIM_SR_UIF; \
  GPIO##_portA->ODR = 0xffff; \
  GPIO##_portB->ODR = 0xffff; \
  \
  uint16_t bitsB = (_bitsB); \
  \
  while (TIM3->CNT < 16) { } \
  GPIO##_portA->ODR = bitsA; \
  GPIO##_portB->ODR = bitsB; \
  \
  while (TIM3->CNT < 48) { } \
  GPIO##_portA->ODR = 0x0000; \
  GPIO##_portB->ODR = 0x0000; \
} while (0)

#pragma GCC optimize("O3")
void run()
{
  // Ports A, B

  TIM3->SR = ~TIM_SR_UIF;

  for (int i = 0; i < N_SUSPEND; i++) {
    // G
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, ~out_buf[i][1] & out_buf[i][2], ~out_buf[i][4] & out_buf[i][5]);
    OUTPUT_BITS(A, B, ~out_buf[i][0] & out_buf[i][2], ~out_buf[i][3] & out_buf[i][5]);

    // R
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, out_buf[i][1] & out_buf[i][2], out_buf[i][4] & out_buf[i][5]);
    OUTPUT_BITS(A, B, out_buf[i][0] & out_buf[i][2], out_buf[i][3] & out_buf[i][5]);

    // B
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, 0, 0);
    OUTPUT_BITS(A, B, out_buf[i][2], out_buf[i][5]);
  }

  // Repeat yourself
  // Ports C, D

  TIM3->SR = ~TIM_SR_UIF;

  for (int i = 0; i < N_SUSPEND; i++) {
    // G
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, ~out_buf_cd[i][1] & out_buf_cd[i][2], ~out_buf_cd[i][1] & out_buf_cd[i][2]);
    OUTPUT_BITS(C, D, ~out_buf_cd[i][0] & out_buf_cd[i][2], ~out_buf_cd[i][0] & out_buf_cd[i][2]);

    // R
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, out_buf_cd[i][1] & out_buf_cd[i][2], out_buf_cd[i][1] & out_buf_cd[i][2]);
    OUTPUT_BITS(C, D, out_buf_cd[i][0] & out_buf_cd[i][2], out_buf_cd[i][0] & out_buf_cd[i][2]);

    // B
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, 0, 0);
    OUTPUT_BITS(C, D, out_buf_cd[i][2], out_buf_cd[i][2]);
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
    process_lights();

    HAL_SPI_Receive_DMA(&spi2, spi_rx_buf, 32 * N_SUSPEND / 2);
    __HAL_DMA_DISABLE_IT(&dma1_ch1, DMA_IT_HT);
    static int parity = 1;
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, parity ^= 1);
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
