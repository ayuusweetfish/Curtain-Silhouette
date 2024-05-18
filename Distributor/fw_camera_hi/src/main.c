#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mlx90640/MLX90640_API.h"

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

extern uint32_t _etext;
#define CODE_START_ADDR     0x08000000
#define CODE_END_ADDR       (uint32_t)(&_etext)
#define SCRATCH_START_ADDR  ((CODE_END_ADDR + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1))
#define SCRATCH_END_ADDR    0x08020000

SPI_HandleTypeDef spi2;
I2C_HandleTypeDef i2c2;

#define N_SUSPEND 200
uint8_t spi_tx_buf[25 * N_SUSPEND / 2];

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
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
  // Clock tree: RM0090 (Rev 20) p. 218 Fig. 21
  // Frequency ranges: DS8626 (Rev 9) p. 104 Tab. 36
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI; // 16 MHz internal RC
  osc_init.PLL.PLLM = 8;              // PLL VCO input 2 MHz (0.95 ~ 2.10 MHz)
    // XXX: HAL says minimum is 0 but RM0090 (Rev 20) p. 229 says 0 and 1 are wrong configurations?
  osc_init.PLL.PLLN = 168;            // VCO output 336 MHz (100 ~ 432 MHz)
  osc_init.PLL.PLLP = RCC_PLLP_DIV2;  // PLL general output 168 MHz (24 ~ 168 MHz)
  osc_init.PLL.PLLQ = 2;              // USB OTG FS, SDIO, RNG 168 MHz
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1 |
    RCC_CLOCKTYPE_PCLK2;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 168 MHz
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_5);  // AN3988 (Rev 2) p. 9 Tab. 2

  // swv_printf("Sys clock = %u\n", HAL_RCC_GetSysClockFreq());

  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

  // ======== SPI2 (PB12 CS, PB13 SCK, PB15 MOSI) ========
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_15;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF5_SPI2;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Pin = GPIO_PIN_12;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);

  __HAL_RCC_SPI2_CLK_ENABLE();
  spi2 = (SPI_HandleTypeDef){
    .Instance = SPI2,
    .Init = (SPI_InitTypeDef){
      .Mode = SPI_MODE_MASTER,
      .Direction = SPI_DIRECTION_2LINES,
      .DataSize = SPI_DATASIZE_8BIT,
      .CLKPolarity = SPI_POLARITY_LOW,  // CPOL = 0
      .CLKPhase = SPI_PHASE_1EDGE,      // CPHA = 0
      .NSS = SPI_NSS_SOFT,
      .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8, // 21 MHz
      .FirstBit = SPI_FIRSTBIT_MSB,
      .TIMode = SPI_TIMODE_DISABLE,
      .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    },
  };
  HAL_SPI_Init(&spi2);

  // Act LEDs
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, 1);

  // ======== MCO (PB8 MCO1) ========
  // GPIO initialisation is done by `HAL_RCC_MCOConfig`
  // 168 MHz / 4 = 42 MHz
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_4);

  // ======== I2C2 (PB10 SCL, PB11 SDA) ========
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_10 | GPIO_PIN_11,
    .Mode = GPIO_MODE_AF_OD,
    .Alternate = GPIO_AF4_I2C2,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOB, &gpio_init);

  __HAL_RCC_I2C2_CLK_ENABLE();
  i2c2 = (I2C_HandleTypeDef){
    .Instance = I2C2,
    .Init = {
      .ClockSpeed = 100000,
      .DutyCycle = I2C_DUTYCYCLE_2,
      .OwnAddress1 = 0x00,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c2);

  uint8_t clkrc = 0b10000111; // Prescale by 8
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x11, I2C_MEMADD_SIZE_8BIT, &clkrc, 1, 1000);
/*
  uint8_t test_pattern_x = 0b10111010;
  uint8_t test_pattern_y = 0b00110101;  // 8-bar color bar
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x70, I2C_MEMADD_SIZE_8BIT, &test_pattern_x, 1, 1000);
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x71, I2C_MEMADD_SIZE_8BIT, &test_pattern_y, 1, 1000);
*/
  swv_printf("err %d\n", i2c2.ErrorCode);

  if (i2c2.ErrorCode != 0) while (1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); HAL_Delay(200);
  }

  // Test

  while (1) {
    static int parity = 1;
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, parity ^= 1);

    // Running lights

    static int count = 0;
    static int phase = 0;

    for (int i = 0; i < sizeof spi_tx_buf / sizeof spi_tx_buf[0]; i++)
      spi_tx_buf[i] = 0;

    if (++count % 2 == 0) phase = (phase + 1) % 8;
    for (int strip = 0; strip < 8; strip++) {
      for (int i = 0; i < N_SUSPEND; i++) {
        uint8_t value;
        if (i % 8 == (strip + count / 50) % 8) value = 0xf;
        else {
          int x = (i / 2 + phase) % 8;
          value = (x >= 4 ? (7 - x) : x);
        }
        spi_tx_buf[(strip * N_SUSPEND + i) / 2] |= (value << (i % 2 == 0 ? 0 : 4));
      }
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
    int result = HAL_SPI_Transmit(&spi2, spi_tx_buf, 25 * N_SUSPEND / 2, 1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
    swv_printf("SPI tx result = %d\n", result);
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
void DMA1_Channel1_IRQHandler() { while (1) { } }
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
