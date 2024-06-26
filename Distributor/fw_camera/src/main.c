#include <stm32g0xx_hal.h>
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

uint32_t frame_count = 0;
uint32_t frame_sum = 0, frame_sum2 = 0;
uint32_t frame_errors = 0;

#define N_SUSPEND 200
uint8_t spi_tx_buf[25 * N_SUSPEND / 2];

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

  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

  // ======== SPI2 (PB9 CS, PB8 SCK, PB7 MOSI) ========
  gpio_init.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF1_SPI2;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  gpio_init.Pin = GPIO_PIN_9;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);

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
      .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4,
      .FirstBit = SPI_FIRSTBIT_MSB,
      .TIMode = SPI_TIMODE_DISABLE,
      .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
      .NSSPMode = SPI_NSS_PULSE_DISABLE,
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

  for (int i = 0; i < 2; i++) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); HAL_Delay(50);
  }

if (0) {
  swv_printf("code: %08x ~ %08x\n", CODE_START_ADDR, CODE_END_ADDR);
  swv_printf("scratch: %08x ~ %08x\n", SCRATCH_START_ADDR, SCRATCH_END_ADDR);
  swv_printf("empty cell [0]: %08x\n", *(uint32_t *)SCRATCH_START_ADDR);

  FLASH_EraseInitTypeDef erase = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .Banks = FLASH_BANK_1,
    .Page = (SCRATCH_START_ADDR - CODE_START_ADDR) / FLASH_PAGE_SIZE,
    .NbPages = 1,
  };
  swv_printf("erase page %08x\n", erase.Page);
  HAL_FLASH_Unlock();
  uint32_t page_err;
  HAL_FLASHEx_Erase(&erase, &page_err);
  swv_printf("empty cell [0]: %08x\n", *(uint32_t *)SCRATCH_START_ADDR);

  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SCRATCH_START_ADDR, 0x1020304a5b6c7daaULL);
  // 5b6c7daa (little-endian)
  swv_printf("empty cell [0]: %08x\n", *(uint32_t *)SCRATCH_START_ADDR);

  uint32_t data[64] = {0x1020304a};
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, SCRATCH_START_ADDR + 8, (uint32_t)&data[0]);
  // 1020304a
  swv_printf("empty cell [2]: %08x\n", *(uint32_t *)(SCRATCH_START_ADDR + 8));
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

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
    int result = HAL_SPI_Transmit(&spi2, spi_tx_buf, 25 * N_SUSPEND / 2, 1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
    swv_printf("SPI tx result = %d\n", result);
  }

  // GPIO initialisation is done by `HAL_RCC_MCOConfig`
/*
  gpio_init.Pin = GPIO_PIN_8;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF0_MCO;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);
*/
  // 64 MHz / 4 = 16 MHz
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_4);

  // ======== GPIO (Camera) ========
  // PA3 CAM_PWDN (active high)
  // PA4 CAM_RESET (active low)
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_3 | GPIO_PIN_4,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
  // Reset
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

  // ======== EXTI (PA9 CAM_VSYNC) ========
  gpio_init.Pin = GPIO_PIN_9;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  EXTI_HandleTypeDef exti_l9 = {
    // .Line = 9,
    .RisingCallback = NULL,
    .FallingCallback = NULL,
  };
  EXTI_ConfigTypeDef exti_cfg_l9 = {
    .Line = EXTI_LINE_9,
    .Mode = EXTI_MODE_INTERRUPT,
    .Trigger = EXTI_TRIGGER_FALLING,
    .GPIOSel = EXTI_GPIOA,
  };
  HAL_EXTI_SetConfigLine(&exti_l9, &exti_cfg_l9);

  // Interrupt
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);

  // ======== GPIO (Camera) ========
  // PA2 CAM_HREF
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_2,
    .Mode = GPIO_MODE_INPUT,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // PB0-2, PB10-114 CAM_D
  // PB15 CAM_PCLK
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 |
           GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
           GPIO_PIN_15,
    .Mode = GPIO_MODE_INPUT,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(GPIOB, &gpio_init);

  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);  // XXX: Does this actually work??

  // ======== I2C2 (PA11 SCL, PA12 SDA) ========
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_11 | GPIO_PIN_12,
    .Mode = GPIO_MODE_AF_PP,
    .Alternate = GPIO_AF6_I2C2,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);

  __HAL_RCC_I2C2_CLK_ENABLE();
  i2c2 = (I2C_HandleTypeDef){
    .Instance = I2C2,
    .Init = {
      // RM0454 Rev 5, pp. 711, 726, 738 (examples), 766
      // APB = 64 MHz
      // PRESC = 7 (1 / (64 MHz / (7+1)) = 0.125 us)
      // SCLDEL = 1, SDADEL = 1
      // SCLH = 9, SCLL = 9 -> f_SCL = 1 / (20 * t_PRESC) = 400 kHz
      .Timing = 0x70110909,
      .OwnAddress1 = 0x00,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c2);

  // MLX90640
  MLX90640_SetResolution(0x33, 0x03);  // 19-bit
  MLX90640_SetRefreshRate(0x33, 0x04);  // 8 Hz

  static uint16_t mlx90640_ee[832];
  MLX90640_DumpEE(0x33, mlx90640_ee);
  paramsMLX90640 mlx90640_params;
  MLX90640_ExtractParameters(mlx90640_ee, &mlx90640_params);

  MLX90640_SynchFrame(0x33);
  // MLX90640_TriggerMeasurement(0x33);
  static uint16_t mlx90640_frame[834];

  MLX90640_GetFrameData(0x33, mlx90640_frame);
  float mlx90640_vdd = MLX90640_GetVdd(mlx90640_frame, &mlx90640_params);
  swv_printf("Vdd = %d mV\n", (int)(mlx90640_vdd * 1000));
  float mlx90640_ta = MLX90640_GetTa(mlx90640_frame, &mlx90640_params);
  swv_printf("Ta = %d /100 C\n", (int)(mlx90640_ta * 100));

  int subpage = MLX90640_GetSubPageNumber(mlx90640_frame);

  static float mlx90640_image[768];
  // MLX90640_GetImage(mlx90640_frame, &mlx90640_params, mlx90640_image);
  MLX90640_CalculateTo(mlx90640_frame, &mlx90640_params,
    0.95, mlx90640_ta - 8, mlx90640_image);
  swv_printf("subpage = %d, temp = %d, %d, %d, %d\n",
    subpage,
    (int)mlx90640_image[0], (int)mlx90640_image[1],
    (int)mlx90640_image[2], (int)mlx90640_image[3]
  );

/*
  uint8_t mfid[2] = {0, 0};
  int result = HAL_I2C_Mem_Read(&i2c2, 0x15 << 1, 0x1c, I2C_MEMADD_SIZE_8BIT, mfid, 1, 1000);
  swv_printf("result %d, err %d, manufacturer id %02x %02x\n",
    result, i2c2.ErrorCode, (int)mfid[0], (int)mfid[1]);
*/

/*
  uint8_t com3  = 0b00001000; // Scale enable
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x0c, I2C_MEMADD_SIZE_8BIT, &com3, 1, 1000);
  uint8_t com7  = 0b00010100; // QVGA, RGB
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x12, I2C_MEMADD_SIZE_8BIT, &com7, 1, 1000);
  uint8_t com15 = 0b11010000; // RGB 565
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x40, I2C_MEMADD_SIZE_8BIT, &com15, 1, 1000);
*/
/*
  uint8_t com14 = 0b00010100; // PCLK divier
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x3e, I2C_MEMADD_SIZE_8BIT, &com14, 1, 1000);
  uint8_t scaling_pclk_div = 0b00000100;
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x73, I2C_MEMADD_SIZE_8BIT, &scaling_pclk_div, 1, 1000);
*/
  uint8_t clkrc = 0b10000111; // Prescale by 8
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x11, I2C_MEMADD_SIZE_8BIT, &clkrc, 1, 1000);
  uint8_t test_pattern_x = 0b10111010;
  uint8_t test_pattern_y = 0b00110101;  // 8-bar color bar
/*
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x70, I2C_MEMADD_SIZE_8BIT, &test_pattern_x, 1, 1000);
  HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, 0x71, I2C_MEMADD_SIZE_8BIT, &test_pattern_y, 1, 1000);
*/
  swv_printf("err %d\n", i2c2.ErrorCode);

  if (i2c2.ErrorCode != 0) while (1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); HAL_Delay(200);
  }

  uint32_t t0 = HAL_GetTick();
  uint32_t count = 0;
  while (0) {
#if 0
#define CLK_PORT GPIOA
#define CLK_PIN  GPIO_PIN_2
#define CLK_NAME "HREF"
#else
#define CLK_PORT GPIOB
#define CLK_PIN  GPIO_PIN_15
#define CLK_NAME "PCLK"
#endif
    while (HAL_GetTick() - t0 < 1000) {
      while ((CLK_PORT->IDR & CLK_PIN) == 0) { }
      while ((CLK_PORT->IDR & CLK_PIN) != 0) { }
      count++;
    }
    swv_printf(CLK_NAME " rate %u\n", count);
    // Before division (by CLKRC):
    // HREF rate 9604 (16 MHz / 784 * (480 / 510) / 2 due to RGB565)
    // PCLK rate 8000000 (16 MHz / 2 due to RGB565)
    t0 += 1000;
    count = 0;
  }

  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  uint32_t last_tick = HAL_GetTick();

  while (1) {
  /*
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); while (HAL_GetTick() - last_tick < 50) { }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); while (HAL_GetTick() - last_tick < 100) { }
    last_tick += 100;
  */
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, !(frame_count > 15));

    if (frame_count > 0) {
      swv_printf("frame count = %u, sum = %u %u, errors = %u\n", frame_count, frame_sum, frame_sum2, frame_errors);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, !(frame_errors > 0));
      frame_count = 0;
      frame_errors = 0;
    }
  }
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
  HAL_I2C_Mem_Read(&i2c2, slaveAddr << 1,
    startAddress, I2C_MEMADD_SIZE_16BIT,
    (uint8_t *)data, nMemAddressRead * 2, 1000);
  for (int i = 0; i < nMemAddressRead; i++)
    data[i] = __builtin_bswap16(data[i]);
  swv_printf("MLX90640 I2C read register %04x @ %u, I2C2 error code %u\n",
    (unsigned)startAddress, (unsigned)nMemAddressRead, (unsigned)i2c2.ErrorCode);
  if (i2c2.ErrorCode != 0) while (1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); HAL_Delay(200);
  }
  return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
  uint8_t bytes[2] = {data >> 8, data & 0xff};
  HAL_I2C_Mem_Write(&i2c2, slaveAddr << 1,
    writeAddress, I2C_MEMADD_SIZE_16BIT,
    (uint8_t *)bytes, 2, 1000);
  swv_printf("MLX90640 I2C write register %04x (value %04x), I2C2 error code %u\n",
    (unsigned)writeAddress, (unsigned)data, (unsigned)i2c2.ErrorCode);
  if (i2c2.ErrorCode != 0) while (1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); HAL_Delay(200);
  }
  return MLX90640_NO_ERROR;
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
#pragma GCC optimize("O3")
void EXTI4_15_IRQHandler() {
  uint32_t sum1 = 0, sum2 = 0;

  for (int row = 0; row < 480; row++) {
    // Wait HREF rise
    while ((GPIOA->IDR & GPIO_PIN_2) == 0) { }

    // Default format is YCbCr
    // Ref: http://embeddedprogrammer.blogspot.com/2012/07/hacking-ov7670-camera-module-sccb-cheat.html
    for (int col = 0; col < 640; col++) {
      // Wait PCLK rise
      uint32_t byte1;
      while (((byte1 = GPIOB->IDR) & GPIO_PIN_15) == 0) { }
      // byte1 = ((byte1 >> 7) & 0xf8) | (byte1 & 0x07);
      // Wait PCLK fall
      while ((GPIOB->IDR & GPIO_PIN_15) != 0) { }
      // Wait PCLK rise
      uint32_t byte2;
      while (((byte2 = GPIOB->IDR) & GPIO_PIN_15) == 0) { }
      byte2 = ((byte2 >> 7) & 0xf8) | (byte2 & 0x07);
      // Wait PCLK fall
      while ((GPIOB->IDR & GPIO_PIN_15) != 0) { }

      if (col < 320) sum1 += byte2;
      else sum2 += byte2;
    }

    // Wait HREF fall
    while ((GPIOA->IDR & GPIO_PIN_2) != 0) { }
  }

  frame_count++;
  frame_sum = sum1;
  frame_sum2 = sum2;
  // if (sum1 != 10598400) frame_errors++;

  static int parity = 0;
  GPIOA->BSRR = 1 << (((parity ^= 1) & 1) ? 1 : 17);

  // Clear at the end, in case of startup midway of a frame
  __HAL_GPIO_EXTI_CLEAR_FALLING_IT(GPIO_PIN_9);
}
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
