#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mlx90640/MLX90640_API.h"

#define PIECE_INDEX 1

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

extern uint32_t _etext;
#define CODE_START_ADDR     0x08000000
#define CODE_END_ADDR       (uint32_t)(&_etext)
#define SCRATCH_START_ADDR  ((CODE_END_ADDR + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1))
#define SCRATCH_END_ADDR    0x08020000

SPI_HandleTypeDef spi2;
I2C_HandleTypeDef i2c2;
DMA_HandleTypeDef dma2_st1_ch1;
DCMI_HandleTypeDef dcmi;

#define N_SUSPEND 200
uint8_t spi_tx_buf[32 * N_SUSPEND / 2];

// 320 words (640 pixels @ 16bpp) per line
// 4 lines for multiple buffering
static uint32_t dcmi_buf[1280] = { 0xaa };
#define dcmi_buf_size (sizeof dcmi_buf / sizeof dcmi_buf[0])

static uint32_t line_count = 0;
static uint32_t frame_count __attribute__ ((section(".ccmram")));

// DMA counter starts at the beginning of the buffer
static int last_pixel_count = dcmi_buf_size;

#define SILHOUETTE_SINGLETON 1
#define SILHOUETTE_SINGLETON_NO_THIS 1
#define SILHOUETTE_BASE_FRAME_ATTR __attribute__ ((section(".ccmram")))
#define SILHOUETTE_CUR_FRAME_ATTR __attribute__ ((section(".ccmram")))
#include "../../misc/camera_vis/silhouette_detection.h"

static volatile bool frame_done = false;

void dcmi_error();
inline void consume_init();
inline void consume_frame();

int main()
{
  // CCMRAM variables need to be manually initialized
  frame_count = 0;

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

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); HAL_Delay(80);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); HAL_Delay(80);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); HAL_Delay(80);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); HAL_Delay(80);

  // ======== Camera controls ========
  // PA2 CAM_PWDN (active high), PA3 CAM_RESET (active low)
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_2 | GPIO_PIN_3,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);  // Reset

  // ======== MCO (PB8 MCO1) ========
  // GPIO initialisation is done by `HAL_RCC_MCOConfig`
  // 168 MHz / 4 = 42 MHz
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_4);

  HAL_Delay(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);  // Release reset
  HAL_Delay(2); // OV7670 datasheet t_S:RESET

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

  void camera_write_reg(uint8_t reg, uint8_t val) {
    HAL_I2C_Mem_Write(&i2c2, 0x21 << 1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);
  }

  // CLKRC: prescale by 1
  camera_write_reg(0x11, 0b00000000);

/*
  // QVGA YUV
  // XXX: Yields 157~158 words per line instead of 160??
  camera_write_reg(0x12, 0x00); // COM7
  camera_write_reg(0x0c, 0x04); // COM3
  camera_write_reg(0x3e, 0x19); // COM14
  camera_write_reg(0x70, 0x3a); // SCALING_XSC
  camera_write_reg(0x71, 0x35); // SCALING_YSC
  camera_write_reg(0x72, 0x11); // SCALING_DCWCTR
  camera_write_reg(0x73, 0xf1); // SCALING_PCLK_DIV
  camera_write_reg(0xa2, 0x02); // SCALING_PCLK_DELAY
*/

  swv_printf("err %d\n", i2c2.ErrorCode);

  if (i2c2.ErrorCode != 0) while (1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1); HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); HAL_Delay(200);
  }

  HAL_Delay(305); // OV7670 datasheet t_S:REG

// Test frequencies
if (0) {
  // PA4 HSYNC, PA6 PIXCLK, PA9 D0, PA10 D1
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_10,
    .Mode = GPIO_MODE_INPUT,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
  // PB6 D5, PB7 VSYNC, PB8 D6, PB9 D7
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
    .Mode = GPIO_MODE_INPUT,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(GPIOB, &gpio_init);
  // PE0 D2, PE1 D3, PE4 D4
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4,
    .Mode = GPIO_MODE_INPUT,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(GPIOE, &gpio_init);

  if (0) {
    // Count lines per frame

    int counts[100] = { 0 };
    // Wait for VSYNC rise
    while ((GPIOB->IDR & (1 << 7)) != 0) { }
    while ((GPIOB->IDR & (1 << 7)) == 0) { }

    for (int i = 0; i < 100; i++) {
      // Wait for VSYNC fall
      while ((GPIOB->IDR & (1 << 7)) != 0) { }

      int count = 0;
      while (1) {
        // Wait for HREF active
        while ((GPIOA->IDR & (1 << 4)) == 0)
          // If VSYNC rises, stop
          if ((GPIOB->IDR & (1 << 7)) != 0) goto done1;

        count++;
        // Wait for HREF inactive
        while ((GPIOA->IDR & (1 << 4)) != 0) { }
      }
    done1:
      counts[i] = count;
    }
    swv_printf("counts %d\n", counts[0]);

  } else {
    // Count bytes per line

    int counts[100] = { 0 };
    // Wait for HREF inactive
    while ((GPIOA->IDR & (1 << 4)) == 0) { }
    while ((GPIOA->IDR & (1 << 4)) != 0) { }

    for (int i = 0; i < 100; i++) {
      // Wait for HREF active
      while ((GPIOA->IDR & (1 << 4)) == 0) { }

      int count = 0;
      while (1) {
        // Wait for PCLK rise
        while ((GPIOA->IDR & (1 << 6)) == 0)
          // If HREF ends, stop
          if ((GPIOA->IDR & (1 << 4)) == 0) goto done2;

        count++;
        // Wait for PCLK fall
        while ((GPIOA->IDR & (1 << 6)) != 0) { }
      }
    done2:
      counts[i] = count;
    }
    swv_printf("counts %d\n", counts[0]);
  }
}

  // ======== DMA (DMA2 Stream 1 Channel 1) ========
  // For DCMI
  // This is not arbitrary as 'F4 does not have DMAMUX
  // Ref. AN4031 (Rev 3) p. 9 Tab. 2
  __HAL_RCC_DMA2_CLK_ENABLE();
  dma2_st1_ch1 = (DMA_HandleTypeDef){
    .Instance = DMA2_Stream1,
    .Init = {
      .Channel = DMA_CHANNEL_1,
      .Direction = DMA_PERIPH_TO_MEMORY,
      .PeriphInc = DMA_PINC_DISABLE,
      .MemInc = DMA_MINC_ENABLE,
      .PeriphDataAlignment = DMA_PDATAALIGN_WORD,
      .MemDataAlignment = DMA_MDATAALIGN_WORD,
      .Mode = DMA_CIRCULAR,
      .Priority = DMA_PRIORITY_HIGH,
      .FIFOMode = DMA_FIFOMODE_ENABLE,
      .FIFOThreshold = DMA_FIFO_THRESHOLD_FULL,
      .MemBurst = DMA_PBURST_SINGLE,
      .PeriphBurst = DMA_MBURST_INC4,
    },
  };
  HAL_DMA_Init(&dma2_st1_ch1);

  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  // ======== DCMI GPIOs ========
  // PA4 HSYNC, PA6 PIXCLK, PA9 D0, PA10 D1
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_10,
    .Mode = GPIO_MODE_AF_PP,
    .Alternate = GPIO_AF13_DCMI,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
  // PB6 D5, PB7 VSYNC, PB8 D6, PB9 D7
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
    .Mode = GPIO_MODE_AF_PP,
    .Alternate = GPIO_AF13_DCMI,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(GPIOB, &gpio_init);
  // PE0 D2, PE1 D3, PE4 D4
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4,
    .Mode = GPIO_MODE_AF_PP,
    .Alternate = GPIO_AF13_DCMI,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(GPIOE, &gpio_init);

  // ======== DCMI ========
  __HAL_RCC_DCMI_CLK_ENABLE();
  dcmi = (DCMI_HandleTypeDef){
    .Instance = DCMI,
    .Init = {
      .SynchroMode = DCMI_SYNCHRO_HARDWARE,
      .PCKPolarity = DCMI_PCKPOLARITY_RISING,
        // RM0090 (Rev 20) p. 471: "[PCKPOL] configures the capture edge of the pixel clock"
      .VSPolarity = DCMI_VSPOLARITY_HIGH,
      .HSPolarity = DCMI_HSPOLARITY_LOW,
        // AN5020 (Rev 3) p. 27: "the data is not valid […] when VSYNC or HSYNC is at [active] level"
      .CaptureRate = DCMI_CR_ALL_FRAME,
      .ExtendedDataMode = DCMI_EXTEND_DATA_8B,
      .JPEGMode = DCMI_JPEG_DISABLE,
    },
  };
  HAL_DCMI_Init(&dcmi);
  __HAL_LINKDMA(&dcmi, DMA_Handle, dma2_st1_ch1);

  // HAL_DCMI_ConfigCrop(&dcmi, 0, 0, 640 * 2, 480);
  // HAL_DCMI_EnableCrop(&dcmi);

  silhouette_init();

  HAL_NVIC_SetPriority(DCMI_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);

  HAL_DCMI_Start_DMA(&dcmi, DCMI_MODE_CONTINUOUS, (uint32_t)&dcmi_buf[0], dcmi_buf_size);

  consume_init();

  int count = 0;

  while (1) {
    while (!frame_done) { }

    consume_frame();

    // if (++count == 20) dcmi_error();

    frame_done = false;
  }

  // Test

  while (1) {
    frame_done = false;

    static int parity = 1;
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, parity ^= 1);

    // Running lights

    static int count = 0;
    static int phase = 0;

    for (int i = 0; i < sizeof spi_tx_buf / sizeof spi_tx_buf[0]; i++)
      spi_tx_buf[i] = 0;

    if (++count % 2 == 0) phase = (phase + 1) % 8;
    for (int strip = 0; strip < 32; strip++) {
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
    int result = HAL_SPI_Transmit(&spi2, spi_tx_buf, 32 * N_SUSPEND / 2, 1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
    swv_printf("SPI tx result = %d\n", result);
  }

}

void consume_init()
{
}

static inline int16_t abs16(int16_t x)
{
  return (x < 0 ? -x : x);
}

// Consume frame
// If this times out, an error will be raised by blinking Act 1
#pragma GCC optimize("O3")
void consume_frame()
{
  if (silhouette_end_frame()) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
  } else {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
  }

  static const uint8_t used_columns[25] = {
#if PIECE_INDEX == 1
     0,  2,  4,  5,
     8,  9, 10, 12, 13, 14, 15,
    16, 18, 19, 20, 21, 22, 23,
    24, 26, 27, 28, 29, 30, 31,
#elif PIECE_INDEX == 2
     0,  2,  3,  5,  6,  7,
     8,  9, 11, 12, 13, 15, 16,
    18, 19, 21, 23, 24,
    25, 26, 27, 28, 29, 30, 31,
#elif PIECE_INDEX == 3
     0,  1,  3,  4,  5,  6,
     8,  9, 10, 11, 13, 14, 15,
    16, 17, 18, 20, 21, 22,
    24, 25, 27, 29, 30, 31,
#elif PIECE_INDEX == 4
     0,  2,  3,  5,  6,
     8,  9, 11, 12, 13, 15,
    16, 17, 18, 19, 20, 22, 23,
    24, 25, 27, 28, 29, 30, 31,
#else
    #error Please define PIECE_INDEX
#endif
  };

  static const uint8_t underlying_pattern[200][25] = {
{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
  };

  memset(spi_tx_buf, 0xff, sizeof spi_tx_buf);
#define write_buf(_strip, _suspend, _value) \
  (spi_tx_buf[((_strip) * N_SUSPEND + (_suspend)) / 2] &= \
    ((_suspend) % 2 == 0 ? ((_value) | 0xf0) : (0x0f | ((_value) << 4))))

  const uint8_t *resid = silhouette_residual_frame();

  for (int i = 0; i < 25; i++) {
    int c = used_columns[i];
    for (int r = 0; r < 200; r++) {
      if (underlying_pattern[r][i]) {
        // width 150 * height 100
        uint8_t pixel = (
          (uint16_t)resid[(r / 2) * 160 + i * 6 + 0] +
          (uint16_t)resid[(r / 2) * 160 + i * 6 + 1] +
          (uint16_t)resid[(r / 2) * 160 + i * 6 + 2] +
          (uint16_t)resid[(r / 2) * 160 + i * 6 + 3] +
          (uint16_t)resid[(r / 2) * 160 + i * 6 + 4] +
          (uint16_t)resid[(r / 2) * 160 + i * 6 + 5]
        ) / 6;
        write_buf(c, r, pixel < 96 ? 0 : 3);
      }
    }
  }

#undef write_buf

  // Transmit SPI, this takes (32 * 200 / 2) * 8 bits / 21 MHz = 1 ms
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
  int result = HAL_SPI_Transmit(&spi2, spi_tx_buf, 32 * N_SUSPEND / 2, 1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void dcmi_error() {
  HAL_DCMI_DeInit(&dcmi);
  HAL_DMA_DeInit(&dma2_st1_ch1);

  for (int i = 0; i < 20; i++) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, i % 2); HAL_Delay(50);
  }
/*
  HAL_Delay(1000);

  line_count = 0;
  frame_count = 0;
  last_pixel_count = dcmi_buf_size;

  HAL_DMA_Init(&dma2_st1_ch1);
  HAL_DCMI_Init(&dcmi);
  HAL_DCMI_Start_DMA(&dcmi, DCMI_MODE_CONTINUOUS, (uint32_t)&dcmi_buf[0], dcmi_buf_size);
*/

  NVIC_SystemReset();
}
void DMA2_Stream1_IRQHandler() {
  HAL_DMA_IRQHandler(&dma2_st1_ch1);
}
#define CAPTURE_RATE 2  // Once every two frames
#define ACCUM_RATE   2  // Once every four frames

static volatile uint32_t running_sum[16] = { 0 };

#pragma GCC optimize("O3")
void DCMI_IRQHandler() {
  uint32_t misr = DCMI->MISR;
  uint32_t ndtr = DMA2_Stream1->NDTR;

  HAL_DCMI_IRQHandler(&dcmi);

  if (frame_count > 0 && (misr & DCMI_MIS_LINE_MIS)) {
    if (ndtr == dcmi_buf_size) ndtr = 0;

    if (last_pixel_count - ndtr != 320) {
      swv_printf("line has %u words\n", last_pixel_count - ndtr);
      dcmi_error();
      return;
    }

    int32_t start = dcmi_buf_size - ndtr - 320;
    if (start < 0) {
      dcmi_error();
      return;
    }

    if (frame_count % CAPTURE_RATE == 1 % CAPTURE_RATE) {
      if (frame_done) {
        swv_printf("processing timed out\n");
        dcmi_error();
        return;
      }

      silhouette_feed_line(dcmi_buf + start, (frame_count / CAPTURE_RATE) % ACCUM_RATE == 1 % ACCUM_RATE);
    }

    line_count++;
    last_pixel_count = (ndtr == 0 ? dcmi_buf_size : ndtr);

    if (line_count > 480) {
      swv_printf("frame has %u lines\n", line_count);
      dcmi_error();
      return;
    }
  }

  if (misr & DCMI_MIS_VSYNC_MIS) {
    if (frame_count >= 1 && line_count != 480) {
      dcmi_error();
      return;
    }

    if (frame_count >= 1 && frame_count % CAPTURE_RATE == 1 % CAPTURE_RATE) {
      // Process frame >o<
      frame_done = true;
    }

    frame_count++;
    line_count = 0;

    // Prevent overflow
    if (frame_count >= 1 + CAPTURE_RATE * ACCUM_RATE)
      frame_count -= CAPTURE_RATE * ACCUM_RATE;
  }
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
