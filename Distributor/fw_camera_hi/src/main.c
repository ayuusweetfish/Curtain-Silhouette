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
DMA_HandleTypeDef dma2_st1_ch1;
DCMI_HandleTypeDef dcmi;

#define N_SUSPEND 200
uint8_t spi_tx_buf[32 * N_SUSPEND / 2];

static int line_count = 0;
static int frame_count = 0;

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

  // CLKRC: prescale by 2
  camera_write_reg(0x11, 0b00000001);

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
if (1) {
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

  } else if (0) {
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

  HAL_NVIC_SetPriority(DCMI_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);

  static uint32_t buf[15000] = { 0xaa };
  for (int i = 0; i < 15000; i++) buf[i] = i;
  HAL_DCMI_Start_DMA(&dcmi, DCMI_MODE_CONTINUOUS, (uint32_t)&buf[0], 15000);
  // Snapshot takes 3600 words = 14400 bytes = 7200 pixels
  // HAL_DCMI_Start_DMA(&dcmi, DCMI_MODE_SNAPSHOT, (uint32_t)&buf[0], 15000);

  while (0) {
    uint32_t sum = 0;
    for (int i = 0; i < 15000; i++) sum += ((buf[i] >> 24) & 0xff) + ((buf[i] >> 8) & 0xff);
  if (0)
    swv_printf("CR %08x, NDT %08x, PA %08x, M0A %08x, M1A %08x, LISR %08x, ErrorCode %08x | buf[0] %02x sum %08x line_count %5d frame_count %5d | DCMI DR %08x CR %08x MIS %08x\n",
      DMA2_Stream1->CR,
      DMA2_Stream1->NDTR,
      DMA2_Stream1->PAR,
      DMA2_Stream1->M0AR,
      DMA2_Stream1->M1AR,
      DMA2->LISR,
      dma2_st1_ch1.ErrorCode,
      (unsigned)((buf[0] >> 24) & 0xff),
      sum,
      line_count,
      frame_count,
      DCMI->DR,
      DCMI->CR,
      DCMI->MISR
    );
    HAL_Delay(300);
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

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void DMA2_Stream1_IRQHandler() {
  HAL_DMA_IRQHandler(&dma2_st1_ch1);
}
// p line_pixels -- {0, 0, 0, 0, 0, 0, 0, 0, 7640, 320 <repeats 991 times>}
// p frame_lines -- {233, 480, 480, 480, 228, 389, 480, 480, 480, 74, 66, 480, 480, 480, 398, 257, 480, 480, 480, 206, 454, 480, 480, 480, 10, 150, 480, 480, 480, 313, 361, 480, 480, 480, 103, 47, 480, 480, 480, 416, 267, ...}
int line_pixels[1000];
int frame_lines[1000];
int mis[9000];
void DCMI_IRQHandler() {
  static int last_pixel_count = 0;
  if (frame_count > 0 && (DCMI->MISR & DCMI_MIS_LINE_MIS)) {
    line_count++;
    if (line_count != 1 && line_count - 2 < 1000)
      line_pixels[line_count - 2] = (15000 - ((int)DMA2_Stream1->NDTR - last_pixel_count)) % 15000;
    last_pixel_count = DMA2_Stream1->NDTR;
  }

  static int last_line_count = 0;
  if (DCMI->MISR & DCMI_MIS_VSYNC_MIS) {
    frame_count++;
    if (frame_count != 1 && frame_count - 2 < 1000)
      frame_lines[frame_count - 2] = line_count - last_line_count;
    last_line_count = line_count;
  }

  static int ptr = 0;
  if (ptr < 9000 - 1 && (DCMI->MISR & ~DCMI_MIS_FRAME_MIS) != 0)
    mis[ptr++] = DCMI->MISR & ~DCMI_MIS_FRAME_MIS;
  HAL_DCMI_IRQHandler(&dcmi);
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
