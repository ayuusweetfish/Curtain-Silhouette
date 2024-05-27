#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct silhouette_detection {
  uint8_t base_frame[120 * 160];
  uint8_t cur_frame[120 * 160];
  uint16_t running_x[120 * 160];
  uint16_t running_x2[120 * 160];

  uint16_t pixel_4l[160];
  uint16_t line_count;
  uint8_t running_count;
  bool base_initialized;
};

static inline void silhouette_init(struct silhouette_detection *d)
{
  memset(d->pixel_4l, 0, sizeof d->pixel_4l);
  d->line_count = 0;
  d->running_count = 0;
  d->base_initialized = false;

  memset(d->running_x, 0, sizeof d->running_x);
  memset(d->running_x2, 0, sizeof d->running_x2);
}

static inline void silhouette_feed_line(struct silhouette_detection *d, uint32_t *buf)
{
  for (int i = 0; i < 320; i++) {
    uint8_t v1 = (buf[i] >> 24) & 0xff;
    uint8_t v2 = (buf[i] >>  8) & 0xff;
    if (d->line_count % 4 == 0) d->pixel_4l[i / 2] = 0;
    d->pixel_4l[i / 2] += ((uint16_t)v1 + v2);
  }

  if (d->line_count % 4 == 3) {
    for (int i = 0; i < 160; i++) {
      uint16_t pixel = d->pixel_4l[i] >> 4;
      d->cur_frame[(d->line_count / 4) * 160 + i] = pixel;
      d->running_x[(d->line_count / 4) * 160 + i] += pixel / 4;
      d->running_x2[(d->line_count / 4) * 160 + i] += (uint16_t)(pixel / 4) * (pixel / 4);
      if (0 && d->line_count == 239 && i == 0) {
        printf(">> %3d %10u %10u\n", (int)pixel, d->running_x[(d->line_count / 4) * 160 + i], d->running_x2[(d->line_count / 4) * 160 + i]);
      }
    }
  }

  d->line_count++;
}

static inline uint8_t absdiff8(uint8_t a, uint8_t b)
{
  return (a > b ? a - b : b - a);
}

static inline void silhouette_end_frame(struct silhouette_detection *d)
{
  d->running_count++;

  if (d->running_count == 16) {
    d->running_count = 0;

    uint32_t sum = 0;
    // Sum(x[i]^2 - x2[i] * 16)
    for (int i = 0; i < 120 * 160; i++)
      sum += (uint32_t)d->running_x2[i] * 16 - (uint32_t)d->running_x[i] * d->running_x[i];
    for (int r = 0; r < 120; r += 3) {
      for (int c = 0; c < 160; c += 2) {
        uint32_t i = r * 160 + c;
        uint32_t v = (uint32_t)d->running_x2[i] * 16 - (uint32_t)d->running_x[i] * d->running_x[i];
        putchar(v >= 4000 ? '*' : ' ');
      }
      putchar('\n');
    }
    printf("sum = %u, avg var = %u\n", (unsigned)sum, (unsigned)(sum / (16 * 160 * 120)));

    if (sum / (16 * 160 * 120) < 30) {
      if (!d->base_initialized) {
        for (int i = 0; i < 120 * 160; i++)
          d->base_frame[i] = d->running_x[i] / 4;
          // divide by 16 (frame count), multiply by 4 (prescale)
        d->base_initialized = true;
      } else {
        for (int i = 0; i < 120 * 160; i++)
          d->base_frame[i] = ((uint16_t)d->base_frame[i] * 3 + d->running_x[i] / 4) / 4;
      }
    }

    memset(d->running_x, 0, sizeof d->running_x);
    memset(d->running_x2, 0, sizeof d->running_x2);
  }

  // Subtract current frame with base frame
  for (int i = 0; i < 160 * 120; i++) {
    d->cur_frame[i] = absdiff8(d->cur_frame[i], d->base_frame[i]);
  }

  d->line_count = 0;
}

static inline const uint8_t *silhouette_base_frame(struct silhouette_detection *d)
{
  return d->base_frame;
}

static inline const uint8_t *silhouette_residual_frame(struct silhouette_detection *d)
{
  return d->cur_frame;
}
