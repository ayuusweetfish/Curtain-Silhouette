#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if SILHOUETTE_SINGLETON
  #define _d(_field) (_field)
  #define _silhouette_field static
#else
  #define _d(_field) (d->_field)
  #define _silhouette_field
#endif

#ifndef SILHOUETTE_BASE_FRAME_ATTR
#define SILHOUETTE_BASE_FRAME_ATTR
#endif
#ifndef SILHOUETTE_CUR_FRAME_ATTR
#define SILHOUETTE_CUR_FRAME_ATTR
#endif
#ifndef SILHOUETTE_RUNNING_X_ATTR
#define SILHOUETTE_RUNNING_X_ATTR
#endif
#ifndef SILHOUETTE_RUNNING_X2_ATTR
#define SILHOUETTE_RUNNING_X2_ATTR
#endif
#ifndef SILHOUETTE_PIXEL_4L_ATTR
#define SILHOUETTE_PIXEL_4L_ATTR
#endif

struct silhouette_detection {

#if SILHOUETTE_SINGLETON
};
#endif

  _silhouette_field uint8_t base_frame[120 * 160] SILHOUETTE_BASE_FRAME_ATTR;
  _silhouette_field uint8_t cur_frame[120 * 160] SILHOUETTE_CUR_FRAME_ATTR;
  _silhouette_field uint16_t running_x[120 * 160] SILHOUETTE_RUNNING_X_ATTR;
  _silhouette_field uint16_t running_x2[120 * 160] SILHOUETTE_RUNNING_X2_ATTR;

  _silhouette_field uint16_t pixel_4l[160] SILHOUETTE_PIXEL_4L_ATTR;
  _silhouette_field uint16_t line_count;
  _silhouette_field uint8_t running_count;
  _silhouette_field bool base_initialized;

#if !SILHOUETTE_SINGLETON
};
#endif

static inline void silhouette_init(struct silhouette_detection *d)
{
  memset(_d(pixel_4l), 0, sizeof _d(pixel_4l));
  _d(line_count) = 0;
  _d(running_count) = 0;
  _d(base_initialized) = false;

  memset(_d(running_x), 0, sizeof _d(running_x));
  memset(_d(running_x2), 0, sizeof _d(running_x2));
}

static inline void silhouette_feed_line(struct silhouette_detection *d, uint32_t *buf)
{
  for (int i = 0; i < 320; i++) {
    uint8_t v1 = (buf[i] >> 24) & 0xff;
    uint8_t v2 = (buf[i] >>  8) & 0xff;
    if (_d(line_count) % 4 == 0) _d(pixel_4l)[i / 2] = 0;
    _d(pixel_4l)[i / 2] += ((uint16_t)v1 + v2);
  }

  if (_d(line_count) % 4 == 3) {
    for (int i = 0; i < 160; i++) {
      uint16_t pixel = _d(pixel_4l)[i] >> 4;
      _d(cur_frame)[(_d(line_count) / 4) * 160 + i] = pixel;
      _d(running_x)[(_d(line_count) / 4) * 160 + i] += pixel / 4;
      _d(running_x2)[(_d(line_count) / 4) * 160 + i] += (uint16_t)(pixel / 4) * (pixel / 4);
      if (0 && _d(line_count) == 239 && i == 0) {
        printf(">> %3d %10u %10u\n", (int)pixel, _d(running_x)[(_d(line_count) / 4) * 160 + i], _d(running_x2)[(_d(line_count) / 4) * 160 + i]);
      }
    }
  }

  _d(line_count)++;
}

static inline uint8_t absdiff8(uint8_t a, uint8_t b)
{
  return (a > b ? a - b : b - a);
}

static inline void silhouette_end_frame(struct silhouette_detection *d)
{
  _d(running_count)++;

  if (_d(running_count) == 16) {
    _d(running_count) = 0;

    uint32_t sum = 0;
    // Sum(x[i]^2 - x2[i] * 16)
    for (int i = 0; i < 120 * 160; i++)
      sum += (uint32_t)_d(running_x2)[i] * 16 - (uint32_t)_d(running_x)[i] * _d(running_x)[i];
    for (int r = 0; r < 120; r += 3) {
      for (int c = 0; c < 160; c += 2) {
        uint32_t i = r * 160 + c;
        uint32_t v = (uint32_t)_d(running_x2)[i] * 16 - (uint32_t)_d(running_x)[i] * _d(running_x)[i];
        putchar(v >= 4000 ? '*' : ' ');
      }
      putchar('\n');
    }
    printf("sum = %u, avg var = %u\n", (unsigned)sum, (unsigned)(sum / (16 * 160 * 120)));

    if (sum / (16 * 160 * 120) < 30) {
      if (!_d(base_initialized)) {
        for (int i = 0; i < 120 * 160; i++)
          _d(base_frame)[i] = _d(running_x)[i] / 4;
          // divide by 16 (frame count), multiply by 4 (prescale)
        _d(base_initialized) = true;
      } else {
        for (int i = 0; i < 120 * 160; i++)
          _d(base_frame)[i] = ((uint16_t)_d(base_frame)[i] * 3 + _d(running_x)[i] / 4) / 4;
      }
    }

    memset(_d(running_x), 0, sizeof _d(running_x));
    memset(_d(running_x2), 0, sizeof _d(running_x2));
  }

  // Subtract current frame with base frame
  for (int i = 0; i < 160 * 120; i++) {
    _d(cur_frame)[i] = absdiff8(_d(cur_frame)[i], _d(base_frame)[i]);
  }

  _d(line_count) = 0;
}

static inline const uint8_t *silhouette_base_frame(struct silhouette_detection *d)
{
  return _d(base_frame);
}

static inline const uint8_t *silhouette_residual_frame(struct silhouette_detection *d)
{
  return _d(cur_frame);
}

#undef _d
