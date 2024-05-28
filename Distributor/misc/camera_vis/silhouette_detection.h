#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define _silhouette_this_arg struct silhouette_detection *d
#define _silhouette_this_arg_ struct silhouette_detection *d,
#if SILHOUETTE_SINGLETON
  #define _d(_field) (_silhouette_f_##_field)
  #define _silhouette_field static
  #define _silhouette_field_name(_field) (_silhouette_f_##_field)
  #if SILHOUETTE_SINGLETON_NO_THIS
    #undef _silhouette_this_arg
    #undef _silhouette_this_arg_
    #define _silhouette_this_arg
    #define _silhouette_this_arg_
  #endif
#else
  #define _d(_field) (d->_field)
  #define _silhouette_field
  #define _silhouette_field_name(_field) (_field)
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
#ifndef SILHOUETTE_STATIONARY_STREAK_ATTR
#define SILHOUETTE_STATIONARY_STREAK_ATTR
#endif

struct silhouette_detection {

#if SILHOUETTE_SINGLETON
};
#endif

  _silhouette_field uint8_t _silhouette_field_name(base_frame)[120 * 160] SILHOUETTE_BASE_FRAME_ATTR;
  _silhouette_field uint8_t _silhouette_field_name(cur_frame)[120 * 160] SILHOUETTE_CUR_FRAME_ATTR;
  _silhouette_field uint16_t _silhouette_field_name(running_x)[120 * 160] SILHOUETTE_RUNNING_X_ATTR;
  _silhouette_field uint16_t _silhouette_field_name(running_x2)[120 * 160] SILHOUETTE_RUNNING_X2_ATTR;
  _silhouette_field uint8_t _silhouette_field_name(stationary_streak)[120 * 160] SILHOUETTE_STATIONARY_STREAK_ATTR;

  _silhouette_field uint16_t _silhouette_field_name(pixel_4l)[160] SILHOUETTE_PIXEL_4L_ATTR;
  _silhouette_field uint16_t _silhouette_field_name(line_count);
  _silhouette_field uint8_t _silhouette_field_name(running_count);
  _silhouette_field bool _silhouette_field_name(base_initialized);

  _silhouette_field bool _silhouette_field_name(cur_frame_accum);

#if !SILHOUETTE_SINGLETON
};
#endif

static inline void silhouette_init(_silhouette_this_arg)
{
  memset(_d(pixel_4l), 0, sizeof _d(pixel_4l));
  memset(_d(stationary_streak), 0, sizeof _d(stationary_streak));
  _d(line_count) = 0;
  _d(running_count) = 0;
  _d(base_initialized) = false;

  memset(_d(running_x), 0, sizeof _d(running_x));
  memset(_d(running_x2), 0, sizeof _d(running_x2));
}

static inline void silhouette_feed_line(_silhouette_this_arg_ uint32_t *buf, bool accum_base)
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
      if (accum_base) {
        _d(running_x)[(_d(line_count) / 4) * 160 + i] += pixel / 4;
        _d(running_x2)[(_d(line_count) / 4) * 160 + i] += (uint16_t)(pixel / 4) * (pixel / 4);
      }
    }
  }

  _d(line_count)++;
  _d(cur_frame_accum) = accum_base;
}

static inline uint8_t absdiff8(uint8_t a, uint8_t b)
{
  return (a > b ? a - b : b - a);
}

#pragma GCC optimize("O3")
static inline bool silhouette_end_frame(_silhouette_this_arg)
{
  _d(line_count) = 0;
  if (_d(cur_frame_accum)) _d(running_count)++;

  bool updated = false;

  if (_d(running_count) == 16) {
    _d(running_count) = 0;

    uint32_t sum = 0;
    // Sum(Sum(x[i]^2) * 16 - Sum(x[i])^2)
    for (int i = 0; i < 120 * 160; i++) {
      uint32_t var = (uint32_t)_d(running_x2)[i] * 16 - (uint32_t)_d(running_x)[i] * _d(running_x)[i];
      sum += var;
      if (var < 16 * 20) {
        _d(stationary_streak)[i]++;
      } else {
        _d(stationary_streak)[i] = 0;
      }
    }
  /*
    for (int r = 0; r < 120; r += 3) {
      for (int c = 0; c < 160; c += 2) {
        uint32_t i = r * 160 + c;
        uint32_t v = (uint32_t)_d(running_x2)[i] * 16 - (uint32_t)_d(running_x)[i] * _d(running_x)[i];
        putchar(v >= 4000 ? '*' : ' ');
      }
      putchar('\n');
    }
    printf("sum = %u, avg var = %u\n", (unsigned)sum, (unsigned)(sum / (16 * 160 * 120)));
  */

    if (sum / (16 * 160 * 120) < 30) {
      if (!_d(base_initialized)) {
        for (int i = 0; i < 120 * 160; i++)
          _d(base_frame)[i] = _d(running_x)[i] / 4;
          // divide by 16 (frame count), multiply by 4 (prescale)
        _d(base_initialized) = true;
      } else {
        // Exponential decay / moving average (weight of most recent = 1/4)
        for (int i = 0; i < 120 * 160; i++)
          _d(base_frame)[i] = ((uint16_t)_d(base_frame)[i] * 3 + _d(running_x)[i] / 4) / 4;
      }
      memset(_d(stationary_streak), 0, sizeof _d(stationary_streak));
      updated = true;
    } else if (_d(base_initialized)) {
      // Update single pixels, but requires an initial set-up beforehand
      for (int i = 0; i < 120 * 160; i++) {
        if (_d(stationary_streak)[i] >= 3) {
          _d(stationary_streak)[i] = 0;
          _d(base_frame)[i] = ((uint16_t)_d(base_frame)[i] * 3 + _d(running_x)[i] / 4) / 4;
        }
      }
      // Does not flag `updated`, since this ought to be a mild, unobtrusive update
    }

    memset(_d(running_x), 0, sizeof _d(running_x));
    memset(_d(running_x2), 0, sizeof _d(running_x2));
  }

  return updated;
}

static inline const uint8_t *silhouette_base_frame(_silhouette_this_arg)
{
  return _d(base_frame);
}

static inline const uint8_t *silhouette_cur_frame(_silhouette_this_arg)
{
  return _d(cur_frame);
}

#pragma GCC optimize("O3")
static inline const uint8_t *silhouette_residual_frame(_silhouette_this_arg)
{
  // Subtract current frame with base frame
  for (int i = 0; i < 160 * 120; i++) {
    _d(cur_frame)[i] = absdiff8(_d(cur_frame)[i], _d(base_frame)[i]);
  }

  const int OPEN_CLOSE_RADIUS = 3;
  const int RMQ_BUF = 8;    // min. OPEN_CLOSE_RADIUS * 2 + 1
  static uint8_t rmq[RMQ_BUF][4][160];
  int rmq_ptr;

  const int tnz[128] = {
    0, 1,
    2, 2,
    3, 3, 3, 3,
    4, 4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
  };

#define max(_a, _b) ((_a) > (_b) ? (_a) : (_b))
#define min(_a, _b) ((_a) < (_b) ? (_a) : (_b))

#define build_rmq(_r, _op) do { \
    for (int c = 0; c < 160; c++) \
      rmq[rmq_ptr][0][c] = _d(cur_frame)[(_r) * 160 + c]; \
    for (int k = 1; k < 4; k++) \
      for (int c = 0; c < 160; c++) { \
        rmq[rmq_ptr][k][c] = rmq[rmq_ptr][k - 1][c]; \
        if (c + (1 << (k - 1)) < 160 && rmq[rmq_ptr][k - 1][c + (1 << (k - 1))] _op rmq[rmq_ptr][k][c]) \
          rmq[rmq_ptr][k][c] = rmq[rmq_ptr][k - 1][c + (1 << (k - 1))]; \
      } \
    rmq_ptr = (rmq_ptr + 1) % RMQ_BUF; \
  } while (0)

#define query_rmq(_best, _line, _c1, _c2, _op) do { \
    uint8_t len = (_c2) - (_c1) + 1; \
    /* int k = (sizeof(len)) * 8 - clz8[len] - 1; */ \
    int k = tnz[len] - 1; \
    if (rmq[_line][k][(_c1)] _op _best) _best = rmq[_line][k][(_c1)]; \
    if (rmq[_line][k][(_c2) - (1 << k) + 1] _op _best) _best = rmq[_line][k][(_c2) - (1 << k) + 1]; \
  } while (0)

  // Closing
  // (1) Dilation
  rmq_ptr = 0;
  for (int r = 0; r < OPEN_CLOSE_RADIUS; r++) build_rmq(r, >);
  for (int r = 0; r < 120; r++) {
    if (r + OPEN_CLOSE_RADIUS < 120)
      build_rmq(r + OPEN_CLOSE_RADIUS, >);
    int r_min = max(r - OPEN_CLOSE_RADIUS, 0);
    int r_max = min(r + OPEN_CLOSE_RADIUS, 119);
    int rmq_line_start = r_min % RMQ_BUF;
    int rmq_line_end = r_max % RMQ_BUF;
    for (int c = 0; c < 160; c++) {
      uint8_t best = 0;
      for (int line = rmq_line_start; line != rmq_line_end; line = (line + 1) % RMQ_BUF) {
        int half_len = OPEN_CLOSE_RADIUS;
        int c_min = max(c - half_len, 0);
        int c_max = min(c + half_len, 159);
        query_rmq(best, line, c_min, c_max, >);
      }
      _d(cur_frame)[r * 160 + c] = best;
    }
  }
  // (2) Erosion
  rmq_ptr = 0;
  for (int r = 0; r < OPEN_CLOSE_RADIUS; r++) build_rmq(r, <);
  for (int r = 0; r < 120; r++) {
    if (r + OPEN_CLOSE_RADIUS < 120)
      build_rmq(r + OPEN_CLOSE_RADIUS, <);
    int r_min = max(r - OPEN_CLOSE_RADIUS, 0);
    int r_max = min(r + OPEN_CLOSE_RADIUS, 119);
    int rmq_line_start = r_min % RMQ_BUF;
    int rmq_line_end = r_max % RMQ_BUF;
    for (int c = 0; c < 160; c++) {
      uint8_t best = 255;
      for (int line = rmq_line_start; line != rmq_line_end; line = (line + 1) % RMQ_BUF) {
        int half_len = OPEN_CLOSE_RADIUS;
        int c_min = max(c - half_len, 0);
        int c_max = min(c + half_len, 159);
        query_rmq(best, line, c_min, c_max, <);
      }
      _d(cur_frame)[r * 160 + c] = best;
    }
  }

  return _d(cur_frame);
}

static inline bool silhouette_base_initialized(_silhouette_this_arg)
{
  return _d(base_initialized);
}

#undef _d
#undef _silhouette_field
#undef _silhouette_field_name
#undef _silhouette_this_arg
#undef _silhouette_this_arg_
