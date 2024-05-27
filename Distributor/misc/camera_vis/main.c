// gcc main.c -Ilibs/minifb/include -Ilibs/openpnp-capture/include build_libs/minifb/libminifb.a build_libs/openpnp-capture/libopenpnp-capture.0.dylib build_libs/stb_image_resize2.o -framework AppKit -framework Metal -framework MetalKit
// DYLD_LIBRARY_PATH=build_libs/openpnp-capture ./a.out

#include "silhouette_detection.h"

#include "MiniFB.h"
#include "openpnp-capture.h"
#include "stb_image_resize2.h"  // stb_image_resize2 - v2.06 (2fb057a)

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void cb_keyboard(struct mfb_window *window, mfb_key key, mfb_key_mod mod, bool is_pressed)
{
  if (is_pressed &&
      (key == KB_KEY_ESCAPE || (key == KB_KEY_Q && (mod & KB_MOD_SUPER))))
    exit(0);
}

static uint64_t millitime()
{
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  return t.tv_sec + t.tv_nsec / 1000000;
}

int main()
{
  // ============ MiniFB initialisation ============

  struct mfb_window *window = mfb_open_ex("camera test", 320, 720, WF_RESIZABLE);
  mfb_set_keyboard_callback(window, cb_keyboard);

  uint8_t *p = malloc(640 * (480 * 3) * 4);
  memset(p, 0, 640 * (480 * 3) * 4);

  // ============ OpenPnP Capture initialisation ============

  CapContext ctx = Cap_createContext();

  int n_dev = Cap_getDeviceCount(ctx);
  for (int i = 0; i < n_dev; i++) {
    const char *name = Cap_getDeviceName(ctx, i);
    printf("Device [%d] name: %s\n", i, name);
  }

  int dev_idx = 1;  // XXX: Change this according to needs
  // FaceTime Camera (Apple Inc.)

  int n_fmt = Cap_getNumFormats(ctx, dev_idx);
  for (int i = 0; i < n_fmt; i++) {
    CapFormatInfo fmt_info;
    Cap_getFormatInfo(ctx, dev_idx, i, &fmt_info);
    printf("Format [%d]: %dx%d, %d FPS, FourCC %08x\n",
      i, fmt_info.width, fmt_info.height, fmt_info.fps, fmt_info.fourcc);
  }

  int fmt_idx = 1;  // XXX: Same as above
  // 848x480, 30 FPS, FourCC 79757673 (yuv2)

  CapFormatInfo fmt_info;
  Cap_getFormatInfo(ctx, dev_idx, fmt_idx, &fmt_info);
  int stream = Cap_openStream(ctx, dev_idx, fmt_idx);

  // openpnp-capture.h: "the frames returned by Cap_captureFrame are always 24-bit RGB."
  uint8_t *p_raw = malloc(fmt_info.width * fmt_info.height * 3);
  uint8_t *p_scaled = malloc(640 * 480 * 3);

  uint8_t *p_ycbcr = malloc(640 * 2);

  static struct silhouette_detection d;
  silhouette_init(&d);

  uint64_t t0 = millitime();

  do {
    if (Cap_hasNewFrame(ctx, stream) == 1) {
      Cap_captureFrame(ctx, stream, p_raw, fmt_info.width * fmt_info.height * 3);
    }

    stbir_resize_uint8_srgb(
      p_raw, fmt_info.width, fmt_info.height, fmt_info.width * 3,
      p_scaled, 640, 480, 640 * 3,
      STBIR_RGB);
    for (int r = 0; r < 480; r++) {
      for (int c = 0; c < 640; c++) {
        // BGRA
        p[(r * 640 + c) * 4 + 0] = p_scaled[(r * 640 + c) * 3 + 2];
        p[(r * 640 + c) * 4 + 1] = p_scaled[(r * 640 + c) * 3 + 1];
        p[(r * 640 + c) * 4 + 2] = p_scaled[(r * 640 + c) * 3 + 0];
        p[(r * 640 + c) * 4 + 3] = 255;
      }
    }

    uint64_t t1 = millitime();
    if (t1 - t0 >= 100) {
      // puts("*");
      t0 = t1;
      for (int r = 0; r < 480; r++) {
        for (int c = 0; c < 640; c++) {
          uint8_t luma = (
            (uint32_t)p_scaled[(r * 640 + c) * 3 + 2] * 299 +
            (uint32_t)p_scaled[(r * 640 + c) * 3 + 1] * 587 +
            (uint32_t)p_scaled[(r * 640 + c) * 3 + 0] * 114
          ) / 1000;
          // Assumes little-endian
          p_ycbcr[(c ^ 1) * 2 + 0] = 0;
          p_ycbcr[(c ^ 1) * 2 + 1] = luma;
        }
        silhouette_feed_line(&d, (uint32_t *)p_ycbcr);
      }
      silhouette_end_frame(&d);

      const uint8_t *base = silhouette_base_frame(&d);
      const uint8_t *resi = silhouette_residual_frame(&d);

      for (int r = 0; r < 120; r++) {
        for (int c = 0; c < 160; c++) {
          uint8_t luma = base[r * 160 + c];
          // Fill 4*4 block in reference frame
          for (int dr = 0; dr < 4; dr++) {
            for (int dc = 0; dc < 4; dc++) {
              p[((480 + r * 4 + dr) * 640 + c * 4 + dc) * 4 + 0] =
              p[((480 + r * 4 + dr) * 640 + c * 4 + dc) * 4 + 1] =
              p[((480 + r * 4 + dr) * 640 + c * 4 + dc) * 4 + 2] = luma;
              p[((480 + r * 4 + dr) * 640 + c * 4 + dc) * 4 + 3] = 255;
            }
          }

          uint8_t diff = resi[r * 160 + c];
          // Fill 4*4 block in difference
          for (int dr = 0; dr < 4; dr++) {
            for (int dc = 0; dc < 4; dc++) {
              p[((960 + r * 4 + dr) * 640 + c * 4 + dc) * 4 + 0] =
              p[((960 + r * 4 + dr) * 640 + c * 4 + dc) * 4 + 1] =
              p[((960 + r * 4 + dr) * 640 + c * 4 + dc) * 4 + 2] = diff;
              p[((960 + r * 4 + dr) * 640 + c * 4 + dc) * 4 + 3] = 255;
            }
          }
        }
      }
    }

    mfb_update_ex(window, p, 640, 480 * 3);
  } while (mfb_wait_sync(window));

  free(p);

  return 0;
}
