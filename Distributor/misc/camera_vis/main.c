// gcc main.c -Ilibs/libs/minifb/include -Ilibs/openpnp-capture/include build_libs/openpnp-capture/libopenpnp-capture.0.dylib
// DYLD_LIBRARY_PATH=build_libs/openpnp-capture ./a.out

#include "openpnp-capture.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int main()
{
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
  uint8_t *p = malloc(fmt_info.width * fmt_info.height * 3);

  for (int i = 0; i < 10; i++) {
    if (Cap_hasNewFrame(ctx, stream) == 1) {
      Cap_captureFrame(ctx, stream, p, fmt_info.width * fmt_info.height * 3);
    }
  }

  free(p);

  return 0;
}
