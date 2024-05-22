// gcc -O2 -DSTB_IMAGE_IMPLEMENTATION -c -x c stb_image.h
// gcc -O2 -DSTB_IMAGE_RESIZE_IMPLEMENTATION -c -x c stb_image_resize2.h
// gcc -std=c99 main.c stb_image.o stb_image_resize2.o

// for i in {1..3}; do ./a.out `echo ~/Desktop/动画x3/动画$i/*.png | awk '{ for (i = 1; i <= NF; i += 2) print $i}'` > anim_seq_$i.h; done

#include "stb_image.h"  // stb_image - v2.29 (0bc88af)
#include "stb_image_resize2.h"  // stb_image_resize2 - v2.06 (2fb057a)

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[])
{
  if (argc <= 1) {
    fprintf(stderr, "Usage: %s <images>\n", argv[0]);
    return 0;
  }

  int n_frames = argc - 1;

  printf("static const uint32_t anim_n_frames = %d;\n", n_frames);
  printf("static const uint8_t anim_seq[%d][25][%d] = {", n_frames, 192 / 8);

  for (int i = 0; i < n_frames; i++) {
    // Load image
    int w, h;
    uint8_t *p = stbi_load(argv[i + 1], &w, &h, NULL, 1);
    if (p == NULL) {
      fprintf(stderr, "Cannot open image %s\n", argv[i + 1]);
      return 1;
    }

    uint8_t *p_scaled = stbir_resize_uint8_linear(
      p, w, h, w * 1,
      NULL, 25, 190, 25 * 1,
      STBIR_1CHANNEL);

    printf("{\n");
    for (int c = 0; c < 25; c++)  // This is not C++
      for (int r = 0; r < 192; r++) {
        if (r == 0) printf("  {0b");
        printf("%d", (r < 190 && p_scaled[r * 25 + c] < 128) ? 1 : 0);
        if (r == 192 - 1) printf("},\n");
        else if (r % 8 == 7) printf(", 0b");
      }
    printf("}");
    if (i != n_frames - 1) printf(", ");

    // Clean up
    free(p_scaled);
    stbi_image_free(p);
  }

  printf("};\n");
  fprintf(stderr, "%d frame(s)\n", n_frames);

  return 0;
}
