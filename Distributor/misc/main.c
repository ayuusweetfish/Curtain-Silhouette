// gcc -O2 -DSTB_IMAGE_IMPLEMENTATION -c -x c stb_image.h
// gcc -std=c99 main.c stb_image.o

#include "stb_image.h"  // stb_image - v2.29 (0bc88af)

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static inline bool is_red(const uint8_t *p)
{
  return (p[0] >= 192 && p[1] < 128 && p[2] < 128);
}
static inline bool is_blue(const uint8_t *p)
{
  return (p[0] < 128 && p[1] < 192 && p[2] >= 192);
}
static inline uint8_t colour(const uint8_t *p)
{
  return (is_red(p) ? 1 : is_blue(p) ? 2 : 0);
}

int main(int argc, char *argv[])
{
  if (argc <= 1) {
    fprintf(stderr, "Usage: %s <file>\n", argv[0]);
    return 0;
  }

  // Load image
  int w, h;
  uint8_t *p = stbi_load(argv[1], &w, &h, NULL, 3);
  if (p == NULL) {
    fprintf(stderr, "Cannot open image %s\n", argv[1]);
    return 1;
  }

  // Extract red and blue regions

  // Visited mark array
  bool *vis = malloc(sizeof(bool) * w * h);
  memset(vis, 0, sizeof(bool) * w * h);

  // Queue for floodfill
  struct pix { int r, c; };
  struct pix *q = malloc(sizeof(struct pix) * w * h);
  int qhead, qtail;

  // Set of extracted centroids
  struct dot { struct pix p; int colour; };
  struct dot *dots = NULL;
  int n_dots = 0;
  int cap_dots = 0;

  for (int r = 0; r < h; r++)
    for (int c = 0; c < w; c++) if (!vis[r * w + c]) {
      uint8_t cur_colour = colour(p + (r * w + c) * 3);
      if (cur_colour != 0 &&
        r < h - 1 && c < w - 1 &&
        cur_colour == colour(p + ((r + 1) * w + c) * 3) &&
        cur_colour == colour(p + (r * w + (c + 1)) * 3) &&
        cur_colour == colour(p + ((r + 1) * w + (c + 1)) * 3)
      ) {
        // Floodfill!
        q[0] = (struct pix){r, c}; qhead = 0; qtail = 1;
        vis[r * w + c] = true;

        // Centroid calculation
        int r_sum = 0, c_sum = 0;

        while (qhead < qtail) {
          int r1 = q[qhead].r;
          int c1 = q[qhead].c;
          qhead++;

          r_sum += r1;
          c_sum += c1;

          // Extend
          const int move[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
          for (int k = 0; k < 4; k++) {
            int r2 = r1 + move[k][0];
            int c2 = c1 + move[k][1];
            if (r2 >= 0 && r2 < h && c2 >= 0 && c2 < w &&
                !vis[r2 * w + c2] &&
                cur_colour == colour(p + (r2 * w + c2) * 3)) {
              q[qtail++] = (struct pix){r2, c2};
              vis[r2 * w + c2] = true;
            }
          }
        }

        int r_cen = r_sum / qtail;
        int c_cen = c_sum / qtail;
        // Append to dots
        if (++n_dots > cap_dots) {
          cap_dots = (cap_dots == 0 ? 16 : cap_dots * 2);
          dots = realloc(dots, sizeof(dots[0]) * cap_dots);
        }
        dots[n_dots - 1] = (struct dot){{r_cen, c_cen}, cur_colour};
      }
    }

  // Cluster coordinates on two axes
  int *rs = malloc(sizeof(int) * h);
  int *cs = malloc(sizeof(int) * w);
  memset(rs, -1, sizeof(int) * h);
  memset(cs, -1, sizeof(int) * w);

  for (int i = 0; i < n_dots; i++) {
    rs[dots[i].p.r] = 0;
    cs[dots[i].p.c] = 0;
  }

  // Count of clustered/discretized rows and columns
  int rn = 0;
  int cn = 0;

  for (int i = 0; i < h; i++)
    if (rs[i] != -1) {
      for (int j = 0; j <= 2; j++) rs[i++] = rn;
      rn++;
    }
  for (int i = 0; i < w; i++)
    if (cs[i] != -1) {
      for (int j = 0; j <= 2; j++) cs[i++] = cn;
      cn++;
    }

  // Discretize and put into array
  int *dot_colours = malloc(sizeof(int) * rn * cn);
  memset(dot_colours, 0, sizeof(int) * rn * cn);

  for (int i = 0; i < n_dots; i++) {
    int r = rs[dots[i].p.r];
    int c = cs[dots[i].p.c];
    dot_colours[r * cn + c] = dots[i].colour;
  }

  // Output
  for (int i = 0; i < rn; i++)
    for (int j = 0; j < cn; j++)
      printf("%d%s", dot_colours[i * cn + j], j == cn - 1 ? ",\n" : ", ");

  stbi_image_free(p);
  return 0;
}
