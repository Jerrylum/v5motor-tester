/* Graphical Terminal library for PROS V5
 * Copyright (c) 2022 Andrew Palardy
 * This code is subject to the BSD 2-clause 'Simplified' license
 * See the LICENSE file for complete terms
 */
#ifndef _GTERM_H_
#define _GTERM_H_

#include <stdio.h>

#pragma GCC diagnostic ignored "-Wwrite-strings"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Definitions of maximum buffer sizes:
 * Default font is 20px tall and between 2 and 17px wide depending on the glyph
 * On the 480x240 screen that gets us:
 * width = 480 / 2 = 240 chars if they are all . or 28 chars if they are all W
 * width could also include control characters to change the color
 * Using 256 bytes as maximum width
 * height = 240 / 20 = 12 lines of text, but there are 2 bytes padding on all but the last line
 * so 11 lines of text
 */
#define GTERM_MAX_WIDTH 256
#define GTERM_MAX_LINES 11
#define GTERM_FONT_HEIGHT 22
#define GTERM_FONT_PAD 2

/* Function to clean gterm */
void gterm_clean();

/* Function to print to the gterm */
void gterm_print_int(const char* str);

/* Macro to print to gterm using printf-like syntax */
#define gterm_print(...)                          \
  do {                                            \
    char temp[GTERM_MAX_WIDTH];                   \
    snprintf(temp, GTERM_MAX_WIDTH, __VA_ARGS__); \
    gterm_print_int(temp);                        \
  } while (0)

#ifdef GTERM_IMPL
#include "display/lvgl.h"

/* Create buffer to store text going on screen */
static char gterm_buf[GTERM_MAX_LINES][GTERM_MAX_WIDTH] = {0};

/* Index to the next line to print to (circular buffer) */
static int gterm_buf_next = 0;

/* Text label object */
static lv_obj_t* gterm_stream = NULL;

/* Number of lines which are active (dep on screen size) */
static int gterm_act_lines = 0;

/* Has been initialized? */
static bool gterm_has_init = false;

/* Function to initialize gterm */
static void gterm_init(lv_obj_t* page) {
  /* If we are already initialized, do nothing */
  if (gterm_has_init) return;

  /* If page is null, use lv_scr_act() */
  if (!page) page = lv_scr_act();

  /* Determine how many lines we can fit on screen */
  gterm_act_lines = (lv_obj_get_height(page) + GTERM_FONT_PAD) / GTERM_FONT_HEIGHT;

  /* Create the label object in the parent page */
  gterm_stream = lv_label_create(page, NULL);
  lv_label_set_text(gterm_stream, "");
  lv_obj_align(gterm_stream, 0, LV_ALIGN_IN_TOP_LEFT, 0, 0);
  lv_label_set_recolor(gterm_stream, true);
  gterm_has_init = true;
}

void gterm_clean() {
  /* Verify that gterm is initialized */
  if (!gterm_has_init) return;

  /* Clear circular buffer pointer and set all buffer lines to null */
  gterm_buf_next = 0;
  for (int i = 0; i < GTERM_MAX_LINES; i++) gterm_buf[i][0] = 0;

  /* Clear the label */
  lv_label_set_text(gterm_stream, "");
}

void gterm_print_int(const char* str) {
  /* Verify that gterm is initialized */
  if (!gterm_has_init) return;

  /* Copy incoming string into print buffer at next entry space */
  strncpy(&gterm_buf[gterm_buf_next][0], str, GTERM_MAX_WIDTH);

  /* Increment next line and roll over */
  gterm_buf_next++;
  if (gterm_buf_next >= gterm_act_lines) gterm_buf_next = 0;

  /* Now concat each string with a newline into a big string */
  static char buf_out[GTERM_MAX_LINES * GTERM_MAX_WIDTH];
  buf_out[0] = 0;
  for (int i = 0; i < gterm_act_lines; i++) {
    /* Get index into buffer and wrap */
    int j = i + gterm_buf_next;
    if (j >= gterm_act_lines) j -= gterm_act_lines;

    /* Concat entry into buffer */
    strcat(buf_out, &gterm_buf[j][0]);
    strcat(buf_out, "\n");
  }

  /* Remove trailing newline by replacing it with null
   * Also ensures that a full buffer always ends with null
   */
  buf_out[strlen(buf_out) - 1] = 0;

  /* Send to graphics layer */
  lv_label_set_text(gterm_stream, buf_out);
}
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _GTERM_H_ */
