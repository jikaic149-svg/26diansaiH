#ifndef APP_DISPLAY_TEST_H
#define APP_DISPLAY_TEST_H

#include <stdbool.h>
#include <stdint.h>

/* ST7735S bench test: initialization and full-screen RGB565 patterns are
 * advanced by the existing 10 ms Task without blocking. */
bool APP_DisplayTest_Init(uint32_t now_ms);
bool APP_DisplayTest_Step(uint32_t now_ms);

/* Each debounced press selects the next color. A press during a refresh keeps
 * only the latest requested color; no unbounded event queue is created. */
bool APP_DisplayTest_NextPattern(void);

#endif
