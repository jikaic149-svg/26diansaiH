#include "app_display_test.h"

#include <stddef.h>

#include "bsp_tft.h"
#include "device_st7735s.h"

static const uint16_t s_test_colors[] = {
    UINT16_C(0xF800), /* red */
    UINT16_C(0x07E0), /* green */
    UINT16_C(0x001F), /* blue */
    UINT16_C(0xFFFF), /* white */
    UINT16_C(0x0000), /* black */
    UINT16_C(0xFFE0), /* yellow */
};

#define DISPLAY_TEST_PATTERN_COUNT \
    (sizeof(s_test_colors) / sizeof(s_test_colors[0]))
#define DISPLAY_TEST_PATTERN_NONE UINT8_C(0xFF)

static Device_ST7735S s_display;
static uint8_t s_requested_pattern;
static uint8_t s_started_pattern;
static bool s_initialized;

bool APP_DisplayTest_Init(uint32_t now_ms)
{
    BSP_TFT_Handle io = BSP_TFT_GetHandle(0U);

    s_requested_pattern = 0U;
    s_started_pattern = DISPLAY_TEST_PATTERN_NONE;
    s_initialized = Device_ST7735S_Init(&s_display, io, now_ms);
    return s_initialized;
}

bool APP_DisplayTest_Step(uint32_t now_ms)
{
    if (!s_initialized || !Device_ST7735S_Step(&s_display, now_ms) ||
        Device_ST7735S_HasError(&s_display)) {
        return false;
    }

    if (Device_ST7735S_IsInitialized(&s_display) &&
        !Device_ST7735S_IsBusy(&s_display) &&
        (s_started_pattern != s_requested_pattern)) {
        if (!Device_ST7735S_FillScreen(
                &s_display, s_test_colors[s_requested_pattern])) {
            return false;
        }
        s_started_pattern = s_requested_pattern;
    }
    return true;
}

bool APP_DisplayTest_NextPattern(void)
{
    if (!s_initialized) {
        return false;
    }
    s_requested_pattern =
        (uint8_t)((s_requested_pattern + 1U) % DISPLAY_TEST_PATTERN_COUNT);
    return true;
}
