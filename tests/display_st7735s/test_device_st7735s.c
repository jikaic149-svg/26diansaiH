#include "device_st7735s.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_display.h"
#include "bsp_time.h"

#define MOCK_MAX_WRITES 64U
#define MOCK_MAX_WRITE_BYTES 128U
#define MOCK_MAX_DELAYS 8U

struct BSP_Display_Resource {
    uint32_t marker;
};

typedef struct {
    bool data_mode;
    size_t length;
    uint8_t bytes[MOCK_MAX_WRITE_BYTES];
    uint32_t timeout_ms;
} MockWrite;

static struct BSP_Display_Resource g_display = {UINT32_C(0x7735)};
static MockWrite g_writes[MOCK_MAX_WRITES];
static size_t g_write_count;
static uint32_t g_delays[MOCK_MAX_DELAYS];
static size_t g_delay_count;
static bool g_reset_values[4];
static size_t g_reset_count;
static bool g_backlight_values[4];
static size_t g_backlight_count;
static size_t g_write_count_when_backlight_enabled;
static uint32_t g_bit_rate_hz;
static bool g_backlight_available;
static BSP_Display_Result g_next_write_result;

static void reset_mock(void)
{
    (void)memset(g_writes, 0, sizeof(g_writes));
    (void)memset(g_delays, 0, sizeof(g_delays));
    (void)memset(g_reset_values, 0, sizeof(g_reset_values));
    (void)memset(g_backlight_values, 0, sizeof(g_backlight_values));
    g_write_count = 0U;
    g_delay_count = 0U;
    g_reset_count = 0U;
    g_backlight_count = 0U;
    g_write_count_when_backlight_enabled = 0U;
    g_bit_rate_hz = UINT32_C(10000000);
    g_backlight_available = true;
    g_next_write_result = BSP_DISPLAY_RESULT_OK;
}

static void clear_writes(void)
{
    (void)memset(g_writes, 0, sizeof(g_writes));
    g_write_count = 0U;
}

static void expect_write(
    size_t index,
    bool data_mode,
    const uint8_t *bytes,
    size_t length)
{
    assert(index < g_write_count);
    assert(g_writes[index].data_mode == data_mode);
    assert(g_writes[index].length == length);
    assert(g_writes[index].timeout_ms == 10U);
    assert(memcmp(g_writes[index].bytes, bytes, length) == 0);
}

size_t BSP_Display_Count(void)
{
    return 1U;
}

BSP_Display_Handle BSP_Display_GetHandle(size_t index)
{
    return (index == 0U) ? &g_display : NULL;
}

bool BSP_Display_GetInfo(
    BSP_Display_Handle display, BSP_Display_Info *info)
{
    if ((display != &g_display) || (info == NULL)) {
        return false;
    }
    info->bit_rate_hz = g_bit_rate_hz;
    return true;
}

bool BSP_Display_SetReset(
    BSP_Display_Handle display, bool reset_asserted)
{
    if ((display != &g_display) ||
        (g_reset_count >= (sizeof(g_reset_values) /
            sizeof(g_reset_values[0])))) {
        return false;
    }
    g_reset_values[g_reset_count++] = reset_asserted;
    return true;
}

bool BSP_Display_SetBacklight(
    BSP_Display_Handle display, bool enabled)
{
    if ((display != &g_display) || !g_backlight_available ||
        (g_backlight_count >= (sizeof(g_backlight_values) /
            sizeof(g_backlight_values[0])))) {
        return false;
    }
    g_backlight_values[g_backlight_count++] = enabled;
    if (enabled) {
        g_write_count_when_backlight_enabled = g_write_count;
    }
    return true;
}

BSP_Display_Result BSP_Display_Write(
    BSP_Display_Handle display,
    bool data_mode,
    const uint8_t *bytes,
    size_t length,
    uint32_t timeout_ms)
{
    BSP_Display_Result result = g_next_write_result;

    if ((display != &g_display) || (bytes == NULL) || (length == 0U) ||
        (length > MOCK_MAX_WRITE_BYTES) ||
        (g_write_count >= MOCK_MAX_WRITES)) {
        return BSP_DISPLAY_RESULT_INVALID_ARGUMENT;
    }
    g_writes[g_write_count].data_mode = data_mode;
    g_writes[g_write_count].length = length;
    g_writes[g_write_count].timeout_ms = timeout_ms;
    (void)memcpy(g_writes[g_write_count].bytes, bytes, length);
    ++g_write_count;
    g_next_write_result = BSP_DISPLAY_RESULT_OK;
    return result;
}

void BSP_Time_DelayMs(uint32_t milliseconds)
{
    assert(g_delay_count < MOCK_MAX_DELAYS);
    g_delays[g_delay_count++] = milliseconds;
}

static Device_ST7735S initialized_device(void)
{
    static const uint8_t sleep_out[] = {0x11U};
    static const uint8_t display_on[] = {0x29U};
    Device_ST7735S_Config config;
    Device_ST7735S device;
    uint16_t width;
    uint16_t height;

    assert(Device_ST7735S_DefaultConfig(0U, &config));
    assert(config.backend_index == 0U);
    assert(config.expected_bit_rate_hz == UINT32_C(10000000));
    assert(config.transfer_timeout_ms == 10U);
    assert(config.rotation == DEVICE_ST7735S_ROTATION_0);
    assert(Device_ST7735S_Init(&device, &config) ==
        DEVICE_ST7735S_RESULT_OK);

    assert(g_reset_count == 2U);
    assert(g_reset_values[0]);
    assert(!g_reset_values[1]);
    assert(g_backlight_count == 2U);
    assert(!g_backlight_values[0]);
    assert(g_backlight_values[1]);
    assert(g_delay_count == 4U);
    assert(g_delays[0] == 100U);
    assert(g_delays[1] == 100U);
    assert(g_delays[2] == 100U);
    assert(g_delays[3] == 120U);
    assert(g_write_count == 30U);
    assert(g_write_count_when_backlight_enabled == g_write_count);
    expect_write(0U, false, sleep_out, sizeof(sleep_out));
    expect_write(29U, false, display_on, sizeof(display_on));
    assert(Device_ST7735S_GetSize(&device, &width, &height));
    assert(width == 128U);
    assert(height == 160U);
    return device;
}

static void test_initialization_and_colors(void)
{
    Device_ST7735S_Config config;
    Device_ST7735S device;

    reset_mock();
    device = initialized_device();
    assert(device.initialized);
    assert(Device_ST7735S_Color565(255U, 0U, 0U) == UINT16_C(0xF800));
    assert(Device_ST7735S_Color565(0U, 255U, 0U) == UINT16_C(0x07E0));
    assert(Device_ST7735S_Color565(0U, 0U, 255U) == UINT16_C(0x001F));
    assert(Device_ST7735S_Color565(255U, 255U, 255U) ==
        UINT16_C(0xFFFF));

    g_bit_rate_hz = UINT32_C(8000000);
    assert(!Device_ST7735S_DefaultConfig(0U, &config));
    assert(!Device_ST7735S_DefaultConfig(1U, &config));
}

static void test_rectangles_rotation_and_errors(void)
{
    static const uint8_t column_command[] = {0x2AU};
    static const uint8_t column_data[] = {0x00U, 0x03U, 0x00U, 0x04U};
    static const uint8_t row_command[] = {0x2BU};
    static const uint8_t row_data[] = {0x00U, 0x03U, 0x00U, 0x03U};
    static const uint8_t memory_write[] = {0x2CU};
    static const uint8_t pixel_data[] = {0xF8U, 0x00U, 0x07U, 0xE0U};
    static const uint8_t rotation_command[] = {0x36U};
    static const uint8_t rotation_data[] = {0x70U};
    const uint16_t pixels[] = {UINT16_C(0xF800), UINT16_C(0x07E0)};
    Device_ST7735S device;
    uint16_t width;
    uint16_t height;

    reset_mock();
    device = initialized_device();
    clear_writes();
    assert(Device_ST7735S_WriteRGB565(
        &device, 1U, 2U, 2U, 2U, pixels, 2U) ==
        DEVICE_ST7735S_RESULT_OK);
    assert(g_write_count == 6U);
    expect_write(0U, false, column_command, sizeof(column_command));
    expect_write(1U, true, column_data, sizeof(column_data));
    expect_write(2U, false, row_command, sizeof(row_command));
    expect_write(3U, true, row_data, sizeof(row_data));
    expect_write(4U, false, memory_write, sizeof(memory_write));
    expect_write(5U, true, pixel_data, sizeof(pixel_data));

    clear_writes();
    assert(Device_ST7735S_Fill(
        &device, 0U, 0U, 64U, 0U, UINT16_C(0x1234)) ==
        DEVICE_ST7735S_RESULT_OK);
    assert(g_write_count == 7U);
    assert(g_writes[5].data_mode);
    assert(g_writes[5].length == 128U);
    assert(g_writes[6].data_mode);
    assert(g_writes[6].length == 2U);
    assert(g_writes[5].bytes[0] == 0x12U);
    assert(g_writes[5].bytes[1] == 0x34U);
    assert(g_writes[6].bytes[0] == 0x12U);
    assert(g_writes[6].bytes[1] == 0x34U);

    clear_writes();
    assert(Device_ST7735S_WriteRGB565(
        &device, 0U, 0U, 1U, 0U, pixels, 1U) ==
        DEVICE_ST7735S_RESULT_INVALID_ARGUMENT);
    assert(Device_ST7735S_DrawPixel(
        &device, 128U, 0U, UINT16_C(0xFFFF)) ==
        DEVICE_ST7735S_RESULT_INVALID_ARGUMENT);
    assert(g_write_count == 0U);

    assert(Device_ST7735S_SetRotation(
        &device, DEVICE_ST7735S_ROTATION_90) ==
        DEVICE_ST7735S_RESULT_OK);
    assert(g_write_count == 2U);
    expect_write(0U, false, rotation_command, sizeof(rotation_command));
    expect_write(1U, true, rotation_data, sizeof(rotation_data));
    assert(Device_ST7735S_GetSize(&device, &width, &height));
    assert(width == 160U);
    assert(height == 128U);

    clear_writes();
    g_next_write_result = BSP_DISPLAY_RESULT_TIMEOUT;
    assert(Device_ST7735S_DrawPixel(
        &device, 0U, 0U, UINT16_C(0xFFFF)) ==
        DEVICE_ST7735S_RESULT_TIMEOUT);
    g_next_write_result = BSP_DISPLAY_RESULT_BUSY;
    assert(Device_ST7735S_DrawPixel(
        &device, 0U, 0U, UINT16_C(0xFFFF)) ==
        DEVICE_ST7735S_RESULT_BUSY);

    g_backlight_available = false;
    assert(Device_ST7735S_SetBacklight(&device, false) ==
        DEVICE_ST7735S_RESULT_BACKEND_ERROR);
}

int main(void)
{
    test_initialization_and_colors();
    test_rectangles_rotation_and_errors();
    puts("ST7735S display Device tests: PASS");
    return 0;
}
