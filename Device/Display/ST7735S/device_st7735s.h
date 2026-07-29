#ifndef DEVICE_ST7735S_H
#define DEVICE_ST7735S_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEVICE_ST7735S_NATIVE_WIDTH 128U
#define DEVICE_ST7735S_NATIVE_HEIGHT 160U

typedef enum {
    DEVICE_ST7735S_RESULT_OK = 0,
    DEVICE_ST7735S_RESULT_INVALID_ARGUMENT,
    DEVICE_ST7735S_RESULT_BACKEND_ERROR,
    DEVICE_ST7735S_RESULT_BUSY,
    DEVICE_ST7735S_RESULT_TIMEOUT,
} Device_ST7735S_Result;

typedef enum {
    DEVICE_ST7735S_ROTATION_0 = 0,
    DEVICE_ST7735S_ROTATION_90,
    DEVICE_ST7735S_ROTATION_180,
    DEVICE_ST7735S_ROTATION_270,
} Device_ST7735S_Rotation;

typedef struct {
    size_t backend_index;
    uint32_t expected_bit_rate_hz;
    uint32_t transfer_timeout_ms;
    Device_ST7735S_Rotation rotation;
} Device_ST7735S_Config;

typedef struct {
    Device_ST7735S_Config config;
    void *backend;
    uint16_t width;
    uint16_t height;
    uint16_t x_offset;
    uint16_t y_offset;
    bool initialized;
} Device_ST7735S;

bool Device_ST7735S_DefaultConfig(
    size_t index, Device_ST7735S_Config *config);

/* Performs the module-specific blocking reset/init sequence (~420 ms) and
 * enables the backlight only after Display On succeeds. */
Device_ST7735S_Result Device_ST7735S_Init(
    Device_ST7735S *device,
    const Device_ST7735S_Config *config);

Device_ST7735S_Result Device_ST7735S_SetRotation(
    Device_ST7735S *device, Device_ST7735S_Rotation rotation);

Device_ST7735S_Result Device_ST7735S_SetBacklight(
    Device_ST7735S *device, bool enabled);

Device_ST7735S_Result Device_ST7735S_DrawPixel(
    Device_ST7735S *device,
    uint16_t x,
    uint16_t y,
    uint16_t rgb565);

Device_ST7735S_Result Device_ST7735S_Fill(
    Device_ST7735S *device,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    uint16_t rgb565);

/* Pixel order is row-major. pixel_count must exactly match the inclusive
 * rectangle; values are transmitted MSB first as RGB565. */
Device_ST7735S_Result Device_ST7735S_WriteRGB565(
    Device_ST7735S *device,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    const uint16_t *pixels,
    size_t pixel_count);

bool Device_ST7735S_GetSize(
    const Device_ST7735S *device,
    uint16_t *width,
    uint16_t *height);

uint16_t Device_ST7735S_Color565(
    uint8_t red, uint8_t green, uint8_t blue);

#endif
