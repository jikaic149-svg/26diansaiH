#include "device_st7735s.h"

#include <stddef.h>

#include "bsp_display.h"
#include "bsp_time.h"

#define ST7735S_CMD_SLEEP_OUT UINT8_C(0x11)
#define ST7735S_CMD_FRAME_RATE_1 UINT8_C(0xB1)
#define ST7735S_CMD_FRAME_RATE_2 UINT8_C(0xB2)
#define ST7735S_CMD_FRAME_RATE_3 UINT8_C(0xB3)
#define ST7735S_CMD_INVERSION UINT8_C(0xB4)
#define ST7735S_CMD_POWER_1 UINT8_C(0xC0)
#define ST7735S_CMD_POWER_2 UINT8_C(0xC1)
#define ST7735S_CMD_POWER_3 UINT8_C(0xC2)
#define ST7735S_CMD_POWER_4 UINT8_C(0xC3)
#define ST7735S_CMD_POWER_5 UINT8_C(0xC4)
#define ST7735S_CMD_VCOM UINT8_C(0xC5)
#define ST7735S_CMD_MEMORY_ACCESS UINT8_C(0x36)
#define ST7735S_CMD_PIXEL_FORMAT UINT8_C(0x3A)
#define ST7735S_CMD_GAMMA_POSITIVE UINT8_C(0xE0)
#define ST7735S_CMD_GAMMA_NEGATIVE UINT8_C(0xE1)
#define ST7735S_CMD_COLUMN_ADDRESS UINT8_C(0x2A)
#define ST7735S_CMD_ROW_ADDRESS UINT8_C(0x2B)
#define ST7735S_CMD_MEMORY_WRITE UINT8_C(0x2C)
#define ST7735S_CMD_DISPLAY_ON UINT8_C(0x29)

#define ST7735S_DEFAULT_BIT_RATE_HZ UINT32_C(10000000)
#define ST7735S_DEFAULT_TIMEOUT_MS UINT32_C(10)
#define ST7735S_PIXEL_CHUNK_COUNT 64U

static bool valid_rotation(Device_ST7735S_Rotation rotation)
{
    return (rotation == DEVICE_ST7735S_ROTATION_0) ||
        (rotation == DEVICE_ST7735S_ROTATION_90) ||
        (rotation == DEVICE_ST7735S_ROTATION_180) ||
        (rotation == DEVICE_ST7735S_ROTATION_270);
}

static Device_ST7735S_Result map_backend_result(
    BSP_Display_Result result)
{
    switch (result) {
        case BSP_DISPLAY_RESULT_OK:
            return DEVICE_ST7735S_RESULT_OK;
        case BSP_DISPLAY_RESULT_BUSY:
            return DEVICE_ST7735S_RESULT_BUSY;
        case BSP_DISPLAY_RESULT_TIMEOUT:
            return DEVICE_ST7735S_RESULT_TIMEOUT;
        default:
            return DEVICE_ST7735S_RESULT_BACKEND_ERROR;
    }
}

static Device_ST7735S_Result write_bytes(
    const Device_ST7735S *device,
    bool data_mode,
    const uint8_t *bytes,
    size_t length)
{
    if ((device == NULL) || (device->backend == NULL)) {
        return DEVICE_ST7735S_RESULT_INVALID_ARGUMENT;
    }
    return map_backend_result(BSP_Display_Write(
        (BSP_Display_Handle)device->backend,
        data_mode,
        bytes,
        length,
        device->config.transfer_timeout_ms));
}

static Device_ST7735S_Result write_command(
    const Device_ST7735S *device,
    uint8_t command,
    const uint8_t *parameters,
    size_t parameter_count)
{
    Device_ST7735S_Result result =
        write_bytes(device, false, &command, 1U);

    if ((result == DEVICE_ST7735S_RESULT_OK) &&
        (parameter_count != 0U)) {
        result = write_bytes(
            device, true, parameters, parameter_count);
    }
    return result;
}

static Device_ST7735S_Result apply_rotation(
    Device_ST7735S *device, Device_ST7735S_Rotation rotation)
{
    uint8_t memory_access;
    uint16_t width;
    uint16_t height;
    uint16_t x_offset;
    uint16_t y_offset;
    Device_ST7735S_Result result;

    switch (rotation) {
        case DEVICE_ST7735S_ROTATION_0:
            memory_access = UINT8_C(0x00);
            width = DEVICE_ST7735S_NATIVE_WIDTH;
            height = DEVICE_ST7735S_NATIVE_HEIGHT;
            x_offset = 2U;
            y_offset = 1U;
            break;
        case DEVICE_ST7735S_ROTATION_90:
            memory_access = UINT8_C(0x70);
            width = DEVICE_ST7735S_NATIVE_HEIGHT;
            height = DEVICE_ST7735S_NATIVE_WIDTH;
            x_offset = 1U;
            y_offset = 2U;
            break;
        case DEVICE_ST7735S_ROTATION_180:
            memory_access = UINT8_C(0xC0);
            width = DEVICE_ST7735S_NATIVE_WIDTH;
            height = DEVICE_ST7735S_NATIVE_HEIGHT;
            x_offset = 2U;
            y_offset = 1U;
            break;
        case DEVICE_ST7735S_ROTATION_270:
            memory_access = UINT8_C(0xA0);
            width = DEVICE_ST7735S_NATIVE_HEIGHT;
            height = DEVICE_ST7735S_NATIVE_WIDTH;
            x_offset = 1U;
            y_offset = 2U;
            break;
        default:
            return DEVICE_ST7735S_RESULT_INVALID_ARGUMENT;
    }

    result = write_command(device,
        ST7735S_CMD_MEMORY_ACCESS, &memory_access, 1U);
    if (result == DEVICE_ST7735S_RESULT_OK) {
        device->config.rotation = rotation;
        device->width = width;
        device->height = height;
        device->x_offset = x_offset;
        device->y_offset = y_offset;
    }
    return result;
}

static Device_ST7735S_Result write_initial_sequence(
    Device_ST7735S *device)
{
    static const uint8_t frame_rate[] = {0x05U, 0x3CU, 0x3CU};
    static const uint8_t frame_rate_3[] = {
        0x05U, 0x3CU, 0x3CU, 0x05U, 0x3CU, 0x3CU};
    static const uint8_t inversion[] = {0x03U};
    static const uint8_t power_1[] = {0x28U, 0x08U, 0x04U};
    static const uint8_t power_2[] = {0xC0U};
    static const uint8_t power_3[] = {0x0DU, 0x00U};
    static const uint8_t power_4[] = {0x8DU, 0x2AU};
    static const uint8_t power_5[] = {0x8DU, 0xEEU};
    static const uint8_t vcom[] = {0x1AU};
    static const uint8_t gamma_positive[] = {
        0x04U, 0x22U, 0x07U, 0x0AU, 0x2EU, 0x30U, 0x25U, 0x2AU,
        0x28U, 0x26U, 0x2EU, 0x3AU, 0x00U, 0x01U, 0x03U, 0x13U};
    static const uint8_t gamma_negative[] = {
        0x04U, 0x16U, 0x06U, 0x0DU, 0x2DU, 0x26U, 0x23U, 0x27U,
        0x27U, 0x25U, 0x2DU, 0x3BU, 0x00U, 0x01U, 0x04U, 0x13U};
    static const uint8_t pixel_format[] = {0x05U};
    Device_ST7735S_Result result;

#define WRITE_INIT(command_, data_) \
    do { \
        result = write_command(device, (command_), \
            (data_), sizeof(data_)); \
        if (result != DEVICE_ST7735S_RESULT_OK) { \
            return result; \
        } \
    } while (0)

    result = write_command(device, ST7735S_CMD_SLEEP_OUT, NULL, 0U);
    if (result != DEVICE_ST7735S_RESULT_OK) {
        return result;
    }
    BSP_Time_DelayMs(120U);
    WRITE_INIT(ST7735S_CMD_FRAME_RATE_1, frame_rate);
    WRITE_INIT(ST7735S_CMD_FRAME_RATE_2, frame_rate);
    WRITE_INIT(ST7735S_CMD_FRAME_RATE_3, frame_rate_3);
    WRITE_INIT(ST7735S_CMD_INVERSION, inversion);
    WRITE_INIT(ST7735S_CMD_POWER_1, power_1);
    WRITE_INIT(ST7735S_CMD_POWER_2, power_2);
    WRITE_INIT(ST7735S_CMD_POWER_3, power_3);
    WRITE_INIT(ST7735S_CMD_POWER_4, power_4);
    WRITE_INIT(ST7735S_CMD_POWER_5, power_5);
    WRITE_INIT(ST7735S_CMD_VCOM, vcom);
    result = apply_rotation(device, device->config.rotation);
    if (result != DEVICE_ST7735S_RESULT_OK) {
        return result;
    }
    WRITE_INIT(ST7735S_CMD_GAMMA_POSITIVE, gamma_positive);
    WRITE_INIT(ST7735S_CMD_GAMMA_NEGATIVE, gamma_negative);
    WRITE_INIT(ST7735S_CMD_PIXEL_FORMAT, pixel_format);

#undef WRITE_INIT

    return write_command(device, ST7735S_CMD_DISPLAY_ON, NULL, 0U);
}

static Device_ST7735S_Result set_window(
    Device_ST7735S *device,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1)
{
    uint16_t physical_x0;
    uint16_t physical_x1;
    uint16_t physical_y0;
    uint16_t physical_y1;
    uint8_t parameters[4];
    Device_ST7735S_Result result;

    if ((device == NULL) || !device->initialized || (x0 > x1) ||
        (y0 > y1) || (x1 >= device->width) ||
        (y1 >= device->height)) {
        return DEVICE_ST7735S_RESULT_INVALID_ARGUMENT;
    }
    physical_x0 = (uint16_t)(x0 + device->x_offset);
    physical_x1 = (uint16_t)(x1 + device->x_offset);
    physical_y0 = (uint16_t)(y0 + device->y_offset);
    physical_y1 = (uint16_t)(y1 + device->y_offset);

    parameters[0] = (uint8_t)(physical_x0 >> 8U);
    parameters[1] = (uint8_t)physical_x0;
    parameters[2] = (uint8_t)(physical_x1 >> 8U);
    parameters[3] = (uint8_t)physical_x1;
    result = write_command(device,
        ST7735S_CMD_COLUMN_ADDRESS, parameters, sizeof(parameters));
    if (result != DEVICE_ST7735S_RESULT_OK) {
        return result;
    }

    parameters[0] = (uint8_t)(physical_y0 >> 8U);
    parameters[1] = (uint8_t)physical_y0;
    parameters[2] = (uint8_t)(physical_y1 >> 8U);
    parameters[3] = (uint8_t)physical_y1;
    result = write_command(device,
        ST7735S_CMD_ROW_ADDRESS, parameters, sizeof(parameters));
    if (result != DEVICE_ST7735S_RESULT_OK) {
        return result;
    }
    return write_command(device, ST7735S_CMD_MEMORY_WRITE, NULL, 0U);
}

bool Device_ST7735S_DefaultConfig(
    size_t index, Device_ST7735S_Config *config)
{
    BSP_Display_Handle handle;
    BSP_Display_Info info;

    if (config == NULL) {
        return false;
    }
    handle = BSP_Display_GetHandle(index);
    if ((handle == NULL) || !BSP_Display_GetInfo(handle, &info) ||
        (info.bit_rate_hz != ST7735S_DEFAULT_BIT_RATE_HZ)) {
        return false;
    }
    *config = (Device_ST7735S_Config){
        .backend_index = index,
        .expected_bit_rate_hz = ST7735S_DEFAULT_BIT_RATE_HZ,
        .transfer_timeout_ms = ST7735S_DEFAULT_TIMEOUT_MS,
        .rotation = DEVICE_ST7735S_ROTATION_0,
    };
    return true;
}

Device_ST7735S_Result Device_ST7735S_Init(
    Device_ST7735S *device,
    const Device_ST7735S_Config *config)
{
    BSP_Display_Handle handle;
    BSP_Display_Info info;
    Device_ST7735S_Result result;

    if (device != NULL) {
        *device = (Device_ST7735S){0};
    }
    if ((device == NULL) || (config == NULL) ||
        (config->expected_bit_rate_hz == 0U) ||
        (config->transfer_timeout_ms == 0U) ||
        !valid_rotation(config->rotation)) {
        return DEVICE_ST7735S_RESULT_INVALID_ARGUMENT;
    }
    handle = BSP_Display_GetHandle(config->backend_index);
    if ((handle == NULL) || !BSP_Display_GetInfo(handle, &info) ||
        (info.bit_rate_hz != config->expected_bit_rate_hz)) {
        return DEVICE_ST7735S_RESULT_BACKEND_ERROR;
    }

    device->config = *config;
    device->backend = (void *)handle;
    if (!BSP_Display_SetBacklight(handle, false) ||
        !BSP_Display_SetReset(handle, true)) {
        *device = (Device_ST7735S){0};
        return DEVICE_ST7735S_RESULT_BACKEND_ERROR;
    }
    BSP_Time_DelayMs(100U);
    if (!BSP_Display_SetReset(handle, false)) {
        *device = (Device_ST7735S){0};
        return DEVICE_ST7735S_RESULT_BACKEND_ERROR;
    }
    BSP_Time_DelayMs(100U);
    BSP_Time_DelayMs(100U);

    result = write_initial_sequence(device);
    if ((result != DEVICE_ST7735S_RESULT_OK) ||
        !BSP_Display_SetBacklight(handle, true)) {
        (void)BSP_Display_SetBacklight(handle, false);
        (void)BSP_Display_SetReset(handle, true);
        *device = (Device_ST7735S){0};
        return (result != DEVICE_ST7735S_RESULT_OK) ? result :
            DEVICE_ST7735S_RESULT_BACKEND_ERROR;
    }
    device->initialized = true;
    return DEVICE_ST7735S_RESULT_OK;
}

Device_ST7735S_Result Device_ST7735S_SetRotation(
    Device_ST7735S *device, Device_ST7735S_Rotation rotation)
{
    if ((device == NULL) || !device->initialized ||
        !valid_rotation(rotation)) {
        return DEVICE_ST7735S_RESULT_INVALID_ARGUMENT;
    }
    return apply_rotation(device, rotation);
}

Device_ST7735S_Result Device_ST7735S_SetBacklight(
    Device_ST7735S *device, bool enabled)
{
    if ((device == NULL) || !device->initialized) {
        return DEVICE_ST7735S_RESULT_INVALID_ARGUMENT;
    }
    return BSP_Display_SetBacklight(
        (BSP_Display_Handle)device->backend, enabled) ?
        DEVICE_ST7735S_RESULT_OK :
        DEVICE_ST7735S_RESULT_BACKEND_ERROR;
}

Device_ST7735S_Result Device_ST7735S_DrawPixel(
    Device_ST7735S *device,
    uint16_t x,
    uint16_t y,
    uint16_t rgb565)
{
    return Device_ST7735S_WriteRGB565(
        device, x, y, x, y, &rgb565, 1U);
}

Device_ST7735S_Result Device_ST7735S_Fill(
    Device_ST7735S *device,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    uint16_t rgb565)
{
    uint8_t bytes[ST7735S_PIXEL_CHUNK_COUNT * 2U];
    size_t remaining;
    Device_ST7735S_Result result;

    result = set_window(device, x0, y0, x1, y1);
    if (result != DEVICE_ST7735S_RESULT_OK) {
        return result;
    }
    for (size_t index = 0U;
         index < ST7735S_PIXEL_CHUNK_COUNT;
         ++index) {
        bytes[index * 2U] = (uint8_t)(rgb565 >> 8U);
        bytes[index * 2U + 1U] = (uint8_t)rgb565;
    }
    remaining = (size_t)(x1 - x0 + 1U) * (size_t)(y1 - y0 + 1U);
    while (remaining != 0U) {
        size_t chunk = (remaining > ST7735S_PIXEL_CHUNK_COUNT) ?
            ST7735S_PIXEL_CHUNK_COUNT : remaining;

        result = write_bytes(device, true, bytes, chunk * 2U);
        if (result != DEVICE_ST7735S_RESULT_OK) {
            return result;
        }
        remaining -= chunk;
    }
    return DEVICE_ST7735S_RESULT_OK;
}

Device_ST7735S_Result Device_ST7735S_WriteRGB565(
    Device_ST7735S *device,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    const uint16_t *pixels,
    size_t pixel_count)
{
    uint8_t bytes[ST7735S_PIXEL_CHUNK_COUNT * 2U];
    size_t expected_count;
    size_t offset = 0U;
    Device_ST7735S_Result result;

    if ((pixels == NULL) || (x0 > x1) || (y0 > y1)) {
        return DEVICE_ST7735S_RESULT_INVALID_ARGUMENT;
    }
    expected_count =
        (size_t)(x1 - x0 + 1U) * (size_t)(y1 - y0 + 1U);
    if (pixel_count != expected_count) {
        return DEVICE_ST7735S_RESULT_INVALID_ARGUMENT;
    }
    result = set_window(device, x0, y0, x1, y1);
    if (result != DEVICE_ST7735S_RESULT_OK) {
        return result;
    }

    while (offset < pixel_count) {
        size_t chunk = pixel_count - offset;

        if (chunk > ST7735S_PIXEL_CHUNK_COUNT) {
            chunk = ST7735S_PIXEL_CHUNK_COUNT;
        }
        for (size_t index = 0U; index < chunk; ++index) {
            const uint16_t pixel = pixels[offset + index];

            bytes[index * 2U] = (uint8_t)(pixel >> 8U);
            bytes[index * 2U + 1U] = (uint8_t)pixel;
        }
        result = write_bytes(device, true, bytes, chunk * 2U);
        if (result != DEVICE_ST7735S_RESULT_OK) {
            return result;
        }
        offset += chunk;
    }
    return DEVICE_ST7735S_RESULT_OK;
}

bool Device_ST7735S_GetSize(
    const Device_ST7735S *device,
    uint16_t *width,
    uint16_t *height)
{
    if ((device == NULL) || !device->initialized ||
        (width == NULL) || (height == NULL)) {
        return false;
    }
    *width = device->width;
    *height = device->height;
    return true;
}

uint16_t Device_ST7735S_Color565(
    uint8_t red, uint8_t green, uint8_t blue)
{
    return (uint16_t)((((uint16_t)red & UINT16_C(0xF8)) << 8U) |
        (((uint16_t)green & UINT16_C(0xFC)) << 3U) |
        ((uint16_t)blue >> 3U));
}
