#include "bsp_display.h"

#include <stddef.h>

#include "bsp_resource_map.h"
#include "bsp_spi.h"

#define BSP_DISPLAY_COUNT 1U
#define BSP_DISPLAY_SPI_INDEX 0U
#define BSP_DISPLAY_SPI_CHIP_SELECT_INDEX 0U

struct BSP_Display_Resource {
    size_t spi_index;
    GPIO_Regs *data_command_port;
    uint32_t data_command_pin;
    GPIO_Regs *reset_port;
    uint32_t reset_pin;
    GPIO_Regs *backlight_port;
    uint32_t backlight_pin;
    bool busy;
};

static struct BSP_Display_Resource g_display[BSP_DISPLAY_COUNT] = {
    {
        .spi_index = BSP_DISPLAY_SPI_INDEX,
        .data_command_port = BSP_RESOURCE_TFT_DC_PORT,
        .data_command_pin = BSP_RESOURCE_TFT_DC_PIN,
        .reset_port = BSP_RESOURCE_TFT_RESET_PORT,
        .reset_pin = BSP_RESOURCE_TFT_RESET_PIN,
        .backlight_port = BSP_RESOURCE_TFT_BACKLIGHT_PORT,
        .backlight_pin = BSP_RESOURCE_TFT_BACKLIGHT_PIN,
    },
};

static struct BSP_Display_Resource *display_from_handle(
    BSP_Display_Handle display)
{
    for (size_t index = 0U; index < BSP_DISPLAY_COUNT; ++index) {
        if (display == &g_display[index]) {
            return &g_display[index];
        }
    }
    return NULL;
}

void BSP_Display_Init(void)
{
    for (size_t index = 0U; index < BSP_DISPLAY_COUNT; ++index) {
        DL_GPIO_clearPins(
            g_display[index].data_command_port,
            g_display[index].data_command_pin);
        DL_GPIO_clearPins(
            g_display[index].reset_port,
            g_display[index].reset_pin);
        DL_GPIO_clearPins(
            g_display[index].backlight_port,
            g_display[index].backlight_pin);
        g_display[index].busy = false;
    }
}

size_t BSP_Display_Count(void)
{
    return BSP_DISPLAY_COUNT;
}

BSP_Display_Handle BSP_Display_GetHandle(size_t index)
{
    return (index < BSP_DISPLAY_COUNT) ? &g_display[index] : NULL;
}

bool BSP_Display_GetInfo(
    BSP_Display_Handle display, BSP_Display_Info *info)
{
    struct BSP_Display_Resource *resource =
        display_from_handle(display);
    BSP_SPI_Info spi_info;
    BSP_SPI_Handle spi;

    if ((resource == NULL) || (info == NULL)) {
        return false;
    }
    spi = BSP_SPI_GetHandle(resource->spi_index);
    if ((spi == NULL) || !BSP_SPI_GetInfo(spi, &spi_info) ||
        spi_info.receive_supported ||
        (spi_info.chip_select_count <=
            BSP_DISPLAY_SPI_CHIP_SELECT_INDEX)) {
        return false;
    }
    *info = (BSP_Display_Info){
        .bit_rate_hz = spi_info.bit_rate_hz,
    };
    return true;
}

bool BSP_Display_SetReset(
    BSP_Display_Handle display, bool reset_asserted)
{
    struct BSP_Display_Resource *resource =
        display_from_handle(display);

    if ((resource == NULL) || resource->busy) {
        return false;
    }
    if (reset_asserted) {
        DL_GPIO_clearPins(resource->reset_port, resource->reset_pin);
    } else {
        DL_GPIO_setPins(resource->reset_port, resource->reset_pin);
    }
    return true;
}

bool BSP_Display_SetBacklight(
    BSP_Display_Handle display, bool enabled)
{
    struct BSP_Display_Resource *resource =
        display_from_handle(display);

    if ((resource == NULL) || resource->busy) {
        return false;
    }
    if (enabled) {
        DL_GPIO_setPins(
            resource->backlight_port, resource->backlight_pin);
    } else {
        DL_GPIO_clearPins(
            resource->backlight_port, resource->backlight_pin);
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
    struct BSP_Display_Resource *resource =
        display_from_handle(display);
    BSP_SPI_Result spi_result;
    BSP_SPI_Handle spi;

    if ((resource == NULL) || ((bytes == NULL) && (length != 0U)) ||
        (timeout_ms == 0U)) {
        return BSP_DISPLAY_RESULT_INVALID_ARGUMENT;
    }
    if (length == 0U) {
        return BSP_DISPLAY_RESULT_OK;
    }
    if (resource->busy) {
        return BSP_DISPLAY_RESULT_BUSY;
    }
    spi = BSP_SPI_GetHandle(resource->spi_index);
    if (spi == NULL) {
        return BSP_DISPLAY_RESULT_INVALID_ARGUMENT;
    }

    resource->busy = true;
    if (data_mode) {
        DL_GPIO_setPins(
            resource->data_command_port,
            resource->data_command_pin);
    } else {
        DL_GPIO_clearPins(
            resource->data_command_port,
            resource->data_command_pin);
    }
    spi_result = BSP_SPI_Transfer(spi,
        BSP_DISPLAY_SPI_CHIP_SELECT_INDEX,
        bytes, NULL, length, timeout_ms);
    resource->busy = false;

    switch (spi_result) {
        case BSP_SPI_RESULT_OK:
            return BSP_DISPLAY_RESULT_OK;
        case BSP_SPI_RESULT_BUSY:
            return BSP_DISPLAY_RESULT_BUSY;
        case BSP_SPI_RESULT_TIMEOUT:
            return BSP_DISPLAY_RESULT_TIMEOUT;
        default:
            return BSP_DISPLAY_RESULT_INVALID_ARGUMENT;
    }
}
