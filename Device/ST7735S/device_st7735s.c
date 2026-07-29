#include "device_st7735s.h"

#include <string.h>

typedef struct {
    uint8_t command;
    uint8_t data_length;
    uint8_t data[16];
} st7735s_init_command_t;

/* Module-specific sequence from the supplied 1.8-inch ST7735S specification.
 * Portrait direction 1 uses MADCTL=0xC0 and RAM offsets X+2, Y+1. */
static const st7735s_init_command_t s_init_commands[] = {
    { 0xB1U, 3U, { 0x05U, 0x3CU, 0x3CU } },
    { 0xB2U, 3U, { 0x05U, 0x3CU, 0x3CU } },
    { 0xB3U, 6U, { 0x05U, 0x3CU, 0x3CU, 0x05U, 0x3CU, 0x3CU } },
    { 0xB4U, 1U, { 0x03U } },
    { 0xC0U, 3U, { 0x28U, 0x08U, 0x04U } },
    { 0xC1U, 1U, { 0xC0U } },
    { 0xC2U, 2U, { 0x0DU, 0x00U } },
    { 0xC3U, 2U, { 0x8DU, 0x2AU } },
    { 0xC4U, 2U, { 0x8DU, 0xEEU } },
    { 0xC5U, 1U, { 0x1AU } },
    { 0x36U, 1U, { 0xC0U } },
    { 0xE0U, 16U, {
        0x04U, 0x22U, 0x07U, 0x0AU, 0x2EU, 0x30U, 0x25U, 0x2AU,
        0x28U, 0x26U, 0x2EU, 0x3AU, 0x00U, 0x01U, 0x03U, 0x13U,
    } },
    { 0xE1U, 16U, {
        0x04U, 0x16U, 0x06U, 0x0DU, 0x2DU, 0x26U, 0x23U, 0x27U,
        0x27U, 0x25U, 0x2DU, 0x3BU, 0x00U, 0x01U, 0x04U, 0x13U,
    } },
    { 0x3AU, 1U, { 0x05U } }, /* RGB565, 65K colors */
    { 0x29U, 0U, { 0U } },    /* Display on */
};

#define ST7735S_INIT_COMMAND_COUNT \
    (sizeof(s_init_commands) / sizeof(s_init_commands[0]))

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool start_write(
    Device_ST7735S *display,
    bool data_mode,
    const uint8_t *data,
    size_t length,
    Device_ST7735S_State next_state)
{
    if (BSP_TFT_StartWrite(display->io, data_mode, data, length) !=
        BSP_TFT_RESULT_OK) {
        display->state = DEVICE_ST7735S_STATE_ERROR;
        return false;
    }
    display->transfer_pending = true;
    display->state = next_state;
    return true;
}

static bool poll_transfer(Device_ST7735S *display)
{
    size_t transferred = 0U;
    const BSP_TFT_Result result =
        BSP_TFT_Poll(display->io, &transferred);

    if (result == BSP_TFT_RESULT_BUSY) {
        return false;
    }
    display->transfer_pending = false;
    if ((result != BSP_TFT_RESULT_OK) || (transferred == 0U)) {
        display->state = DEVICE_ST7735S_STATE_ERROR;
    }
    return true;
}

bool Device_ST7735S_Init(
    Device_ST7735S *display, BSP_TFT_Handle io, uint32_t now_ms)
{
    BSP_TFT_Info info;

    if ((display == NULL) || (io == NULL) ||
        !BSP_TFT_GetInfo(io, &info) || !info.supports_tx_dma ||
        (info.maximum_transfer_length < DEVICE_ST7735S_FILL_BYTES)) {
        return false;
    }

    memset(display, 0, sizeof(*display));
    display->io = io;
    if (!BSP_TFT_SetReset(io, false)) {
        return false;
    }
    display->deadline_ms = now_ms + UINT32_C(100);
    display->state = DEVICE_ST7735S_STATE_RESET_LOW_WAIT;
    return true;
}

bool Device_ST7735S_Step(Device_ST7735S *display, uint32_t now_ms)
{
    const st7735s_init_command_t *entry;

    if ((display == NULL) || (display->io == NULL) ||
        (display->state == DEVICE_ST7735S_STATE_UNINITIALIZED) ||
        (display->state == DEVICE_ST7735S_STATE_ERROR)) {
        return false;
    }

    if (display->transfer_pending && !poll_transfer(display)) {
        return true;
    }
    if (display->state == DEVICE_ST7735S_STATE_ERROR) {
        return false;
    }

    switch (display->state) {
    case DEVICE_ST7735S_STATE_RESET_LOW_WAIT:
        if (deadline_reached(now_ms, display->deadline_ms)) {
            if (!BSP_TFT_SetReset(display->io, true)) {
                display->state = DEVICE_ST7735S_STATE_ERROR;
                return false;
            }
            display->deadline_ms = now_ms + UINT32_C(100);
            display->state = DEVICE_ST7735S_STATE_RESET_HIGH_WAIT;
        }
        break;

    case DEVICE_ST7735S_STATE_RESET_HIGH_WAIT:
        if (deadline_reached(now_ms, display->deadline_ms)) {
            display->command_byte = 0x11U;
            return start_write(
                display, false, &display->command_byte, 1U,
                DEVICE_ST7735S_STATE_SLEEP_OUT_COMMAND);
        }
        break;

    case DEVICE_ST7735S_STATE_SLEEP_OUT_COMMAND:
        display->deadline_ms = now_ms + UINT32_C(120);
        display->state = DEVICE_ST7735S_STATE_SLEEP_OUT_WAIT;
        break;

    case DEVICE_ST7735S_STATE_SLEEP_OUT_WAIT:
        if (deadline_reached(now_ms, display->deadline_ms)) {
            display->init_index = 0U;
            display->state = DEVICE_ST7735S_STATE_INIT_COMMAND;
        }
        break;

    case DEVICE_ST7735S_STATE_INIT_COMMAND:
        entry = &s_init_commands[display->init_index];
        display->command_byte = entry->command;
        return start_write(
            display, false, &display->command_byte, 1U,
            DEVICE_ST7735S_STATE_INIT_DATA);

    case DEVICE_ST7735S_STATE_INIT_DATA:
        entry = &s_init_commands[display->init_index];
        if (entry->data_length == 0U) {
            display->state = DEVICE_ST7735S_STATE_INIT_NEXT;
        }
        else {
            return start_write(
                display, true, entry->data, entry->data_length,
                DEVICE_ST7735S_STATE_INIT_NEXT);
        }
        break;

    case DEVICE_ST7735S_STATE_INIT_NEXT:
        ++display->init_index;
        display->state =
            (display->init_index < ST7735S_INIT_COMMAND_COUNT) ?
                DEVICE_ST7735S_STATE_INIT_COMMAND :
                DEVICE_ST7735S_STATE_READY;
        break;

    case DEVICE_ST7735S_STATE_READY:
        break;

    case DEVICE_ST7735S_STATE_FILL_COLUMN_COMMAND:
        display->command_byte = 0x2AU;
        return start_write(
            display, false, &display->command_byte, 1U,
            DEVICE_ST7735S_STATE_FILL_COLUMN_DATA);

    case DEVICE_ST7735S_STATE_FILL_COLUMN_DATA:
        display->address_data[0] = 0x00U;
        display->address_data[1] = 0x02U;
        display->address_data[2] = 0x00U;
        display->address_data[3] = 0x81U;
        return start_write(
            display, true, display->address_data,
            sizeof(display->address_data),
            DEVICE_ST7735S_STATE_FILL_ROW_COMMAND);

    case DEVICE_ST7735S_STATE_FILL_ROW_COMMAND:
        display->command_byte = 0x2BU;
        return start_write(
            display, false, &display->command_byte, 1U,
            DEVICE_ST7735S_STATE_FILL_ROW_DATA);

    case DEVICE_ST7735S_STATE_FILL_ROW_DATA:
        display->address_data[0] = 0x00U;
        display->address_data[1] = 0x01U;
        display->address_data[2] = 0x00U;
        display->address_data[3] = 0xA0U;
        return start_write(
            display, true, display->address_data,
            sizeof(display->address_data),
            DEVICE_ST7735S_STATE_FILL_MEMORY_COMMAND);

    case DEVICE_ST7735S_STATE_FILL_MEMORY_COMMAND:
        display->command_byte = 0x2CU;
        return start_write(
            display, false, &display->command_byte, 1U,
            DEVICE_ST7735S_STATE_FILL_PIXELS);

    case DEVICE_ST7735S_STATE_FILL_PIXELS:
        if (display->rows_written >= DEVICE_ST7735S_HEIGHT) {
            display->state = DEVICE_ST7735S_STATE_READY;
            break;
        }
        display->rows_written += DEVICE_ST7735S_FILL_ROWS;
        return start_write(
            display, true, display->fill_data,
            sizeof(display->fill_data),
            DEVICE_ST7735S_STATE_FILL_PIXELS);

    default:
        display->state = DEVICE_ST7735S_STATE_ERROR;
        return false;
    }

    return true;
}

bool Device_ST7735S_FillScreen(
    Device_ST7735S *display, uint16_t rgb565)
{
    if ((display == NULL) ||
        (display->state != DEVICE_ST7735S_STATE_READY) ||
        display->transfer_pending) {
        return false;
    }

    for (size_t index = 0U;
         index < DEVICE_ST7735S_FILL_BYTES;
         index += 2U) {
        display->fill_data[index] = (uint8_t)(rgb565 >> 8U);
        display->fill_data[index + 1U] = (uint8_t)rgb565;
    }
    display->rows_written = 0U;
    display->state = DEVICE_ST7735S_STATE_FILL_COLUMN_COMMAND;
    return true;
}

bool Device_ST7735S_IsInitialized(const Device_ST7735S *display)
{
    return (display != NULL) &&
           (display->state >= DEVICE_ST7735S_STATE_READY) &&
           (display->state < DEVICE_ST7735S_STATE_ERROR);
}

bool Device_ST7735S_IsBusy(const Device_ST7735S *display)
{
    return (display != NULL) &&
           (display->state != DEVICE_ST7735S_STATE_READY) &&
           (display->state != DEVICE_ST7735S_STATE_ERROR);
}

bool Device_ST7735S_HasError(const Device_ST7735S *display)
{
    return (display == NULL) ||
           (display->state == DEVICE_ST7735S_STATE_ERROR);
}
