#ifndef DEVICE_ST7735S_H
#define DEVICE_ST7735S_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_tft.h"

#define DEVICE_ST7735S_WIDTH          128U
#define DEVICE_ST7735S_HEIGHT         160U
#define DEVICE_ST7735S_FILL_ROWS      4U
#define DEVICE_ST7735S_FILL_BYTES     \
    (DEVICE_ST7735S_WIDTH * DEVICE_ST7735S_FILL_ROWS * 2U)

typedef enum {
    DEVICE_ST7735S_STATE_UNINITIALIZED = 0,
    DEVICE_ST7735S_STATE_RESET_LOW_WAIT,
    DEVICE_ST7735S_STATE_RESET_HIGH_WAIT,
    DEVICE_ST7735S_STATE_SLEEP_OUT_COMMAND,
    DEVICE_ST7735S_STATE_SLEEP_OUT_WAIT,
    DEVICE_ST7735S_STATE_INIT_COMMAND,
    DEVICE_ST7735S_STATE_INIT_DATA,
    DEVICE_ST7735S_STATE_INIT_NEXT,
    DEVICE_ST7735S_STATE_READY,
    DEVICE_ST7735S_STATE_FILL_COLUMN_COMMAND,
    DEVICE_ST7735S_STATE_FILL_COLUMN_DATA,
    DEVICE_ST7735S_STATE_FILL_ROW_COMMAND,
    DEVICE_ST7735S_STATE_FILL_ROW_DATA,
    DEVICE_ST7735S_STATE_FILL_MEMORY_COMMAND,
    DEVICE_ST7735S_STATE_FILL_PIXELS,
    DEVICE_ST7735S_STATE_ERROR,
} Device_ST7735S_State;

typedef struct {
    BSP_TFT_Handle io;
    Device_ST7735S_State state;
    uint32_t deadline_ms;
    size_t init_index;
    size_t rows_written;
    bool transfer_pending;
    uint8_t command_byte;
    uint8_t address_data[4];
    uint8_t fill_data[DEVICE_ST7735S_FILL_BYTES];
} Device_ST7735S;

/* Starts the datasheet reset/initialization sequence without waiting. */
bool Device_ST7735S_Init(
    Device_ST7735S *display, BSP_TFT_Handle io, uint32_t now_ms);

/* Advance at most one asynchronous transfer/state transition. */
bool Device_ST7735S_Step(Device_ST7735S *display, uint32_t now_ms);

/* Queueing belongs to APP. FillScreen accepts only an initialized idle panel. */
bool Device_ST7735S_FillScreen(
    Device_ST7735S *display, uint16_t rgb565);
bool Device_ST7735S_IsInitialized(const Device_ST7735S *display);
bool Device_ST7735S_IsBusy(const Device_ST7735S *display);
bool Device_ST7735S_HasError(const Device_ST7735S *display);

#endif
