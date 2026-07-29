#ifndef BSP_DISPLAY_H
#define BSP_DISPLAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef const struct BSP_Display_Resource *BSP_Display_Handle;

typedef enum {
    BSP_DISPLAY_RESULT_OK = 0,
    BSP_DISPLAY_RESULT_INVALID_ARGUMENT,
    BSP_DISPLAY_RESULT_BUSY,
    BSP_DISPLAY_RESULT_TIMEOUT,
} BSP_Display_Result;

typedef struct {
    uint32_t bit_rate_hz;
} BSP_Display_Info;

/* Initialize display control pins to reset asserted and backlight off. */
void BSP_Display_Init(void);

size_t BSP_Display_Count(void);
BSP_Display_Handle BSP_Display_GetHandle(size_t index);
bool BSP_Display_GetInfo(
    BSP_Display_Handle display, BSP_Display_Info *info);

/* reset_asserted hides the board's active-low RESET polarity. */
bool BSP_Display_SetReset(
    BSP_Display_Handle display, bool reset_asserted);

/* enabled hides the board's active-high backlight polarity. */
bool BSP_Display_SetBacklight(
    BSP_Display_Handle display, bool enabled);

/* Write command bytes (data_mode=false) or parameter/pixel bytes
 * (data_mode=true). CS remains active for the complete finite transfer. */
BSP_Display_Result BSP_Display_Write(
    BSP_Display_Handle display,
    bool data_mode,
    const uint8_t *bytes,
    size_t length,
    uint32_t timeout_ms);

#endif
