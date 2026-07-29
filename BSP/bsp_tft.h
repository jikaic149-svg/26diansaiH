#ifndef BSP_TFT_H
#define BSP_TFT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef const struct BSP_TFT_Resource *BSP_TFT_Handle;

typedef enum {
    BSP_TFT_RESULT_OK = 0,
    BSP_TFT_RESULT_BUSY,
    BSP_TFT_RESULT_NO_COMPLETION,
    BSP_TFT_RESULT_INVALID_ARGUMENT,
    BSP_TFT_RESULT_IO_ERROR,
} BSP_TFT_Result;

typedef struct {
    uint32_t bit_rate;
    size_t maximum_transfer_length;
    bool supports_tx_dma;
} BSP_TFT_Info;

/* Return the TFT transports wired by this board BSP. */
size_t BSP_TFT_Count(void);
BSP_TFT_Handle BSP_TFT_GetHandle(size_t index);
bool BSP_TFT_GetInfo(BSP_TFT_Handle tft, BSP_TFT_Info *info);

/* released=false asserts the active-low hardware reset signal. */
bool BSP_TFT_SetReset(BSP_TFT_Handle tft, bool released);

/* Start one complete 4-wire SPI write. data_mode=false sends a command byte,
 * data_mode=true sends command parameters or display RAM data. The caller owns
 * data until Poll reports a terminal result. This function never waits. */
BSP_TFT_Result BSP_TFT_StartWrite(
    BSP_TFT_Handle tft,
    bool data_mode,
    const uint8_t *data,
    size_t length);

/* Finalize a DMA transfer only after the SPI shift register is idle. BUSY is
 * an in-progress transfer; NO_COMPLETION means no transfer is pending. */
BSP_TFT_Result BSP_TFT_Poll(
    BSP_TFT_Handle tft, size_t *transferred);
bool BSP_TFT_IsBusy(BSP_TFT_Handle tft);

#endif
