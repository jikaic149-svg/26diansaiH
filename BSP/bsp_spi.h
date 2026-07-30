#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef const struct BSP_SPI_Resource *BSP_SPI_Handle;

/* Called once from the DMA ISR when a transfer reaches a terminal DMA state.
 * Keep the callback ISR-safe and short; the Task must still call Poll to
 * release chip select and obtain the final result. */
typedef void (*BSP_SPI_CompletionCallbackFromISR)(void *context);

typedef enum {
    BSP_SPI_RESULT_OK = 0,
    BSP_SPI_RESULT_NO_TRANSFER,
    BSP_SPI_RESULT_BUSY,
    BSP_SPI_RESULT_INVALID_ARGUMENT,
    BSP_SPI_RESULT_IO_ERROR,
} BSP_SPI_Result;

typedef struct {
    uint32_t bit_rate_hz;
    size_t chip_select_count;
    size_t maximum_transfer_length;
} BSP_SPI_Info;

size_t BSP_SPI_Count(void);
BSP_SPI_Handle BSP_SPI_GetHandle(size_t index);
bool BSP_SPI_GetInfo(BSP_SPI_Handle spi, BSP_SPI_Info *info);

/* Register the single owner's DMA completion notification before starting a
 * transfer. Passing NULL unregisters it. Registration fails while active. */
bool BSP_SPI_SetCompletionCallbackFromISR(
    BSP_SPI_Handle spi,
    BSP_SPI_CompletionCallbackFromISR callback,
    void *context);

/* Start one full-duplex DMA transaction. The caller owns both buffers until
 * Poll returns a terminal result. This function and Poll never wait. */
BSP_SPI_Result BSP_SPI_StartTransfer(
    BSP_SPI_Handle spi,
    size_t chip_select,
    const uint8_t *tx_data,
    uint8_t *rx_data,
    size_t length);

/* BUSY means DMA or the final SPI shift is still active. OK/IO_ERROR end the
 * transaction and release chip select; transferred is required. */
BSP_SPI_Result BSP_SPI_Poll(
    BSP_SPI_Handle spi,
    size_t *transferred);

bool BSP_SPI_Abort(BSP_SPI_Handle spi);

#endif
