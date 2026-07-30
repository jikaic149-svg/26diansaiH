#ifndef DEVICE_BMI088_H
#define DEVICE_BMI088_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_spi.h"

#define DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT 0U
#define DEVICE_BMI088_GYROSCOPE_CHIP_SELECT     1U
#define DEVICE_BMI088_TARGET_FRAME_RATE_HZ      UINT32_C(1000)
#define DEVICE_BMI088_FRAME_PERIOD_MS           UINT32_C(1)

typedef enum {
    DEVICE_BMI088_STATUS_UNINITIALIZED = 0,
    DEVICE_BMI088_STATUS_INITIALIZING,
    DEVICE_BMI088_STATUS_ONLINE,
    DEVICE_BMI088_STATUS_INVALID_ARGUMENT,
    DEVICE_BMI088_STATUS_BSP_UNAVAILABLE,
    DEVICE_BMI088_STATUS_TRANSFER_TIMEOUT,
    DEVICE_BMI088_STATUS_TRANSPORT_ERROR,
    DEVICE_BMI088_STATUS_ACCELEROMETER_ID_MISMATCH,
    DEVICE_BMI088_STATUS_GYROSCOPE_ID_MISMATCH,
    DEVICE_BMI088_STATUS_CONFIGURATION_MISMATCH,
} Device_BMI088_Status;

typedef struct {
    size_t transport_index;

    /* Zero accepts any non-zero BMI088-compatible SPI rate up to 10 MHz. */
    uint32_t expected_bit_rate_hz;

    /* Bounds both waiting for a shared transport and an active DMA transfer. */
    uint32_t transfer_timeout_ms;
} Device_BMI088_Config;

typedef struct {
    float acceleration_m_s2[3];
    float angular_velocity_rad_s[3];
    uint32_t sequence;

    /* Timestamp at the start of the accelerometer transaction for this frame. */
    uint32_t sample_ms;
    Device_BMI088_Status status;
    bool online;
} Device_BMI088_Snapshot;

typedef struct {
    Device_BMI088_Config config;
    Device_BMI088_Snapshot snapshot;
    BSP_SPI_Handle spi;
    float pending_acceleration_m_s2[3];
    uint32_t deadline_ms;
    uint32_t state_started_ms;
    uint32_t transfer_started_ms;
    uint32_t next_frame_ms;
    uint32_t pending_sample_ms;
    uint32_t transfer_sequence;
    size_t transfer_length;
    uint8_t internal_state;
    uint8_t configuration_index;
    bool transfer_pending;
    bool initialized;

    /* Fixed-lifetime, DMA-safe transaction storage owned by this instance. */
    _Alignas(4) uint8_t tx_buffer[8];
    _Alignas(4) uint8_t rx_buffer[8];
} Device_BMI088;

Device_BMI088_Config Device_BMI088_DefaultConfig(void);

/* Begin the non-blocking power-on and register initialization sequence. */
bool Device_BMI088_Init(
    Device_BMI088 *device,
    const Device_BMI088_Config *config,
    uint32_t now_ms);

/* Advance initialization or acquisition without waiting. The owning Task
 * waits for each DMA terminal notification with a finite timeout, then calls
 * this function to poll completion and start the next ACC/GYRO transaction. */
bool Device_BMI088_Step(Device_BMI088 *device, uint32_t now_ms);

bool Device_BMI088_GetSnapshot(
    const Device_BMI088 *device, Device_BMI088_Snapshot *snapshot);
bool Device_BMI088_IsTransferPending(const Device_BMI088 *device);
uint32_t Device_BMI088_GetTransferSequence(const Device_BMI088 *device);
Device_BMI088_Status Device_BMI088_GetStatus(const Device_BMI088 *device);
bool Device_BMI088_IsOnline(const Device_BMI088 *device);

#endif
