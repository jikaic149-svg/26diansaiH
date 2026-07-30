#include "device_bmi088.h"

#include <limits.h>
#include <string.h>

#define BMI088_MAX_SPI_BIT_RATE_HZ UINT32_C(10000000)
#define BMI088_DEFAULT_TIMEOUT_MS  UINT32_C(5)

#define BMI088_ACCEL_CHIP_ID_REG   UINT8_C(0x00)
#define BMI088_ACCEL_CHIP_ID_VALUE UINT8_C(0x1E)
#define BMI088_ACCEL_DATA_REG      UINT8_C(0x12)
#define BMI088_ACCEL_CONF_REG      UINT8_C(0x40)
#define BMI088_ACCEL_RANGE_REG     UINT8_C(0x41)
#define BMI088_ACCEL_PWR_CONF_REG  UINT8_C(0x7C)
#define BMI088_ACCEL_PWR_CTRL_REG  UINT8_C(0x7D)
#define BMI088_ACCEL_RESET_REG     UINT8_C(0x7E)

#define BMI088_GYRO_CHIP_ID_REG    UINT8_C(0x00)
#define BMI088_GYRO_CHIP_ID_VALUE  UINT8_C(0x0F)
#define BMI088_GYRO_DATA_REG       UINT8_C(0x02)
#define BMI088_GYRO_RANGE_REG      UINT8_C(0x0F)
#define BMI088_GYRO_BANDWIDTH_REG  UINT8_C(0x10)
#define BMI088_GYRO_POWER_REG      UINT8_C(0x11)
#define BMI088_GYRO_RESET_REG      UINT8_C(0x14)

#define BMI088_SOFT_RESET_VALUE    UINT8_C(0xB6)
#define BMI088_ACCEL_CONF_VALUE    UINT8_C(0x8C)
#define BMI088_ACCEL_RANGE_6G      UINT8_C(0x01)
#define BMI088_ACCEL_ACTIVE        UINT8_C(0x00)
#define BMI088_ACCEL_ENABLE        UINT8_C(0x04)
#define BMI088_GYRO_RANGE_500_DPS  UINT8_C(0x02)
#define BMI088_GYRO_1000_116_VALUE UINT8_C(0x82)
#define BMI088_GYRO_NORMAL         UINT8_C(0x00)

#define BMI088_POWER_ON_DELAY_MS      UINT32_C(10)
#define BMI088_ACCEL_RESET_DELAY_MS   UINT32_C(1)
#define BMI088_ACCEL_POWER_DELAY_MS   UINT32_C(5)
#define BMI088_ACCEL_CONFIG_DELAY_MS  UINT32_C(40)
#define BMI088_GYRO_RESET_DELAY_MS    UINT32_C(30)
#define BMI088_GYRO_POWER_DELAY_MS    UINT32_C(30)
#define BMI088_GYRO_CONFIG_DELAY_MS   UINT32_C(10)

#define BMI088_GRAVITY_M_S2        9.80665F
#define BMI088_DEG_TO_RAD          0.01745329251994329577F
#define BMI088_ACCEL_SCALE_M_S2    \
    ((6.0F * BMI088_GRAVITY_M_S2) / 32768.0F)
#define BMI088_GYRO_SCALE_RAD_S    \
    ((500.0F * BMI088_DEG_TO_RAD) / 32768.0F)

typedef enum {
    BMI088_STATE_UNINITIALIZED = 0,
    BMI088_STATE_POWER_ON_WAIT,
    BMI088_STATE_ACCEL_INITIAL_DUMMY_ID,
    BMI088_STATE_ACCEL_INITIAL_ID,
    BMI088_STATE_GYRO_INITIAL_ID,
    BMI088_STATE_ACCEL_RESET,
    BMI088_STATE_ACCEL_RESET_WAIT,
    BMI088_STATE_ACCEL_POST_RESET_DUMMY_ID,
    BMI088_STATE_ACCEL_POST_RESET_ID,
    BMI088_STATE_ACCEL_CONFIG_WRITE,
    BMI088_STATE_ACCEL_CONFIG_WAIT,
    BMI088_STATE_ACCEL_CONFIG_READ,
    BMI088_STATE_GYRO_RESET,
    BMI088_STATE_GYRO_RESET_WAIT,
    BMI088_STATE_GYRO_POST_RESET_ID,
    BMI088_STATE_GYRO_CONFIG_WRITE,
    BMI088_STATE_GYRO_CONFIG_WAIT,
    BMI088_STATE_GYRO_CONFIG_READ,
    BMI088_STATE_READY,
    BMI088_STATE_SAMPLE_ACCEL,
    BMI088_STATE_SAMPLE_GYRO,
    BMI088_STATE_ERROR,
} bmi088_state_t;

typedef struct {
    uint8_t address;
    uint8_t value;
    uint32_t settle_ms;
} bmi088_register_config_t;

static const bmi088_register_config_t s_accel_config[] = {
    { BMI088_ACCEL_PWR_CONF_REG, BMI088_ACCEL_ACTIVE,
      BMI088_ACCEL_POWER_DELAY_MS },
    { BMI088_ACCEL_PWR_CTRL_REG, BMI088_ACCEL_ENABLE,
      BMI088_ACCEL_POWER_DELAY_MS },
    { BMI088_ACCEL_RANGE_REG, BMI088_ACCEL_RANGE_6G, UINT32_C(0) },
    { BMI088_ACCEL_CONF_REG, BMI088_ACCEL_CONF_VALUE,
      BMI088_ACCEL_CONFIG_DELAY_MS },
};

static const bmi088_register_config_t s_gyro_config[] = {
    { BMI088_GYRO_POWER_REG, BMI088_GYRO_NORMAL,
      BMI088_GYRO_POWER_DELAY_MS },
    { BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_500_DPS, UINT32_C(0) },
    { BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_1000_116_VALUE,
      BMI088_GYRO_CONFIG_DELAY_MS },
};

#define BMI088_ACCEL_CONFIG_COUNT \
    (sizeof(s_accel_config) / sizeof(s_accel_config[0]))
#define BMI088_GYRO_CONFIG_COUNT \
    (sizeof(s_gyro_config) / sizeof(s_gyro_config[0]))

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool timeout_reached(
    uint32_t now_ms, uint32_t started_ms, uint32_t timeout_ms)
{
    return (uint32_t)(now_ms - started_ms) >= timeout_ms;
}

static void set_state(
    Device_BMI088 *device, bmi088_state_t state, uint32_t now_ms)
{
    device->internal_state = (uint8_t)state;
    device->state_started_ms = now_ms;
}

static void set_status(
    Device_BMI088 *device, Device_BMI088_Status status, bool online)
{
    device->snapshot.status = status;
    device->snapshot.online = online;
}

static bool fail(
    Device_BMI088 *device,
    Device_BMI088_Status status,
    uint32_t now_ms,
    bool abort_transfer)
{
    if (abort_transfer && device->transfer_pending) {
        (void)BSP_SPI_Abort(device->spi);
    }
    device->transfer_pending = false;
    set_status(device, status, false);
    set_state(device, BMI088_STATE_ERROR, now_ms);
    return false;
}

static int16_t decode_i16_le(const uint8_t bytes[2])
{
    const uint16_t raw = (uint16_t)bytes[0] |
        (uint16_t)((uint16_t)bytes[1] << 8U);

    return (raw <= (uint16_t)INT16_MAX) ?
        (int16_t)raw : (int16_t)((int32_t)raw - INT32_C(65536));
}

static void decode_vector(
    const uint8_t bytes[6], float scale, float output[3])
{
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        output[axis] =
            (float)decode_i16_le(&bytes[(size_t)axis * 2U]) * scale;
    }
}

static size_t payload_offset(size_t chip_select)
{
    return (chip_select == DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT) ?
        2U : 1U;
}

static bool start_prepared_transfer(
    Device_BMI088 *device,
    size_t chip_select,
    size_t length,
    uint32_t now_ms)
{
    const uint32_t previous_sequence = device->transfer_sequence;
    uint32_t next_sequence = previous_sequence + UINT32_C(1);

    if (next_sequence == 0U) {
        next_sequence = 1U;
    }
    /* Publish the transaction identity before DMA can raise its ISR. */
    device->transfer_sequence = next_sequence;
    const BSP_SPI_Result result = BSP_SPI_StartTransfer(
        device->spi, chip_select,
        device->tx_buffer, device->rx_buffer, length);

    if (result == BSP_SPI_RESULT_BUSY) {
        device->transfer_sequence = previous_sequence;
        if (timeout_reached(now_ms, device->state_started_ms,
                device->config.transfer_timeout_ms)) {
            return fail(device, DEVICE_BMI088_STATUS_TRANSFER_TIMEOUT,
                now_ms, false);
        }
        return true;
    }
    if (result != BSP_SPI_RESULT_OK) {
        device->transfer_sequence = previous_sequence;
        return fail(device, DEVICE_BMI088_STATUS_TRANSPORT_ERROR,
            now_ms, false);
    }
    device->transfer_pending = true;
    device->transfer_length = length;
    device->transfer_started_ms = now_ms;
    return true;
}

static bool start_read(
    Device_BMI088 *device,
    size_t chip_select,
    uint8_t register_address,
    size_t payload_length,
    uint32_t now_ms)
{
    const size_t length = payload_offset(chip_select) + payload_length;

    memset(device->tx_buffer, 0, length);
    memset(device->rx_buffer, 0, length);
    device->tx_buffer[0] = (uint8_t)(register_address | UINT8_C(0x80));
    return start_prepared_transfer(
        device, chip_select, length, now_ms);
}

static bool start_write(
    Device_BMI088 *device,
    size_t chip_select,
    uint8_t register_address,
    uint8_t value,
    uint32_t now_ms)
{
    device->tx_buffer[0] = (uint8_t)(register_address & UINT8_C(0x7F));
    device->tx_buffer[1] = value;
    device->rx_buffer[0] = 0U;
    device->rx_buffer[1] = 0U;
    return start_prepared_transfer(device, chip_select, 2U, now_ms);
}

static uint8_t read_byte(
    const Device_BMI088 *device, size_t chip_select)
{
    return device->rx_buffer[payload_offset(chip_select)];
}

static void commit_sample(Device_BMI088 *device)
{
    uint32_t next_sequence = device->snapshot.sequence + UINT32_C(1);

    if (next_sequence == 0U) {
        next_sequence = 1U;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        device->snapshot.acceleration_m_s2[axis] =
            device->pending_acceleration_m_s2[axis];
    }
    decode_vector(&device->rx_buffer[1], BMI088_GYRO_SCALE_RAD_S,
        device->snapshot.angular_velocity_rad_s);
    device->snapshot.sequence = next_sequence;
    device->snapshot.sample_ms = device->pending_sample_ms;
    set_status(device, DEVICE_BMI088_STATUS_ONLINE, true);
}

static bool complete_transfer(Device_BMI088 *device, uint32_t now_ms)
{
    const bmi088_state_t state = (bmi088_state_t)device->internal_state;
    const bmi088_register_config_t *entry;
    uint8_t value;

    switch (state) {
    case BMI088_STATE_ACCEL_INITIAL_DUMMY_ID:
        set_state(device, BMI088_STATE_ACCEL_INITIAL_ID, now_ms);
        break;

    case BMI088_STATE_ACCEL_INITIAL_ID:
        if (read_byte(device,
                DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT) !=
            BMI088_ACCEL_CHIP_ID_VALUE) {
            return fail(device,
                DEVICE_BMI088_STATUS_ACCELEROMETER_ID_MISMATCH,
                now_ms, false);
        }
        set_state(device, BMI088_STATE_GYRO_INITIAL_ID, now_ms);
        break;

    case BMI088_STATE_GYRO_INITIAL_ID:
        if (read_byte(device, DEVICE_BMI088_GYROSCOPE_CHIP_SELECT) !=
            BMI088_GYRO_CHIP_ID_VALUE) {
            return fail(device,
                DEVICE_BMI088_STATUS_GYROSCOPE_ID_MISMATCH,
                now_ms, false);
        }
        set_state(device, BMI088_STATE_ACCEL_RESET, now_ms);
        break;

    case BMI088_STATE_ACCEL_RESET:
        device->deadline_ms = now_ms + BMI088_ACCEL_RESET_DELAY_MS;
        set_state(device, BMI088_STATE_ACCEL_RESET_WAIT, now_ms);
        break;

    case BMI088_STATE_ACCEL_POST_RESET_DUMMY_ID:
        set_state(device, BMI088_STATE_ACCEL_POST_RESET_ID, now_ms);
        break;

    case BMI088_STATE_ACCEL_POST_RESET_ID:
        if (read_byte(device,
                DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT) !=
            BMI088_ACCEL_CHIP_ID_VALUE) {
            return fail(device,
                DEVICE_BMI088_STATUS_ACCELEROMETER_ID_MISMATCH,
                now_ms, false);
        }
        device->configuration_index = 0U;
        set_state(device, BMI088_STATE_ACCEL_CONFIG_WRITE, now_ms);
        break;

    case BMI088_STATE_ACCEL_CONFIG_WRITE:
        entry = &s_accel_config[device->configuration_index];
        if (entry->settle_ms > 0U) {
            device->deadline_ms = now_ms + entry->settle_ms;
            set_state(device, BMI088_STATE_ACCEL_CONFIG_WAIT, now_ms);
        }
        else {
            set_state(device, BMI088_STATE_ACCEL_CONFIG_READ, now_ms);
        }
        break;

    case BMI088_STATE_ACCEL_CONFIG_READ:
        entry = &s_accel_config[device->configuration_index];
        value = read_byte(
            device, DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT);
        if (value != entry->value) {
            return fail(device,
                DEVICE_BMI088_STATUS_CONFIGURATION_MISMATCH,
                now_ms, false);
        }
        ++device->configuration_index;
        if (device->configuration_index < BMI088_ACCEL_CONFIG_COUNT) {
            set_state(device, BMI088_STATE_ACCEL_CONFIG_WRITE, now_ms);
        }
        else {
            set_state(device, BMI088_STATE_GYRO_RESET, now_ms);
        }
        break;

    case BMI088_STATE_GYRO_RESET:
        device->deadline_ms = now_ms + BMI088_GYRO_RESET_DELAY_MS;
        set_state(device, BMI088_STATE_GYRO_RESET_WAIT, now_ms);
        break;

    case BMI088_STATE_GYRO_POST_RESET_ID:
        if (read_byte(device, DEVICE_BMI088_GYROSCOPE_CHIP_SELECT) !=
            BMI088_GYRO_CHIP_ID_VALUE) {
            return fail(device,
                DEVICE_BMI088_STATUS_GYROSCOPE_ID_MISMATCH,
                now_ms, false);
        }
        device->configuration_index = 0U;
        set_state(device, BMI088_STATE_GYRO_CONFIG_WRITE, now_ms);
        break;

    case BMI088_STATE_GYRO_CONFIG_WRITE:
        entry = &s_gyro_config[device->configuration_index];
        if (entry->settle_ms > 0U) {
            device->deadline_ms = now_ms + entry->settle_ms;
            set_state(device, BMI088_STATE_GYRO_CONFIG_WAIT, now_ms);
        }
        else {
            set_state(device, BMI088_STATE_GYRO_CONFIG_READ, now_ms);
        }
        break;

    case BMI088_STATE_GYRO_CONFIG_READ:
        entry = &s_gyro_config[device->configuration_index];
        value = read_byte(device, DEVICE_BMI088_GYROSCOPE_CHIP_SELECT);
        if (value != entry->value) {
            return fail(device,
                DEVICE_BMI088_STATUS_CONFIGURATION_MISMATCH,
                now_ms, false);
        }
        ++device->configuration_index;
        if (device->configuration_index < BMI088_GYRO_CONFIG_COUNT) {
            set_state(device, BMI088_STATE_GYRO_CONFIG_WRITE, now_ms);
        }
        else {
            device->next_frame_ms = now_ms;
            set_status(device, DEVICE_BMI088_STATUS_ONLINE, true);
            set_state(device, BMI088_STATE_READY, now_ms);
        }
        break;

    case BMI088_STATE_SAMPLE_ACCEL:
        decode_vector(&device->rx_buffer[2], BMI088_ACCEL_SCALE_M_S2,
            device->pending_acceleration_m_s2);
        set_state(device, BMI088_STATE_SAMPLE_GYRO, now_ms);
        break;

    case BMI088_STATE_SAMPLE_GYRO:
        commit_sample(device);
        set_state(device, BMI088_STATE_READY, now_ms);
        break;

    default:
        return fail(device, DEVICE_BMI088_STATUS_TRANSPORT_ERROR,
            now_ms, false);
    }
    return true;
}

static bool poll_transfer(Device_BMI088 *device, uint32_t now_ms)
{
    size_t transferred = 0U;
    const BSP_SPI_Result result = BSP_SPI_Poll(device->spi, &transferred);

    if (result == BSP_SPI_RESULT_BUSY) {
        if (timeout_reached(now_ms, device->transfer_started_ms,
                device->config.transfer_timeout_ms)) {
            return fail(device, DEVICE_BMI088_STATUS_TRANSFER_TIMEOUT,
                now_ms, true);
        }
        return true;
    }
    device->transfer_pending = false;
    if ((result != BSP_SPI_RESULT_OK) ||
        (transferred != device->transfer_length)) {
        return fail(device, DEVICE_BMI088_STATUS_TRANSPORT_ERROR,
            now_ms, false);
    }
    return complete_transfer(device, now_ms);
}

Device_BMI088_Config Device_BMI088_DefaultConfig(void)
{
    const Device_BMI088_Config config = {
        .transport_index = 0U,
        .expected_bit_rate_hz = 0U,
        .transfer_timeout_ms = BMI088_DEFAULT_TIMEOUT_MS,
    };

    return config;
}

bool Device_BMI088_Init(
    Device_BMI088 *device,
    const Device_BMI088_Config *config,
    uint32_t now_ms)
{
    BSP_SPI_Info info;
    BSP_SPI_Handle spi;

    if (device != NULL) {
        *device = (Device_BMI088){0};
    }
    if ((device == NULL) || (config == NULL) ||
        (config->transfer_timeout_ms == 0U) ||
        (config->transfer_timeout_ms > (UINT32_MAX / 2U))) {
        if (device != NULL) {
            set_status(device, DEVICE_BMI088_STATUS_INVALID_ARGUMENT, false);
        }
        return false;
    }
    if (config->transport_index >= BSP_SPI_Count()) {
        set_status(device, DEVICE_BMI088_STATUS_BSP_UNAVAILABLE, false);
        return false;
    }
    spi = BSP_SPI_GetHandle(config->transport_index);
    if ((spi == NULL) || !BSP_SPI_GetInfo(spi, &info) ||
        (info.bit_rate_hz == 0U) ||
        (info.bit_rate_hz > BMI088_MAX_SPI_BIT_RATE_HZ) ||
        (info.chip_select_count <=
            DEVICE_BMI088_GYROSCOPE_CHIP_SELECT) ||
        (info.maximum_transfer_length < sizeof(device->tx_buffer)) ||
        ((config->expected_bit_rate_hz != 0U) &&
         (info.bit_rate_hz != config->expected_bit_rate_hz))) {
        set_status(device, DEVICE_BMI088_STATUS_BSP_UNAVAILABLE, false);
        return false;
    }

    device->config = *config;
    device->spi = spi;
    device->deadline_ms = now_ms + BMI088_POWER_ON_DELAY_MS;
    device->initialized = true;
    set_status(device, DEVICE_BMI088_STATUS_INITIALIZING, false);
    set_state(device, BMI088_STATE_POWER_ON_WAIT, now_ms);
    return true;
}

bool Device_BMI088_Step(Device_BMI088 *device, uint32_t now_ms)
{
    const bmi088_register_config_t *entry;

    if ((device == NULL) || !device->initialized ||
        ((bmi088_state_t)device->internal_state == BMI088_STATE_ERROR) ||
        ((bmi088_state_t)device->internal_state ==
            BMI088_STATE_UNINITIALIZED)) {
        return false;
    }
    if (device->transfer_pending && !poll_transfer(device, now_ms)) {
        return false;
    }
    if (device->transfer_pending) {
        return true;
    }

    switch ((bmi088_state_t)device->internal_state) {
    case BMI088_STATE_POWER_ON_WAIT:
        if (deadline_reached(now_ms, device->deadline_ms)) {
            set_state(device, BMI088_STATE_ACCEL_INITIAL_DUMMY_ID, now_ms);
            return start_read(device,
                DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT,
                BMI088_ACCEL_CHIP_ID_REG, 1U, now_ms);
        }
        break;

    case BMI088_STATE_ACCEL_INITIAL_DUMMY_ID:
    case BMI088_STATE_ACCEL_INITIAL_ID:
    case BMI088_STATE_ACCEL_POST_RESET_DUMMY_ID:
    case BMI088_STATE_ACCEL_POST_RESET_ID:
        return start_read(device,
            DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT,
            BMI088_ACCEL_CHIP_ID_REG, 1U, now_ms);

    case BMI088_STATE_GYRO_INITIAL_ID:
    case BMI088_STATE_GYRO_POST_RESET_ID:
        return start_read(device,
            DEVICE_BMI088_GYROSCOPE_CHIP_SELECT,
            BMI088_GYRO_CHIP_ID_REG, 1U, now_ms);

    case BMI088_STATE_ACCEL_RESET:
        return start_write(device,
            DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT,
            BMI088_ACCEL_RESET_REG, BMI088_SOFT_RESET_VALUE, now_ms);

    case BMI088_STATE_ACCEL_RESET_WAIT:
        if (deadline_reached(now_ms, device->deadline_ms)) {
            set_state(device,
                BMI088_STATE_ACCEL_POST_RESET_DUMMY_ID, now_ms);
            return start_read(device,
                DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT,
                BMI088_ACCEL_CHIP_ID_REG, 1U, now_ms);
        }
        break;

    case BMI088_STATE_ACCEL_CONFIG_WRITE:
        entry = &s_accel_config[device->configuration_index];
        return start_write(device,
            DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT,
            entry->address, entry->value, now_ms);

    case BMI088_STATE_ACCEL_CONFIG_WAIT:
        if (deadline_reached(now_ms, device->deadline_ms)) {
            set_state(device, BMI088_STATE_ACCEL_CONFIG_READ, now_ms);
            entry = &s_accel_config[device->configuration_index];
            return start_read(device,
                DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT,
                entry->address, 1U, now_ms);
        }
        break;

    case BMI088_STATE_ACCEL_CONFIG_READ:
        entry = &s_accel_config[device->configuration_index];
        return start_read(device,
            DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT,
            entry->address, 1U, now_ms);

    case BMI088_STATE_GYRO_RESET:
        return start_write(device,
            DEVICE_BMI088_GYROSCOPE_CHIP_SELECT,
            BMI088_GYRO_RESET_REG, BMI088_SOFT_RESET_VALUE, now_ms);

    case BMI088_STATE_GYRO_RESET_WAIT:
        if (deadline_reached(now_ms, device->deadline_ms)) {
            set_state(device, BMI088_STATE_GYRO_POST_RESET_ID, now_ms);
            return start_read(device,
                DEVICE_BMI088_GYROSCOPE_CHIP_SELECT,
                BMI088_GYRO_CHIP_ID_REG, 1U, now_ms);
        }
        break;

    case BMI088_STATE_GYRO_CONFIG_WRITE:
        entry = &s_gyro_config[device->configuration_index];
        return start_write(device,
            DEVICE_BMI088_GYROSCOPE_CHIP_SELECT,
            entry->address, entry->value, now_ms);

    case BMI088_STATE_GYRO_CONFIG_WAIT:
        if (deadline_reached(now_ms, device->deadline_ms)) {
            set_state(device, BMI088_STATE_GYRO_CONFIG_READ, now_ms);
            entry = &s_gyro_config[device->configuration_index];
            return start_read(device,
                DEVICE_BMI088_GYROSCOPE_CHIP_SELECT,
                entry->address, 1U, now_ms);
        }
        break;

    case BMI088_STATE_GYRO_CONFIG_READ:
        entry = &s_gyro_config[device->configuration_index];
        return start_read(device,
            DEVICE_BMI088_GYROSCOPE_CHIP_SELECT,
            entry->address, 1U, now_ms);

    case BMI088_STATE_READY:
        if (deadline_reached(now_ms, device->next_frame_ms)) {
            device->pending_sample_ms = now_ms;
            device->next_frame_ms += DEVICE_BMI088_FRAME_PERIOD_MS;
            if (deadline_reached(now_ms, device->next_frame_ms)) {
                device->next_frame_ms =
                    now_ms + DEVICE_BMI088_FRAME_PERIOD_MS;
            }
            set_state(device, BMI088_STATE_SAMPLE_ACCEL, now_ms);
            return start_read(device,
                DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT,
                BMI088_ACCEL_DATA_REG, 6U, now_ms);
        }
        break;

    case BMI088_STATE_SAMPLE_ACCEL:
        return start_read(device,
            DEVICE_BMI088_ACCELEROMETER_CHIP_SELECT,
            BMI088_ACCEL_DATA_REG, 6U, now_ms);

    case BMI088_STATE_SAMPLE_GYRO:
        return start_read(device,
            DEVICE_BMI088_GYROSCOPE_CHIP_SELECT,
            BMI088_GYRO_DATA_REG, 6U, now_ms);

    default:
        return fail(device, DEVICE_BMI088_STATUS_TRANSPORT_ERROR,
            now_ms, false);
    }

    return true;
}

bool Device_BMI088_GetSnapshot(
    const Device_BMI088 *device, Device_BMI088_Snapshot *snapshot)
{
    if ((device == NULL) || !device->initialized || (snapshot == NULL)) {
        return false;
    }
    *snapshot = device->snapshot;
    return true;
}

bool Device_BMI088_IsTransferPending(const Device_BMI088 *device)
{
    return (device != NULL) && device->initialized &&
        device->transfer_pending;
}

uint32_t Device_BMI088_GetTransferSequence(const Device_BMI088 *device)
{
    return ((device != NULL) && device->initialized) ?
        device->transfer_sequence : 0U;
}

Device_BMI088_Status Device_BMI088_GetStatus(const Device_BMI088 *device)
{
    return (device == NULL) ? DEVICE_BMI088_STATUS_INVALID_ARGUMENT :
        device->snapshot.status;
}

bool Device_BMI088_IsOnline(const Device_BMI088 *device)
{
    return (device != NULL) && device->initialized &&
        device->snapshot.online &&
        (device->snapshot.status == DEVICE_BMI088_STATUS_ONLINE);
}
