#include "device_ganwei.h"

#include <stddef.h>

static bool valid_ganwei_backend(BSP_LineSensor_Handle backend)
{
    BSP_LineSensor_Info info;

    if ((backend == NULL) || !BSP_LineSensor_GetInfo(backend, &info) ||
        (info.channel_count != DEVICE_GANWEI_CHANNEL_COUNT)) {
        return false;
    }
    for (size_t index = 0U;
         index < DEVICE_GANWEI_CHANNEL_COUNT;
         ++index) {
        if (info.channel_kind[index] != BSP_LINE_SENSOR_CHANNEL_DIGITAL) {
            return false;
        }
    }
    return true;
}

bool Device_Ganwei_DefaultConfig(
    size_t transport_index, Device_Ganwei_Config *config)
{
    BSP_LineSensor_Handle backend;

    if (config == NULL) {
        return false;
    }
    *config = (Device_Ganwei_Config){0};
    backend = BSP_LineSensor_GetHandle(transport_index);
    return valid_ganwei_backend(backend) &&
        Device_LineTracking_DefaultConfig(backend, config);
}

bool Device_Ganwei_Init(
    Device_Ganwei *device, const Device_Ganwei_Config *config)
{
    if (device != NULL) {
        *device = (Device_Ganwei){0};
    }
    if ((device == NULL) || (config == NULL) ||
        !valid_ganwei_backend(config->backend) ||
        (config->channel_count != DEVICE_GANWEI_CHANNEL_COUNT) ||
        !Device_LineTracking_Init(&device->digital_tracking, config)) {
        return false;
    }
    device->initialized = true;
    return true;
}

bool Device_Ganwei_Update(Device_Ganwei *device, uint32_t now_ms)
{
    return (device != NULL) && device->initialized &&
        Device_LineTracking_Update(&device->digital_tracking, now_ms);
}

bool Device_Ganwei_GetSample(
    const Device_Ganwei *device, Device_Ganwei_Sample *sample)
{
    return (device != NULL) && device->initialized &&
        Device_LineTracking_GetSample(&device->digital_tracking, sample);
}
