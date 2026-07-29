#ifndef DEVICE_GANWEI_H
#define DEVICE_GANWEI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "device_line_tracking.h"

#define DEVICE_GANWEI_CHANNEL_COUNT 8U

typedef Device_LineTracking_Config Device_Ganwei_Config;
typedef Device_LineTracking_Sample Device_Ganwei_Sample;

typedef struct {
    Device_LineTracking digital_tracking;
    bool initialized;
} Device_Ganwei;

/* Bind the Ganwei DATA/CLK interface and build the digital line-tracking
 * defaults. Callers may adjust polarity, thresholds and position weights
 * before Device_Ganwei_Init(). */
bool Device_Ganwei_DefaultConfig(
    size_t transport_index, Device_Ganwei_Config *config);
bool Device_Ganwei_Init(
    Device_Ganwei *device, const Device_Ganwei_Config *config);

/* Acquire exactly eight digital channels through the BSP DATA/CLK backend.
 * This always uses the thresholded digital path, never the analog relative-
 * difference preprocessing used by Xinliu T8. */
bool Device_Ganwei_Update(Device_Ganwei *device, uint32_t now_ms);
bool Device_Ganwei_GetSample(
    const Device_Ganwei *device, Device_Ganwei_Sample *sample);

#endif
