#ifndef ALGORITHM_ROD_ANGLE_MAP_H
#define ALGORITHM_ROD_ANGLE_MAP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t raw;
    /* Absolute indication used while calibrating. A constant IMU mounting
     * offset is removed by anchoring the table at reference_raw. */
    float indicated_angle_deg;
} Algorithm_RodAngleMap_Point;

typedef struct {
    const Algorithm_RodAngleMap_Point *points;
    size_t point_count;
    float reference_raw;
    float reference_indicated_angle_deg;
    float minimum_rod_angle_deg;
    float maximum_rod_angle_deg;
    bool raw_increasing;
    bool indication_increasing;
    bool initialized;
} Algorithm_RodAngleMap;

/* Initialize one measured, monotonic motion branch. The reference raw value
 * must lie inside the measured interval; no endpoint extrapolation is used. */
bool Algorithm_RodAngleMap_Init(
    Algorithm_RodAngleMap *map,
    const Algorithm_RodAngleMap_Point *points,
    size_t point_count,
    uint16_t reference_raw);

/* Convert between QD4310 raw position and rod angle relative to reference_raw.
 * Queries outside the measured branch are rejected. */
bool Algorithm_RodAngleMap_RawToRodDeg(
    const Algorithm_RodAngleMap *map,
    float raw,
    float *rod_angle_deg);
bool Algorithm_RodAngleMap_RodDegToRaw(
    const Algorithm_RodAngleMap *map,
    float rod_angle_deg,
    float *raw);

bool Algorithm_RodAngleMap_GetLimits(
    const Algorithm_RodAngleMap *map,
    float *minimum_rod_angle_deg,
    float *maximum_rod_angle_deg);

#endif
