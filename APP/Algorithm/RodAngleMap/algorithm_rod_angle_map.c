#include "algorithm_rod_angle_map.h"

#include <math.h>
#include <stddef.h>

static bool between_inclusive(float value, float first, float second)
{
    return (first <= second) ?
        ((value >= first) && (value <= second)) :
        ((value >= second) && (value <= first));
}

static float interpolate(
    float input, float input0, float input1, float output0, float output1)
{
    const float fraction = (input - input0) / (input1 - input0);

    return output0 + fraction * (output1 - output0);
}

static bool interpolate_by_raw(
    const Algorithm_RodAngleMap *map,
    float raw,
    float *indicated_angle_deg)
{
    if ((map == NULL) || !map->initialized || !isfinite(raw) ||
        (indicated_angle_deg == NULL) ||
        !between_inclusive(
            raw, (float)map->points[0].raw,
            (float)map->points[map->point_count - 1U].raw)) {
        return false;
    }

    for (size_t index = 0U; index + 1U < map->point_count; ++index) {
        const Algorithm_RodAngleMap_Point *first = &map->points[index];
        const Algorithm_RodAngleMap_Point *second = &map->points[index + 1U];

        if (between_inclusive(raw, (float)first->raw, (float)second->raw)) {
            *indicated_angle_deg = interpolate(
                raw, (float)first->raw, (float)second->raw,
                first->indicated_angle_deg, second->indicated_angle_deg);
            return isfinite(*indicated_angle_deg);
        }
    }
    return false;
}

bool Algorithm_RodAngleMap_Init(
    Algorithm_RodAngleMap *map,
    const Algorithm_RodAngleMap_Point *points,
    size_t point_count,
    uint16_t reference_raw)
{
    bool raw_increasing;
    bool indication_increasing;
    float reference_indication;

    if (map != NULL) {
        *map = (Algorithm_RodAngleMap){0};
    }
    if ((map == NULL) || (points == NULL) || (point_count < 2U)) {
        return false;
    }
    if (!isfinite(points[0].indicated_angle_deg) ||
        !isfinite(points[1].indicated_angle_deg) ||
        (points[0].raw == points[1].raw) ||
        (points[0].indicated_angle_deg == points[1].indicated_angle_deg)) {
        return false;
    }
    raw_increasing = points[1].raw > points[0].raw;
    indication_increasing =
        points[1].indicated_angle_deg > points[0].indicated_angle_deg;

    for (size_t index = 1U; index < point_count; ++index) {
        if (!isfinite(points[index].indicated_angle_deg) ||
            ((points[index].raw > points[index - 1U].raw) != raw_increasing) ||
            (points[index].raw == points[index - 1U].raw) ||
            ((points[index].indicated_angle_deg >
                points[index - 1U].indicated_angle_deg) !=
                indication_increasing) ||
            (points[index].indicated_angle_deg ==
                points[index - 1U].indicated_angle_deg)) {
            return false;
        }
    }
    if (!between_inclusive(
            (float)reference_raw, (float)points[0].raw,
            (float)points[point_count - 1U].raw)) {
        return false;
    }

    map->points = points;
    map->point_count = point_count;
    map->reference_raw = (float)reference_raw;
    map->raw_increasing = raw_increasing;
    map->indication_increasing = indication_increasing;
    map->initialized = true;
    if (!interpolate_by_raw(map, map->reference_raw, &reference_indication)) {
        *map = (Algorithm_RodAngleMap){0};
        return false;
    }
    map->reference_indicated_angle_deg = reference_indication;
    {
        const float first = points[0].indicated_angle_deg -
            reference_indication;
        const float last = points[point_count - 1U].indicated_angle_deg -
            reference_indication;

        map->minimum_rod_angle_deg = (first < last) ? first : last;
        map->maximum_rod_angle_deg = (first > last) ? first : last;
    }
    return true;
}

bool Algorithm_RodAngleMap_RawToRodDeg(
    const Algorithm_RodAngleMap *map,
    float raw,
    float *rod_angle_deg)
{
    float indication;

    if ((rod_angle_deg == NULL) ||
        !interpolate_by_raw(map, raw, &indication)) {
        return false;
    }
    *rod_angle_deg = indication - map->reference_indicated_angle_deg;
    return isfinite(*rod_angle_deg);
}

bool Algorithm_RodAngleMap_RodDegToRaw(
    const Algorithm_RodAngleMap *map,
    float rod_angle_deg,
    float *raw)
{
    float desired_indication;

    if ((map == NULL) || !map->initialized || !isfinite(rod_angle_deg) ||
        (raw == NULL) ||
        (rod_angle_deg < map->minimum_rod_angle_deg) ||
        (rod_angle_deg > map->maximum_rod_angle_deg)) {
        return false;
    }
    desired_indication =
        map->reference_indicated_angle_deg + rod_angle_deg;

    for (size_t index = 0U; index + 1U < map->point_count; ++index) {
        const Algorithm_RodAngleMap_Point *first = &map->points[index];
        const Algorithm_RodAngleMap_Point *second = &map->points[index + 1U];

        if (between_inclusive(
                desired_indication, first->indicated_angle_deg,
                second->indicated_angle_deg)) {
            *raw = interpolate(
                desired_indication,
                first->indicated_angle_deg,
                second->indicated_angle_deg,
                (float)first->raw,
                (float)second->raw);
            return isfinite(*raw);
        }
    }
    return false;
}

bool Algorithm_RodAngleMap_GetLimits(
    const Algorithm_RodAngleMap *map,
    float *minimum_rod_angle_deg,
    float *maximum_rod_angle_deg)
{
    if ((map == NULL) || !map->initialized ||
        (minimum_rod_angle_deg == NULL) ||
        (maximum_rod_angle_deg == NULL)) {
        return false;
    }
    *minimum_rod_angle_deg = map->minimum_rod_angle_deg;
    *maximum_rod_angle_deg = map->maximum_rod_angle_deg;
    return true;
}
