#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "algorithm_rod_angle_map.h"

static const Algorithm_RodAngleMap_Point s_points[] = {
    { 58499U, 20.640F },
    { 57999U, 21.160F },
    { 57499U, 21.650F },
    { 56999U, 22.110F },
    { 56499U, 22.570F },
    { 55999U, 23.030F },
    { 55499U, 23.460F },
    { 54999U, 24.000F },
    { 54499U, 24.470F },
    { 53999U, 25.080F },
    { 53577U, 25.450F },
};

static bool close_enough(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void test_reference_and_limits(void)
{
    Algorithm_RodAngleMap map;
    float angle;
    float minimum;
    float maximum;

    assert(Algorithm_RodAngleMap_Init(
        &map, s_points, sizeof(s_points) / sizeof(s_points[0]), 54379U));
    assert(Algorithm_RodAngleMap_RawToRodDeg(&map, 54379.0F, &angle));
    assert(close_enough(angle, 0.0F, 0.0001F));
    assert(close_enough(
        map.reference_indicated_angle_deg, 24.6164F, 0.0001F));
    assert(Algorithm_RodAngleMap_GetLimits(&map, &minimum, &maximum));
    assert(close_enough(minimum, -3.9764F, 0.0002F));
    assert(close_enough(maximum, 0.8336F, 0.0002F));
}

static void test_inverse_round_trip_and_no_extrapolation(void)
{
    Algorithm_RodAngleMap map;

    assert(Algorithm_RodAngleMap_Init(
        &map, s_points, sizeof(s_points) / sizeof(s_points[0]), 54379U));
    for (size_t index = 0U;
        index < sizeof(s_points) / sizeof(s_points[0]); ++index) {
        float angle;
        float raw;

        assert(Algorithm_RodAngleMap_RawToRodDeg(
            &map, (float)s_points[index].raw, &angle));
        assert(Algorithm_RodAngleMap_RodDegToRaw(&map, angle, &raw));
        assert(close_enough(raw, (float)s_points[index].raw, 0.02F));
    }
    {
        float value;
        assert(!Algorithm_RodAngleMap_RawToRodDeg(
            &map, 58500.0F, &value));
        assert(!Algorithm_RodAngleMap_RodDegToRaw(
            &map, 0.84F, &value));
    }
}

static void test_invalid_non_monotonic_table(void)
{
    Algorithm_RodAngleMap map;
    const Algorithm_RodAngleMap_Point invalid[] = {
        { 100U, 1.0F }, { 90U, 2.0F }, { 95U, 3.0F },
    };

    assert(!Algorithm_RodAngleMap_Init(
        &map, invalid, sizeof(invalid) / sizeof(invalid[0]), 95U));
}

int main(void)
{
    test_reference_and_limits();
    test_inverse_round_trip_and_no_extrapolation();
    test_invalid_non_monotonic_table();
    puts("rod angle map tests: PASS");
    return 0;
}
