#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "algorithm_chassis_acceleration_fusion.h"

static bool close_enough(float first, float second, float tolerance)
{
    return fabsf(first - second) <= tolerance;
}

static Algorithm_ChassisAccelerationFusion_Config default_config(void)
{
    return (Algorithm_ChassisAccelerationFusion_Config){
        .plan_gain = 1.0F,
        .response_time_constant_s = 0.12F,
        .plan_delay_ms = 30U,
        .wheel_speed_measurement_variance_mm2_s2 = 225.0F,
        .imu_acceleration_measurement_variance_mm2_s4 = 14400.0F,
        .velocity_noise_density_mm2_s3 = 400.0F,
        .acceleration_noise_density_mm2_s5 = 160000.0F,
        .imu_bias_walk_noise_density_mm2_s5 = 4.0F,
        .initial_velocity_variance_mm2_s2 = 900.0F,
        .initial_acceleration_variance_mm2_s4 = 10000.0F,
        .initial_imu_bias_variance_mm2_s4 = 6400.0F,
        .innovation_gate_sigma = 5.0F,
        .maximum_absolute_velocity_mm_s = 2000.0F,
        .maximum_absolute_acceleration_mm_s2 = 3000.0F,
        .maximum_absolute_imu_bias_mm_s2 = 1000.0F,
        .trusted_acceleration_variance_mm2_s4 = 40000.0F,
        .maximum_prediction_step_ms = 10U,
        .maximum_measurement_age_ms = 30U,
        .minimum_accepted_wheel_measurements = 3U,
        .minimum_accepted_imu_measurements = 3U,
    };
}

static void test_plan_delay_and_first_order_response(void)
{
    Algorithm_ChassisAccelerationFusion fusion;
    Algorithm_ChassisAccelerationFusion_State state;
    const Algorithm_ChassisAccelerationFusion_Config config = default_config();

    assert(Algorithm_ChassisAccelerationFusion_Init(&fusion, &config, 0U));
    for (uint32_t now_ms = 2U; now_ms <= 30U; now_ms += 2U) {
        assert(Algorithm_ChassisAccelerationFusion_Predict(
            &fusion, now_ms, 180.0F));
    }
    assert(Algorithm_ChassisAccelerationFusion_GetState(
        &fusion, &state, NULL));
    assert(close_enough(state.acceleration_mm_s2, 0.0F, 0.001F));

    for (uint32_t now_ms = 32U; now_ms <= 150U; now_ms += 2U) {
        assert(Algorithm_ChassisAccelerationFusion_Predict(
            &fusion, now_ms, 180.0F));
    }
    assert(Algorithm_ChassisAccelerationFusion_GetState(
        &fusion, &state, NULL));
    assert(state.acceleration_mm_s2 > 105.0F);
    assert(state.acceleration_mm_s2 < 120.0F);
    assert(state.velocity_mm_s > 5.0F);
}

static void test_sensor_fusion_and_trust_gate(void)
{
    Algorithm_ChassisAccelerationFusion fusion;
    Algorithm_ChassisAccelerationFusion_State state;
    Algorithm_ChassisAccelerationFusion_Config config = default_config();
    float actual_velocity = 0.0F;

    config.innovation_gate_sigma = 20.0F;
    assert(Algorithm_ChassisAccelerationFusion_Init(&fusion, &config, 0U));
    for (uint32_t now_ms = 2U; now_ms <= 400U; now_ms += 2U) {
        actual_velocity += 150.0F * 0.002F;
        assert(Algorithm_ChassisAccelerationFusion_Predict(
            &fusion, now_ms, 180.0F));
        if ((now_ms % 20U) == 0U) {
            assert(Algorithm_ChassisAccelerationFusion_Correct(
                &fusion,
                now_ms,
                true,
                actual_velocity,
                false,
                0.0F));
            assert(Algorithm_ChassisAccelerationFusion_Correct(
                &fusion,
                now_ms,
                false,
                0.0F,
                true,
                200.0F));
        }
    }
    assert(Algorithm_ChassisAccelerationFusion_GetState(
        &fusion, &state, NULL));
    assert(fusion.accepted_wheel_measurement_count >= 3U);
    assert(fusion.accepted_imu_measurement_count >= 3U);
    assert(Algorithm_ChassisAccelerationFusion_IsTrusted(&fusion, 400U));
    assert(state.acceleration_mm_s2 > 120.0F);
    assert(state.acceleration_mm_s2 < 185.0F);
    assert(state.imu_bias_mm_s2 > 5.0F);
    assert(!Algorithm_ChassisAccelerationFusion_IsTrusted(&fusion, 430U));
}

static void test_innovation_rejection_and_gap_reset(void)
{
    Algorithm_ChassisAccelerationFusion fusion;
    Algorithm_ChassisAccelerationFusion_State before;
    Algorithm_ChassisAccelerationFusion_State after;
    const Algorithm_ChassisAccelerationFusion_Config config = default_config();

    assert(Algorithm_ChassisAccelerationFusion_Init(&fusion, &config, 0U));
    assert(Algorithm_ChassisAccelerationFusion_Predict(
        &fusion, 2U, 0.0F));
    assert(Algorithm_ChassisAccelerationFusion_GetState(
        &fusion, &before, NULL));
    assert(Algorithm_ChassisAccelerationFusion_Correct(
        &fusion, 2U, true, 1000.0F, false, 0.0F));
    assert(fusion.wheel_innovation_reject_count == 1U);
    assert(Algorithm_ChassisAccelerationFusion_GetState(
        &fusion, &after, NULL));
    assert(close_enough(before.velocity_mm_s, after.velocity_mm_s, 0.001F));

    assert(!Algorithm_ChassisAccelerationFusion_Predict(
        &fusion, 20U, 0.0F));
    assert(fusion.reset_count == 1U);
    assert(Algorithm_ChassisAccelerationFusion_GetState(
        &fusion, &after, NULL));
    assert(close_enough(after.velocity_mm_s, 0.0F, 0.001F));
}

int main(void)
{
    test_plan_delay_and_first_order_response();
    test_sensor_fusion_and_trust_gate();
    test_innovation_rejection_and_gap_reset();
    puts("chassis acceleration fusion tests: PASS");
    return 0;
}
