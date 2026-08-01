#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "task3_rod_sequence.h"

static bool close_enough(float actual, float expected)
{
    return fabsf(actual - expected) <= 0.0001F;
}

static Task3_RodSequence_Config default_config(void)
{
    return (Task3_RodSequence_Config){
        .origin_tolerance_mm = 10.0F,
        .positive_crossing_mm = 50.0F,
        .negative_crossing_mm = -18.0F,
        .raise_rate_deg_s = 0.40F,
        .maximum_raise_offset_deg = 0.60F,
        .initial_reverse_offset_deg = 0.20F,
        .reverse_rate_deg_s = 0.40F,
        .maximum_reverse_offset_deg = 2.00F,
        .maximum_step_ms = 100U,
    };
}

static void test_launch_requires_request_and_fresh_origin(void)
{
    Task3_RodSequence sequence;
    const Task3_RodSequence_Config config = default_config();
    Task3_RodSequence_Output output;

    assert(Task3_RodSequence_Init(&sequence, &config, 0U));
    assert(Task3_RodSequence_Step(
        &sequence, 0U, false, false, true, true, 0.0F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_LEVEL);
    assert(Task3_RodSequence_Step(
        &sequence, 1U, true, false, true, false, 0.0F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_LEVEL);
    assert(Task3_RodSequence_Step(
        &sequence, 2U, true, false, true, true, 10.0F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_OPEN_LOOP_RAISE);
    assert(close_enough(output.rod_offset_deg, 0.0F));
}

static void test_forward_ramp_reverse_bias_and_position_handoff(void)
{
    Task3_RodSequence sequence;
    const Task3_RodSequence_Config config = default_config();
    Task3_RodSequence_Output output;

    assert(Task3_RodSequence_Init(&sequence, &config, 100U));
    assert(Task3_RodSequence_Step(
        &sequence, 100U, true, false, true, true, 0.0F, &output));
    assert(Task3_RodSequence_Step(
        &sequence, 200U, true, false, true, false, 20.0F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_OPEN_LOOP_RAISE);
    assert(close_enough(output.rod_offset_deg, 0.04F));
    assert(Task3_RodSequence_Step(
        &sequence, 300U, true, false, true, true, 49.9F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_OPEN_LOOP_RAISE);
    assert(close_enough(output.rod_offset_deg, 0.08F));

    assert(Task3_RodSequence_Step(
        &sequence, 301U, false, true, true, false, 49.9F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_OPEN_LOOP_REVERSE);
    assert(!output.entered_position_return);
    assert(close_enough(output.rod_offset_deg, -0.20F));

    assert(Task3_RodSequence_Step(
        &sequence, 401U, false, true, true, false, 49.9F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_OPEN_LOOP_REVERSE);
    assert(close_enough(output.rod_offset_deg, -0.24F));

    assert(Task3_RodSequence_Step(
        &sequence, 402U, false, true, true, true, -17.9F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_OPEN_LOOP_REVERSE);
    assert(!output.entered_position_return);

    assert(Task3_RodSequence_Step(
        &sequence, 403U, false, true, true, true, -18.0F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_POSITION_RETURN);
    assert(output.entered_position_return);
    assert(close_enough(output.rod_offset_deg, 0.0F));
}

static void test_crossing_and_visual_loss(void)
{
    Task3_RodSequence sequence;
    const Task3_RodSequence_Config config = default_config();
    Task3_RodSequence_Output output;

    assert(Task3_RodSequence_Init(&sequence, &config, 0U));
    assert(Task3_RodSequence_Step(
        &sequence, 0U, true, false, true, true, -2.0F, &output));
    assert(Task3_RodSequence_Step(
        &sequence, 100U, true, false, true, true, 50.0F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_OPEN_LOOP_REVERSE);
    assert(close_enough(output.rod_offset_deg, -0.20F));
    assert(!output.entered_position_return);
    assert(Task3_RodSequence_Step(
        &sequence, 200U, true, true, true, true, -18.0F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_POSITION_RETURN);
    assert(output.entered_position_return);

    Task3_RodSequence_Reset(&sequence, 200U);
    assert(Task3_RodSequence_Step(
        &sequence, 200U, true, false, true, true, 1.0F, &output));
    assert(Task3_RodSequence_Step(
        &sequence, 300U, true, false, false, false, 0.0F, &output));
    assert(output.mode == TASK3_ROD_SEQUENCE_OUTPUT_LEVEL);
    assert(sequence.state == TASK3_ROD_SEQUENCE_WAIT_ORIGIN);
    assert(close_enough(sequence.raise_offset_deg, 0.0F));
}

static void test_both_ramps_and_elapsed_step_are_bounded(void)
{
    Task3_RodSequence sequence;
    const Task3_RodSequence_Config config = default_config();
    Task3_RodSequence_Output output;

    assert(Task3_RodSequence_Init(&sequence, &config, 0U));
    assert(Task3_RodSequence_Step(
        &sequence, 0U, true, false, true, true, 0.0F, &output));
    assert(Task3_RodSequence_Step(
        &sequence, 1000U, true, false, true, false, 0.0F, &output));
    assert(close_enough(output.rod_offset_deg, 0.04F));

    for (uint32_t now_ms = 1100U; now_ms <= 3000U; now_ms += 100U) {
        assert(Task3_RodSequence_Step(
            &sequence, now_ms, true, false, true, false, 0.0F, &output));
    }
    assert(close_enough(output.rod_offset_deg, 0.60F));

    assert(Task3_RodSequence_Step(
        &sequence, 3001U, false, true, true, false, 0.0F, &output));
    assert(close_enough(output.rod_offset_deg, -0.20F));
    for (uint32_t now_ms = 3101U; now_ms <= 8101U; now_ms += 100U) {
        assert(Task3_RodSequence_Step(
            &sequence, now_ms, false, true, true, false, 0.0F, &output));
    }
    assert(close_enough(output.rod_offset_deg, -2.00F));
    assert(sequence.state == TASK3_ROD_SEQUENCE_REVERSE_RAISE);
}

static void test_configuration_validation(void)
{
    Task3_RodSequence sequence;
    Task3_RodSequence_Config config = default_config();

    config.positive_crossing_mm = config.origin_tolerance_mm;
    assert(!Task3_RodSequence_Init(&sequence, &config, 0U));
    config = default_config();
    config.negative_crossing_mm = -config.origin_tolerance_mm;
    assert(!Task3_RodSequence_Init(&sequence, &config, 0U));
    config = default_config();
    config.initial_reverse_offset_deg = 0.0F;
    assert(!Task3_RodSequence_Init(&sequence, &config, 0U));
    config = default_config();
    config.maximum_reverse_offset_deg = 0.10F;
    assert(!Task3_RodSequence_Init(&sequence, &config, 0U));
    assert(!Task3_RodSequence_Init(NULL, &config, 0U));
}

int main(void)
{
    test_launch_requires_request_and_fresh_origin();
    test_forward_ramp_reverse_bias_and_position_handoff();
    test_crossing_and_visual_loss();
    test_both_ramps_and_elapsed_step_are_bounded();
    test_configuration_validation();
    puts("task3 rod sequence tests: PASS");
    return 0;
}
