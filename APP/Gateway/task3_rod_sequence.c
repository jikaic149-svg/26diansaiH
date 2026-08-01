#include "task3_rod_sequence.h"

#include <math.h>

static bool valid_config(const Task3_RodSequence_Config *config)
{
    return (config != NULL) &&
        isfinite(config->origin_tolerance_mm) &&
        (config->origin_tolerance_mm >= 0.0F) &&
        isfinite(config->positive_crossing_mm) &&
        (config->positive_crossing_mm > config->origin_tolerance_mm) &&
        isfinite(config->raise_rate_deg_s) &&
        (config->raise_rate_deg_s > 0.0F) &&
        isfinite(config->maximum_raise_offset_deg) &&
        (config->maximum_raise_offset_deg > 0.0F) &&
        (config->maximum_step_ms > 0U);
}

static void make_output(
    const Task3_RodSequence *sequence,
    Task3_RodSequence_Output *output)
{
    output->entered_position_return = false;
    if (sequence->state == TASK3_ROD_SEQUENCE_RAISE) {
        output->mode = TASK3_ROD_SEQUENCE_OUTPUT_OPEN_LOOP_RAISE;
        output->rod_offset_deg = sequence->raise_offset_deg;
    }
    else if (sequence->state == TASK3_ROD_SEQUENCE_POSITION_RETURN) {
        output->mode = TASK3_ROD_SEQUENCE_OUTPUT_POSITION_RETURN;
        output->rod_offset_deg = 0.0F;
    }
    else {
        output->mode = TASK3_ROD_SEQUENCE_OUTPUT_LEVEL;
        output->rod_offset_deg = 0.0F;
    }
}

bool Task3_RodSequence_Init(
    Task3_RodSequence *sequence,
    const Task3_RodSequence_Config *config,
    uint32_t now_ms)
{
    if (sequence != NULL) {
        *sequence = (Task3_RodSequence){0};
    }
    if ((sequence == NULL) || !valid_config(config)) {
        return false;
    }
    sequence->config = *config;
    sequence->last_step_ms = now_ms;
    sequence->initialized = true;
    return true;
}

void Task3_RodSequence_Reset(
    Task3_RodSequence *sequence,
    uint32_t now_ms)
{
    if ((sequence == NULL) || !sequence->initialized) {
        return;
    }
    sequence->state = TASK3_ROD_SEQUENCE_WAIT_ORIGIN;
    sequence->raise_offset_deg = 0.0F;
    sequence->last_step_ms = now_ms;
}

bool Task3_RodSequence_Step(
    Task3_RodSequence *sequence,
    uint32_t now_ms,
    bool launch_requested,
    bool return_requested,
    bool measurement_valid,
    bool new_measurement,
    float camera_position_mm,
    Task3_RodSequence_Output *output)
{
    uint32_t elapsed_ms;

    if ((output != NULL)) {
        *output = (Task3_RodSequence_Output){0};
    }
    if ((sequence == NULL) || !sequence->initialized || (output == NULL) ||
        (measurement_valid && !isfinite(camera_position_mm))) {
        return false;
    }

    if (!measurement_valid) {
        /* Preserve the existing visual-loss behavior: remove the open-loop
         * offset and require a fresh origin observation before relaunching. */
        Task3_RodSequence_Reset(sequence, now_ms);
        make_output(sequence, output);
        return true;
    }

    make_output(sequence, output);
    if (sequence->state == TASK3_ROD_SEQUENCE_POSITION_RETURN) {
        return true;
    }
    if (sequence->state == TASK3_ROD_SEQUENCE_WAIT_ORIGIN) {
        sequence->last_step_ms = now_ms;
        if (launch_requested && new_measurement &&
            (fabsf(camera_position_mm) <=
                sequence->config.origin_tolerance_mm)) {
            sequence->state = TASK3_ROD_SEQUENCE_RAISE;
            make_output(sequence, output);
        }
        return true;
    }
    if (return_requested ||
        (new_measurement &&
         (camera_position_mm >= sequence->config.positive_crossing_mm))) {
        sequence->state = TASK3_ROD_SEQUENCE_POSITION_RETURN;
        sequence->raise_offset_deg = 0.0F;
        make_output(sequence, output);
        output->entered_position_return = true;
        return true;
    }

    elapsed_ms = now_ms - sequence->last_step_ms;
    sequence->last_step_ms = now_ms;
    if (elapsed_ms > sequence->config.maximum_step_ms) {
        elapsed_ms = sequence->config.maximum_step_ms;
    }
    sequence->raise_offset_deg += sequence->config.raise_rate_deg_s *
        ((float)elapsed_ms / 1000.0F);
    if (sequence->raise_offset_deg >
        sequence->config.maximum_raise_offset_deg) {
        sequence->raise_offset_deg =
            sequence->config.maximum_raise_offset_deg;
    }
    make_output(sequence, output);
    return true;
}
