#ifndef TASK3_ROD_SEQUENCE_H
#define TASK3_ROD_SEQUENCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TASK3_ROD_SEQUENCE_WAIT_ORIGIN = 0,
    TASK3_ROD_SEQUENCE_RAISE,
    TASK3_ROD_SEQUENCE_POSITION_RETURN,
} Task3_RodSequence_State;

typedef enum {
    TASK3_ROD_SEQUENCE_OUTPUT_LEVEL = 0,
    TASK3_ROD_SEQUENCE_OUTPUT_OPEN_LOOP_RAISE,
    TASK3_ROD_SEQUENCE_OUTPUT_POSITION_RETURN,
} Task3_RodSequence_OutputMode;

typedef struct {
    float origin_tolerance_mm;
    float positive_crossing_mm;
    float raise_rate_deg_s;
    float maximum_raise_offset_deg;
    uint32_t maximum_step_ms;
} Task3_RodSequence_Config;

typedef struct {
    Task3_RodSequence_Config config;
    Task3_RodSequence_State state;
    float raise_offset_deg;
    uint32_t last_step_ms;
    bool initialized;
} Task3_RodSequence;

typedef struct {
    Task3_RodSequence_OutputMode mode;
    float rod_offset_deg;
    bool entered_position_return;
} Task3_RodSequence_Output;

bool Task3_RodSequence_Init(
    Task3_RodSequence *sequence,
    const Task3_RodSequence_Config *config,
    uint32_t now_ms);

void Task3_RodSequence_Reset(
    Task3_RodSequence *sequence,
    uint32_t now_ms);

bool Task3_RodSequence_Step(
    Task3_RodSequence *sequence,
    uint32_t now_ms,
    bool launch_requested,
    bool return_requested,
    bool measurement_valid,
    bool new_measurement,
    float camera_position_mm,
    Task3_RodSequence_Output *output);

#endif
