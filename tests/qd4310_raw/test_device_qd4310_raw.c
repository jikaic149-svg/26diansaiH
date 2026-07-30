#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "device_qd4310.h"

struct BSP_CAN_Resource {
    uint8_t token;
};

static struct BSP_CAN_Resource s_can;

bool BSP_CAN_GetInfo(BSP_CAN_Handle can, BSP_CAN_Info *info)
{
    if ((can != &s_can) || (info == NULL)) {
        return false;
    }
    *info = (BSP_CAN_Info){
        .nominal_bit_rate = DEVICE_QD4310_CAN_BIT_RATE,
    };
    return true;
}

BSP_CAN_Result BSP_CAN_Send(
    BSP_CAN_Handle can, const BSP_CAN_Frame *frame, uint32_t timeout_ms)
{
    (void)frame;
    (void)timeout_ms;
    return (can == &s_can) ?
        BSP_CAN_RESULT_OK : BSP_CAN_RESULT_INVALID_ARGUMENT;
}

BSP_CAN_Result BSP_CAN_ReadAvailable(
    BSP_CAN_Handle can, BSP_CAN_Frame *frame)
{
    (void)frame;
    return (can == &s_can) ?
        BSP_CAN_RESULT_NO_DATA : BSP_CAN_RESULT_INVALID_ARGUMENT;
}

static Device_QD4310 make_motor(void)
{
    Device_QD4310 motor;
    Device_QD4310_Config config;

    assert(Device_QD4310_DefaultConfig(&s_can, 1U, &config));
    assert(Device_QD4310_Init(&motor, &config));
    return motor;
}

static Device_QD4310_Feedback consume_raw(uint16_t raw)
{
    Device_QD4310 motor = make_motor();
    BSP_CAN_Frame frame = {
        .identifier = DEVICE_QD4310_FEEDBACK_ID_BASE + 1U,
        .data_length = 8U,
    };
    Device_QD4310_Feedback feedback;

    frame.data[6] = (uint8_t)(raw & UINT16_C(0x00FF));
    frame.data[7] = (uint8_t)(raw >> 8U);
    assert(Device_QD4310_ConsumeFrame(&motor, &frame, 10U));
    assert(Device_QD4310_GetFeedback(&motor, 10U, &feedback));
    assert(feedback.online);
    return feedback;
}

static void test_raw_value_is_exact_and_angle_is_derived(void)
{
    const uint16_t raw = UINT16_C(0xEE77);
    const Device_QD4310_Feedback feedback = consume_raw(raw);
    const float expected_rad = Device_QD4310_AngleRawToRad(raw);

    assert(feedback.single_turn_angle_raw == raw);
    assert(fabsf(feedback.single_turn_angle_rad - expected_rad) < 0.000001F);
}

static void test_raw_boundaries(void)
{
    Device_QD4310_Feedback feedback = consume_raw(0U);

    assert(feedback.single_turn_angle_raw == 0U);
    assert(feedback.single_turn_angle_rad == 0.0F);
    feedback = consume_raw(UINT16_MAX);
    assert(feedback.single_turn_angle_raw == UINT16_MAX);
    assert(fabsf(feedback.single_turn_angle_rad -
        6.2831853071795864769F) < 0.000001F);
    assert(Device_QD4310_AngleRawToRad(0U) == 0.0F);
    assert(fabsf(Device_QD4310_AngleRawToRad(UINT16_MAX) -
        6.2831853071795864769F) < 0.000001F);
}

static void test_nonblocking_send_timeout_is_valid(void)
{
    Device_QD4310 motor;
    Device_QD4310_Config config;

    assert(Device_QD4310_DefaultConfig(&s_can, 1U, &config));
    config.send_timeout_ms = 0U;
    assert(Device_QD4310_Init(&motor, &config));
    assert(motor.config.send_timeout_ms == 0U);
}

int main(void)
{
    test_raw_value_is_exact_and_angle_is_derived();
    test_raw_boundaries();
    test_nonblocking_send_timeout_is_valid();
    puts("QD4310 raw encoder tests: PASS");
    return 0;
}
