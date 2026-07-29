/* 静态网关 Task，骨架来自 TI-Template：静态栈/TCB + vTaskDelayUntil 固定周期。
 * 本模板不含 MSPM0 看门狗；F407 WWDG/IWDG 未启用。 */

#include "task_app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_gateway.h"
#include "bsp_key.h"
#include "core_config.h"
#include "device_peer_link.h"

#define TASK_GATEWAY_PRIORITY \
    (tskIDLE_PRIORITY + GATEWAY_TASK_PRIORITY_OFFSET)
#define TASK_PEER_UART_INDEX 0U  /* BSP transport 0 = USART1 */

static StaticTask_t g_TASK_GatewayTCB;
static StackType_t g_TASK_GatewayStack[GATEWAY_TASK_STACK_DEPTH];

static bool s_key_candidate_pressed;
static bool s_key_stable_pressed;
static uint32_t s_key_candidate_since_ms;
static uint32_t s_key_press_count;
static uint32_t s_key_command_errors;
static Device_PeerLink s_peer_link;
static bool s_peer_link_ready;
static uint32_t s_peer_link_fault;
static uint32_t s_next_peer_telemetry_ms;

volatile uint32_t g_TASK_Heartbeat = 0U;
volatile uint32_t g_TASK_Fault = 0U;
volatile uint32_t g_TASK_Ready = 0U;
volatile uint32_t g_TASK_NowMs = 0U;

static void Task_KeyInit(uint32_t now_ms)
{
    const bool pressed = BSP_Key_IsPressed();

    s_key_candidate_pressed = pressed;
    s_key_stable_pressed = pressed;
    s_key_candidate_since_ms = now_ms;
    s_key_press_count = 0U;
    s_key_command_errors = 0U;
}

static void Task_KeyStep(uint32_t now_ms)
{
    const bool pressed = BSP_Key_IsPressed();

    if (pressed != s_key_candidate_pressed) {
        s_key_candidate_pressed = pressed;
        s_key_candidate_since_ms = now_ms;
        return;
    }
    if ((pressed == s_key_stable_pressed) ||
        ((uint32_t)(now_ms - s_key_candidate_since_ms) <
            GATEWAY_KEY_DEBOUNCE_MS)) {
        return;
    }

    s_key_stable_pressed = pressed;
    if (pressed) {
        ++s_key_press_count;
#if GATEWAY_LEVEL_CALIBRATION_KEY_MODE == 1U
        if (!APP_Gateway_AdvancePrimaryMotorCalibration(now_ms)) {
#else
        if (!APP_Gateway_TogglePrimaryMotor(now_ms)) {
#endif
            ++s_key_command_errors;
        }
    }
}

static void Task_PeerLinkInit(uint32_t now_ms)
{
    const Device_PeerLink_Config config = {
        .transport_index = TASK_PEER_UART_INDEX,
        .local_node_id = PEER_LINK_LOCAL_NODE_ID,
        .remote_node_id = PEER_LINK_REMOTE_NODE_ID,
        .heartbeat_period_ms = PEER_LINK_HEARTBEAT_PERIOD_MS,
        .offline_timeout_ms = PEER_LINK_OFFLINE_TIMEOUT_MS,
        .maximum_rx_bytes_per_step = PEER_LINK_RX_BYTES_PER_STEP,
    };

    s_peer_link_fault = 0U;
    s_peer_link_ready = Device_PeerLink_Init(
        &s_peer_link, &config, now_ms);
    s_next_peer_telemetry_ms = now_ms;
    if (!s_peer_link_ready) {
        s_peer_link_fault = 1U;
    }
}

static int32_t Task_ScaleSigned(float value, float scale)
{
    const float scaled = value * scale;

    return (int32_t)(scaled + ((scaled >= 0.0F) ? 0.5F : -0.5F));
}

static void Task_SendPeerTelemetry(
    uint32_t now_ms, const APP_Gateway_Telemetry *gateway)
{
    Device_PeerLink_Telemetry telemetry;
    Device_PeerLink_AbsoluteEncoder encoder;

    if (!s_peer_link_ready || (gateway == NULL) ||
        ((int32_t)(now_ms - s_next_peer_telemetry_ms) < 0)) {
        return;
    }
    s_next_peer_telemetry_ms = now_ms + PEER_LINK_TELEMETRY_PERIOD_MS;
    telemetry = (Device_PeerLink_Telemetry){
        .motor_id = gateway->motor_id,
        .enabled = gateway->enabled,
        .current_milliamps =
            Task_ScaleSigned(gateway->current_a, 1000.0F),
        .speed_centi_rpm =
            Task_ScaleSigned(gateway->speed_rpm, 100.0F),
        .angle_milliradians = (uint32_t)(
            gateway->single_turn_angle_rad * 1000.0F + 0.5F),
        .online = gateway->online,
    };
    if (!Device_PeerLink_SendTelemetry(&s_peer_link, &telemetry)) {
        s_peer_link_fault = 3U;
    }
    encoder = (Device_PeerLink_AbsoluteEncoder){
        .motor_id = gateway->motor_id,
        .angle_raw = gateway->single_turn_angle_raw,
        .online = gateway->online,
    };
    if (!Device_PeerLink_SendAbsoluteEncoder(&s_peer_link, &encoder)) {
        s_peer_link_fault = 4U;
    }
}

static void Task_Gateway(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    if (!APP_Gateway_Init(
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS))) {
        g_TASK_Fault = 2U;
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }
    Task_KeyInit(
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    Task_PeerLinkInit(
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));

    for (;;) {
        APP_Gateway_Telemetry gateway_telemetry;

        g_TASK_NowMs =
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        ++g_TASK_Heartbeat;
        Task_KeyStep(g_TASK_NowMs);
        if (s_peer_link_ready &&
            !Device_PeerLink_Step(&s_peer_link, g_TASK_NowMs)) {
            s_peer_link_fault = 2U;
        }
        APP_Gateway_Step(g_TASK_NowMs, &gateway_telemetry);
        Task_SendPeerTelemetry(g_TASK_NowMs, &gateway_telemetry);
        vTaskDelayUntil(
            &last_wake, pdMS_TO_TICKS(GATEWAY_TASK_PERIOD_MS));
    }
}

bool Task_Init(void)
{
    TaskHandle_t task;

    g_TASK_Heartbeat = 0U;
    g_TASK_Fault = 0U;
    g_TASK_Ready = 0U;
    g_TASK_NowMs = 0U;

    task = xTaskCreateStatic(
        Task_Gateway,
        GATEWAY_TASK_NAME,
        GATEWAY_TASK_STACK_DEPTH,
        NULL,
        TASK_GATEWAY_PRIORITY,
        g_TASK_GatewayStack,
        &g_TASK_GatewayTCB);
    if (task == NULL) {
        g_TASK_Fault = 1U;
        return false;
    }

    g_TASK_Ready = 1U;
    return true;
}
