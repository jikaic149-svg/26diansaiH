#include "task_bringup_chassis.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "core_config.h"
#include "device_encoder.h"
#include "device_icm45686_spi.h"
#include "device_key_input.h"
#include "device_line_tracking.h"
#include "device_motor.h"
#include "MG513X/device_motor_mg513x_gmr.h"
#include "device_peer_link.h"
#include "device_rgb.h"
#include "device_serial.h"
#include "device_st7735s.h"
#include "device_xinliu_t8.h"
#include "mode_car.h"
#include "mode_car_control.h"
#include "mode_communicate.h"

#define BRINGUP_TASK_NAME "Bringup"
#define BRINGUP_TASK_PERIOD_MS UINT32_C(10)
#define BRINGUP_TASK_STACK_DEPTH 512U
#define BRINGUP_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)

/* BSP UART resource order on G3519: 0 debug, 1 aux, 2 Bluetooth (PB10/PB11),
 * 3 line-tracking (PC6 TX/PC7 RX). */
#define BRINGUP_BLUETOOTH_TRANSPORT_INDEX 2U
#define BRINGUP_GRAYSCALE_TRANSPORT_INDEX 3U
#define BRINGUP_PEER_LINK_TRANSPORT_INDEX 0U
#define BRINGUP_UART_BAUD UINT32_C(115200)

/* BSP SPI resource order on G3519: 0 TFT, 1 ICM-45686 (CS index 0). */
#define BRINGUP_TFT_DISPLAY_INDEX 0U
#define BRINGUP_ICM45686_SPI_INDEX 1U
#define BRINGUP_ICM45686_SPI_BAUD UINT32_C(10000000)

#define BRINGUP_MOTOR_COUNT 2U

/* MG513X drivetrain: 500-line GMR, x4 QEI decode, 28:1 gearbox. The wheel
 * circumference is the user-confirmed nominal value; the effective rolling
 * circumference still needs the push-1-m calibration. */
#define BRINGUP_COUNTS_PER_WHEEL_REV \
    ((float)DEVICE_MOTOR_MG513X_GMR_ENCODER_LINES * 4.0F * \
        DEVICE_MOTOR_MG513X_GEAR_RATIO)
#define BRINGUP_WHEEL_CIRCUMFERENCE_MM 204.20F

/* Product-confirmed T8 channel order is CH1..CH8 from left to right. The two
 * center channels share zero weight because they straddle the car center. */
static const float g_line_position_weight[
    DEVICE_XINLIU_T8_CHANNEL_COUNT] = {
    -TEMPLATE_BRINGUP_LINE_OUTER_WEIGHT,
    -TEMPLATE_BRINGUP_LINE_MIDDLE_WEIGHT,
    -TEMPLATE_BRINGUP_LINE_INNER_WEIGHT,
    0.0F,
    0.0F,
    TEMPLATE_BRINGUP_LINE_INNER_WEIGHT,
    TEMPLATE_BRINGUP_LINE_MIDDLE_WEIGHT,
    TEMPLATE_BRINGUP_LINE_OUTER_WEIGHT,
};

/* VOFA+ FireWater: one ASCII line per frame, comma-separated values ending
 * with \n. Integer-only formatting keeps newlib-nano printf small; yaw is
 * printed as a fixed-point value with two decimals. */
#define BRINGUP_TELEMETRY_LINE_CAPACITY 224U

#define BRINGUP_DISPLAY_WIDTH 128U
#define BRINGUP_DISPLAY_LINE_HEIGHT 16U
#define BRINGUP_DISPLAY_CHARACTER_WIDTH 8U
#define BRINGUP_DISPLAY_CHARACTER_COUNT \
    (BRINGUP_DISPLAY_WIDTH / BRINGUP_DISPLAY_CHARACTER_WIDTH)
#define BRINGUP_DISPLAY_STATUS_LINE_COUNT 10U

#define BRINGUP_DISPLAY_COLOR_BLACK UINT16_C(0x0000)
#define BRINGUP_DISPLAY_COLOR_RED UINT16_C(0xF800)
#define BRINGUP_DISPLAY_COLOR_GREEN UINT16_C(0x07E0)
#define BRINGUP_DISPLAY_COLOR_YELLOW UINT16_C(0xFFE0)
#define BRINGUP_DISPLAY_COLOR_CYAN UINT16_C(0x07FF)
#define BRINGUP_DISPLAY_COLOR_WHITE UINT16_C(0xFFFF)

/* Compact Task-owned 5x7 uppercase font. Each row uses bits 4..0 from left
 * to right; the renderer doubles rows into an 8x16 status cell. */
static const uint8_t g_bringup_font_5x7['Z' - ' ' + 1U][7] = {
    ['+' - ' '] = {0x00U, 0x04U, 0x04U, 0x1FU, 0x04U, 0x04U, 0x00U},
    ['-' - ' '] = {0x00U, 0x00U, 0x00U, 0x1FU, 0x00U, 0x00U, 0x00U},
    ['.' - ' '] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x0CU, 0x0CU},
    ['/' - ' '] = {0x01U, 0x02U, 0x02U, 0x04U, 0x08U, 0x08U, 0x10U},
    ['0' - ' '] = {0x0EU, 0x11U, 0x13U, 0x15U, 0x19U, 0x11U, 0x0EU},
    ['1' - ' '] = {0x04U, 0x0CU, 0x04U, 0x04U, 0x04U, 0x04U, 0x0EU},
    ['2' - ' '] = {0x0EU, 0x11U, 0x01U, 0x02U, 0x04U, 0x08U, 0x1FU},
    ['3' - ' '] = {0x1EU, 0x01U, 0x01U, 0x0EU, 0x01U, 0x01U, 0x1EU},
    ['4' - ' '] = {0x02U, 0x06U, 0x0AU, 0x12U, 0x1FU, 0x02U, 0x02U},
    ['5' - ' '] = {0x1FU, 0x10U, 0x10U, 0x1EU, 0x01U, 0x01U, 0x1EU},
    ['6' - ' '] = {0x0EU, 0x10U, 0x10U, 0x1EU, 0x11U, 0x11U, 0x0EU},
    ['7' - ' '] = {0x1FU, 0x01U, 0x02U, 0x04U, 0x08U, 0x08U, 0x08U},
    ['8' - ' '] = {0x0EU, 0x11U, 0x11U, 0x0EU, 0x11U, 0x11U, 0x0EU},
    ['9' - ' '] = {0x0EU, 0x11U, 0x11U, 0x0FU, 0x01U, 0x01U, 0x0EU},
    [':' - ' '] = {0x00U, 0x0CU, 0x0CU, 0x00U, 0x0CU, 0x0CU, 0x00U},
    ['A' - ' '] = {0x0EU, 0x11U, 0x11U, 0x1FU, 0x11U, 0x11U, 0x11U},
    ['B' - ' '] = {0x1EU, 0x11U, 0x11U, 0x1EU, 0x11U, 0x11U, 0x1EU},
    ['C' - ' '] = {0x0EU, 0x11U, 0x10U, 0x10U, 0x10U, 0x11U, 0x0EU},
    ['D' - ' '] = {0x1EU, 0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x1EU},
    ['E' - ' '] = {0x1FU, 0x10U, 0x10U, 0x1EU, 0x10U, 0x10U, 0x1FU},
    ['F' - ' '] = {0x1FU, 0x10U, 0x10U, 0x1EU, 0x10U, 0x10U, 0x10U},
    ['G' - ' '] = {0x0EU, 0x11U, 0x10U, 0x17U, 0x11U, 0x11U, 0x0FU},
    ['H' - ' '] = {0x11U, 0x11U, 0x11U, 0x1FU, 0x11U, 0x11U, 0x11U},
    ['I' - ' '] = {0x0EU, 0x04U, 0x04U, 0x04U, 0x04U, 0x04U, 0x0EU},
    ['J' - ' '] = {0x07U, 0x02U, 0x02U, 0x02U, 0x12U, 0x12U, 0x0CU},
    ['K' - ' '] = {0x11U, 0x12U, 0x14U, 0x18U, 0x14U, 0x12U, 0x11U},
    ['L' - ' '] = {0x10U, 0x10U, 0x10U, 0x10U, 0x10U, 0x10U, 0x1FU},
    ['M' - ' '] = {0x11U, 0x1BU, 0x15U, 0x15U, 0x11U, 0x11U, 0x11U},
    ['N' - ' '] = {0x11U, 0x19U, 0x15U, 0x13U, 0x11U, 0x11U, 0x11U},
    ['O' - ' '] = {0x0EU, 0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x0EU},
    ['P' - ' '] = {0x1EU, 0x11U, 0x11U, 0x1EU, 0x10U, 0x10U, 0x10U},
    ['Q' - ' '] = {0x0EU, 0x11U, 0x11U, 0x11U, 0x15U, 0x12U, 0x0DU},
    ['R' - ' '] = {0x1EU, 0x11U, 0x11U, 0x1EU, 0x14U, 0x12U, 0x11U},
    ['S' - ' '] = {0x0FU, 0x10U, 0x10U, 0x0EU, 0x01U, 0x01U, 0x1EU},
    ['T' - ' '] = {0x1FU, 0x04U, 0x04U, 0x04U, 0x04U, 0x04U, 0x04U},
    ['U' - ' '] = {0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x0EU},
    ['V' - ' '] = {0x11U, 0x11U, 0x11U, 0x11U, 0x0AU, 0x0AU, 0x04U},
    ['W' - ' '] = {0x11U, 0x11U, 0x11U, 0x15U, 0x15U, 0x1BU, 0x11U},
    ['X' - ' '] = {0x11U, 0x11U, 0x0AU, 0x04U, 0x0AU, 0x11U, 0x11U},
    ['Y' - ' '] = {0x11U, 0x11U, 0x0AU, 0x04U, 0x04U, 0x04U, 0x04U},
    ['Z' - ' '] = {0x1FU, 0x01U, 0x02U, 0x04U, 0x08U, 0x10U, 0x1FU},
};

volatile uint32_t g_BRINGUP_Fault = 0U;
volatile uint32_t g_BRINGUP_DisplayFault = 0U;

static StaticTask_t g_bringup_tcb;
static StackType_t g_bringup_stack[BRINGUP_TASK_STACK_DEPTH];

static Device_Motor g_motor[BRINGUP_MOTOR_COUNT];
static Device_Encoder g_encoder[BRINGUP_MOTOR_COUNT];
static Device_KeyInput g_start_key;
static Device_RGB g_rgb;
static Device_Serial g_bluetooth;
static Device_Serial g_grayscale_serial;
static Device_XinliuT8 g_grayscale;
static Device_LineTracking g_line_tracking;
static Device_ST7735S g_display;
static Device_ICM45686_SPI g_imu;
static Mode_Communicate_BluetoothPCContext g_bluetooth_mode;
static Mode_Car_SpeedController g_speed_controller;
static Mode_Car_AnalogTrackingContext g_tracking_controller;
static Device_PeerLink g_peer_link;
static uint16_t g_display_line_pixels[
    BRINGUP_DISPLAY_WIDTH * BRINGUP_DISPLAY_LINE_HEIGHT];
static char g_display_last_text[BRINGUP_DISPLAY_STATUS_LINE_COUNT]
    [BRINGUP_DISPLAY_CHARACTER_COUNT + 1U];
static uint16_t
    g_display_last_foreground[BRINGUP_DISPLAY_STATUS_LINE_COUNT];

static bool g_devices_ready = false;
static bool g_peer_link_ready = false;
/* 0 none, 1 init, 2 periodic step, 3 motor-disable request send. */
static volatile uint32_t g_peer_link_fault = 0U;
static bool g_tracking_active = false;
static size_t g_display_status_line = 0U;
static Device_LineTracking_Sample g_line_sample;
static bool g_line_sample_available = false;
static uint32_t g_processed_grayscale_sequence = 0U;
static uint32_t g_previous_line_sample_ms = 0U;
static bool g_has_previous_line_sample_time = false;
static float g_target_left_speed = 0.0F;
static float g_target_right_speed = 0.0F;
static bool g_tracking_drive_valid = false;

static uint32_t bringup_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static Mode_Car_AnalogTrackingConfig bringup_tracking_config(void)
{
    Mode_Car_AnalogTrackingConfig config =
        Mode_Car_AnalogTracking_DefaultConfig();

    config.steering.kp = TEMPLATE_BRINGUP_LINE_STEERING_KP;
    return config;
}

static void bringup_show_status(void)
{
    Device_RGB_Color color;

    if (g_BRINGUP_Fault != 0U) {
        color = (Device_RGB_Color){
            TEMPLATE_BRINGUP_RGB_BRIGHTNESS, 0U, 0U};
    } else if (g_tracking_active) {
        color = (Device_RGB_Color){
            0U, 0U, TEMPLATE_BRINGUP_RGB_BRIGHTNESS};
    } else {
        color = (Device_RGB_Color){
            0U, TEMPLATE_BRINGUP_RGB_BRIGHTNESS, 0U};
    }
    /* StartColor is non-blocking; a busy frame is retried next cycle. */
    (void)Device_RGB_StartColor(&g_rgb, color);
}

static void bringup_init_peer_link(uint32_t now_ms)
{
    const Device_PeerLink_Config config = {
        .transport_index = BRINGUP_PEER_LINK_TRANSPORT_INDEX,
        .local_node_id = TEMPLATE_PEER_LINK_LOCAL_NODE_ID,
        .remote_node_id = TEMPLATE_PEER_LINK_REMOTE_NODE_ID,
        .heartbeat_period_ms = TEMPLATE_PEER_LINK_HEARTBEAT_PERIOD_MS,
        .offline_timeout_ms = TEMPLATE_PEER_LINK_OFFLINE_TIMEOUT_MS,
        .maximum_rx_bytes_per_step =
            TEMPLATE_PEER_LINK_RX_BYTES_PER_STEP,
    };

    g_peer_link_fault = 0U;
    g_peer_link_ready = Device_PeerLink_Init(
        &g_peer_link, &config, now_ms);
    if (!g_peer_link_ready) {
        g_peer_link_fault = 1U;
    }
}

static void bringup_init_display(void)
{
    Device_ST7735S_Config config;
    uint16_t width;
    uint16_t height;

    g_BRINGUP_DisplayFault = 0U;
    if (!Device_ST7735S_DefaultConfig(
            BRINGUP_TFT_DISPLAY_INDEX, &config)) {
        g_BRINGUP_DisplayFault = 1U;
        return;
    }
    if (Device_ST7735S_Init(&g_display, &config) !=
        DEVICE_ST7735S_RESULT_OK) {
        g_BRINGUP_DisplayFault = 2U;
        return;
    }
    if (!Device_ST7735S_GetSize(&g_display, &width, &height)) {
        g_BRINGUP_DisplayFault = 3U;
        return;
    }
    if (Device_ST7735S_Fill(&g_display,
            0U, 0U, (uint16_t)(width - 1U),
            (uint16_t)(height - 1U),
            Device_ST7735S_Color565(255U, 0U, 0U)) !=
        DEVICE_ST7735S_RESULT_OK) {
        g_BRINGUP_DisplayFault = 4U;
    }
}

static const uint8_t *bringup_display_glyph(char character)
{
    unsigned int value = (unsigned char)character;

    if ((value < (unsigned int)' ') || (value > (unsigned int)'Z')) {
        value = (unsigned int)' ';
    }
    return g_bringup_font_5x7[value - (unsigned int)' '];
}

static bool bringup_display_write_line(
    size_t line_index, const char *text, uint16_t foreground)
{
    if ((line_index >= BRINGUP_DISPLAY_STATUS_LINE_COUNT) ||
        (text == NULL)) {
        return false;
    }
    for (size_t index = 0U;
         index < (sizeof(g_display_line_pixels) /
             sizeof(g_display_line_pixels[0]));
         ++index) {
        g_display_line_pixels[index] = BRINGUP_DISPLAY_COLOR_BLACK;
    }
    for (size_t character_index = 0U;
         (character_index < BRINGUP_DISPLAY_CHARACTER_COUNT) &&
         (text[character_index] != '\0');
         ++character_index) {
        const uint8_t *glyph =
            bringup_display_glyph(text[character_index]);

        for (size_t glyph_y = 0U; glyph_y < 7U; ++glyph_y) {
            const size_t pixel_y = 1U + (2U * glyph_y);

            for (size_t glyph_x = 0U; glyph_x < 5U; ++glyph_x) {
                if ((glyph[glyph_y] &
                        (uint8_t)(UINT8_C(1) << (4U - glyph_x))) != 0U) {
                    const size_t pixel_x =
                        (character_index * BRINGUP_DISPLAY_CHARACTER_WIDTH) +
                        1U + glyph_x;

                    g_display_line_pixels[
                        pixel_y * BRINGUP_DISPLAY_WIDTH + pixel_x] =
                        foreground;
                    g_display_line_pixels[
                        (pixel_y + 1U) * BRINGUP_DISPLAY_WIDTH + pixel_x] =
                        foreground;
                }
            }
        }
    }
    return Device_ST7735S_WriteRGB565(&g_display,
        0U,
        (uint16_t)(line_index * BRINGUP_DISPLAY_LINE_HEIGHT),
        BRINGUP_DISPLAY_WIDTH - 1U,
        (uint16_t)(((line_index + 1U) *
            BRINGUP_DISPLAY_LINE_HEIGHT) - 1U),
        g_display_line_pixels,
        sizeof(g_display_line_pixels) /
            sizeof(g_display_line_pixels[0])) == DEVICE_ST7735S_RESULT_OK;
}

static uint32_t bringup_display_magnitude(int32_t value)
{
    return (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static bool bringup_line_sample_fresh(uint32_t now_ms)
{
    return g_line_sample_available &&
        ((uint32_t)(now_ms - g_line_sample.sampled_at_ms) <=
            TEMPLATE_BRINGUP_GRAYSCALE_TIMEOUT_MS);
}

static void bringup_update_display(uint32_t now_ms)
{
    Device_PeerLink_Telemetry telemetry = {0};
    Device_PeerLink_AbsoluteEncoder encoder = {0};
    char text[BRINGUP_DISPLAY_CHARACTER_COUNT + 1U];
    uint16_t foreground = BRINGUP_DISPLAY_COLOR_WHITE;
    const bool link_online =
        g_peer_link_ready && g_peer_link.status.online;
    const bool telemetry_valid = g_peer_link_ready &&
        Device_PeerLink_GetRemoteTelemetry(
            &g_peer_link, now_ms,
            TEMPLATE_PEER_LINK_TELEMETRY_TIMEOUT_MS, &telemetry);
    const bool encoder_valid = g_peer_link_ready &&
        Device_PeerLink_GetRemoteAbsoluteEncoder(
            &g_peer_link, now_ms,
            TEMPLATE_PEER_LINK_ENCODER_TIMEOUT_MS, &encoder) &&
        encoder.online;

    if (g_BRINGUP_DisplayFault != 0U) {
        return;
    }
    switch (g_display_status_line) {
        case 0U: {
            const bool sample_fresh = bringup_line_sample_fresh(now_ms);

            if (!sample_fresh) {
                (void)snprintf(text, sizeof(text), "LINE:STALE");
                foreground = BRINGUP_DISPLAY_COLOR_RED;
            } else if (!g_line_sample.line_present) {
                (void)snprintf(text, sizeof(text), "LINE:LOST");
                foreground = BRINGUP_DISPLAY_COLOR_RED;
            } else {
                const int32_t error_milli =
                    (int32_t)(g_line_sample.error * 1000.0F);
                uint32_t magnitude =
                    bringup_display_magnitude(error_milli);

                if (magnitude > 1000U) {
                    magnitude = 1000U;
                }
                text[0] = 'L';
                text[1] = 'I';
                text[2] = 'N';
                text[3] = 'E';
                text[4] = ':';
                text[5] = (error_milli < 0) ? '-' : '+';
                text[6] = (char)('0' + (magnitude / 1000U));
                text[7] = '.';
                text[8] = (char)('0' + ((magnitude / 100U) % 10U));
                text[9] = (char)('0' + ((magnitude / 10U) % 10U));
                text[10] = (char)('0' + (magnitude % 10U));
                text[11] = '\0';
                foreground = BRINGUP_DISPLAY_COLOR_CYAN;
            }
            break;
        }
        case 1U:
            (void)snprintf(text, sizeof(text),
                "CAR:%s", g_tracking_active ? "RUN" : "STOP");
            foreground = g_tracking_active ? BRINGUP_DISPLAY_COLOR_GREEN :
                BRINGUP_DISPLAY_COLOR_YELLOW;
            break;
        case 2U:
            (void)snprintf(text, sizeof(text),
                "LINK:%s F:%lu", link_online ? "OK" : "LOST",
                (unsigned long)g_peer_link_fault);
            foreground = link_online ? BRINGUP_DISPLAY_COLOR_GREEN :
                BRINGUP_DISPLAY_COLOR_RED;
            break;
        case 3U:
            (void)snprintf(text, sizeof(text),
                "QD:%s", (telemetry_valid && telemetry.online) ?
                    "ONLINE" : "OFFLINE");
            foreground = (telemetry_valid && telemetry.online) ?
                BRINGUP_DISPLAY_COLOR_GREEN : BRINGUP_DISPLAY_COLOR_RED;
            break;
        case 4U:
            if (telemetry_valid) {
                (void)snprintf(text, sizeof(text), "ID:%u EN:%u",
                    (unsigned)telemetry.motor_id,
                    telemetry.enabled ? 1U : 0U);
            } else {
                (void)snprintf(text, sizeof(text), "ID:-- EN:--");
            }
            break;
        case 5U:
            if (telemetry_valid) {
                const uint32_t magnitude =
                    bringup_display_magnitude(telemetry.current_milliamps);

                (void)snprintf(text, sizeof(text), "I:%s%lu.%03luA",
                    (telemetry.current_milliamps < 0) ? "-" : "",
                    (unsigned long)(magnitude / 1000U),
                    (unsigned long)(magnitude % 1000U));
            } else {
                (void)snprintf(text, sizeof(text), "I:--");
            }
            break;
        case 6U:
            if (telemetry_valid) {
                const uint32_t magnitude =
                    bringup_display_magnitude(telemetry.speed_centi_rpm);

                (void)snprintf(text, sizeof(text), "RPM:%s%lu.%02lu",
                    (telemetry.speed_centi_rpm < 0) ? "-" : "",
                    (unsigned long)(magnitude / 100U),
                    (unsigned long)(magnitude % 100U));
            } else {
                (void)snprintf(text, sizeof(text), "RPM:--");
            }
            break;
        case 7U:
            if (telemetry_valid) {
                (void)snprintf(text, sizeof(text), "ANG:%lu.%03lu",
                    (unsigned long)(telemetry.angle_milliradians / 1000U),
                    (unsigned long)(telemetry.angle_milliradians % 1000U));
            } else {
                (void)snprintf(text, sizeof(text), "ANG:--");
            }
            break;
        case 8U:
            if (encoder_valid) {
                (void)snprintf(text, sizeof(text), "ABS:%u",
                    (unsigned)encoder.angle_raw);
            } else {
                (void)snprintf(text, sizeof(text), "ABS:--");
            }
            break;
        default:
            (void)snprintf(text, sizeof(text), "HB:%lu F:%lu",
                (unsigned long)g_peer_link.status.heartbeat_rx_count,
                (unsigned long)g_BRINGUP_Fault);
            foreground = (g_BRINGUP_Fault == 0U) ?
                BRINGUP_DISPLAY_COLOR_WHITE : BRINGUP_DISPLAY_COLOR_RED;
            break;
    }
    if ((foreground !=
            g_display_last_foreground[g_display_status_line]) ||
        (strcmp(text,
            g_display_last_text[g_display_status_line]) != 0)) {
        if (!bringup_display_write_line(
                g_display_status_line, text, foreground)) {
            g_BRINGUP_DisplayFault = 5U;
            return;
        }
        (void)snprintf(g_display_last_text[g_display_status_line],
            sizeof(g_display_last_text[g_display_status_line]),
            "%s", text);
        g_display_last_foreground[g_display_status_line] = foreground;
    }
    g_display_status_line =
        (g_display_status_line + 1U) % BRINGUP_DISPLAY_STATUS_LINE_COUNT;
}

/* Every motor command passes through this clamp so no code path can exceed
 * the TB6612 protection limit. */
static Device_Motor_Result bringup_command_motor(
    Device_Motor *motor, int32_t command)
{
    if (command > TEMPLATE_BRINGUP_MOTOR_COMMAND_LIMIT) {
        command = TEMPLATE_BRINGUP_MOTOR_COMMAND_LIMIT;
    } else if (command < -TEMPLATE_BRINGUP_MOTOR_COMMAND_LIMIT) {
        command = -TEMPLATE_BRINGUP_MOTOR_COMMAND_LIMIT;
    }
    return Device_Motor_SetCommand(motor, command);
}

static bool bringup_init_devices(void)
{
    Device_RGB_Config rgb_config;
    Device_KeyInput_Config key_config = {
        .key = {
            .debounce_ms = TEMPLATE_BRINGUP_KEY_DEBOUNCE_MS,
            .long_press_ms = TEMPLATE_BRINGUP_KEY_LONG_PRESS_MS,
            .double_click_ms = TEMPLATE_BRINGUP_KEY_DOUBLE_CLICK_MS,
        },
        .input_index = 0U,
    };
    const Device_Serial_Config bluetooth_config = {
        .transport_index = BRINGUP_BLUETOOTH_TRANSPORT_INDEX,
        .expected_baud_rate = BRINGUP_UART_BAUD,
        .minimum_tx_capacity = BRINGUP_TELEMETRY_LINE_CAPACITY,
        .minimum_rx_capacity = 0U,
        .require_buffered_tx = true,
        .require_buffered_rx = false,
    };
    const Device_Serial_Config grayscale_config = {
        .transport_index = BRINGUP_GRAYSCALE_TRANSPORT_INDEX,
        .expected_baud_rate = BRINGUP_UART_BAUD,
        .minimum_tx_capacity = DEVICE_XINLIU_T8_READ_ANALOG_QUERY_SIZE,
        .minimum_rx_capacity = 2U * DEVICE_XINLIU_T8_ANALOG_RESPONSE_SIZE,
        .require_buffered_tx = true,
        .require_buffered_rx = true,
    };
    const Device_ICM45686_SPI_Config imu_transport = {
        .transport_index = BRINGUP_ICM45686_SPI_INDEX,
        .chip_select_index = 0U,
        .expected_bit_rate_hz = BRINGUP_ICM45686_SPI_BAUD,
        .transfer_timeout_ms = 5U,
        .startup_delay_ms = 10U,
    };
    Device_IMU_Config imu_config = Device_IMU_DefaultConfig();
    Device_LineTracking_Config line_config;

    if (!Device_RGB_DefaultConfig(0U, &rgb_config) ||
        !Device_RGB_Init(&g_rgb, &rgb_config)) {
        g_BRINGUP_Fault = 1U;
        return false;
    }
    if (!Device_KeyInput_Init(&g_start_key, &key_config, bringup_now_ms())) {
        g_BRINGUP_Fault = 2U;
        return false;
    }
    for (size_t index = 0U; index < BRINGUP_MOTOR_COUNT; ++index) {
        static const int8_t encoder_direction[BRINGUP_MOTOR_COUNT] = {
            TEMPLATE_BRINGUP_ENCODER_A_DIRECTION,
            TEMPLATE_BRINGUP_ENCODER_B_DIRECTION,
        };
        Device_Motor_Config motor_config;
        Device_Encoder_Config encoder_config;

        if (!Device_Motor_DefaultConfig(index, &motor_config) ||
            !Device_Motor_Init(&g_motor[index], &motor_config)) {
            g_BRINGUP_Fault = 3U;
            return false;
        }
        if (!Device_Encoder_MakeConfig(index,
                BRINGUP_COUNTS_PER_WHEEL_REV,
                BRINGUP_WHEEL_CIRCUMFERENCE_MM,
                encoder_direction[index],
                &encoder_config) ||
            !Device_Encoder_Init(&g_encoder[index], &encoder_config)) {
            g_BRINGUP_Fault = 4U;
            return false;
        }
    }
    if (!Device_Serial_Init(&g_bluetooth, &bluetooth_config)) {
        g_BRINGUP_Fault = 5U;
        return false;
    }
    /* Template policy layer: PC Bluetooth is a raw byte stream whose upper
     * format is decided by this product Task (FireWater text lines). */
    {
        Mode_Communicate_BluetoothOutput bluetooth_policy;

        if (!Mode_Communicate_BluetoothPC_Init(&g_bluetooth_mode) ||
            !Mode_Communicate_BluetoothPC_Update(
                &g_bluetooth_mode, &bluetooth_policy) ||
            (bluetooth_policy.format !=
                MODE_COMMUNICATE_BLUETOOTH_FORMAT_RAW_STREAM)) {
            g_BRINGUP_Fault = 5U;
            return false;
        }
    }
    /* Stationary gyro-bias calibration runs right after boot; keep the car
     * still for the first ~2 s so yaw drift stays low. */
    imu_config.start_calibration_on_init = true;
    imu_config.calibration_required_samples = 200U;
    if (!Device_ICM45686_SPI_Init(&g_imu, &imu_transport, &imu_config)) {
        g_BRINGUP_Fault = 6U;
        return false;
    }
    if (!Device_Serial_Init(&g_grayscale_serial, &grayscale_config) ||
        !Device_XinliuT8_Init(&g_grayscale)) {
        g_BRINGUP_Fault = 8U;
        return false;
    }
    if (!Device_LineTracking_DefaultRawConfig(
            DEVICE_XINLIU_T8_CHANNEL_COUNT, 0U, 4095U, &line_config)) {
        g_BRINGUP_Fault = 10U;
        return false;
    }
    for (size_t index = 0U;
         index < DEVICE_XINLIU_T8_CHANNEL_COUNT;
         ++index) {
        line_config.channel[index].position_weight =
            g_line_position_weight[index];
    }
    if (!Device_LineTracking_Init(&g_line_tracking, &line_config)) {
        g_BRINGUP_Fault = 10U;
        return false;
    }
    {
        const Mode_Car_SpeedControllerConfig speed_config =
            Mode_Car_SpeedController_DefaultConfig();

        if (!Mode_Car_SpeedController_Init(
                &g_speed_controller, &speed_config)) {
            g_BRINGUP_Fault = 9U;
            return false;
        }
    }
    {
        const Mode_Car_AnalogTrackingConfig tracking_config =
            bringup_tracking_config();

        if ((TEMPLATE_BRINGUP_LINE_BASE_SPEED_MM_PER_SECOND <= 0.0F) ||
            (TEMPLATE_BRINGUP_LINE_BASE_SPEED_MM_PER_SECOND >
                TEMPLATE_CAR_SPEED_MAXIMUM_TARGET_MM_PER_SECOND) ||
            !Mode_Car_AnalogTracking_Init(
                &g_tracking_controller, &tracking_config)) {
            g_BRINGUP_Fault = 10U;
            return false;
        }
    }
    return true;
}

static void bringup_stop_motors(void)
{
    for (size_t index = 0U; index < BRINGUP_MOTOR_COUNT; ++index) {
        (void)Device_Motor_Stop(&g_motor[index]);
        (void)Device_Motor_SetUnlocked(&g_motor[index], false);
    }
    Mode_Car_SpeedController_Reset(&g_speed_controller);
    {
        const Mode_Car_AnalogTrackingConfig tracking_config =
            bringup_tracking_config();

        (void)Mode_Car_AnalogTracking_Init(
            &g_tracking_controller, &tracking_config);
    }
    g_target_left_speed = 0.0F;
    g_target_right_speed = 0.0F;
    g_tracking_drive_valid = false;
    g_has_previous_line_sample_time = false;
    g_tracking_active = false;
}

static void bringup_start_tracking(void)
{
    for (size_t index = 0U; index < BRINGUP_MOTOR_COUNT; ++index) {
        if (Device_Motor_SetUnlocked(&g_motor[index], true) !=
                DEVICE_MOTOR_RESULT_OK) {
            g_BRINGUP_Fault = 3U;
            bringup_stop_motors();
            return;
        }
    }
    Mode_Car_SpeedController_Reset(&g_speed_controller);
    {
        const Mode_Car_AnalogTrackingConfig tracking_config =
            bringup_tracking_config();

        if (!Mode_Car_AnalogTracking_Init(
                &g_tracking_controller, &tracking_config)) {
            g_BRINGUP_Fault = 10U;
            bringup_stop_motors();
            return;
        }
    }
    g_processed_grayscale_sequence = 0U;
    g_has_previous_line_sample_time = false;
    g_target_left_speed = 0.0F;
    g_target_right_speed = 0.0F;
    g_tracking_drive_valid = false;
    g_tracking_active = true;
}

static bool bringup_pause_tracking(uint32_t now_ms, float dt_seconds)
{
    const Mode_Car_AnalogTrackingInput tracking_input = {
        .time = {
            .now_ms = now_ms,
            .dt_seconds = (dt_seconds > 0.0F) ? dt_seconds :
                (float)BRINGUP_TASK_PERIOD_MS * 0.001F,
        },
        .base_drive = 0.0F,
        .analog_error = 0.0F,
        .confidence = 0.0F,
        .line_valid = false,
        .line_lost = true,
        .intersection = false,
    };
    Mode_Car_Output stop_output;

    g_target_left_speed = 0.0F;
    g_target_right_speed = 0.0F;
    g_tracking_drive_valid = false;
    Mode_Car_SpeedController_Reset(&g_speed_controller);
    if (!Mode_Car_AnalogTracking_Update(
            &g_tracking_controller, &tracking_input, &stop_output)) {
        g_BRINGUP_Fault = 10U;
        return false;
    }
    if ((bringup_command_motor(&g_motor[0], 0) !=
            DEVICE_MOTOR_RESULT_OK) ||
        (bringup_command_motor(&g_motor[1], 0) !=
            DEVICE_MOTOR_RESULT_OK)) {
        g_BRINGUP_Fault = 3U;
        return false;
    }
    return true;
}

/* A fresh T8 frame updates the analog position/PID outer loop. Between T8
 * frames, the latest wheel-speed targets are held while the verified 10 ms
 * dual-wheel speed loop continues to close on encoder feedback. */
static void bringup_update_tracking(
    uint32_t now_ms,
    float dt_seconds,
    const Device_Encoder_Sample *speed_a,
    const Device_Encoder_Sample *speed_b)
{
    Device_XinliuT8_Snapshot snapshot;
    Mode_Car_SpeedControllerInput speed_input;
    Mode_Car_SpeedControllerOutput speed_output;

    if (dt_seconds <= 0.0F) {
        return;
    }
    if (!Device_XinliuT8_GetLatest(&g_grayscale, &snapshot) ||
        !Device_XinliuT8_SnapshotFresh(&snapshot, now_ms,
            TEMPLATE_BRINGUP_GRAYSCALE_TIMEOUT_MS)) {
        if (!bringup_pause_tracking(now_ms, dt_seconds)) {
            bringup_stop_motors();
        }
        return;
    }
    if (snapshot.sequence != g_processed_grayscale_sequence) {
        Device_LineTracking_RawSample raw = {
            .channel_count = DEVICE_XINLIU_T8_CHANNEL_COUNT,
            .valid_mask = UINT32_C(0xFF),
        };
        Mode_Car_AnalogTrackingInput tracking_input;
        Mode_Car_Output tracking_output;
        float tracking_dt_seconds =
            (float)BRINGUP_TASK_PERIOD_MS * 0.001F;

        for (size_t index = 0U;
             index < DEVICE_XINLIU_T8_CHANNEL_COUNT;
             ++index) {
            raw.value[index] = snapshot.adc[index];
        }
        if (g_has_previous_line_sample_time) {
            tracking_dt_seconds =
                (float)((uint32_t)(snapshot.received_at_ms -
                    g_previous_line_sample_ms)) * 0.001F;
        }
        if (!Device_LineTracking_UpdateAnalogRaw(
                &g_line_tracking, &raw, snapshot.received_at_ms) ||
            !Device_LineTracking_GetSample(
                &g_line_tracking, &g_line_sample)) {
            g_BRINGUP_Fault = 10U;
            bringup_stop_motors();
            return;
        }
        g_line_sample_available = true;
        tracking_input = (Mode_Car_AnalogTrackingInput){
            .time = {
                .now_ms = snapshot.received_at_ms,
                .dt_seconds = tracking_dt_seconds,
            },
            .base_drive =
                TEMPLATE_BRINGUP_LINE_BASE_SPEED_MM_PER_SECOND /
                TEMPLATE_CAR_SPEED_MAXIMUM_TARGET_MM_PER_SECOND,
            .analog_error = g_line_sample.error,
            .confidence = g_line_sample.confidence,
            .line_valid = g_line_sample.line_present,
            .line_lost = g_line_sample.line_lost,
            .intersection = g_line_sample.intersection,
        };
        if (!Mode_Car_AnalogTracking_Update(
                &g_tracking_controller, &tracking_input,
                &tracking_output)) {
            g_BRINGUP_Fault = 10U;
            bringup_stop_motors();
            return;
        }
        g_processed_grayscale_sequence = snapshot.sequence;
        g_previous_line_sample_ms = snapshot.received_at_ms;
        g_has_previous_line_sample_time = true;
        if (!tracking_output.motion_requested) {
            if (!bringup_pause_tracking(now_ms, dt_seconds)) {
                bringup_stop_motors();
            }
            return;
        }
        g_target_left_speed = tracking_output.left_drive *
            TEMPLATE_CAR_SPEED_MAXIMUM_TARGET_MM_PER_SECOND;
        g_target_right_speed = tracking_output.right_drive *
            TEMPLATE_CAR_SPEED_MAXIMUM_TARGET_MM_PER_SECOND;
        g_tracking_drive_valid = true;
    }
    if (!g_tracking_drive_valid || !g_line_sample_available ||
        !g_line_sample.line_present) {
        if (!bringup_pause_tracking(now_ms, dt_seconds)) {
            bringup_stop_motors();
        }
        return;
    }
    speed_input = (Mode_Car_SpeedControllerInput){
        .dt_seconds = dt_seconds,
        .target_left_speed = g_target_left_speed,
        .target_right_speed = g_target_right_speed,
        .measured_left_speed = speed_a->speed_distance_per_second,
        .measured_right_speed = speed_b->speed_distance_per_second,
    };
    if (!Mode_Car_SpeedController_Update(
            &g_speed_controller, &speed_input, &speed_output)) {
        g_BRINGUP_Fault = 9U;
        bringup_stop_motors();
        return;
    }
    if ((bringup_command_motor(&g_motor[0],
            (int32_t)(speed_output.left_effort_percent * 100.0F)) !=
            DEVICE_MOTOR_RESULT_OK) ||
        (bringup_command_motor(&g_motor[1],
            (int32_t)(speed_output.right_effort_percent * 100.0F)) !=
            DEVICE_MOTOR_RESULT_OK)) {
        g_BRINGUP_Fault = 3U;
        bringup_stop_motors();
    }
}

/* Poll the grayscale link: send one 0x19 query per telemetry period and
 * feed every buffered response byte into the frame parser. */
static void bringup_update_grayscale(uint32_t now_ms, bool send_query)
{
    uint8_t buffer[DEVICE_XINLIU_T8_ANALOG_RESPONSE_SIZE];
    size_t received = 0U;

    if (send_query) {
        uint8_t query[DEVICE_XINLIU_T8_READ_ANALOG_QUERY_SIZE];
        size_t length = 0U;

        if (Device_XinliuT8_BuildReadAnalogQuery(
                query, sizeof(query), &length)) {
            (void)Device_Serial_WriteBuffered(
                &g_grayscale_serial, query, length);
        }
    }
    while ((Device_Serial_ReadAvailable(&g_grayscale_serial,
                buffer, sizeof(buffer), &received) ==
            DEVICE_SERIAL_RESULT_OK) &&
        (received > 0U)) {
        for (size_t index = 0U; index < received; ++index) {
            (void)Device_XinliuT8_FeedByte(
                &g_grayscale, now_ms, buffer[index]);
        }
    }
}

static void bringup_send_firewater(
    uint32_t now_ms,
    const Device_Encoder_Sample *speed_a,
    const Device_Encoder_Sample *speed_b,
    float yaw_deg)
{
    char line[BRINGUP_TELEMETRY_LINE_CAPACITY];
    Device_XinliuT8_Snapshot grayscale = {0};
    Device_PeerLink_Telemetry f407 = {0};
    Device_PeerLink_AbsoluteEncoder f407_encoder = {0};
    int32_t command_a = 0;
    int32_t command_b = 0;
    int32_t line_error_milli = bringup_line_sample_fresh(now_ms)
        ? (int32_t)(g_line_sample.error * 1000.0F) : 0;
    int32_t yaw_centi = (int32_t)(yaw_deg * 100.0F);
    const char *line_error_sign = "";
    const char *yaw_sign = "";
    const char *f407_current_sign = "";
    const char *f407_speed_sign = "";
    int length;

    (void)Device_Motor_GetCommand(&g_motor[0], &command_a);
    (void)Device_Motor_GetCommand(&g_motor[1], &command_b);
    (void)Device_XinliuT8_GetLatest(&g_grayscale, &grayscale);
    if (line_error_milli < 0) {
        line_error_sign = "-";
        line_error_milli = -line_error_milli;
    }
    if (yaw_centi < 0) {
        yaw_sign = "-";
        yaw_centi = -yaw_centi;
    }
    if (!g_peer_link_ready ||
        !Device_PeerLink_GetRemoteTelemetry(
            &g_peer_link, now_ms,
            TEMPLATE_PEER_LINK_TELEMETRY_TIMEOUT_MS, &f407)) {
        f407 = (Device_PeerLink_Telemetry){0};
    }
    if (!g_peer_link_ready ||
        !Device_PeerLink_GetRemoteAbsoluteEncoder(
            &g_peer_link, now_ms,
            TEMPLATE_PEER_LINK_ENCODER_TIMEOUT_MS, &f407_encoder) ||
        !f407_encoder.online) {
        f407_encoder = (Device_PeerLink_AbsoluteEncoder){0};
    }
    if (f407.current_milliamps < 0) {
        f407_current_sign = "-";
        f407.current_milliamps = -f407.current_milliamps;
    }
    if (f407.speed_centi_rpm < 0) {
        f407_speed_sign = "-";
        f407.speed_centi_rpm = -f407.speed_centi_rpm;
    }

    /* Channel order: normalized analog line error (-1..1, 0 when stale),
     * wheel A/B speed mm/s, motor A/B command, measured yaw deg, grayscale
     * ADC 0..7, then F407 QD4310 ID/enabled/current A/speed rpm/angle
     * rad/online/raw angle. */
    length = snprintf(line, sizeof(line),
        "%s%ld.%03ld,%ld,%ld,%ld,%ld,%s%ld.%02ld,%u,%u,%u,%u,%u,%u,%u,%u,"
        "%u,%u,%s%ld.%03ld,%s%ld.%02ld,%lu.%03lu,%u,%u\n",
        line_error_sign,
        (long)(line_error_milli / 1000),
        (long)(line_error_milli % 1000),
        (long)speed_a->speed_distance_per_second,
        (long)speed_b->speed_distance_per_second,
        (long)command_a,
        (long)command_b,
        yaw_sign,
        (long)(yaw_centi / 100),
        (long)(yaw_centi % 100),
        (unsigned)grayscale.adc[0],
        (unsigned)grayscale.adc[1],
        (unsigned)grayscale.adc[2],
        (unsigned)grayscale.adc[3],
        (unsigned)grayscale.adc[4],
        (unsigned)grayscale.adc[5],
        (unsigned)grayscale.adc[6],
        (unsigned)grayscale.adc[7],
        (unsigned)f407.motor_id,
        f407.enabled ? 1U : 0U,
        f407_current_sign,
        (long)(f407.current_milliamps / 1000),
        (long)(f407.current_milliamps % 1000),
        f407_speed_sign,
        (long)(f407.speed_centi_rpm / 100),
        (long)(f407.speed_centi_rpm % 100),
        (unsigned long)(f407.angle_milliradians / 1000U),
        (unsigned long)(f407.angle_milliradians % 1000U),
        f407.online ? 1U : 0U,
        (unsigned)f407_encoder.angle_raw);
    if ((length > 0) && ((size_t)length < sizeof(line))) {
        /* Dropping a frame on BUSY is acceptable for bring-up telemetry. */
        (void)Device_Serial_WriteBuffered(
            &g_bluetooth, (const uint8_t *)line, (size_t)length);
    }
}

static void Task_BringupChassis(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t previous_ms;
    uint32_t last_telemetry_ms = 0U;
    uint32_t last_display_ms = 0U;

    (void)argument;
    g_devices_ready = bringup_init_devices();
    bringup_init_peer_link(bringup_now_ms());
    /* The panel reset/init is a bounded blocking startup operation. Running
     * it here, after the scheduler starts, lets the higher-priority Control
     * task continue feeding WWDT1. Display failure stays independent from
    * chassis and communication readiness. */
    bringup_init_display();
    last_wake = xTaskGetTickCount();
    previous_ms = bringup_now_ms();

    for (;;) {
        const uint32_t now_ms = bringup_now_ms();
        const float dt_seconds = (float)(now_ms - previous_ms) * 0.001F;

        previous_ms = now_ms;
        if (g_peer_link_ready &&
            !Device_PeerLink_Step(&g_peer_link, now_ms)) {
            g_peer_link_fault = 2U;
        }
        if (g_devices_ready) {
            Device_Encoder_Sample sample_a;
            Device_Encoder_Sample sample_b;
            Device_IMU_Snapshot imu_snapshot;
            float yaw_deg = 0.0F;
            bool pressed_edge = false;
            bool telemetry_due = false;

            (void)Device_Encoder_Update(&g_encoder[0], dt_seconds);
            (void)Device_Encoder_Update(&g_encoder[1], dt_seconds);
            (void)Device_Encoder_GetSample(&g_encoder[0], &sample_a);
            (void)Device_Encoder_GetSample(&g_encoder[1], &sample_b);

            if ((Device_ICM45686_SPI_Update(&g_imu, now_ms, dt_seconds,
                    NULL, &imu_snapshot) == DEVICE_ICM45686_STATUS_OK) &&
                imu_snapshot.has_sample) {
                yaw_deg = imu_snapshot.yaw_deg;
            }

            telemetry_due = (now_ms - last_telemetry_ms) >=
                TEMPLATE_BRINGUP_TELEMETRY_PERIOD_MS;
            bringup_update_grayscale(now_ms, telemetry_due);

            (void)Device_KeyInput_UpdatePressedEdge(
                &g_start_key, now_ms, &pressed_edge);

            if (pressed_edge && (g_BRINGUP_Fault == 0U)) {
                if (g_tracking_active) {
                    bringup_stop_motors();
                    if (!g_peer_link_ready ||
                        !Device_PeerLink_SendMotorDisableRequest(
                            &g_peer_link)) {
                        g_peer_link_fault = 3U;
                    }
                } else {
                    bringup_start_tracking();
                }
            }
            if (g_tracking_active) {
                bringup_update_tracking(
                    now_ms, dt_seconds, &sample_a, &sample_b);
            }

            if (telemetry_due) {
                last_telemetry_ms = now_ms;
                bringup_send_firewater(
                    now_ms, &sample_a, &sample_b, yaw_deg);
            }
        }
        if ((now_ms - last_display_ms) >=
            TEMPLATE_BRINGUP_TELEMETRY_PERIOD_MS) {
            last_display_ms = now_ms;
            bringup_update_display(now_ms);
        }
        bringup_show_status();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BRINGUP_TASK_PERIOD_MS));
    }
}

bool Task_BringupChassis_Init(void)
{
    TaskHandle_t task = xTaskCreateStatic(
        Task_BringupChassis,
        BRINGUP_TASK_NAME,
        BRINGUP_TASK_STACK_DEPTH,
        NULL,
        BRINGUP_TASK_PRIORITY,
        g_bringup_stack,
        &g_bringup_tcb);

    if (task == NULL) {
        g_BRINGUP_Fault = 7U;
        return false;
    }
    return true;
}
