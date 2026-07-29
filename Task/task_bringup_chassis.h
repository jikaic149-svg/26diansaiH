#ifndef TASK_BRINGUP_CHASSIS_H
#define TASK_BRINGUP_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Chassis bring-up (H-title): the first key press enables Xinliu T8 analog
 * line tracking. Each fresh eight-channel ADC frame is converted to a
 * normalized line error and confidence, then an analog tracking PID produces
 * left/right wheel-speed targets around a 250 mm/s forward target. The
 * verified dual-wheel speed controller remains the 10 ms inner loop. Lost,
 * low-confidence or older-than-100-ms line data clears both motor commands;
 * a later valid frame resumes tracking without another key press. The next
 * key press exits tracking, disables both chassis motors, and queues one
 * non-blocking DIS request for the F407 primary motor. Every motor command
 * stays inside the TB6612 protection clamp. Telemetry streams
 * continuously over the Bluetooth UART as VOFA+ FireWater ASCII lines
 * (comma-separated, \n terminated) with 21 values per line:
 *   0 normalized analog line error (-1..1; 0 when stale)
 *   1 wheel A speed mm/s        2 wheel B speed mm/s
 *   3 motor A command           4 motor B command
 *   5 yaw deg (ICM-45686 Mahony, two decimals)
 *   6..13 Xinliu T8 grayscale ADC channels 0..7
 *   14 F407 QD4310 ID           15 enabled
 *   16 current A                17 speed rpm
 *   18 single-turn angle rad    19 online
 *   20 QD4310 absolute encoder protocol raw value (0 when stale/offline)
 * The RGB pixel shows idle/running/fault. At startup the G3519 ST7735S is
 * initialized once and filled red. No ball control runs on this MCU.
 *
 * Fault codes (g_BRINGUP_Fault): 0 none, 1 RGB, 2 key, 3 motor, 4 encoder,
 * 5 Bluetooth serial, 6 IMU, 7 task creation, 8 grayscale serial,
 * 9 speed controller, 10 analog line estimator/controller.
 */
extern volatile uint32_t g_BRINGUP_Fault;

/* Display-only status: 0 success, 1 default configuration, 2 panel
 * initialization, 3 geometry query, 4 red full-screen fill, 5 status-line
 * update. A TFT failure does not change g_BRINGUP_Fault or stop
 * chassis/communication. */
extern volatile uint32_t g_BRINGUP_DisplayFault;

/* Create the bring-up task after BSP_Init(); returns false only when the
 * FreeRTOS task itself could not be created. Device failures are reported
 * through g_BRINGUP_Fault and the red RGB indicator instead. */
bool Task_BringupChassis_Init(void);

#endif
