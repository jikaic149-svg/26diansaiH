#ifndef TASK_BRINGUP_CHASSIS_H
#define TASK_BRINGUP_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Chassis bring-up (H-title): the first key press captures the current yaw
 * and enables an absolute-angle outer loop around the verified dual-wheel
 * speed inner loop. Left/right wheel-speed targets are equal and opposite,
 * so commanded linear speed stays zero; the next key press exits the lock
 * and disables both chassis motors, then queues one non-blocking DIS request
 * for the F407 primary motor. Every motor command stays inside the TB6612
 * protection clamp. Telemetry streams
 * continuously over the Bluetooth UART as VOFA+ FireWater ASCII lines
 * (comma-separated, \n terminated) with 21 values per line:
 *   0 locked yaw target deg (0 while idle)
 *   1 wheel A speed mm/s        2 wheel B speed mm/s
 *   3 motor A command           4 motor B command
 *   5 yaw deg (ICM-45686 Mahony, two decimals)
 *   6..13 Xinliu T8 grayscale ADC channels 0..7
 *   14 F407 QD4310 ID           15 enabled
 *   16 current A                17 speed rpm
 *   18 single-turn angle rad    19 online
 *   20 QD4310 absolute encoder protocol raw value (0 when stale/offline)
 * The RGB pixel shows idle/running/fault. At startup the G3519 ST7735S is
 * initialized once and filled red. No line following, no ball control.
 *
 * Fault codes (g_BRINGUP_Fault): 0 none, 1 RGB, 2 key, 3 motor, 4 encoder,
 * 5 Bluetooth serial, 6 IMU, 7 task creation, 8 grayscale serial,
 * 9 speed controller, 10 angle controller.
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
