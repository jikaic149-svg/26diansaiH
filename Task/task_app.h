#ifndef TASK_APP_H
#define TASK_APP_H

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t g_TASK_Heartbeat;
extern volatile uint32_t g_TASK_Fault;
extern volatile uint32_t g_TASK_Ready;
extern volatile uint32_t g_TASK_NowMs;
extern volatile uint32_t g_TASK_BMI088_Status;
extern volatile uint32_t g_TASK_BMI088_Sequence;
extern volatile int32_t g_TASK_BMI088_AccelX_mm_s2;
extern volatile int32_t g_TASK_BMI088_AccelY_mm_s2;
extern volatile int32_t g_TASK_BMI088_AccelZ_mm_s2;
extern volatile int32_t g_TASK_BMI088_ProcessedAccel_mm_s2;

/* 创建静态网关 Task；必须在 vTaskStartScheduler() 之前调用。 */
bool Task_Init(void);

#endif
