#ifndef TASK_APP_H
#define TASK_APP_H

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t g_TASK_Heartbeat;
extern volatile uint32_t g_TASK_Fault;
extern volatile uint32_t g_TASK_Ready;
extern volatile uint32_t g_TASK_NowMs;

/* 创建静态网关 Task；必须在 vTaskStartScheduler() 之前调用。 */
bool Task_Init(void);

#endif
