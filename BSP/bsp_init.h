#ifndef BSP_INIT_H
#define BSP_INIT_H

#include <stdbool.h>

/* 初始化时钟(HSE 25MHz -> SYSCLK 168MHz)、UART 与 CAN。
 * 必须在启动 FreeRTOS 调度器之前调用一次；失败时外设不可用。 */
bool BSP_Init(void);

#endif
