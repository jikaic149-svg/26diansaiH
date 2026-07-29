/* F407-Gateway 入口：初始化 BSP，创建网关 Task，启动 FreeRTOS。
 * 流程与故障门与 TI-Template main.c 保持一致。 */

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_init.h"
#include "freertos_support.h"
#include "task_app.h"

#include "stm32f4xx.h"

volatile uint32_t g_MAIN_Fault = 0U;

int main(void)
{
    if (!BSP_Init()) {
        g_MAIN_Fault = 1U;
        for (;;) {
            __WFI();
        }
    }

    if (!FreeRTOS_Port_Validate()) {
        g_MAIN_Fault = 2U;
        for (;;) {
            __WFI();
        }
    }

    if (!Task_Init()) {
        g_MAIN_Fault = 3U;
        for (;;) {
            __WFI();
        }
    }

    vTaskStartScheduler();
    g_MAIN_Fault = 4U;

    for (;;) {
        __WFI();
    }
}
