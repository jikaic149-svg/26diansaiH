#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_init.h"
#include "freertos_support.h"
#include "task.h"
#include "task_app.h"
#include "ti_msp_dl_config.h"

#if defined(TEMPLATE_ENABLE_TASK_BRINGUP_CHASSIS)
#include "task_bringup_chassis.h"
#endif

volatile uint32_t g_MAIN_Fault = 0U;

int main(void)
{
    SYSCFG_DL_init();
    BSP_Init();

    if (!FreeRTOS_Port_Validate()) {
        g_MAIN_Fault = 1U;
        for (;;) {
            __WFI();
        }
    }

    if (!Task_Init()) {
        g_MAIN_Fault = 2U;
        for (;;) {
            __WFI();
        }
    }

#if defined(TEMPLATE_ENABLE_TASK_BRINGUP_CHASSIS)
    if (!Task_BringupChassis_Init()) {
        g_MAIN_Fault = 4U;
        for (;;) {
            __WFI();
        }
    }
#endif

    vTaskStartScheduler();
    g_MAIN_Fault = 3U;

    for (;;) {
        __WFI();
    }
}
