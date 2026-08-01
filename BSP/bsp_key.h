#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdbool.h>

/* Board KEY1: PA0 input with pull-down, active high. */
bool BSP_Key_IsPressed(void);

/* Board KEY2: PA1 input with pull-up, active low. */
bool BSP_Key2_IsPressed(void);

#endif
