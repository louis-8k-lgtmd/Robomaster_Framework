#ifndef RM26_CONTROL_TASK_H
#define RM26_CONTROL_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register the motor and prepare its controller; does not leave RELAX mode. */
uint8_t ControlTask_Init(void);
/* Set a finite output-shaft target in radians after the first encoder feedback. */
uint8_t ControlTask_SetPositionTarget(float position_rad);
/* Clear the target and return the motor to RELAX mode. */
void ControlTask_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
