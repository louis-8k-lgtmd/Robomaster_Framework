#ifndef RM26_CONTROL_TASK_H
#define RM26_CONTROL_TASK_H

#include <stdint.h>
#include "can.h"

/* C610 green LED blinks three times: ESC ID 3, feedback standard ID 0x203. */
#define M2006_FEEDBACK_STD_ID 0x203U
#define M2006_TRACE_CAPACITY 512U

#ifdef __cplusplus
extern "C" {
#endif

/* Debugger commands: 0 = relax, 1 = speed loop, 2 = position/speed loop.
 * Targets use the gearbox output shaft, in rad/s and rad respectively.
 * Automatic position/speed rotation starts at the first valid feedback frame. */
typedef struct
{
    uint8_t requested_mode;
    uint8_t auto_rotate_enabled;
    uint8_t auto_rotate_started;
    float speed_target_rad_s;
    float position_target_rad;
    uint32_t thread_entries;
    uint32_t init_ok;
    uint32_t can_ok;
    uint32_t control_cycles;
    uint32_t feedback_frames;
    uint32_t feedback_dropped;
    uint32_t tx_successes;
    uint32_t tx_failures;
    uint32_t last_feedback_id;
    uint8_t last_rx_data[8];
    uint8_t last_tx_data[8];
    uint8_t feedback_online;
    uint8_t active_mode;
    uint16_t encoder_count;
    int16_t rotor_speed_rpm;
    int16_t current_command;
    float output_speed_rad_s;
    float output_position_rad;
    float position_error_rad;
    float speed_target_active_rad_s;
    float speed_error_rad_s;
    float position_pid_speed_rad_s;
    float speed_pid_current;
} M2006_DebugState;

extern volatile M2006_DebugState g_m2006_debug;

/* Latest 512 control cycles (about 5.12 s at 100 Hz), for curve export. */
typedef struct
{
    uint32_t tick;
    float position_target_rad;
    float position_feedback_rad;
    float speed_target_rad_s;
    float speed_feedback_rad_s;
    float position_pid_speed_rad_s;
    float speed_pid_current;
    int16_t current_command;
    uint8_t mode;
    uint8_t feedback_online;
} M2006_TraceSample;

extern volatile M2006_TraceSample g_m2006_trace[M2006_TRACE_CAPACITY];
extern volatile uint32_t g_m2006_trace_next;
extern volatile uint32_t g_m2006_trace_count;

/* Called once by the motor thread before CAN_Init starts reception. */
uint8_t ControlTask_Init(void);
/* Called from CAN FIFO ISR; only queues the raw frame, without parsing/PID. */
void ControlTask_QueueFeedbackFromISR(CAN_HandleTypeDef *hcan,
                                      uint32_t id, const uint8_t *data);
/* Called every 10 ms by the motor thread: parse, control, pack and send. */
uint8_t ControlTask_Step(void);
uint8_t ControlTask_SetSpeedTarget(float speed_rad_s);
uint8_t ControlTask_SetPositionTarget(float position_rad);
void ControlTask_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
