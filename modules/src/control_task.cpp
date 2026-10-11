#include "control_task.h"
#include "DJIMotorHandler.hpp"
#include "M2006.hpp"
#include "bsp_can.hpp"
#include "tx_api.h"

#include <cmath>
#include <cstring>

namespace
{
M2006 motor;
TX_QUEUE feedback_queue;
CHAR feedback_queue_name[] = "M2006 feedback";
constexpr uint32_t kQueueDepth = 32U;
constexpr ULONG kFeedbackTimeoutTicks = TX_TIMER_TICKS_PER_SECOND / 10U;
/* Output-shaft sine reference, centered on the first valid feedback angle. */
constexpr float kSineAmplitudeRad = 0.5f;
constexpr ULONG kSinePeriodTicks = 6U * TX_TIMER_TICKS_PER_SECOND;
constexpr float kTwoPi = 6.28318530718f;
static_assert(kSinePeriodTicks > 0U, "Sine period is too short");
static_assert(sizeof(M2006_TraceSample) == 32U, "Trace layout used by plot script");
/* ThreadX queue message size is four ULONG words on this Cortex-M4 target. */
struct FeedbackFrame
{
    uint32_t id;
    CAN_HandleTypeDef *hcan;
    uint8_t data[8];
};
static_assert(sizeof(FeedbackFrame) == 4U * sizeof(ULONG), "CAN queue message size");
ULONG feedback_queue_storage[kQueueDepth * 4U];
bool queue_ready;
ULONG last_feedback_tick;
ULONG auto_start_tick;
float auto_center_rad;
bool auto_target_active;

void UpdateAutoPositionTarget(bool online, ULONG now)
{
    if (!g_m2006_debug.auto_rotate_enabled)
    {
        auto_target_active = false;
        motor.positionFeedforwardRadS = 0.0f;
        g_m2006_debug.auto_rotate_started = 0U;
        return;
    }
    if (!online)
    {
        auto_target_active = false;
        motor.positionFeedforwardRadS = 0.0f;
        g_m2006_debug.auto_rotate_started = 0U;
        g_m2006_debug.requested_mode = DJIMotor::RELAX_MODE;
        return;
    }
    if (!auto_target_active)
    {
        auto_center_rad = motor.motorFeedback.positionFdb;
        auto_start_tick = now;
        auto_target_active = true;
    }

    const ULONG phase_ticks = (now - auto_start_tick) % kSinePeriodTicks;
    const float omega = kTwoPi * static_cast<float>(TX_TIMER_TICKS_PER_SECOND) /
                        static_cast<float>(kSinePeriodTicks);
    const float phase_rad = kTwoPi * static_cast<float>(phase_ticks) /
                            static_cast<float>(kSinePeriodTicks);
    g_m2006_debug.position_target_rad = auto_center_rad +
        kSineAmplitudeRad * std::sin(phase_rad);
    motor.positionFeedforwardRadS = kSineAmplitudeRad * omega * std::cos(phase_rad);
    g_m2006_debug.requested_mode = DJIMotor::POS_MODE;
    g_m2006_debug.auto_rotate_started = 1U;
}

void ApplyRequestedMode(bool online)
{
    const uint8_t requested = g_m2006_debug.requested_mode;
    const float speed = g_m2006_debug.speed_target_rad_s;
    const float position = g_m2006_debug.position_target_rad;
    DJIMotor::MotorControlModeType next = DJIMotor::RELAX_MODE;
    if (online && requested == DJIMotor::SPD_MODE &&
        std::isfinite(speed) && std::fabs(speed) <= 50.0f)
        next = DJIMotor::SPD_MODE;
    else if (online && requested == DJIMotor::POS_MODE && std::isfinite(position))
        next = DJIMotor::POS_MODE;

    if (motor.controlMode != next)
    {
        motor.speedPid.Clear();
        motor.positionPid.Clear();
        motor.controlMode = next;
    }
    motor.speedSet = speed;
    motor.positionSet = position;
    if (next != DJIMotor::POS_MODE || !g_m2006_debug.auto_rotate_enabled)
        motor.positionFeedforwardRadS = 0.0f;
    g_m2006_debug.active_mode = static_cast<uint8_t>(next);
}
}

extern "C"
{
volatile M2006_DebugState g_m2006_debug = {};
volatile M2006_TraceSample g_m2006_trace[M2006_TRACE_CAPACITY] = {};
volatile uint32_t g_m2006_trace_next = 0U;
volatile uint32_t g_m2006_trace_count = 0U;
}

extern "C" uint8_t ControlTask_Init(void)
{
    if (!queue_ready)
    {
        if (tx_queue_create(&feedback_queue, feedback_queue_name, 4U,
                            feedback_queue_storage,
                            sizeof(feedback_queue_storage)) != TX_SUCCESS)
            return 0U;
        queue_ready = true;
    }

    DJIMotorHandler *handler = DJIMotorHandler::Instance();
    handler->registerMotor(&motor, &hcan1, M2006_FEEDBACK_STD_ID);
    motor.controlMode = DJIMotor::RELAX_MODE;
    motor.currentSet = 0;
    auto_target_active = false;
    g_m2006_debug.auto_rotate_enabled = 1U;
    g_m2006_debug.auto_rotate_started = 0U;
    return handler->DJIMotorList[0][M2006_FEEDBACK_STD_ID - 0x201U] == &motor ? 1U : 0U;
}

extern "C" void ControlTask_QueueFeedbackFromISR(CAN_HandleTypeDef *hcan,
                                                    uint32_t id,
                                                    const uint8_t *data)
{
    if (!queue_ready || hcan != &hcan1 || id != M2006_FEEDBACK_STD_ID || data == nullptr)
        return;
    FeedbackFrame frame{};
    frame.id = id;
    frame.hcan = hcan;
    std::memcpy(frame.data, data, sizeof(frame.data));
    if (tx_queue_send(&feedback_queue, &frame, TX_NO_WAIT) != TX_SUCCESS)
        ++g_m2006_debug.feedback_dropped;
}

extern "C" uint8_t ControlTask_Step(void)
{
    CAN_PollReceive();
    FeedbackFrame frame{};
    DJIMotorHandler *handler = DJIMotorHandler::Instance();
    while (tx_queue_receive(&feedback_queue, &frame, TX_NO_WAIT) == TX_SUCCESS)
    {
        handler->updateFeedback(frame.hcan, frame.data,
                                static_cast<int>(frame.id - 0x201U));
        last_feedback_tick = tx_time_get();
        ++g_m2006_debug.feedback_frames;
        g_m2006_debug.last_feedback_id = frame.id;
        for (uint32_t i = 0; i < sizeof(frame.data); ++i)
            g_m2006_debug.last_rx_data[i] = frame.data[i];
    }

    const ULONG now = tx_time_get();
    const bool online = motor.motorFeedback.ecdInitialized &&
                        (now - last_feedback_tick <= kFeedbackTimeoutTicks);
    if (!online && g_m2006_debug.feedback_online)
        g_m2006_debug.requested_mode = DJIMotor::RELAX_MODE;
    motor.MotorState = online ? DJIMotor::MOTOR_ONLINE : DJIMotor::MOTOR_OFFLINE;
    g_m2006_debug.feedback_online = online ? 1U : 0U;
    UpdateAutoPositionTarget(online, now);
    ApplyRequestedMode(online);

    const bool sent = handler->sendControlData();
    ++g_m2006_debug.control_cycles;
    if (sent)
        ++g_m2006_debug.tx_successes;
    else
        ++g_m2006_debug.tx_failures;

    const auto &feedback = motor.motorFeedback;
    g_m2006_debug.encoder_count = feedback.ecd;
    g_m2006_debug.rotor_speed_rpm = feedback.speed_rpm;
    g_m2006_debug.output_speed_rad_s = feedback.speedFdb;
    g_m2006_debug.output_position_rad = feedback.positionFdb;
    g_m2006_debug.current_command = motor.currentSet;
    g_m2006_debug.position_error_rad =
        motor.controlMode == DJIMotor::POS_MODE
            ? motor.positionPid.err[0] : 0.0f;
    g_m2006_debug.speed_target_active_rad_s =
        motor.controlMode == DJIMotor::RELAX_MODE ? 0.0f : motor.speedPid.ref;
    g_m2006_debug.speed_error_rad_s =
        motor.controlMode == DJIMotor::RELAX_MODE ? 0.0f : motor.speedPid.err[0];
    g_m2006_debug.position_pid_speed_rad_s =
        motor.controlMode == DJIMotor::POS_MODE ? motor.positionPid.result : 0.0f;
    g_m2006_debug.speed_pid_current =
        motor.controlMode == DJIMotor::RELAX_MODE ? 0.0f : motor.speedPid.result;
    for (uint32_t i = 0; i < sizeof(g_m2006_debug.last_tx_data); ++i)
        g_m2006_debug.last_tx_data[i] = handler->can1_send_data_0[i];

    const uint32_t next = g_m2006_trace_next;
    volatile M2006_TraceSample &trace = g_m2006_trace[next];
    trace.tick = static_cast<uint32_t>(now);
    trace.position_target_rad = g_m2006_debug.position_target_rad;
    trace.position_feedback_rad = feedback.positionFdb;
    trace.speed_target_rad_s = g_m2006_debug.speed_target_active_rad_s;
    trace.speed_feedback_rad_s = feedback.speedFdb;
    trace.position_pid_speed_rad_s = g_m2006_debug.position_pid_speed_rad_s;
    trace.speed_pid_current = g_m2006_debug.speed_pid_current;
    trace.current_command = motor.currentSet;
    trace.mode = g_m2006_debug.active_mode;
    trace.feedback_online = g_m2006_debug.feedback_online;
    g_m2006_trace_next = (next + 1U) % M2006_TRACE_CAPACITY;
    if (g_m2006_trace_count < M2006_TRACE_CAPACITY)
        ++g_m2006_trace_count;
    return sent ? 1U : 0U;
}

extern "C" uint8_t ControlTask_SetSpeedTarget(float speed_rad_s)
{
    if (!std::isfinite(speed_rad_s) || std::fabs(speed_rad_s) > 50.0f)
        return 0U;
    const UINT irq = tx_interrupt_control(TX_INT_DISABLE);
    g_m2006_debug.auto_rotate_enabled = 0U;
    g_m2006_debug.speed_target_rad_s = speed_rad_s;
    g_m2006_debug.requested_mode = DJIMotor::SPD_MODE;
    tx_interrupt_control(irq);
    return 1U;
}

extern "C" uint8_t ControlTask_SetPositionTarget(float position_rad)
{
    if (!std::isfinite(position_rad) || !g_m2006_debug.feedback_online)
        return 0U;
    const UINT irq = tx_interrupt_control(TX_INT_DISABLE);
    g_m2006_debug.auto_rotate_enabled = 0U;
    g_m2006_debug.position_target_rad = position_rad;
    g_m2006_debug.requested_mode = DJIMotor::POS_MODE;
    tx_interrupt_control(irq);
    return 1U;
}

extern "C" void ControlTask_Stop(void)
{
    g_m2006_debug.auto_rotate_enabled = 0U;
    g_m2006_debug.requested_mode = DJIMotor::RELAX_MODE;
}
