/** @file track_speed.c
 *  @brief 单编码器速度闭环：参考系数换算、基础 PWM 前馈、PID 修正、抗饱和及堵转保护。
 */
#include "track_speed.h"

static float speed_clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void TrackSpeed_Stop(TrackSpeedState *state)
{
    state->command = 0;
    state->target = 0;
    state->integral = 0;
    state->derivative = 0;
    state->p_term = 0;
    state->d_term = 0;
    state->output = 0;
    state->no_pulse_ms = 0;
    state->initialized = 0;
    state->saturated = 0;
}

uint8_t TrackSpeed_Measure(TrackSpeedState *state, uint16_t pulses, uint16_t elapsed_ms)
{
    if (elapsed_ms == 0 || elapsed_ms > 2U * TRACK_SPEED_PERIOD_MS ||
        (uint32_t)pulses * 10UL > (uint32_t)TRACK_ENCODER_MAX_PER_10MS * elapsed_ms)
    {
        return 0;
    }
    state->raw_feedback = (float)pulses * 10.0f / elapsed_ms;
    if (!state->measured)
    {
        state->feedback = state->raw_feedback;
        state->measured = 1;
    }
    else
    {
        state->feedback += ((float)elapsed_ms / (TRACK_SPEED_FEEDBACK_TAU_MS + elapsed_ms)) *
                           (state->raw_feedback - state->feedback);
    }
    return 1;
}

TrackFault TrackSpeed_Calculate(TrackSpeedState *state, const TrackParams *params,
                               uint16_t requested, uint16_t pwm_limit, uint16_t elapsed_ms)
{
    float dt;
    float error;
    float candidate_integral;
    float unconstrained;
    float output;

    if (!TrackParams_Valid(params) || requested > TRACK_SPEED_COMMAND_MAX || pwm_limit > TRACK_DUTY_MAX)
    {
        TrackSpeed_Stop(state);
        return TRACK_BAD_PARAMS;
    }
    if (elapsed_ms == 0 || elapsed_ms > 2U * TRACK_SPEED_PERIOD_MS || !state->measured)
    {
        TrackSpeed_Stop(state);
        return TRACK_ENCODER_ERROR;
    }
    if (requested == 0 || pwm_limit == 0)
    {
        TrackSpeed_Stop(state);
        return TRACK_OK;
    }
    dt = elapsed_ms / 1000.0f;
    state->command = requested;
    /* 参考式是 20ms 计数 = 指令 * 0.104；内部 10ms 标度折算为指令 * 0.052。 */
    state->target = requested * TRACK_ENCODER_COUNTS_PER_COMMAND_10MS;
    error = state->target - state->feedback;
    if (!state->initialized)
    {
        state->last_feedback = state->feedback;
        state->initialized = 1;
    }
    /* 对测量做微分，避免按键改变目标时产生微分突跳。 */
    state->derivative += ((float)elapsed_ms / (TRACK_SPEED_D_TAU_MS + elapsed_ms)) *
        ((state->last_feedback - state->feedback) / dt - state->derivative);
    state->last_feedback = state->feedback;
    state->p_term = params->speed_kp100 * 0.01f * error;
    state->d_term = params->speed_kd100 * 0.01f * state->derivative;
    state->integral = speed_clamp(state->integral, -(float)pwm_limit, (float)pwm_limit);
    candidate_integral = speed_clamp(state->integral + params->speed_ki100 * 0.01f * error * dt,
                                     -(float)pwm_limit, (float)pwm_limit);
    /* 指令提供基础动力；PID 只补偿误差，积分允许为负以减小过剩动力。 */
    unconstrained = requested + state->p_term + candidate_integral + state->d_term;
    output = speed_clamp(unconstrained, 0, (float)pwm_limit);
    /* 只保留 PWM 绝对限幅与抗积分饱和，输出不受额外加速斜率限制。 */
    if (output == unconstrained || (unconstrained > output && error < 0) ||
        (unconstrained < output && error > 0))
    {
        state->integral = candidate_integral;
    }
    else
    {
        output = speed_clamp(requested + state->p_term + state->integral + state->d_term, 0, (float)pwm_limit);
    }
    state->saturated = output != unconstrained;
    state->output = (uint16_t)output;
    if (state->raw_feedback == 0 && state->output >= TRACK_SPEED_STALL_DUTY)
    {
        state->no_pulse_ms += elapsed_ms;
        if (state->no_pulse_ms >= TRACK_SPEED_STALL_MS)
        {
            TrackSpeed_Stop(state);
            return TRACK_ENCODER_STALL;
        }
    }
    else
    {
        state->no_pulse_ms = 0;
    }
    return TRACK_OK;
}

/** @brief 将滤波轮速反算为 PWM 指令刻度；不裁到 4000，保留真实超速反馈。 */
uint32_t TrackSpeed_FeedbackCommand(const TrackSpeedState *state)
{
    return (uint32_t)(state->feedback / TRACK_ENCODER_COUNTS_PER_COMMAND_10MS + 0.5f);
}

static uint16_t speed_interpolate(uint16_t straight, uint16_t curve, int16_t error, uint8_t crossing)
{
    uint16_t magnitude = error < 0 ? (uint16_t)(-(int32_t)error) : (uint16_t)error;
    if (crossing || magnitude >= 1000U) return curve;
    return straight - (uint16_t)((uint32_t)(straight - curve) * magnitude / 1000UL);
}

uint16_t TrackSpeed_Target(const TrackParams *params, int16_t error, uint8_t crossing)
{
    return speed_interpolate(params->base_command, params->curve_command, error, crossing);
}
