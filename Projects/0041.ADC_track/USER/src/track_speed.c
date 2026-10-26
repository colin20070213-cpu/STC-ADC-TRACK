/** @file track_speed.c
 *  @brief 单编码器速度闭环：测量滤波、目标斜率、微分滤波、抗饱和、堵转保护。
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
        state->feedback += TRACK_SPEED_FEEDBACK_ALPHA *
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
    float rising_limit;

    if (!TrackParams_Valid(params) || requested > TRACK_SPEED_TARGET_MAX || pwm_limit > TRACK_DUTY_MAX)
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
    state->target += TRACK_SPEED_RAMP_PER_SECOND * dt;
    if (state->target > requested) state->target = requested;
    error = state->target - state->feedback;
    if (!state->initialized)
    {
        state->last_feedback = state->feedback;
        state->initialized = 1;
    }
    /* 对测量做微分，避免按键改变目标时产生微分突跳。 */
    state->derivative += TRACK_SPEED_D_FILTER *
        ((state->last_feedback - state->feedback) / dt - state->derivative);
    state->last_feedback = state->feedback;
    state->p_term = params->speed_kp100 * 0.01f * error;
    state->d_term = params->speed_kd100 * 0.01f * state->derivative;
    state->integral = speed_clamp(state->integral, 0, (float)pwm_limit);
    candidate_integral = speed_clamp(state->integral + params->speed_ki100 * 0.01f * error * dt,
                                     0, (float)pwm_limit);
    unconstrained = state->p_term + candidate_integral + state->d_term;
    rising_limit = state->output + (float)TRACK_SPEED_PWM_RISE_PER_MS * elapsed_ms;
    if (rising_limit > pwm_limit) rising_limit = pwm_limit;
    output = speed_clamp(unconstrained, 0, rising_limit);
    /* 输出上限、加速上限均参与抗积分饱和；减速不受加速斜率限制。 */
    if (output == unconstrained || (unconstrained > output && error < 0) ||
        (unconstrained < output && error > 0))
    {
        state->integral = candidate_integral;
    }
    else
    {
        output = speed_clamp(state->p_term + state->integral + state->d_term, 0, rising_limit);
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

static uint16_t speed_interpolate(uint16_t straight, uint16_t curve, int16_t error, uint8_t crossing)
{
    uint16_t magnitude = error < 0 ? (uint16_t)(-(int32_t)error) : (uint16_t)error;
    if (crossing || magnitude >= 1000U) return curve;
    return straight - (uint16_t)((uint32_t)(straight - curve) * magnitude / 1000UL);
}

uint16_t TrackSpeed_Target(const TrackParams *params, int16_t error, uint8_t crossing)
{
    return speed_interpolate(params->speed_base, params->speed_curve, error, crossing);
}

uint16_t TrackSpeed_Limit(const TrackParams *params, int16_t error, uint8_t crossing)
{
    return speed_interpolate(params->base_duty, params->curve_duty, error, crossing);
}
