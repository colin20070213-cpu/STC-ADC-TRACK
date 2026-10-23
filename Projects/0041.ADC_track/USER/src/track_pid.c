/**
 * @file    track_pid.c
 * @brief   位置式 PID 纯计算。输出是绝对偏移量，不是 PWM 增量累加。
 */
#include "track_pid.h"
#include <string.h>

/** @brief 将浮点数限制在指定闭区间。 */
static float pid_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

void TrackPID_Reset(TrackPIDState *state)
{
    memset(state, 0, sizeof(*state));
}

float TrackPID_Calculate(TrackPIDState *state, const TrackPIDGains *gains,
                         float error, float output_min, float output_max,
                         int8_t output_direction)
{
    float candidate_integral;
    float directed_error;

    if (!state->initialized)
    {
        state->last_error = error;
        state->initialized = 1;
    }

    /* 先求误差变化率，再做一阶低通，减轻 ADC 噪声对 D 项的放大。 */
    state->derivative += TRACK_PID_D_FILTER *
        ((error - state->last_error) * (1000.0f / TRACK_PERIOD_MS) - state->derivative);
    state->last_error = error;
    candidate_integral = state->integral + error * (TRACK_PERIOD_MS / 1000.0f);
    candidate_integral = pid_clamp(candidate_integral,
                                   -TRACK_PID_INTEGRAL_MAX, TRACK_PID_INTEGRAL_MAX);

    state->p_term = gains->kp * error;
    state->i_term = gains->ki * candidate_integral;
    state->d_term = gains->kd * state->derivative;
    state->unclamped = output_direction * (state->p_term + state->i_term + state->d_term);
    state->output = pid_clamp(state->unclamped, output_min, output_max);

    /* 饱和时只接受有助于退出饱和的积分，避免打到限位后继续积累。 */
    directed_error = output_direction * error;
    if (state->unclamped == state->output ||
        (state->unclamped > output_max && directed_error < 0) ||
        (state->unclamped < output_min && directed_error > 0))
    {
        state->integral = candidate_integral;
    }
    else
    {
        state->i_term = gains->ki * state->integral;
        state->unclamped = output_direction * (state->p_term + state->i_term + state->d_term);
        state->output = pid_clamp(state->unclamped, output_min, output_max);
    }

    state->saturated = (state->unclamped != state->output);
    return state->output;
}
