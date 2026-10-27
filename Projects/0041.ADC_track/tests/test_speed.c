/** @file test_speed.c @brief 参考系数换算、基础 PWM、负载闭环及停车保护回归。 */
#include "track_speed.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    TrackSpeedState state;
    TrackParams params;
    float actual = 0, phase = 0, gain = 1.0f / 0.052f;
    uint16_t pulses, first_output = 0;
    unsigned n;
    const uint16_t period = TRACK_SPEED_PERIOD_MS;
    TrackFault result;

    memset(&state, 0, sizeof(state));
    TrackParams_Default(&params);
    params.base_command = 2500;
    params.curve_command = 1400;
    params.legacy_speed_base = 1000;
    params.legacy_speed_curve = 700; /* 旧额外目标不参与控制。 */
    assert(TrackSpeed_Target(&params, 500, 0) == 1950);
    assert(TrackSpeed_Target(&params, -500, 0) == 1950);
    assert(TrackSpeed_Target(&params, 0, 1) == 1400);
    assert(TrackSpeed_Target(&params, 0, 0) == 2500);
    assert(!TrackSpeed_Measure(&state, 1, 0));
    assert(!TrackSpeed_Measure(&state, 5000, 10));
    assert(period == 2);
    assert(TrackSpeed_Measure(&state, 6, 3) && state.feedback == 20.0f);
    assert(!TrackSpeed_Measure(&state, 1, 5));

    /* 独立已知值：2500 PWM * 0.104 = 260 计数/20ms = 26 计数/2ms。 */
    memset(&state, 0, sizeof(state));
    params.speed_kp100 = 5000;
    params.speed_ki100 = params.speed_kd100 = 0;
    assert(TrackSpeed_Measure(&state, 26, 2));
    assert(TrackSpeed_FeedbackCommand(&state) == 2500);
    assert(TrackSpeed_Calculate(&state, &params, 2500, 4000, 2) == TRACK_OK);
    assert(fabsf(state.target - 130.0f) < 0.001f);
    assert(fabsf(state.p_term) < 0.001f && state.command == 2500 && state.output == 2500);
    memset(&state, 0, sizeof(state));
    assert(TrackSpeed_Measure(&state, 52, 4));
    assert(TrackSpeed_FeedbackCommand(&state) == 2500); /* 实际时间归一后速度一致。 */
    assert(TrackSpeed_Calculate(&state, &params, 2500, 4000, 4) == TRACK_OK && state.output == 2500);
    state.feedback = 104.0f; /* 实测等效 2000 PWM，误差 26 计数/10ms。 */
    assert(TrackSpeed_Calculate(&state, &params, 2500, 4000, 2) == TRACK_OK);
    assert(fabsf(state.p_term - 1300.0f) < 0.1f && state.output == 3800);
    state.feedback = 156.0f; /* 实测等效 3000 PWM，减小基础动力。 */
    assert(TrackSpeed_Calculate(&state, &params, 2500, 4000, 2) == TRACK_OK);
    assert(state.output == 1200 && TrackSpeed_FeedbackCommand(&state) == 3000);

    /* 无慢爬升：起步第一帧即可达到限幅；超速、零目标可立即降到零。 */
    memset(&state, 0, sizeof(state));
    assert(TrackSpeed_Measure(&state, 0, period));
    assert(TrackSpeed_Calculate(&state, &params, 2500, 4000, period) == TRACK_OK);
    assert(state.command == 2500 && state.output == 4000);
    state.feedback = 1000.0f;
    assert(TrackSpeed_Calculate(&state, &params, 2500, 4000, period) == TRACK_OK && !state.output);
    assert(TrackSpeed_Calculate(&state, &params, 0, 4000, period) == TRACK_OK);
    assert(!state.output && !state.command && state.integral == 0 && state.target == 0);

    /* 基础动力过大时允许负积分，停止后不携带旧积分重新启动。 */
    params.speed_kp100 = 0;
    params.speed_ki100 = 10000;
    state.feedback = 140.0f;
    assert(TrackSpeed_Calculate(&state, &params, 2500, 4000, 2) == TRACK_OK);
    assert(fabsf(state.integral + 2.0f) < 0.01f && state.output == 2498);
    TrackSpeed_Stop(&state);
    assert(state.integral == 0 && !state.command);

    /* 一阶电机/离散脉冲模型，后半段同 PWM 轮速下降，积分应增加动力。 */
    memset(&state, 0, sizeof(state));
    params.speed_kp100 = 3000;
    params.speed_ki100 = 3000;
    params.speed_kd100 = 0;
    for (n = 0; n < 50000; ++n)
    {
        if (n == 25000) { first_output = state.output; gain *= 1.2f; }
        actual += (float)period / (125.0f + period) * (state.output / gain - actual);
        phase += actual * period / 10.0f;
        pulses = (uint16_t)phase;
        phase -= pulses;
        assert(TrackSpeed_Measure(&state, pulses, period));
        assert(TrackSpeed_Calculate(&state, &params, 2000, 3000, period) == TRACK_OK);
        assert(state.output <= 3000);
    }
    printf("plant=%.4f feedback=%.4f pwm=%u first=%u integral=%.4f\n",
           actual, state.feedback, state.output, first_output, state.integral);
    assert(fabsf(actual - 104.0f) < 0.8f && state.output > first_output + 200);

    /* 双向积分边界与高低饱和冻结，避免前馈改变后继续积累错误动力。 */
    params.speed_kp100 = params.speed_ki100 = 50000;
    memset(&state, 0, sizeof(state));
    for (n = 0; n < 500; ++n)
    {
        assert(TrackSpeed_Measure(&state, 0, period));
        assert(TrackSpeed_Calculate(&state, &params, 1000, 100, period) == TRACK_OK);
        assert(state.integral == 0 && state.output == 100);
    }
    state.integral = 3000;
    assert(TrackSpeed_Calculate(&state, &params, 1000, 100, period) == TRACK_OK);
    assert(state.output <= 100 && fabsf(state.integral) <= 100);
    state.integral = 0;
    state.feedback = 1000;
    assert(TrackSpeed_Calculate(&state, &params, 1000, 100, period) == TRACK_OK);
    assert(state.output == 0 && state.integral == 0);

    memset(&state, 0, sizeof(state));
    for (n = period; n < TRACK_SPEED_STALL_MS; n += period)
    {
        assert(TrackSpeed_Measure(&state, 0, period));
        assert(TrackSpeed_Calculate(&state, &params, 1000, 1500, period) == TRACK_OK);
    }
    result = TrackSpeed_Calculate(&state, &params, 1000, 1500, period);
    assert(result == TRACK_ENCODER_STALL && !state.output && !state.command && state.integral == 0);
    puts("PASS: 0.104/20ms mapping, PWM feedforward + PID, signed anti-windup, load tracking, no ramp, zero stop and stall shutdown");
    return 0;
}
