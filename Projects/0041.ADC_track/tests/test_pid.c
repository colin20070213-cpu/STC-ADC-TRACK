/** @file test_pid.c
 *  @brief 独立 PID 的数值与保护测试：验证采样时间、滤波、积分和反向输出。
 */
#include "track_pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

int main(void)
{
    TrackPIDState state;
    TrackPIDGains gains;
    unsigned sample;

    gains.kp = 40.0f;
    gains.ki = 0.0f;
    gains.kd = 0.15f;
    TrackPID_Reset(&state);
    assert_near(TrackPID_Calculate(&state, &gains, 0.5f, -50, 50, 1), 20.0f);
    assert_near(state.derivative, 0.0f);
    /* 2ms 内误差增加 0.25，变化率为 125；滤波后 31.25，D 项为 4.6875。 */
    assert_near(TrackPID_Calculate(&state, &gains, 0.75f, -50, 50, 1), 34.6875f);
    assert_near(state.derivative, 31.25f);

    gains.kp = 200.0f;
    gains.ki = 20.0f;
    gains.kd = 0.0f;
    TrackPID_Reset(&state);
    for (sample = 0; sample < 1000; ++sample)
    {
        assert_near(TrackPID_Calculate(&state, &gains, 1.0f, -50, 50, 1), 50.0f);
    }
    assert(state.saturated);
    assert_near(state.integral, 0.0f);
    TrackPID_Reset(&state);
    assert_near(TrackPID_Calculate(&state, &gains, 1.0f, -50, 50, -1), -50.0f);
    assert_near(state.integral, 0.0f);

    /* 已有积分导致饱和时，反方向误差必须允许积分逐步退回。 */
    TrackPID_Reset(&state);
    gains.kp = 0.0f;
    state.integral = 1.0f;
    TrackPID_Calculate(&state, &gains, 1.0f, -10, 10, 1);
    assert_near(state.integral, 1.0f);
    TrackPID_Calculate(&state, &gains, -1.0f, -10, 10, 1);
    assert_near(state.integral, 0.998f);

    TrackPID_Reset(&state);
    gains.ki = 0.0f;
    for (sample = 0; sample < 1000; ++sample)
    {
        TrackPID_Calculate(&state, &gains, 1.0f, -50, 50, 1);
    }
    assert_near(state.integral, TRACK_PID_INTEGRAL_MAX);
    TrackPID_Reset(&state);
    assert(!state.initialized && !state.saturated);
    assert_near(state.integral, 0.0f);
    assert_near(state.output, 0.0f);
    puts("PASS: independent PID numerical response, filtered D, anti-windup, reverse direction and reset");
    return 0;
}
