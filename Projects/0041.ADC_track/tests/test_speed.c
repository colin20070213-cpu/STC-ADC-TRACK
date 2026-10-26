/** @file test_speed.c @brief 速度环闭环响应、负载扰动、限幅、堵转及停车回归。 */
#include "track_speed.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    TrackSpeedState state;
    TrackParams params;
    float actual=0, phase=0, gain=100.0f;
    uint16_t pulses, first_output;
    unsigned n;
    const uint16_t period=TRACK_SPEED_PERIOD_MS;
    TrackFault result;

    memset(&state,0,sizeof(state));TrackParams_Default(&params);
    params.base_speed_pps=2500;params.curve_speed_pps=1400;
    params.legacy_speed_base=1000;params.legacy_speed_curve=700; /* 旧额外目标不参与控制。 */
    /* 固定仿真增益，不让用户实车调参宏改变被测电机模型的验收条件。 */
    params.speed_kp100=3000;params.speed_ki100=3000;params.speed_kd100=0;
    assert(TrackSpeed_Target(&params,500,0)==1950);
    assert(TrackSpeed_Target(&params,-500,0)==1950);
    assert(TrackSpeed_Target(&params,0,1)==1400);
    assert(TrackSpeed_Target(&params,0,0)==2500);
    assert(!TrackSpeed_Measure(&state,1,0));
    assert(!TrackSpeed_Measure(&state,5000,10));
    assert(period==2);
    assert(TrackSpeed_Measure(&state,6,3) && state.feedback==20.0f);
    assert(!TrackSpeed_Measure(&state,1,5));
    memset(&state,0,sizeof(state));
    for(n=0;n<50000;n++)
    {
        if(n==25000){first_output=state.output;gain=120.0f;}
        /* 一阶电机和离散脉冲计数模型，后半段施加负载使同 PWM 轮速下降。 */
        actual+=(float)period/(125.0f+period)*(state.output/gain-actual);
        phase+=actual*period/10.0f;pulses=(uint16_t)phase;phase-=pulses;
        assert(TrackSpeed_Measure(&state,pulses,period));
        assert(TrackSpeed_Calculate(&state,&params,2000,3000,period)==TRACK_OK);
        assert(state.output<=3000);
    }
    printf("plant=%.4f feedback=%.4f pwm=%u first=%u integral=%.4f\n",actual,state.feedback,state.output,first_output,state.integral);fflush(stdout);
    assert(fabsf(actual-20.0f)<0.8f && state.output>first_output+200);
    assert(TrackSpeed_Calculate(&state,&params,0,3000,period)==TRACK_OK);
    assert(!state.output && state.integral==0 && state.target==0);

    memset(&state,0,sizeof(state));params.speed_kp100=100000UL;params.speed_ki100=params.speed_kd100=0;
    assert(TrackSpeed_Measure(&state,0,period));state.target=1.0f;state.output=2000;
    assert(TrackSpeed_Calculate(&state,&params,100,3000,period)==TRACK_OK);
    assert(fabsf(state.p_term-1000.0f)<0.1f && state.output==1000);
    /* 第一次 2ms 就能输出完整 PID 动力，不再额外爬升几百毫秒。 */
    TrackSpeed_Stop(&state);
    assert(TrackSpeed_Calculate(&state,&params,2500,TRACK_DUTY_MAX,period)==TRACK_OK);
    assert(state.target==25.0f && state.output==TRACK_DUTY_MAX);
    state.feedback=100.0f;
    assert(TrackSpeed_Calculate(&state,&params,2500,TRACK_DUTY_MAX,period)==TRACK_OK && !state.output);
    assert(TrackSpeed_Calculate(&state,&params,0,TRACK_DUTY_MAX,period)==TRACK_OK && !state.output);

    params.base_speed_pps=1000;params.curve_speed_pps=0;params.speed_kp100=50000;params.speed_ki100=50000;
    memset(&state,0,sizeof(state));
    for(n=0;n<500;n++)
    {
        assert(TrackSpeed_Measure(&state,0,period));
        assert(TrackSpeed_Calculate(&state,&params,1000,100,period)==TRACK_OK);
    }
    assert(state.output<=100 && state.integral<=100.0f);
    state.integral=3000;state.output=3000;
    assert(TrackSpeed_Calculate(&state,&params,1000,100,period)==TRACK_OK);
    assert(state.output<=100 && state.integral<=100.0f);
    state.output=1500;state.target=1000;
    for(n=period;n<TRACK_SPEED_STALL_MS;n+=period)
    {
        assert(TrackSpeed_Measure(&state,0,period));
        assert(TrackSpeed_Calculate(&state,&params,1000,1500,period)==TRACK_OK);
    }
    result=TrackSpeed_Calculate(&state,&params,1000,1500,period);
    assert(result==TRACK_ENCODER_STALL && !state.output && state.integral==0);
    puts("PASS: 2ms speed PI tracks a quantized plant/load change, actual sample time, target interpolation, anti-windup, zero stop and stall shutdown");
    return 0;
}
