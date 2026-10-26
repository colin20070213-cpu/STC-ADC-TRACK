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
    TrackFault result;

    memset(&state,0,sizeof(state));TrackParams_Default(&params);
    params.speed_base=20;params.speed_curve=10;
    assert(TrackSpeed_Target(&params,500,0)==15);
    assert(TrackSpeed_Target(&params,-500,0)==15);
    assert(TrackSpeed_Target(&params,0,1)==10);
    assert(TrackSpeed_Limit(&params,1000,0)==params.curve_duty);
    assert(!TrackSpeed_Measure(&state,1,0));
    assert(!TrackSpeed_Measure(&state,5000,10));
    assert(TrackSpeed_Measure(&state,30,15) && state.feedback==20.0f);
    memset(&state,0,sizeof(state));
    for(n=0;n<10000;n++)
    {
        if(n==5000){first_output=state.output;gain=120.0f;}
        /* 一阶电机和离散脉冲计数模型，后半段施加负载使同 PWM 轮速下降。 */
        actual+=0.08f*(state.output/gain-actual);
        phase+=actual;pulses=(uint16_t)phase;phase-=pulses;
        assert(TrackSpeed_Measure(&state,pulses,10));
        assert(TrackSpeed_Calculate(&state,&params,20,3000,10)==TRACK_OK);
        assert(state.output<=3000);
    }
    assert(fabsf(actual-20.0f)<0.8f && state.output>first_output+200);
    assert(TrackSpeed_Calculate(&state,&params,0,3000,10)==TRACK_OK);
    assert(!state.output && state.integral==0 && state.target==0);

    params.speed_base=1000;params.speed_curve=0;params.speed_kp100=50000;params.speed_ki100=50000;
    memset(&state,0,sizeof(state));
    for(n=0;n<500;n++)
    {
        assert(TrackSpeed_Measure(&state,0,10));
        assert(TrackSpeed_Calculate(&state,&params,1000,100,10)==TRACK_OK);
    }
    assert(state.output<=100 && state.integral<=100.0f);
    state.integral=3000;state.output=3000;
    assert(TrackSpeed_Calculate(&state,&params,1000,100,10)==TRACK_OK);
    assert(state.output<=100 && state.integral<=100.0f);
    state.output=1500;state.target=1000;
    for(n=10;n<TRACK_SPEED_STALL_MS;n+=10)
    {
        assert(TrackSpeed_Measure(&state,0,10));
        assert(TrackSpeed_Calculate(&state,&params,1000,1500,10)==TRACK_OK);
    }
    result=TrackSpeed_Calculate(&state,&params,1000,1500,10);
    assert(result==TRACK_ENCODER_STALL && !state.output && state.integral==0);
    puts("PASS: speed PI tracks a quantized plant/load change, actual sample time, target interpolation, anti-windup, zero stop and stall shutdown");
    return 0;
}
