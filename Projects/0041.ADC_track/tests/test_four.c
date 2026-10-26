/**
 * @file    test_four.c
 * @brief   独立 64 位参考公式验证四电感 C4、左右对称、分母保护和失线输入。
 */
#include "track_core.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int16_t reference_error(unsigned left, unsigned right,
                               unsigned middle_left, unsigned middle_right, const TrackParams *params)
{
    int64_t horizontal = (int64_t)left - right;
    int64_t vertical = (int64_t)middle_left - middle_right;
    int64_t numerator = horizontal * params->c4_a + vertical * params->c4_b;
    int64_t denominator = (int64_t)(left + right) * params->c4_a +
                          (vertical < 0 ? -vertical : vertical) * params->c4_c;
    int64_t result;

    if (denominator < (int64_t)TRACK_FOUR_DEN_MIN * params->c4_a)
    {
        return 0;
    }
    result = numerator * TRACK_FOUR_SCALE1000 / denominator;
    if (result > 1000) result = 1000;
    if (result < -1000) result = -1000;
    return (int16_t)(result * TRACK_ERROR_SIGN);
}

/** @brief 运行时权重独立对照、边界、比例不变性与居中参考清除。 */
static void runtime_weight_tests(void)
{
    static const uint16_t weights[][3]={{1,1000,0},{1000,0,1000},{100,200,100},
                                      {100,100,200},{1,1000,1000},{1000,1000,1000}};
    TrackParams params,other;
    TrackSensors sensors;
    unsigned i,l,r,lm,rm;
    int16_t error;
    memset(&sensors,0,sizeof(sensors));sensors.valid=1;TrackParams_Default(&params);
    for(i=0;i<sizeof(weights)/sizeof(weights[0]);i++)
    {
        params.c4_a=weights[i][0];params.c4_b=weights[i][1];params.c4_c=weights[i][2];
        assert(TrackParams_Valid(&params));
        for(l=0;l<=1000;l+=250)for(r=0;r<=1000;r+=250)
        for(lm=0;lm<=1000;lm+=250)for(rm=0;rm<=1000;rm+=250)
        {
            sensors.norm[0]=l;sensors.norm[1]=r;sensors.norm[2]=lm;sensors.norm[3]=rm;
            error=TrackSensors_Error(&sensors,&params);
            assert(error==reference_error(l,r,lm,rm,&params));
            sensors.norm[0]=r;sensors.norm[1]=l;sensors.norm[2]=rm;sensors.norm[3]=lm;
            assert(TrackSensors_Error(&sensors,&params)==-error);
        }
    }
    TrackParams_Default(&params);other=params;
    other.c4_a=2*params.c4_a;other.c4_b=2*params.c4_b;other.c4_c=2*params.c4_c;
    sensors.norm[0]=600;sensors.norm[1]=400;sensors.norm[2]=300;sensors.norm[3]=100;
    assert(TrackSensors_Error(&sensors,&params)==TrackSensors_Error(&sensors,&other));
    params.c4_a=0;assert(!TrackParams_Valid(&params));
    assert(!TrackSensors_SteeringValid(&sensors,&params) && !TrackSensors_Error(&sensors,&params));
    params.c4_a=1;params.c4_b=0;params.c4_c=0;assert(TrackParams_Valid(&params));
    params.c4_b=1001;assert(!TrackParams_Valid(&params));
    params.c4_b=0;params.c4_c=1001;assert(!TrackParams_Valid(&params));
    TrackParams_Default(&params);params.c4_b=200;
    sensors.norm[0]=525;sensors.norm[1]=475;sensors.norm[2]=200;sensors.norm[3]=100;
    TrackBodyCenter_Begin(&sensors);
    for(i=0;i<TRACK_BODY_SAMPLES;i++)TrackBodyCenter_Sample(&sensors,&params);
    assert(TrackBodyCenter_Commit(&sensors) && sensors.center_bias==reference_error(525,475,200,100,&params));
    assert(TrackSensors_Feedback(&sensors,&params)==0);
    sensors.center_samples=22;sensors.high[0]=999;
    TrackBodyCenter_Invalidate(&sensors);
    assert(!sensors.center_valid && !sensors.center_bias && !sensors.center_samples && sensors.valid && sensors.high[0]==999);
    puts("PASS: runtime C4 weights vs independent 64-bit reference (3750 combinations), mirror/scale invariance, limits and center invalidation");
}

int main(void)
{
    TrackSensors sensors;
    TrackControl control;
    TrackParams params;
    uint16_t left_strength, right_strength;
    unsigned left, right, middle_left, middle_right, elapsed;
    int16_t error;

    runtime_weight_tests();
    assert(TRACK_SENSOR_COUNT == 4 && TRACK_ADC_FRAME_MS == 1);
    assert(TrackSensors_MathCheck());
    TrackParams_Default(&params);
    memset(&sensors, 0, sizeof(sensors));
    sensors.valid = 1;
    for (left = 0; left <= 1000; left += 125)
    for (right = 0; right <= 1000; right += 125)
    for (middle_left = 0; middle_left <= 1000; middle_left += 125)
    for (middle_right = 0; middle_right <= 1000; middle_right += 125)
    {
        sensors.norm[0] = (uint16_t)left;
        sensors.norm[1] = (uint16_t)right;
        sensors.norm[2] = (uint16_t)middle_left;
        sensors.norm[3] = (uint16_t)middle_right;
        error = TrackSensors_Error(&sensors, &params);
        assert(error == reference_error(left, right, middle_left, middle_right, &params));
        sensors.norm[0] = (uint16_t)right;
        sensors.norm[1] = (uint16_t)left;
        sensors.norm[2] = (uint16_t)middle_right;
        sensors.norm[3] = (uint16_t)middle_left;
        assert(TrackSensors_Error(&sensors, &params) == -error);
    }

    /* 外侧横电感都很弱，但左纵电感仍有方向信息，不能仅凭外侧判断失线。 */
    sensors.norm[0] = sensors.norm[1] = 0;
    sensors.norm[2] = 700;
    sensors.norm[3] = 100;
    TrackSensors_Strength(&sensors, &left_strength, &right_strength);
    assert(left_strength == 700 && right_strength == 100);
    TrackParams_Default(&params);
    TrackControl_Reset(&control, &params);
    assert(TrackControl_Step(&control, &sensors, &params, 1) == TRACK_OK);
    assert(control.signal_ok && control.error == reference_error(0, 0, 700, 100, &params));
    assert(control.servo > params.center);

    /* 两只纵电感等强、横电感为零时，C4 没有有效分母；不能当作正常直线。 */
    sensors.norm[2] = sensors.norm[3] = 700;
    assert(!TrackSensors_SteeringValid(&sensors, &params));
    for (elapsed = TRACK_PERIOD_MS; elapsed < TRACK_LOST_CONFIRM_MS; elapsed += TRACK_PERIOD_MS)
    {
        assert(TrackControl_Step(&control, &sensors, &params, 1) == TRACK_OK);
        assert(!control.signal_ok && control.left_motor <= TRACK_LOST_DUTY_MAX);
    }
    assert(TrackControl_Step(&control, &sensors, &params, 1) == TRACK_LOST);

    /* 弱信号恢复必须重新满足强度和有效分母；四路全零使用较短确认。 */
    TrackControl_Reset(&control, &params);
    memset(sensors.norm, 0, sizeof(sensors.norm));
    for (elapsed = TRACK_PERIOD_MS; elapsed < TRACK_ZERO_CONFIRM_MS; elapsed += TRACK_PERIOD_MS)
    {
        assert(TrackControl_Step(&control, &sensors, &params, 1) == TRACK_OK);
    }
    assert(TrackControl_Step(&control, &sensors, &params, 1) == TRACK_LOST);
    puts("PASS: paper C4 vs independent 64-bit reference (6561 combinations), mirror symmetry, middle-coil tracking and denominator/loss guards");
    return 0;
}
