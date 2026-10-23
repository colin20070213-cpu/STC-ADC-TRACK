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
                               unsigned middle_left, unsigned middle_right)
{
    int64_t horizontal = (int64_t)left - right;
    int64_t vertical = (int64_t)middle_left - middle_right;
    int64_t numerator = horizontal * TRACK_FOUR_WEIGHT_A + vertical * TRACK_FOUR_WEIGHT_B;
    int64_t denominator = (int64_t)(left + right) * TRACK_FOUR_WEIGHT_A +
                          (vertical < 0 ? -vertical : vertical) * TRACK_FOUR_WEIGHT_C;
    int64_t result;

    if (denominator < (int64_t)TRACK_FOUR_DEN_MIN * TRACK_FOUR_WEIGHT_A)
    {
        return 0;
    }
    result = numerator * TRACK_FOUR_SCALE1000 / denominator;
    if (result > 1000) result = 1000;
    if (result < -1000) result = -1000;
    return (int16_t)(result * TRACK_ERROR_SIGN);
}

int main(void)
{
    TrackSensors sensors;
    TrackControl control;
    TrackParams params;
    uint16_t left_strength, right_strength;
    unsigned left, right, middle_left, middle_right, elapsed;
    int16_t error;

    assert(TRACK_SENSOR_COUNT == 4 && TRACK_ADC_FRAME_MS == 1);
    assert(TrackSensors_MathCheck());
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
        error = TrackSensors_Error(&sensors);
        assert(error == reference_error(left, right, middle_left, middle_right));
        sensors.norm[0] = (uint16_t)right;
        sensors.norm[1] = (uint16_t)left;
        sensors.norm[2] = (uint16_t)middle_right;
        sensors.norm[3] = (uint16_t)middle_left;
        assert(TrackSensors_Error(&sensors) == -error);
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
    assert(control.signal_ok && control.error == reference_error(0, 0, 700, 100));
    assert(control.servo > params.center);

    /* 两只纵电感等强、横电感为零时，C4 没有有效分母；不能当作正常直线。 */
    sensors.norm[2] = sensors.norm[3] = 700;
    assert(!TrackSensors_SteeringValid(&sensors));
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
