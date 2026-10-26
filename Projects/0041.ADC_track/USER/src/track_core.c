/**
 * @file    track_core.c
 * @brief   电感标定、居中参考、循迹决策及参数序列化。
 * @note    本模块不直接驱动引脚；应用层检查结果后调用硬件模块。
 */
#include "track_core.h"
#include <string.h>

/** @brief 以 32 位中间值进行有符号限幅，避免减法溢出。 */
static int16_t clamp16(int32_t value, int16_t minimum, int16_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return (int16_t)value;
}

void TrackParams_RestorePID(TrackParams *params)
{
    /* 宏使用实际数值；四舍五入转换成屏幕/Flash 使用的百分之一单位。 */
    params->kp100 = (uint16_t)(TRACK_PID_DEFAULT_KP * 100.0f + 0.5f);
    params->ki100 = (uint16_t)(TRACK_PID_DEFAULT_KI * 100.0f + 0.5f);
    params->kd100 = (uint16_t)(TRACK_PID_DEFAULT_KD * 100.0f + 0.5f);
}

void TrackParams_RestoreSpeedPID(TrackParams *params)
{
    params->speed_kp100 = (uint16_t)(TRACK_SPEED_DEFAULT_KP * 100.0f + 0.5f);
    params->speed_ki100 = (uint16_t)(TRACK_SPEED_DEFAULT_KI * 100.0f + 0.5f);
    params->speed_kd100 = (uint16_t)(TRACK_SPEED_DEFAULT_KD * 100.0f + 0.5f);
}

void TrackParams_RestoreC4(TrackParams *params)
{
    params->c4_a = TRACK_FOUR_WEIGHT_A;
    params->c4_b = TRACK_FOUR_WEIGHT_B;
    params->c4_c = TRACK_FOUR_WEIGHT_C;
}

/** @brief C4 运行时边界与宏初值边界一致，保证 32 位分子缩放不溢出。 */
static uint8_t c4_weights_valid(const TrackParams *params)
{
    return params->c4_a >= 1 && params->c4_a <= TRACK_FOUR_WEIGHT_MAX &&
           params->c4_b <= TRACK_FOUR_WEIGHT_MAX && params->c4_c <= TRACK_FOUR_WEIGHT_MAX;
}

void TrackParams_Default(TrackParams *params)
{
    TrackParams_RestorePID(params);
    TrackParams_RestoreSpeedPID(params);
    TrackParams_RestoreC4(params);
    params->speed_base = TRACK_SPEED_BASE_DEFAULT;
    params->speed_curve = TRACK_SPEED_CURVE_DEFAULT;
    params->center = TRACK_SERVO_DEFAULT_CENTER;
    params->left = TRACK_SERVO_LEFT_SIGN == 1 ?
                   TRACK_SERVO_DEFAULT_LEFT : TRACK_SERVO_DEFAULT_RIGHT;
    params->right = TRACK_SERVO_LEFT_SIGN == 1 ?
                    TRACK_SERVO_DEFAULT_RIGHT : TRACK_SERVO_DEFAULT_LEFT;
    params->base_duty = TRACK_BASE_DUTY;
    params->curve_duty = TRACK_CURVE_MIN_DUTY;
    params->lost_sum = TRACK_LOST_SUM;
    params->servo_left_sign = TRACK_SERVO_LEFT_SIGN;
    params->left_reverse = TRACK_LEFT_REVERSE;
    params->right_reverse = TRACK_RIGHT_REVERSE;
}

uint8_t TrackParams_Valid(const TrackParams *params)
{
    if (!c4_weights_valid(params)) return 0;
    if (params->speed_base > TRACK_SPEED_TARGET_MAX || params->speed_curve > params->speed_base ||
        params->speed_kp100 > TRACK_SPEED_GAIN_MAX_X100 ||
        params->speed_ki100 > TRACK_SPEED_GAIN_MAX_X100 || params->speed_kd100 > TRACK_SPEED_GAIN_MAX_X100)
    {
        return 0;
    }
    if ((params->servo_left_sign != 1 && params->servo_left_sign != -1) ||
        params->left_reverse > 1 || params->right_reverse > 1)
    {
        return 0;
    }
    if (params->kp100 > TRACK_PID_KP_MAX_X100 ||
        params->ki100 > TRACK_PID_KI_MAX_X100 ||
        params->kd100 > TRACK_PID_KD_MAX_X100)
    {
        return 0;
    }
    if (params->left < TRACK_SERVO_ABS_MIN || params->left > TRACK_SERVO_ABS_MAX ||
        params->right < TRACK_SERVO_ABS_MIN || params->right > TRACK_SERVO_ABS_MAX ||
        params->center < TRACK_SERVO_ABS_MIN || params->center > TRACK_SERVO_ABS_MAX)
    {
        return 0;
    }
    /* 左右数值顺序服从已保存的方向，两边至少保留 5 个单位行程。 */
    if (params->servo_left_sign * ((int32_t)params->left - params->center) < 5 ||
        params->servo_left_sign * ((int32_t)params->center - params->right) < 5)
    {
        return 0;
    }
    if (params->base_duty > TRACK_DUTY_MAX || params->curve_duty > params->base_duty)
    {
        return 0;
    }
    /* 合法范围与配置共用，允许现场将失线阈值设为 40 等小于 50 的正数。 */
    if (params->lost_sum < TRACK_LOST_SUM_MIN || params->lost_sum > TRACK_LOST_SUM_MAX)
    {
        return 0;
    }
    return 1;
}

uint8_t TrackParams_SetServoDirection(TrackParams *params, int16_t left_sign)
{
    TrackParams candidate;
    uint16_t original_left;

    if ((left_sign != 1 && left_sign != -1) || !TrackParams_Valid(params))
    {
        return 0;
    }
    candidate = *params;
    if (candidate.servo_left_sign != left_sign)
    {
        original_left = candidate.left;
        candidate.left = candidate.right;
        candidate.right = original_left;
        candidate.servo_left_sign = left_sign;
    }
    if (!TrackParams_Valid(&candidate))
    {
        return 0;
    }
    *params = candidate;
    return 1;
}

void TrackCal_Begin(TrackSensors *sensors)
{
    uint8_t channel;

    sensors->samples = 0;
    sensors->bad_channel = 0;
    for (channel = 0; channel < TRACK_SENSOR_MAX; ++channel)
    {
        sensors->work_low[channel] = 4095;
        sensors->work_high[channel] = 0;
    }
}

void TrackCal_Sample(TrackSensors *sensors, const uint16_t *raw)
{
    uint8_t channel;

    for (channel = 0; channel < TRACK_SENSOR_COUNT; ++channel)
    {
        sensors->raw[channel] = raw[channel];
        if (raw[channel] < sensors->work_low[channel])
        {
            sensors->work_low[channel] = raw[channel];
        }
        if (raw[channel] > sensors->work_high[channel])
        {
            sensors->work_high[channel] = raw[channel];
        }
    }
    if (sensors->samples < 65535U)
    {
        ++sensors->samples;
    }
}

uint8_t TrackCal_Commit(TrackSensors *sensors)
{
    uint8_t channel;

    if (sensors->samples < TRACK_CAL_MIN_SAMPLES)
    {
        sensors->bad_channel = 255;
        return 0;
    }
    /* 所有通道先检查，全部合格后一起提交，防止只更新了一部分量程。 */
    for (channel = 0; channel < TRACK_SENSOR_COUNT; ++channel)
    {
        if (sensors->work_high[channel] <= sensors->work_low[channel] ||
            sensors->work_high[channel] - sensors->work_low[channel] < TRACK_CAL_MIN_SPAN ||
            sensors->work_high[channel] >= TRACK_ADC_RAIL)
        {
            sensors->bad_channel = channel + 1;
            return 0;
        }
    }
    for (channel = 0; channel < TRACK_SENSOR_COUNT; ++channel)
    {
        sensors->low[channel] = sensors->work_low[channel];
        sensors->high[channel] = sensors->work_high[channel];
    }
    sensors->valid = 1;
    sensors->bad_channel = 0;
    /* 量程变化后旧差比和参考不再适用，必须重新做车身居中标定。 */
    sensors->center_valid = 0;
    sensors->center_bias = 0;
    sensors->center_status = TRACK_BODY_IDLE;
    sensors->center_samples = 0;
    return 1;
}

/**
 * @brief 计算 numerator/denominator 的千分比，输入为非负数。
 * @note 以 1000 的二进制位递推商和余数，十轮完成；不调用 32 位乘除库。
 *       每轮余数小于三倍分母，使用 32 位加减移位避免 16 位溢出。
 */
static uint16_t ratio_per_mille(uint16_t numerator, uint16_t denominator)
{
    uint32_t remainder = 0;
    uint16_t result = 0;
    uint16_t mask;

    if (denominator == 0)
    {
        return 0;
    }
    if (numerator >= denominator)
    {
        return 1000;
    }
    for (mask = 512; mask != 0; mask >>= 1)
    {
        result <<= 1;
        remainder <<= 1;
        if (1000U & mask)
        {
            remainder += numerator;
        }
        while (remainder >= denominator)
        {
            remainder -= denominator;
            ++result;
        }
    }
    return result;
}

/** @brief 求居中样本的有符号均值，按绝对值长除后恢复符号。 */
static int16_t center_average(int32_t sum)
{
    uint32_t magnitude = sum < 0 ? (uint32_t)(-sum) : (uint32_t)sum;
    uint32_t remainder = 0;
    uint32_t quotient = 0;
    uint32_t mask;

    for (mask = 0x80000000UL; mask != 0; mask >>= 1)
    {
        remainder = (remainder << 1) | ((magnitude & mask) ? 1U : 0U);
        if (remainder >= TRACK_BODY_SAMPLES)
        {
            remainder -= TRACK_BODY_SAMPLES;
            quotient |= mask;
        }
    }
    return sum < 0 ? -(int16_t)quotient : (int16_t)quotient;
}

uint16_t TrackNormalize(uint16_t value, uint16_t low, uint16_t high)
{
    if (high <= low || value <= low)
    {
        return 0;
    }
    if (value >= high)
    {
        return 1000;
    }
    return ratio_per_mille((uint16_t)(value - low), (uint16_t)(high - low));
}

void TrackSensors_Update(TrackSensors *sensors, const uint16_t *raw)
{
    uint8_t channel;

    for (channel = 0; channel < TRACK_SENSOR_COUNT; ++channel)
    {
        sensors->raw[channel] = raw[channel];
        sensors->norm[channel] = sensors->valid ?
            TrackNormalize(raw[channel], sensors->low[channel], sensors->high[channel]) : 0;
    }
}

/** @brief 将输入限制在归一化范围，防止异常数据破坏误差及强度判断。 */
static uint16_t sensor_signal(const TrackSensors *sensors, uint8_t channel)
{
    return sensors->norm[channel] > 1000U ? 1000U : sensors->norm[channel];
}

void TrackSensors_Strength(const TrackSensors *sensors, uint16_t *left, uint16_t *right)
{
    *left = sensor_signal(sensors, 0);
    *right = sensor_signal(sensors, 1);
#if TRACK_SENSOR_COUNT == 4
    /* 每侧取横/纵较强值，保留原失线阈值的 0～2000 标度。 */
    if (sensor_signal(sensors, 2) > *left)
    {
        *left = sensor_signal(sensors, 2);
    }
    if (sensor_signal(sensors, 3) > *right)
    {
        *right = sensor_signal(sensors, 3);
    }
#endif
}

#if TRACK_SENSOR_COUNT == 4
/** @brief 用四个 8 位分块乘积构造精确 16×16→32 结果，避免依赖 32 位乘法库。
 *  @note 每个局部乘积最多 65025，显式无符号 16 位计算不会溢出。
 */
static uint32_t c4_weight_product(uint16_t value, uint16_t weight)
{
    uint16_t value_low = value & 0xffU;
    uint16_t value_high = value >> 8;
    uint16_t weight_low = weight & 0xffU;
    uint16_t weight_high = weight >> 8;
    uint16_t low = value_low * weight_low;
    uint16_t middle_first = value_low * weight_high;
    uint16_t middle_second = value_high * weight_low;
    uint16_t high = value_high * weight_high;

    return (uint32_t)low + ((uint32_t)middle_first << 8) +
           ((uint32_t)middle_second << 8) + ((uint32_t)high << 16);
}

/** @brief 给差值乘非负权重，先算幅值再恢复符号。 */
static int32_t c4_signed_weight(int16_t difference, uint16_t weight)
{
    uint16_t magnitude = difference < 0 ? -difference : difference;
    int32_t result = (int32_t)c4_weight_product(magnitude, weight);

    return difference < 0 ? -result : result;
}

/** @brief 论文 C4 分子/分母；整数权重同标度，分母保留纵电感差的绝对值。 */
static void four_sensor_terms(const TrackSensors *sensors, const TrackParams *params, int32_t *numerator,
                              uint32_t *denominator)
{
    int16_t horizontal_difference = (int16_t)sensor_signal(sensors, 0) -
                                    (int16_t)sensor_signal(sensors, 1);
    int16_t vertical_difference = (int16_t)sensor_signal(sensors, 2) -
                                  (int16_t)sensor_signal(sensors, 3);
    uint16_t horizontal_sum = sensor_signal(sensors, 0) + sensor_signal(sensors, 1);
    uint16_t vertical_magnitude = vertical_difference < 0 ?
                                 -vertical_difference : vertical_difference;

    *numerator = c4_signed_weight(horizontal_difference, params->c4_a) +
                 c4_signed_weight(vertical_difference, params->c4_b);
    *denominator = c4_weight_product(horizontal_sum, params->c4_a) +
                   c4_weight_product(vertical_magnitude, params->c4_c);
}

/** @brief 无乘除库的 32 位长除，结果限幅到 1000；固定执行 32 轮。 */
static uint16_t divide_capped1000(uint32_t numerator, uint32_t denominator)
{
    uint32_t remainder = 0;
    uint32_t quotient = 0;
    uint32_t mask;

    if (denominator == 0)
    {
        return 0;
    }
    for (mask = 0x80000000UL; mask != 0; mask >>= 1)
    {
        remainder = (remainder << 1) | ((numerator & mask) ? 1U : 0U);
        if (remainder >= denominator)
        {
            remainder -= denominator;
            quotient |= mask;
        }
    }
    return quotient > 1000UL ? 1000U : (uint16_t)quotient;
}
#endif

uint8_t TrackSensors_SteeringValid(const TrackSensors *sensors, const TrackParams *params)
{
#if TRACK_SENSOR_COUNT == 4
    int32_t numerator;
    uint32_t denominator;

    if (!c4_weights_valid(params)) return 0;
    four_sensor_terms(sensors, params, &numerator, &denominator);
    return denominator >= (uint32_t)TRACK_FOUR_DEN_MIN * params->c4_a;
#else
    (void)params;
    return sensor_signal(sensors, 0) + sensor_signal(sensors, 1) != 0;
#endif
}

#if TRACK_SENSOR_COUNT == 2
static int16_t two_sensor_error(const TrackSensors *sensors)
{
    uint16_t left = sensors->norm[0] > 1000 ? 1000 : sensors->norm[0];
    uint16_t right = sensors->norm[1] > 1000 ? 1000 : sensors->norm[1];
    uint16_t signal_sum = left + right;
    int16_t magnitude;

    if (signal_sum == 0)
    {
        return 0;
    }
    /* 先计算非负差的比例，再恢复左右符号，结果明确限制在 -1000～1000。 */
    if (left >= right)
    {
        magnitude = (int16_t)ratio_per_mille((uint16_t)(left - right), signal_sum);
    }
    else
    {
        magnitude = -(int16_t)ratio_per_mille((uint16_t)(right - left), signal_sum);
    }
    return TRACK_ERROR_SIGN == 1 ? magnitude : -magnitude;
}

#endif

int16_t TrackSensors_Error(const TrackSensors *sensors, const TrackParams *params)
{
#if TRACK_SENSOR_COUNT == 4
    int32_t numerator;
    uint32_t denominator;
    uint32_t magnitude;
    int16_t error;

    if (!c4_weights_valid(params)) return 0;
    four_sensor_terms(sensors, params, &numerator, &denominator);
    if (denominator < (uint32_t)TRACK_FOUR_DEN_MIN * params->c4_a)
    {
        return 0; /* 低分母不作为有效反馈，运行状态会进入限速失线确认。 */
    }
    magnitude = numerator < 0 ? (uint32_t)(-numerator) : (uint32_t)numerator;
    /* 宏边界保证乘积不超过 4,000,000,000，适合 uint32_t。 */
    error = (int16_t)divide_capped1000(magnitude * TRACK_FOUR_SCALE1000, denominator);
    if (numerator < 0)
    {
        error = -error;
    }
    return TRACK_ERROR_SIGN == 1 ? error : -error;
#else
    (void)params;
    return two_sensor_error(sensors);
#endif
}

int16_t TrackSensors_Feedback(const TrackSensors *sensors, const TrackParams *params)
{
    int32_t feedback = TrackSensors_Error(sensors, params);

    if (sensors->center_valid)
    {
        feedback -= sensors->center_bias;
    }
    return clamp16(feedback, -1000, 1000);
}

uint8_t TrackSensors_MathCheck(void)
{
    TrackSensors probe;
    TrackParams params;
#if TRACK_SENSOR_COUNT == 4
    const int16_t horizontal_200 = (int16_t)(200UL * TRACK_FOUR_SCALE1000 / 1000UL);
    const int16_t horizontal_20 = (int16_t)(20UL * TRACK_FOUR_SCALE1000 / 1000UL);
#else
    const int16_t horizontal_200 = 200;
    const int16_t horizontal_20 = 20;
#endif

    TrackParams_Default(&params);
    memset(&probe, 0, sizeof(probe));
    if (TrackNormalize(40, 0, 80) != 500 || TrackNormalize(5, 0, 10) != 500 ||
        TrackNormalize(20, 0, 80) != 250 || TrackNormalize(60, 0, 80) != 750)
    {
        return 0;
    }
    probe.norm[0] = 600;
    probe.norm[1] = 400;
    if (TrackSensors_Error(&probe, &params) != TRACK_ERROR_SIGN * horizontal_200)
    {
        return 0;
    }
    probe.norm[0] = 400;
    probe.norm[1] = 600;
    if (TrackSensors_Error(&probe, &params) != TRACK_ERROR_SIGN * -horizontal_200)
    {
        return 0;
    }
#if TRACK_SENSOR_COUNT == 4
    if (c4_weight_product(2000U, 1000U) != 2000000UL ||
        c4_signed_weight(-1000, 1000U) != -1000000L) return 0;
    probe.norm[0] = 500;
    probe.norm[1] = 500;
    probe.norm[2] = 600;
    probe.norm[3] = 100;
    {
        /* 独立常量表达式推导宏配置的预期值，运行误差路径不用除法库。 */
        uint32_t expected_value = (((uint32_t)500 * TRACK_FOUR_WEIGHT_B *
                                      TRACK_FOUR_SCALE1000) /
                                    ((uint32_t)1000 * TRACK_FOUR_WEIGHT_A +
                                     (uint32_t)500 * TRACK_FOUR_WEIGHT_C));
        int16_t expected = expected_value > 1000UL ? 1000 : (int16_t)expected_value;
        if (TrackSensors_Error(&probe, &params) != TRACK_ERROR_SIGN * expected)
        {
            return 0;
        }
        probe.norm[2] = 100;
        probe.norm[3] = 600;
        if (TrackSensors_Error(&probe, &params) != TRACK_ERROR_SIGN * -expected)
        {
            return 0;
        }
    }
    probe.norm[2] = 0;
    probe.norm[3] = 0;
#endif
    probe.norm[0] = 510;
    probe.norm[1] = 490;
    return TrackSensors_Error(&probe, &params) == TRACK_ERROR_SIGN * horizontal_20 &&
           center_average((int32_t)TRACK_BODY_SAMPLES * 200L) == 200 &&
           center_average((int32_t)TRACK_BODY_SAMPLES * -200L) == -200;
}

void TrackBodyCenter_Invalidate(TrackSensors *sensors)
{
    sensors->center_valid = 0;
    sensors->center_bias = 0;
    sensors->center_samples = 0;
    sensors->center_sum = 0;
    sensors->center_min = 0;
    sensors->center_max = 0;
    sensors->center_status = TRACK_BODY_IDLE;
}

void TrackBodyCenter_Begin(TrackSensors *sensors)
{
    sensors->center_samples = 0;
    sensors->center_sum = 0;
    sensors->center_min = 1000;
    sensors->center_max = -1000;
    sensors->center_status = TRACK_BODY_CAPTURING;
}

void TrackBodyCenter_Sample(TrackSensors *sensors, const TrackParams *params)
{
    int16_t measured_error;
    uint8_t channel;

    if (sensors->center_status != TRACK_BODY_CAPTURING)
    {
        return;
    }
    for (channel = 0; channel < TRACK_SENSOR_COUNT; ++channel)
    {
        if (sensors->raw[channel] >= TRACK_ADC_RAIL)
        {
            sensors->center_status = TRACK_BODY_ADC;
            return;
        }
    }
    if (!sensors->valid || sensors->norm[0] < TRACK_BODY_MIN_EACH ||
        sensors->norm[1] < TRACK_BODY_MIN_EACH ||
        sensors->norm[0] + sensors->norm[1] < params->lost_sum)
    {
        sensors->center_status = TRACK_BODY_WEAK;
        return;
    }
    measured_error = TrackSensors_Error(sensors, params);
    if (measured_error < -TRACK_BODY_MAX_BIAS || measured_error > TRACK_BODY_MAX_BIAS)
    {
        sensors->center_status = TRACK_BODY_OFFSET;
        return;
    }
    if (measured_error < sensors->center_min)
    {
        sensors->center_min = measured_error;
    }
    if (measured_error > sensors->center_max)
    {
        sensors->center_max = measured_error;
    }
    if (sensors->center_max - sensors->center_min > TRACK_BODY_MAX_SPREAD)
    {
        sensors->center_status = TRACK_BODY_MOVING;
        return;
    }
    sensors->center_sum += measured_error;
    ++sensors->center_samples;
    if (sensors->center_samples >= TRACK_BODY_SAMPLES)
    {
        sensors->center_status = TRACK_BODY_READY;
    }
}

uint8_t TrackBodyCenter_Commit(TrackSensors *sensors)
{
    if (!sensors->valid || sensors->center_status != TRACK_BODY_READY ||
        sensors->center_samples != TRACK_BODY_SAMPLES)
    {
        return 0;
    }
    sensors->center_bias = center_average(sensors->center_sum);
    sensors->center_valid = 1;
    return 1;
}

void TrackControl_Reset(TrackControl *control, const TrackParams *params)
{
    memset(control, 0, sizeof(*control));
    TrackPID_Reset(&control->steering_pid);
    control->servo = params->center;
}

/**
 * @brief 更新十字候选和通过状态，返回该状态应交给 PID 的误差。
 * @note  这里只保留已有双电感启发式；不代表闭环航向控制。
 */
static int16_t update_crossing(TrackControl *control, uint16_t signal_sum,
                                int16_t error, uint8_t drive)
{
    if (signal_sum < TRACK_CROSS_SUM - 200)
    {
        control->cross_blocked = 0;
    }
    if (drive && !control->crossing && !control->cross_blocked &&
        signal_sum >= TRACK_CROSS_SUM && control->previous_sum != 0 &&
        control->previous_error > -TRACK_CROSS_ERROR &&
        control->previous_error < TRACK_CROSS_ERROR &&
        error > -TRACK_CROSS_ERROR && error < TRACK_CROSS_ERROR &&
        signal_sum > control->previous_sum + 100)
    {
        control->crossing = 1;
        control->cross_ms = 0;
        control->cross_blocked = 1;
    }
    control->previous_sum = signal_sum;
    control->previous_error = error;
    if (control->crossing)
    {
        control->cross_ms += TRACK_PERIOD_MS;
        if (control->cross_ms >= TRACK_CROSS_MAX_MS || signal_sum < TRACK_CROSS_SUM - 200)
        {
            control->crossing = 0;
        }
        else
        {
            error = 0;
        }
    }
    return error;
}

/** @brief 静态转向调参禁止电机；正常行驶的公共动力由独立速度环生成。 */
static void enforce_motor_interlock(TrackControl *control, uint8_t drive)
{
    if (!drive)
    {
        control->left_motor = 0;
        control->right_motor = 0;
    }
    /* drive=1 时保留上一实际输出，供下一帧弱信号保持/限速使用。
       应用层随后以速度环共同输出覆盖左右命令，不再计算旧开环 PWM。 */
}

/** @brief 连续弱信号计时及限速保持；不依据弱信号重新计算舵机位置。 */
static TrackFault hold_weak_signal(TrackControl *control, const TrackParams *params,
                                  uint16_t left, uint16_t right, uint8_t drive)
{
    uint16_t limit = params->curve_duty;

    control->signal_ok = 0;
    if (control->weak_ms == 0)
    {
        /* 恢复后的第一次 PID 不沿用旧积分/微分，舵机命令保持不变。 */
        TrackPID_Reset(&control->steering_pid);
    }
    if (control->weak_ms < TRACK_LOST_CONFIRM_MS)
    {
        control->weak_ms += TRACK_PERIOD_MS;
    }
    if (left < TRACK_HARD_LOST_EACH && right < TRACK_HARD_LOST_EACH)
    {
        if (control->zero_ms < TRACK_ZERO_CONFIRM_MS)
        {
            control->zero_ms += TRACK_PERIOD_MS;
        }
    }
    else
    {
        control->zero_ms = 0;
    }
    control->crossing = 0;
    control->cross_ms = 0;
    control->previous_sum = 0;
    if (!drive)
    {
        control->left_motor = 0;
        control->right_motor = 0;
        return TRACK_OK;
    }
    if (control->weak_ms >= TRACK_LOST_CONFIRM_MS || control->zero_ms >= TRACK_ZERO_CONFIRM_MS)
    {
        control->left_motor = 0;
        control->right_motor = 0;
        return TRACK_LOST;
    }
    if (limit > TRACK_LOST_DUTY_MAX)
    {
        limit = TRACK_LOST_DUTY_MAX;
    }
    /* 只降低已有动力，不因进入确认状态而加速或从静止自行起步。 */
    if (control->left_motor > limit) control->left_motor = limit;
    if (control->right_motor > limit) control->right_motor = limit;
    return TRACK_OK;
}

TrackFault TrackControl_Step(TrackControl *control, const TrackSensors *sensors,
                            const TrackParams *params, uint8_t drive)
{
    uint16_t left_signal;
    uint16_t right_signal;
    uint16_t signal_sum;
    uint16_t required_sum;
    int16_t error;
    int16_t output_min;
    int16_t output_max;
    float steering_offset;
    TrackPIDGains gains;

    if (!sensors->valid)
    {
        return TRACK_NO_CAL;
    }
    if (!TrackParams_Valid(params))
    {
        return TRACK_BAD_PARAMS;
    }

    /* 四路按左右横/纵信号共同判断强度，误差则使用论文加权公式。 */
    TrackSensors_Strength(sensors, &left_signal, &right_signal);
    signal_sum = left_signal + right_signal;
    required_sum = params->lost_sum;
    if (drive && required_sum > TRACK_RUN_LOST_SUM_MAX)
    {
        /* 只放宽行驶判定，居中标定和静态 PID 仍使用原参数。 */
        required_sum = TRACK_RUN_LOST_SUM_MAX;
    }
    if (control->weak_ms != 0)
    {
        required_sum += TRACK_LOST_RECOVER_MARGIN;
    }
    if (signal_sum < required_sum || !TrackSensors_SteeringValid(sensors, params))
    {
        return hold_weak_signal(control, params, left_signal, right_signal, drive);
    }
    control->signal_ok = 1;
    control->weak_ms = 0;
    control->zero_ms = 0;

    /* 沿用本车纠偏符号：误差 = 位置反馈 - 目标，再由舵机方向决定输出符号。 */
    error = clamp16((int32_t)TrackSensors_Feedback(sensors, params) -
                    TRACK_STEERING_TARGET1000, -1000, 1000);
    /* 十字候选仍只看外侧横电感，避免纵电感增强单独触发直行压制。 */
    control->error = update_crossing(control,
        sensor_signal(sensors, 0) + sensor_signal(sensors, 1), error, drive);

    /* Flash 增益是整数 ×100；只在这里转换成独立 PID 所需的实际增益。 */
    gains.kp = params->kp100 * 0.01f;
    gains.ki = params->ki100 * 0.01f;
    gains.kd = params->kd100 * 0.01f;
    output_min = (int16_t)((params->left < params->right ? params->left : params->right) -
                           params->center);
    output_max = (int16_t)((params->left > params->right ? params->left : params->right) -
                           params->center);
    steering_offset = TrackPID_Calculate(&control->steering_pid, &gains,
                                         control->error / 1000.0f,
                                         output_min, output_max, (int8_t)params->servo_left_sign);
    control->servo = params->center + (int16_t)steering_offset;
    control->p_x10 = (int16_t)(control->steering_pid.p_term * 10.0f);
    control->i_x10 = (int16_t)(control->steering_pid.i_term * 10.0f);
    control->d_x10 = (int16_t)(control->steering_pid.d_term * 10.0f);
    control->unclamped_x10 = (int16_t)(control->steering_pid.unclamped * 10.0f);
    control->saturated = control->steering_pid.saturated;
    enforce_motor_interlock(control, drive);
    return TRACK_OK;
}

uint16_t TrackCRC16(const uint8_t *bytes, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint8_t bit_index;

    while (length--)
    {
        crc ^= (uint16_t)*bytes++ << 8;
        for (bit_index = 0; bit_index < 8; ++bit_index)
        {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

/** @brief 以小端格式写入 16 位数，避免依赖 C 结构体内存布局。 */
static void write_u16(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8);
}

/** @brief 读取小端 16 位数。 */
static uint16_t read_u16(const uint8_t *buffer)
{
    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

void TrackRecord_Encode(uint8_t *record, const TrackParams *params, uint32_t sequence)
{
    memset(record, 0xFF, TRACK_RECORD_BYTES);
    record[0] = 'P';
    record[1] = 'I';
    record[2] = 'E';
    record[3] = 2; /* v2 使用原预留字节保存三个方向，记录总长不变。 */
    record[4] = (uint8_t)sequence;
    record[5] = (uint8_t)(sequence >> 8);
    record[6] = (uint8_t)(sequence >> 16);
    record[7] = (uint8_t)(sequence >> 24);
    write_u16(record + 8, params->kp100);
    write_u16(record + 10, params->ki100);
    write_u16(record + 12, params->kd100);
    write_u16(record + 14, params->center);
    write_u16(record + 16, params->left);
    write_u16(record + 18, params->right);
    write_u16(record + 20, params->base_duty);
    write_u16(record + 22, params->curve_duty);
    write_u16(record + 24, params->lost_sum);
    record[26] = (params->servo_left_sign < 0 ? 1 : 0) |
                 (params->left_reverse ? 2 : 0) | (params->right_reverse ? 4 : 0);
    write_u16(record + 28, TrackCRC16(record, 28));
    record[31] = TRACK_RECORD_COMMIT;
}

uint8_t TrackRecord_Decode(const uint8_t *record, TrackParams *params, uint32_t *sequence)
{
    TrackParams candidate;

    if (record[0] != 'P' || record[1] != 'I' || record[2] != 'E' ||
        (record[3] != 1 && record[3] != 2) ||
        record[31] != TRACK_RECORD_COMMIT || read_u16(record + 28) != TrackCRC16(record, 28))
    {
        return 0;
    }
    TrackParams_Default(&candidate); /* 旧记录的新速度字段使用代码初值，不把旧 PWM 当速度。 */
    candidate.kp100 = read_u16(record + 8);
    candidate.ki100 = read_u16(record + 10);
    candidate.kd100 = read_u16(record + 12);
    candidate.center = read_u16(record + 14);
    candidate.left = read_u16(record + 16);
    candidate.right = read_u16(record + 18);
    candidate.base_duty = read_u16(record + 20);
    candidate.curve_duty = read_u16(record + 22);
    candidate.lost_sum = read_u16(record + 24);
    if (record[3] == 2)
    {
        if (record[26] & 0xF8)
        {
            return 0;
        }
        candidate.servo_left_sign = (record[26] & 1) ? -1 : 1;
        candidate.left_reverse = (record[26] & 2) ? 1 : 0;
        candidate.right_reverse = (record[26] & 4) ? 1 : 0;
    }
    else
    {
        /* 旧记录未保存方向：舵机由已有左右限位推断，电机使用代码默认方向。
           读取不擦写；下次手动保存才写 v2，另一槽旧记录仍保留。 */
        candidate.servo_left_sign = candidate.left > candidate.center ? 1 : -1;
        candidate.left_reverse = TRACK_LEFT_REVERSE;
        candidate.right_reverse = TRACK_RIGHT_REVERSE;
    }
    if (!TrackParams_Valid(&candidate))
    {
        return 0;
    }
    /* 放大板更换后，将弱信号版本的默认阈值恢复；其他配置保持原样。
       这里只调整 RAM，下次明确保存才写回 EEPROM。 */
    if (candidate.lost_sum == 100)
    {
        candidate.lost_sum = TRACK_LOST_SUM;
    }
    *params = candidate;
    *sequence = (uint32_t)record[4] | ((uint32_t)record[5] << 8) |
                ((uint32_t)record[6] << 16) | ((uint32_t)record[7] << 24);
    return 1;
}

uint8_t TrackSequence_Newer(uint32_t first, uint32_t second)
{
    return (int32_t)(first - second) > 0;
}
