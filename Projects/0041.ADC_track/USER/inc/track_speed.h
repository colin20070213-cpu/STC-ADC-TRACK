/** @file track_speed.h
 *  @brief 右后轮单反馈速度 PID。输出为左右电机共用 PWM，不提供差速转向。
 */
#ifndef TRACK_SPEED_H
#define TRACK_SPEED_H
#include "track_core.h"

typedef struct
{
    float feedback;        /**< 滤波轮速，P04 计数/10ms。 */
    float raw_feedback;    /**< 未滤波轮速，同上。 */
    float target;          /**< 限斜率后的目标轮速。 */
    float integral;        /**< I 分量，单位 PWM。 */
    float derivative;      /**< 负反馈变化率的低通结果。 */
    float last_feedback;
    float p_term;
    float d_term;
    uint16_t output;       /**< 两个电机共用的动力命令。 */
    uint16_t no_pulse_ms;
    uint8_t measured;
    uint8_t initialized;
    uint8_t saturated;
} TrackSpeedState;

/** @brief 停车/失线清空控制历史，保留最新轮速显示值。 */
void TrackSpeed_Stop(TrackSpeedState *state);
/** @brief 按实际间隔换算测速；异常计数/时间返回 0，不污染旧测量。 */
uint8_t TrackSpeed_Measure(TrackSpeedState *state, uint16_t pulses, uint16_t elapsed_ms);
/** @brief 用实际间隔计算速度 PID；零目标立即停，堵转/无脉冲返回对应故障。 */
TrackFault TrackSpeed_Calculate(TrackSpeedState *state, const TrackParams *params,
                               uint16_t requested, uint16_t pwm_limit, uint16_t elapsed_ms);
/** @brief 依据转向误差插值目标轮速；十字使用弯道目标。 */
uint16_t TrackSpeed_Target(const TrackParams *params, int16_t error, uint8_t crossing);
/** @brief 同样插值直线/弯道 PWM 上限；与目标速度分开。 */
uint16_t TrackSpeed_Limit(const TrackParams *params, int16_t error, uint8_t crossing);
#endif
