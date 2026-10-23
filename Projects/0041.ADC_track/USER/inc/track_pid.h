/**
 * @file    track_pid.h
 * @brief   独立位置式 PID：只计算输出，不访问 ADC、菜单或硬件寄存器。
 */
#ifndef TRACK_PID_H
#define TRACK_PID_H

#ifdef TRACK_HOST_TEST
#include <stdint.h>
#else
#include "common.h"
#endif
#include "track_config.h"

/** @brief 实际增益；例如 kp=40.0 表示 Kp 为 40，不需要乘 100。 */
typedef struct
{
    float kp;
    float ki;
    float kd;
} TrackPIDGains;

/** @brief 一个 PID 实例的运行状态；重置时清零，禁止写入 Flash。 */
typedef struct
{
    float integral;       /**< 误差积分，单位：误差·秒。 */
    float derivative;     /**< 滤波后的误差变化率，单位：误差/秒。 */
    float last_error;     /**< 上一控制周期的误差。 */
    float p_term;         /**< 本次比例分量。 */
    float i_term;         /**< 本次积分分量。 */
    float d_term;         /**< 本次微分分量。 */
    float unclamped;      /**< 含方向符号、尚未限幅的输出。 */
    float output;         /**< 最终限幅输出，本车将其作为舵机中值偏移。 */
    uint8_t initialized;  /**< 首次计算时抑制启动微分突变。 */
    uint8_t saturated;    /**< 本周期是否发生输出限幅。 */
} TrackPIDState;

/**
 * @brief 清空积分、微分和输出；换模式或修改参数后调用。
 * @param state 待重置的 PID 状态，不能为空。
 */
void TrackPID_Reset(TrackPIDState *state);

/**
 * @brief 按 TRACK_PERIOD_MS 计算一次带微分滤波、抗积分饱和的位置式 PID。
 * @param state            同一控制器跨周期复用的状态。
 * @param gains            实际 Kp/Ki/Kd，不采用放大 100 倍的存储格式。
 * @param error            本车输入为 -1～1 的有符号横向偏差。
 * @param output_min       输出下限，本车为限位相对中值的较小偏移。
 * @param output_max       输出上限，必须不小于 output_min。
 * @param output_direction 输出方向，取 +1 或 -1。
 * @return 限幅后的输出，同时更新 state 中的各分量。
 * @note  参数和限位由调用层校验；在前台调用，不放进 1ms 中断。
 */
float TrackPID_Calculate(TrackPIDState *state, const TrackPIDGains *gains,
                         float error, float output_min, float output_max,
                         int8_t output_direction);
#endif
