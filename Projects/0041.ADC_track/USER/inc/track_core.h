/**
 * @file    track_core.h
 * @brief   循迹数据模型和业务算法接口；不直接访问硬件。
 * @note    Flash 只保存 TrackParams，电感极值/车身参考每次上电重新标定。
 */
#ifndef TRACK_CORE_H
#define TRACK_CORE_H

#include "track_pid.h"

#define TRACK_STORE_RECORD_BYTES 80U /* 32B 基础记录 + 48B SPD2 扩展，原提交字节最后写。 */
#define TRACK_RECORD_BYTES  32
#define TRACK_RECORD_COMMIT 0xA5

/** @brief 应用状态；已有数值保持不变，方向页追加在末尾。 */
typedef enum
{
    TRACK_MENU = 0,
    TRACK_CAL,
    TRACK_PID,
    TRACK_RUN,
    TRACK_CENTER,
    TRACK_LIMIT_LEFT,
    TRACK_LIMIT_RIGHT,
    TRACK_BODY_CENTER,
    TRACK_SPEED,
    TRACK_SERVO_DIRECTION,
    TRACK_LEFT_DIRECTION,
    TRACK_RIGHT_DIRECTION,
    TRACK_SPEED_TARGET,
    TRACK_SPEED_PID,
    TRACK_C4_TUNE
} TrackMode;

/** @brief 停车/启动失败原因，在故障菜单中显示，串口仍只发送四列目标/反馈。 */
typedef enum
{
    TRACK_OK = 0,
    TRACK_NO_CAL,
    TRACK_ADC_ERROR,
    TRACK_LOST,
    TRACK_TIMEOUT,
    TRACK_MANUAL_STOP,
    TRACK_BAD_PARAMS,
    TRACK_NO_CENTER,
    TRACK_MATH_ERROR,
    TRACK_NO_SPEED,
    TRACK_ENCODER_ERROR,
    TRACK_ENCODER_STALL
} TrackFault;

/** @brief 车身居中采集状态；采集失败不覆盖原有效参考。 */
typedef enum
{
    TRACK_BODY_IDLE = 0,
    TRACK_BODY_CAPTURING,
    TRACK_BODY_READY,
    TRACK_BODY_WEAK,
    TRACK_BODY_MOVING,
    TRACK_BODY_OFFSET,
    TRACK_BODY_ADC
} TrackBodyStatus;

/** @brief 可断电保存的参数；字段顺序不等于 Flash 字节顺序，基础记录由 core、速度/C4 扩展由 store 明确序列化。 */
typedef struct
{
    uint16_t kp100;      /**< Kp ×100，例如 4000 对应 40.00。 */
    uint16_t ki100;      /**< Ki ×100。 */
    uint16_t kd100;      /**< Kd ×100。 */
    uint16_t center;     /**< 舵机机械中值，1 单位=2us 脉宽，与载波频率解耦。 */
    uint16_t left;       /**< 舵机物理左限位，与运行时方向配合使用。 */
    uint16_t right;      /**< 舵机物理右限位。 */
    uint16_t base_duty;  /**< 基础动力 PWM；1000 对应 10.00%。 */
    uint16_t curve_duty; /**< 弯道最低动力，不得大于 base_duty。 */
    uint16_t lost_sum;   /**< 左右有效强度之和的失线阈值（每侧取横/纵较强值）。 */
    int16_t servo_left_sign; /**< +1=占空比增大向左，-1=占空比减小向左。 */
    uint8_t left_reverse;    /**< 逻辑左电机反向开关，0=正向，1=反向。 */
    uint8_t right_reverse;   /**< 逻辑右电机反向开关，0=正向，1=反向。 */
    uint16_t speed_base;     /**< 基础目标，P04 计数/10ms；0 表示未设置。 */
    uint16_t speed_curve;    /**< 大偏差时的目标速度，不大于 speed_base。 */
    uint32_t speed_kp100;    /**< 独立速度环 Kp ×100。 */
    uint32_t speed_ki100;    /**< 独立速度环 Ki ×100。 */
    uint32_t speed_kd100;    /**< 独立速度环 Kd ×100，默认 0。 */
    uint16_t c4_a;           /**< 横电感权重 A ×100，1~1000，100 对应 1.00。 */
    uint16_t c4_b;           /**< 纵电感分子权重 B ×100，0~1000。 */
    uint16_t c4_c;           /**< 纵电感分母权重 C ×100，0~1000。 */
} TrackParams;

/** @brief 电感测量、极值标定及车身参考，全部仅保存在 RAM。 */
typedef struct
{
    uint16_t raw[TRACK_SENSOR_MAX];       /**< 去极值滤波后的 ADC，0～4095。 */
    uint16_t norm[TRACK_SENSOR_MAX];      /**< 每通道独立归一化结果，0～1000。 */
    uint16_t low[TRACK_SENSOR_MAX];       /**< 已提交的最小值。 */
    uint16_t high[TRACK_SENSOR_MAX];      /**< 已提交的最大值。 */
    uint16_t work_low[TRACK_SENSOR_MAX];  /**< 正在采集的最小值，不直接用于循迹。 */
    uint16_t work_high[TRACK_SENSOR_MAX]; /**< 正在采集的最大值。 */
    uint16_t samples;                     /**< 极值采集帧数，饱和到 65535。 */
    uint8_t valid;                        /**< 极值标定是否有效。 */
    uint8_t bad_channel;                  /**< 错误通道从 1 起；255 表示帧数不足。 */
    int16_t center_bias;                  /**< 已提交的居中参考，标度 1000。 */
    int16_t center_min;                   /**< 本次静态采集的最小偏差。 */
    int16_t center_max;                   /**< 本次静态采集的最大偏差。 */
    int32_t center_sum;                   /**< 有符号偏差累计，用于计算均值。 */
    uint16_t center_samples;              /**< 本次居中采集的有效帧数。 */
    uint8_t center_valid;                 /**< 当前极值和 C4 权重下的居中参考是否有效。 */
    uint8_t center_status;                /**< 取值见 TrackBodyStatus。 */
} TrackSensors;

/** @brief 一次循迹控制的持久状态及输出；使用前必须 Reset。 */
typedef struct
{
    TrackPIDState steering_pid; /**< 独立转向 PID 的积分、微分等状态。 */
    int16_t error;              /**< 居中补偿后误差，-1000～1000。 */
    int16_t p_x10;              /**< 遥测比例分量，实际值 ×10。 */
    int16_t i_x10;              /**< 遥测积分分量，实际值 ×10。 */
    int16_t d_x10;              /**< 遥测微分分量，实际值 ×10。 */
    int16_t unclamped_x10;      /**< 限幅前、有方向符号的偏移 ×10。 */
    uint16_t servo;             /**< 舵机最终命令，1 单位=2us 脉宽。 */
    uint16_t left_motor;        /**< 逻辑左轮动力 PWM。 */
    uint16_t right_motor;       /**< 逻辑右轮动力 PWM。 */
    uint8_t saturated;         /**< 舵机输出是否限幅。 */
    uint8_t signal_ok;          /**< 信号强度是否满足循迹条件。 */
    uint16_t weak_ms;           /**< 尚未恢复有效信号的累计标称毫秒数。 */
    uint16_t zero_ms;           /**< 两路连续近零的累计标称毫秒数。 */
    uint8_t crossing;           /**< 是否处于短时十字通过状态。 */
    uint8_t cross_blocked;      /**< 同一次增强尚未结束，禁止重复触发。 */
    uint16_t cross_ms;          /**< 十字通过累计标称毫秒。 */
    uint16_t previous_sum;      /**< 前一帧左右归一化值总和。 */
    int16_t previous_error;     /**< 前一帧居中补偿后的误差。 */
} TrackControl;

/** @brief 加载全部代码初值；仅在 Flash 无有效记录时自动使用。 */
void TrackParams_Default(TrackParams *params);

/** @brief 只恢复 Kp/Ki/Kd 宏初值；不改舵机、速度和失线参数。 */
void TrackParams_RestorePID(TrackParams *params);
/** @brief 只恢复速度环三项增益，不改速度目标和原转向参数。 */
void TrackParams_RestoreSpeedPID(TrackParams *params);
/** @brief 只恢复 C4 的 A/B/C 宏初值，不改变两套 PID、舵机和速度。 */
void TrackParams_RestoreC4(TrackParams *params);

/**
 * @brief 修改舵机方向并交换左右限位标签，保持中值和实际脉宽范围不变。
 * @param params 待修改的有效参数；失败时不修改。
 * @param left_sign +1=增大占空比向左，-1=减小占空比向左。
 * @return 参数及新方向合法时返回 1。
 */
uint8_t TrackParams_SetServoDirection(TrackParams *params, int16_t left_sign);

/** @brief 校验全部持久参数的范围及相互关系；合法返回 1。 */
uint8_t TrackParams_Valid(const TrackParams *params);

/** @brief 开始新的极值采集，保留原有效标定供取消时恢复。 */
void TrackCal_Begin(TrackSensors *sensors);
/** @brief 纳入一帧滤波 ADC；raw 数组长度不得小于启用通道数。 */
void TrackCal_Sample(TrackSensors *sensors, const uint16_t *raw);
/** @brief 校验并提交极值，成功返回 1，同时使旧居中参考失效。 */
uint8_t TrackCal_Commit(TrackSensors *sensors);
/** @brief 更新测量值；未标定时归一化结果为 0。 */
void TrackSensors_Update(TrackSensors *sensors, const uint16_t *raw);
/** @brief 线性归一化并限幅到 0～1000；无效量程返回 0。 */
uint16_t TrackNormalize(uint16_t value, uint16_t low, uint16_t high);
/** @brief 获取左右有效强度；四路时每侧取横/纵较强值，双路时取原值。 */
void TrackSensors_Strength(const TrackSensors *sensors, uint16_t *left, uint16_t *right);
/** @brief 是否有可用于转向的有效分母；强度和有效性必须同时满足。 */
uint8_t TrackSensors_SteeringValid(const TrackSensors *sensors, const TrackParams *params);
/** @brief 未居中补偿的误差 ×1000；四路使用论文 C4，双路保留差比和。 */
int16_t TrackSensors_Error(const TrackSensors *sensors, const TrackParams *params);
/** @brief PID 位置反馈 ×1000：差比和减去有效居中参考，限幅到 -1000～1000。
 *  @note 这是电感估计的横向偏差，不是舵机实测角度；十字处理不会覆盖此值。
 */
int16_t TrackSensors_Feedback(const TrackSensors *sensors, const TrackParams *params);
/** @brief 上电检查实际编译的归一化、正负差比和与有符号均值，成功为 1。 */
uint8_t TrackSensors_MathCheck(void);
/** @brief C4 权重等测量规则变化后清除居中参考，保留原电感极值标定。 */
void TrackBodyCenter_Invalidate(TrackSensors *sensors);
/** @brief 启动一次居中采集，不清除旧参考。 */
void TrackBodyCenter_Begin(TrackSensors *sensors);
/** @brief 检查并纳入居中样本；弱信号、晃动、偏离或饱和即中止。 */
void TrackBodyCenter_Sample(TrackSensors *sensors, const TrackParams *params);
/** @brief 仅提交完整且合格的居中均值，成功返回 1。 */
uint8_t TrackBodyCenter_Commit(TrackSensors *sensors);
/** @brief 清空循迹历史、重置 PID，舵机目标设为机械中值。 */
void TrackControl_Reset(TrackControl *control, const TrackParams *params);

/**
 * @brief 计算一帧转向与失线保持；正常公共动力由独立速度环和应用层生成。
 * @param control 跨周期复用的状态和输出。
 * @param sensors 本帧测量及已提交的标定数据。
 * @param params  已校验范围的运行参数，函数内部仍会复核。
 * @param drive   1=行驶，保留上一实际动力供失线限速；0=静态调 PID，清零电机。
 * @return 故障原因；弱信号确认期间返回 OK，但 signal_ok 为 0。
 * @note   drive=1 时确认期间保留受限输出；确认失线才返回 LOST 并清零电机。
 *         drive=0 时信号不足不输出电机，应用层关闭舵机预览。
 */
TrackFault TrackControl_Step(TrackControl *control, const TrackSensors *sensors,
                            const TrackParams *params, uint8_t drive);

/** @brief CRC16-CCITT，初值 FFFF；用于 Flash 记录校验。 */
uint16_t TrackCRC16(const uint8_t *bytes, uint16_t length);
/** @brief 编码为 v2 的 32 字节记录，方向位受 CRC 保护，commit 最后写入。 */
void TrackRecord_Encode(uint8_t *record, const TrackParams *params, uint32_t sequence);
/** @brief 校验并解码记录；失败返回 0，不修改输出参数。 */
uint8_t TrackRecord_Decode(const uint8_t *record, TrackParams *params, uint32_t *sequence);
/** @brief 判断序号是否更新，兼容 32 位计数回绕。 */
uint8_t TrackSequence_Newer(uint32_t first, uint32_t second);
#endif
