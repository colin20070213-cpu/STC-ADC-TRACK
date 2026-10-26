/**
 * @file    track_config.h
 * @brief   用户配置入口：PID 初值、动力 PWM、标定条件和硬件方向。
 * @note    有效 Flash 参数优先。修改 PID 宏后，在 PID 页恢复初值并保存。
 */
#ifndef TRACK_CONFIG_H
#define TRACK_CONFIG_H

/* PID 初值直接填写实际值；例如 40.00f 表示 Kp=40.00。 */
#ifndef TRACK_PID_DEFAULT_KP
#define TRACK_PID_DEFAULT_KP    130.00f
#endif
#ifndef TRACK_PID_DEFAULT_KI
#define TRACK_PID_DEFAULT_KI     0.00f
#endif
#ifndef TRACK_PID_DEFAULT_KD
#define TRACK_PID_DEFAULT_KD     8.15f
#endif
#define TRACK_PID_KP_MAX_X100   50000U /* Kp 上限 500.00；代码初值与现场保存值可分别调整。 */
#define TRACK_PID_KI_MAX_X100    2000U
#define TRACK_PID_KD_MAX_X100    2000U
#define TRACK_PID_D_FILTER      0.25f  /* 微分低通系数，越大响应越快。 */
#define TRACK_PID_INTEGRAL_MAX  2.0f   /* 积分累计上限，单位：误差·秒。 */
#define TRACK_STEERING_TARGET1000 0   /* 居中补偿后的目标偏差，正常循迹保持为 0。 */

/* 通道与周期：时基中断维护按键/保护并触发 ADC 扫描，PID 在前台执行。 */
#define TRACK_SENSOR_MAX       4      /* 数组容量，当前预留到四路。 */
#ifndef TRACK_SENSOR_COUNT
#define TRACK_SENSOR_COUNT     4      /* 实际启用的 ADC 通道数。 */
#endif
#define TRACK_ADC_FRAME_MS     1U     /* TIM0 每 1ms 启动一次全部电感的非阻塞扫描。 */
#define TRACK_ADC_STALE_MS     3U     /* 最新整帧超过此年龄即禁止使用，单位 ms。 */
#define TRACK_PERIOD_MS        2U     /* 转向 PID 标称周期 2ms，独立于 1ms ADC 扫描。 */
/* 数组顺序保持 L、R、LM、RM；物理从左到右为 L、LM、RM、R。
 * 两侧 L/R 是水平横置电感；中间 LM/RM 是同平面的纵置电感。
 * 中间引脚仅为占位：接车运行前必须按实际接线修改下面两个宏。 */
#define TRACK_ADC_LEFT_PIN          ADC_P13
#define TRACK_ADC_RIGHT_PIN         ADC_P01
#define TRACK_ADC_MIDDLE_LEFT_PIN   ADC_P10 /* TODO：确认左纵电感实际引脚。 */
#define TRACK_ADC_MIDDLE_RIGHT_PIN  ADC_P05 /* TODO：确认右纵电感实际引脚。 */
/* 论文 C4：Err=P*[A*(L-R)+B*(LM-RM)]/[A*(L+R)+C*abs(LM-RM)]。
 * A/B/C 使用相同整数权重，100 对应 1.00；P 使用千分比。
 * 这些是待实车整定的起点，独立于舵机 PID 的 Kp/Ki/Kd。 */
#define TRACK_FOUR_WEIGHT_A        100U
#define TRACK_FOUR_WEIGHT_B        100U
#define TRACK_FOUR_WEIGHT_C        100U
#define TRACK_FOUR_SCALE1000       1000U
#define TRACK_FOUR_DEN_MIN         10U /* 分母折算为 A=1 时的最低归一化强度。 */
#define TRACK_KEY_DEBOUNCE_MS   20     /* 按键确认时间，单位 ms。 */
#define TRACK_CONTROL_TIMEOUT  20     /* 无输出更新时的中断截止时间，单位 ms。 */
#define TRACK_DISPLAY_PERIOD_MS 100   /* OLED 重绘周期，单位 ms。 */
#define TRACK_UART_BAUD         9600UL /* 与蓝牙模块串口波特率保持一致，8N1。 */
#define TRACK_TELEMETRY_MS      500    /* 增加速度闭环字段后按 2Hz 发送，适配 9600 串口。 */
/* 极值与车身居中标定；强度标度为 0～1000，ADC 原始值为 0～4095。 */
#define TRACK_CAL_MIN_SAMPLES  ((500U + TRACK_PERIOD_MS - 1U) / TRACK_PERIOD_MS) /* 至少约 0.5 秒。 */
#define TRACK_CAL_MIN_SPAN     100    /* 放大板正常时，每路原始 ADC 的最低有效跨度。 */
#define TRACK_BODY_SAMPLES     ((1000U + TRACK_PERIOD_MS - 1U) / TRACK_PERIOD_MS) /* 保持约 1 秒。 */
#define TRACK_BODY_MIN_EACH    100    /* 居中时每路归一化信号的最低值。 */
#define TRACK_BODY_MAX_BIAS    300    /* 居中允许的偏差绝对值，标度 1000。 */
#define TRACK_BODY_MAX_SPREAD  60     /* 静态采集期间最大偏差变化范围。 */
#define TRACK_ADC_RAIL_CONFIRM_MS 15U /* 保留原约 15ms 连续饱和确认；2ms 下取 8 帧。 */
#define TRACK_ADC_RAIL_FRAMES   ((TRACK_ADC_RAIL_CONFIRM_MS + TRACK_PERIOD_MS - 1U) / TRACK_PERIOD_MS)
#define TRACK_ADC_RAIL         4090U  /* 12 位 ADC 高端饱和判据。 */
/* 舵机三值保留旧界面标度：1 单位=2us 脉宽；换频率时三值不变。
 * 用户已确认 SPT5410LV，采用产品资料标称的 330Hz 信号频率。 */
#ifndef TRACK_SERVO_FREQUENCY
#define TRACK_SERVO_FREQUENCY  330UL
#endif
#define TRACK_SERVO_UNIT_US    2UL
#define TRACK_SERVO_ABS_MIN    600    /* 绝对调试边界，还需服从实际机械限制。 */
#define TRACK_SERVO_ABS_MAX    900
#define TRACK_SERVO_DEFAULT_CENTER 760U
#define TRACK_SERVO_DEFAULT_LEFT   840U /* 正向安装时的左限位，允许不对称行程。 */
#define TRACK_SERVO_DEFAULT_RIGHT  670U /* 反向安装时由初始化函数交换左右标签。 */
/* 默认舵机方向；运行时可在“舵机设置”更改并保存，Flash 优先。 */
#define TRACK_SERVO_LEFT_SIGN  1
#define TRACK_ERROR_SIGN       1     /* 电感误差方向，仅允许 +1/-1。 */
/* 电机：这些值是动力 PWM，不是 cm/s。Flash 可覆盖默认基础/弯道动力。 */
#define TRACK_MOTOR_FREQUENCY  10000UL /* 电机 PWM 频率，单位 Hz。 */
#define TRACK_MOTOR_SWAP       0      /* 两组电机对调：0=否，1=是。 */
#define TRACK_LEFT_REVERSE     0      /* 默认逻辑左电机方向，菜单可覆盖并保存。 */
#define TRACK_RIGHT_REVERSE    0      /* 默认逻辑右电机方向，菜单可覆盖并保存。 */
#define TRACK_BASE_DUTY        3000   /* 速度闭环直线动力上限，当前初值 30.00%。 */
#define TRACK_CURVE_MIN_DUTY   1500   /* 速度闭环弯道动力上限，当前初值 15.00%。 */
#define TRACK_DUTY_MAX         4000   /* 菜单校验和实际输出共同上限 40.00%。 */
#define TRACK_SPEED_FINE_STEP  10U  /* 速度页精调：0.10% 占空比。 */
#define TRACK_SPEED_COARSE_STEP 100U /* 速度页粗调：1.00% 占空比。 */
#define TRACK_MOTOR_RAMP       (2U * TRACK_PERIOD_MS) /* 每毫秒 2 单位，2ms 每次增加 4。 */
#define TRACK_LEFT_TRIM        0     /* 左轮开环动力补偿，可正可负。 */
#define TRACK_RIGHT_TRIM       0     /* 右轮开环动力补偿，可正可负。 */
/* 右后轮单编码器速度环。速度单位为 P04 计数/10ms；未知每圈计数时不换算 m/s。
 * 默认目标为 0，需在“电机设置→速度设定”设置非零基础速度后发车。
 * 原 BASE/CURVE PWM 参数继续作为直线/弯道动力上限，不能当作速度值迁移。 */
#define TRACK_ENCODER_SAMPLE_MS     1U  /* TIM0 保存原始计数快照，不等待脉冲。 */
#define TRACK_ENCODER_STALE_MS      2U  /* 前台拒绝超期的计数快照。 */
#define TRACK_SPEED_PERIOD_MS       2U  /* 速度闭环每两个编码器快照更新。 */
#define TRACK_SPEED_BASE_DEFAULT    0U
#define TRACK_SPEED_CURVE_DEFAULT   0U
#define TRACK_SPEED_TARGET_MAX      1000U
#ifndef TRACK_SPEED_DEFAULT_KP
#define TRACK_SPEED_DEFAULT_KP      30.00f
#endif
#ifndef TRACK_SPEED_DEFAULT_KI
#define TRACK_SPEED_DEFAULT_KI      30.00f
#endif
#ifndef TRACK_SPEED_DEFAULT_KD
#define TRACK_SPEED_DEFAULT_KD       0.00f
#endif
#define TRACK_SPEED_GAIN_MAX_X100    50000U
#define TRACK_SPEED_FEEDBACK_TAU_MS  10.0f /* 轮速低通时间常数，抑制短窗口量化跳动。 */
#define TRACK_SPEED_D_TAU_MS         30.0f /* 测量微分低通时间常数，与实际 dt 配合。 */
#define TRACK_SPEED_RAMP_PER_SECOND  25.0f /* 每秒增加的目标计数/10ms。 */
#define TRACK_SPEED_PWM_RISE_PER_MS  2U
#define TRACK_SPEED_STALL_DUTY       1000U
#define TRACK_SPEED_STALL_MS         1000U
#define TRACK_ENCODER_MAX_PER_10MS   2000U /* 单边沿计数合理上界，防止异常读数。 */

/* 失线与十字判据；使用未扣除居中参考的电感强度。 */
#define TRACK_LOST_SUM_MIN     1U    /* 保存参数的合法下界；0 会使全零信号无法触发失线。 */
#define TRACK_LOST_SUM_MAX     500U  /* 保存参数的合法上界，与代码初值分开定义。 */
#define TRACK_LOST_SUM         40   /* 左右归一化总和低于此值进入弱信号确认。 */
#define TRACK_RUN_LOST_SUM_MAX 20U  /* 仅运行时限制失线门槛；旧 EEPROM 较大值不会抵消本次放宽。 */
#define TRACK_HARD_LOST_EACH   5    /* 全部电感均低于此值，采用较短失线确认。 */
#define TRACK_LOST_CONFIRM_MS  250U  /* 普通弱信号持续时间，按标称控制周期累计。 */
#define TRACK_ZERO_CONFIRM_MS  120U   /* 全部电感同时近零持续时间，单位 ms。 */
#define TRACK_LOST_RECOVER_MARGIN 10U /* 恢复需高于进入阈值，避免临界值反复切换。 */
#define TRACK_LOST_DUTY_MAX    600U  /* 确认期间保持转向，动力不超过 6%，且不加速。 */
#define TRACK_CROSS_SUM        1500  /* 十字候选的总强度下限。 */
#define TRACK_CROSS_ERROR      120   /* 十字进入前后允许的小偏差。 */
#define TRACK_CROSS_MAX_MS     120   /* 十字状态的最长标称时间。 */
/* STC32G12K128 用户 EEPROM 从 FE0000 开始，ISP 中至少分配 1KB。
 * IAP 使用从 0 开始的偏移；不能使用程序 Flash 末尾的偏移作为 EEPROM。 */
#define TRACK_STORE_SLOT_A     0x000000UL
#define TRACK_STORE_SLOT_B     0x000200UL
#define TRACK_STORE_PAGE_SIZE  512U
#define TRACK_TX_SIZE          512U

#if TRACK_LEFT_TRIM != TRACK_RIGHT_TRIM
#error Single-encoder common speed control requires equal left and right trim
#endif
#if TRACK_ENCODER_SAMPLE_MS != 1 || TRACK_SPEED_PERIOD_MS % TRACK_ENCODER_SAMPLE_MS != 0
#error Encoder snapshots require the 1ms tick and an integer speed period multiple
#endif
#if TRACK_SPEED_PERIOD_MS < TRACK_PERIOD_MS || TRACK_SPEED_PERIOD_MS % TRACK_PERIOD_MS != 0
#error Invalid speed loop period
#endif
#if TRACK_SERVO_FREQUENCY < 50 || TRACK_SERVO_FREQUENCY > 500
#error Servo frequency must be within 50 to 500 Hz and supported by the actual servo
#endif
#if TRACK_ADC_FRAME_MS != 1 || TRACK_ADC_STALE_MS < 2
#error ADC scanner requires a 1ms frame and at least 2ms freshness limit
#endif
#if TRACK_FOUR_WEIGHT_A < 1 || TRACK_FOUR_WEIGHT_A > 1000 || TRACK_FOUR_WEIGHT_B > 1000 || TRACK_FOUR_WEIGHT_C > 1000
#error Four sensor weights must be integers from 0 to 1000 with A positive
#endif
#if TRACK_FOUR_SCALE1000 < 1 || TRACK_FOUR_SCALE1000 > 2000 || TRACK_FOUR_DEN_MIN < 1 || TRACK_FOUR_DEN_MIN > 1000
#error Invalid four sensor scale or denominator guard
#endif
#if TRACK_SENSOR_COUNT != 2 && TRACK_SENSOR_COUNT != 4
#error Configure exactly two or four sensor channels
#endif
#if TRACK_ERROR_SIGN != 1 && TRACK_ERROR_SIGN != -1
#error TRACK_ERROR_SIGN must be +1 or -1
#endif
#if TRACK_LOST_SUM < TRACK_LOST_SUM_MIN || TRACK_LOST_SUM > TRACK_LOST_SUM_MAX
#error TRACK_LOST_SUM is outside the valid parameter range
#endif
#if TRACK_RUN_LOST_SUM_MAX < TRACK_LOST_SUM_MIN || TRACK_RUN_LOST_SUM_MAX > TRACK_LOST_SUM_MAX
#error Invalid run-only signal loss threshold cap
#endif
#if TRACK_ZERO_CONFIRM_MS < TRACK_PERIOD_MS || TRACK_LOST_CONFIRM_MS < TRACK_ZERO_CONFIRM_MS
#error Invalid signal loss confirmation times
#endif
#if (TRACK_TX_SIZE & (TRACK_TX_SIZE - 1)) != 0
#error UART buffer size must be a power of two
#endif
#endif
