/**
 * @file    track_config.h
 * @brief   用户配置入口：PID 初值、动力 PWM、标定条件和硬件方向。
 * @note    有效 Flash 参数优先。修改 PID 宏后，在 PID 页恢复初值并保存。
 */
#ifndef TRACK_CONFIG_H
#define TRACK_CONFIG_H

/* PID 初值直接填写实际值；例如 40.00f 表示 Kp=40.00。 */
#ifndef TRACK_PID_DEFAULT_KP
#define TRACK_PID_DEFAULT_KP    115.00f
#endif
#ifndef TRACK_PID_DEFAULT_KI
#define TRACK_PID_DEFAULT_KI     0.00f
#endif
#ifndef TRACK_PID_DEFAULT_KD
#define TRACK_PID_DEFAULT_KD     2.15f
#endif
#define TRACK_PID_KP_MAX_X100   50000U /* Kp 上限 500.00；代码初值与现场保存值可分别调整。 */
#define TRACK_PID_KI_MAX_X100    2000U
#define TRACK_PID_KD_MAX_X100    2000U
#define TRACK_PID_D_FILTER      0.25f  /* 微分低通系数，越大响应越快。 */
#define TRACK_PID_INTEGRAL_MAX  2.0f   /* 积分累计上限，单位：误差·秒。 */
#define TRACK_STEERING_TARGET1000 0   /* 居中补偿后的目标偏差，正常循迹保持为 0。 */

/* 通道与周期：中断只维护时基、按键和截止保护，PID 在前台执行。 */
#define TRACK_SENSOR_MAX       4      /* 数组容量，当前预留到四路。 */
#ifndef TRACK_SENSOR_COUNT
#define TRACK_SENSOR_COUNT     2      /* 实际启用的 ADC 通道数。 */
#endif
#define TRACK_PERIOD_MS        5      /* 标称控制周期，单位 ms。 */
#define TRACK_KEY_DEBOUNCE_MS   20     /* 按键确认时间，单位 ms。 */
#define TRACK_CONTROL_TIMEOUT  20     /* 无输出更新时的中断截止时间，单位 ms。 */
#define TRACK_DISPLAY_PERIOD_MS 100   /* OLED 重绘周期，单位 ms。 */
#define TRACK_UART_BAUD         9600UL /* 与蓝牙模块串口波特率保持一致，8N1。 */
#define TRACK_TELEMETRY_MS      250    /* 完整 CSV 按 4Hz 发送，为 9600 串口留出余量。 */
/* 极值与车身居中标定；强度标度为 0～1000，ADC 原始值为 0～4095。 */
#define TRACK_CAL_MIN_SAMPLES  100    /* 极值标定至少采集的帧数。 */
#define TRACK_CAL_MIN_SPAN     100    /* 放大板正常时，每路原始 ADC 的最低有效跨度。 */
#define TRACK_BODY_SAMPLES     200    /* 静态居中采集帧数，约 1 秒。 */
#define TRACK_BODY_MIN_EACH    100    /* 居中时每路归一化信号的最低值。 */
#define TRACK_BODY_MAX_BIAS    300    /* 居中允许的偏差绝对值，标度 1000。 */
#define TRACK_BODY_MAX_SPREAD  60     /* 静态采集期间最大偏差变化范围。 */
#define TRACK_ADC_TIMEOUT      2000U  /* 等待 ADC 的循环次数上限，不是毫秒。 */
#define TRACK_ADC_RAIL         4090U  /* 12 位 ADC 高端饱和判据。 */
/* 舵机：占空比使用 0～10000 标度，50Hz 下每个单位约 2us。 */
#define TRACK_SERVO_FREQUENCY  50UL
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
#define TRACK_BASE_DUTY        2000   /* 无有效 Flash 时基础动力 10.00%。 */
#define TRACK_CURVE_MIN_DUTY   1000    /* 无有效 Flash 时弯道最低动力 6.00%。 */
#define TRACK_DUTY_MAX         4000   /* 菜单校验和实际输出共同上限 40.00%。 */
#define TRACK_SPEED_FINE_STEP  10U  /* 速度页精调：0.10% 占空比。 */
#define TRACK_SPEED_COARSE_STEP 100U /* 速度页粗调：1.00% 占空比。 */
#define TRACK_MOTOR_RAMP       10    /* 每个周期最多增加的动力单位。 */
#define TRACK_LEFT_TRIM        0     /* 左轮开环动力补偿，可正可负。 */
#define TRACK_RIGHT_TRIM       0     /* 右轮开环动力补偿，可正可负。 */
/* 失线与十字判据；使用未扣除居中参考的电感强度。 */
#define TRACK_LOST_SUM_MIN     1U    /* 保存参数的合法下界；0 会使全零信号无法触发失线。 */
#define TRACK_LOST_SUM_MAX     500U  /* 保存参数的合法上界，与代码初值分开定义。 */
#define TRACK_LOST_SUM         40   /* 左右归一化总和低于此值进入弱信号确认。 */
#define TRACK_HARD_LOST_EACH   15    /* 两路均低于此值，采用较短的失线确认时间。 */
#define TRACK_LOST_CONFIRM_MS  100U  /* 普通弱信号持续时间，按标称控制周期累计。 */
#define TRACK_ZERO_CONFIRM_MS  50U   /* 两路同时近零持续时间，单位 ms。 */
#define TRACK_LOST_RECOVER_MARGIN 50U /* 恢复需高于进入阈值，避免临界值反复切换。 */
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

#if TRACK_SENSOR_COUNT < 2 || TRACK_SENSOR_COUNT > TRACK_SENSOR_MAX
#error Configure between two and four sensor channels
#endif
#if TRACK_ERROR_SIGN != 1 && TRACK_ERROR_SIGN != -1
#error TRACK_ERROR_SIGN must be +1 or -1
#endif
#if TRACK_LOST_SUM < TRACK_LOST_SUM_MIN || TRACK_LOST_SUM > TRACK_LOST_SUM_MAX
#error TRACK_LOST_SUM is outside the valid parameter range
#endif
#if TRACK_ZERO_CONFIRM_MS < TRACK_PERIOD_MS || TRACK_LOST_CONFIRM_MS < TRACK_ZERO_CONFIRM_MS
#error Invalid signal loss confirmation times
#endif
#if (TRACK_TX_SIZE & (TRACK_TX_SIZE - 1)) != 0
#error UART buffer size must be a power of two
#endif
#endif
