/**
 * @file    track_hw.h
 * @brief   应用层硬件接口；除 ISR 标记接口外均从前台调用。
 */
#ifndef TRACK_HW_H
#define TRACK_HW_H
#include "track_core.h"

/* 五向按键事件位，可组合；左键在业务层拥有退出/撤销优先级。 */
#define KEY_UP_EVENT     0x01
#define KEY_DOWN_EVENT   0x02
#define KEY_LEFT_EVENT   0x04
#define KEY_RIGHT_EVENT  0x08
#define KEY_OK_EVENT     0x10

extern volatile uint32_t track_milliseconds; /**< 中断维护的毫秒计数。 */
extern volatile uint8_t track_guard_armed;   /**< 执行器截止保护是否启用。 */
extern volatile uint8_t track_guard_age;     /**< 距离上次有效输出更新的毫秒数。 */
extern volatile uint8_t track_guard_trip;    /**< 截止触发锁存，需重新 Arm 才能清除。 */
extern volatile uint8_t track_key_events;    /**< 中断发布、前台取走的按键事件。 */
extern volatile uint16_t track_adc_frames;   /**< 1ms 扫描完成整帧数，16 位自然回绕。 */
extern volatile uint16_t track_adc_overruns; /**< 来不及在下一毫秒启动新帧的累计次数。 */
extern volatile uint16_t track_tx_dropped;   /**< 串口队列满时丢弃的新帧累计。 */

/** @brief 先设置安全电平，再初始化 PWM、ADC、按键、UART 和 1ms 时基。 */
void TrackHW_Init(void);
/** @brief 由 TIM0 每 1ms 调用，处理截止保护、按键消抖和编码器快照，不做浮点运算。 */
void TrackHW_TickISR(void);
/** @brief ADC 转换完成中断入口；无等待、浮点或乘除库调用。 */
void TrackHW_AdcISR(void);
/** @brief 关闭电机 PWM、两组输入置同高；关闭舵机脉冲并拉低 P74。可从 ISR 调用。 */
void TrackHW_Stop(void);
/**
 * @brief 写入执行器命令；电机使用万分比占空比，舵机使用 2us 脉宽单位。
 * @param servo        舵机脉宽值（1 单位=2us），必须在绝对调试边界内。
 * @param left         逻辑左电机动力，不得超过 TRACK_DUTY_MAX。
 * @param right        逻辑右电机动力，不得超过 TRACK_DUTY_MAX。
 * @param motor_enable 仅运行模式传 1，其余模式传 0。
 * @note  未 Arm 或截止已触发时不允许重新输出；合法更新会重置截止计时。
 */
void TrackHW_Set(uint16_t servo, uint16_t left, uint16_t right, uint8_t motor_enable);
/** @brief 先停输出、清截止状态，再按 enable 开关保护；不会自行启动电机。 */
void TrackHW_Arm(uint8_t enable);
/** @brief 只在解除 Arm 后配置逻辑左右电机方向；参数非 0/1 或仍 Arm 则拒绝。 */
uint8_t TrackHW_SetMotorDirections(uint8_t left_reverse, uint8_t right_reverse);
/** @brief 原子读取毫秒计数；调用者用无符号减法处理回绕。 */
uint32_t TrackHW_Now(void);
/** @brief 原子取走并清空按键事件，返回 KEY_*_EVENT 位组合。 */
uint8_t TrackHW_Keys(void);
/** @brief 复制最新完整 ADC 扫描并去极值平均；未就绪/过期/故障返回 0，不阻塞。 */
uint8_t TrackHW_Sample(uint16_t *values);
/** @brief 喂硬件看门狗；不从周期中断持续喂狗。 */
void TrackHW_FeedWatchdog(void);
/** @brief 原子复制 TIM0 发布的最近计数快照；失败返回 0，不在前台读 TIM3。 */
uint8_t TrackHW_EncoderSnapshot(uint16_t *count, uint8_t *direction);
/** @brief 同时复制计数、方向电平及采集时刻；未就绪或采集错误返回 0。
 *  @note 1ms 中断只发布整数快照；前台按 sample_ms 的实际差值换算速度。
 *        任何两次读取间发生的采集错误会锁存至读取，不会被后续成功样本掩盖。
 */
uint8_t TrackHW_EncoderSample(uint16_t *count, uint8_t *direction, uint32_t *sample_ms);
/** @brief 读取最新有效 Flash 记录；无记录时加载代码初值并返回 0。 */
uint8_t TrackStore_Load(TrackParams *params);
/** @brief 最近一次保存结果，用于区分参数、擦除、写入及回读故障。 */
typedef enum
{
    TRACK_STORE_OK = 0,
    TRACK_STORE_PARAMS,
    TRACK_STORE_BUSY,
    TRACK_STORE_ERASE,
    TRACK_STORE_WRITE,
    TRACK_STORE_READ,
    TRACK_STORE_VERIFY
} TrackStoreError;
/** @brief 关闭输出且解除 Arm 后才能保存；成功返回 1，失败保留原槽。 */
uint8_t TrackStore_Save(const TrackParams *params);
/** @brief 查询最近一次保存的错误阶段；不会触发 EEPROM 操作。 */
TrackStoreError TrackStore_LastError(void);
/** @brief 队列及在途最后一字节均发送完才返回 1；只查询，不等待。 */
uint8_t TrackUART_IsIdle(void);
/** @brief 非阻塞入队完整字符串；空间不足返回 0 并累计丢帧。 */
uint8_t TrackUART_Queue(const char *text);
/** @brief 由 UART1 发送中断调用，最多发送一个字节。 */
void TrackUART_TxISR(void);
#endif
