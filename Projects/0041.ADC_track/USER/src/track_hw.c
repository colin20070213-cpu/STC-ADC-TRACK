/**
 * @file    track_hw.c
 * @brief   ADC、电机/舵机 PWM、按键、时基及非阻塞串口。
 * @note    带 ISR 后缀的函数由中断调用；计算和显示由前台执行。
 */
#include "track_hw.h"
#include <string.h>
#ifdef TRACK_HOST_TEST
#include "track_hw_test_device.h"
#else
#include "CNU_PIE_ADC.h"
#include "CNU_PIE_GPIO.h"
#include "CNU_PIE_PWM.h"
#include "CNU_PIE_TIMER.h"
#include "CNU_PIE_UART.h"
#endif

volatile uint32_t track_milliseconds;
volatile uint8_t track_guard_armed;
volatile uint8_t track_guard_age;
volatile uint8_t track_guard_trip;
volatile uint8_t track_key_events;
volatile uint16_t track_tx_dropped;

static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint8_t tx_active;
static uint8_t tx_data[TRACK_TX_SIZE];
static uint8_t key_count[5];  /**< 按下/松开的消抖计数。 */
static uint8_t key_stable[5]; /**< 已确认的按下状态。 */
static uint16_t key_hold[5];  /**< 长按计时；只有上下键连发。 */
static uint16_t motor_period;
static uint16_t servo_period;
static uint8_t motor_left_reverse = TRACK_LEFT_REVERSE;
static uint8_t motor_right_reverse = TRACK_RIGHT_REVERSE;

/* 通道顺序：左 P13、右 P01、预留左 P16、预留右 P05。未接线不要启用。 */
static const ADC_PIN_ENUM sensor_pins[TRACK_SENSOR_MAX] =
{
    ADC_P13, ADC_P01, ADC_P16, ADC_P05
};

void TrackHW_Stop(void)
{
    /* 中断也可直接调用：不使用浮点、库函数或与前台共用的临时变量。 */
    PWMA_ENO &= ~0x55;
    P60 = 1;
    P62 = 1;
    P64 = 1;
    P66 = 1;
    PWMB_ENO &= ~0x01;
    P74 = 0;
}

void TrackHW_Arm(uint8_t enable)
{
    uint8_t previous_irq = EA;

    EA = 0;
    track_guard_armed = 0;
    TrackHW_Stop();
    track_guard_age = 0;
    track_guard_trip = 0;
    track_guard_armed = enable;
    EA = previous_irq;
}

uint8_t TrackHW_SetMotorDirections(uint8_t left_reverse, uint8_t right_reverse)
{
    if (track_guard_armed || left_reverse > 1 || right_reverse > 1)
    {
        return 0;
    }
    TrackHW_Stop();
    motor_left_reverse = left_reverse;
    motor_right_reverse = right_reverse;
    return 1;
}

void TrackHW_TickISR(void)
{
    uint8_t key_index;
    uint8_t pressed;
    uint8_t mask;

    ++track_milliseconds;
    if (track_guard_armed && ++track_guard_age >= TRACK_CONTROL_TIMEOUT)
    {
        track_guard_trip = 1;
        TrackHW_Stop();
    }
    pressed = (!P42 ? KEY_UP_EVENT : 0) | (!P46 ? KEY_DOWN_EVENT : 0) |
              (!P45 ? KEY_LEFT_EVENT : 0) | (!P41 ? KEY_RIGHT_EVENT : 0) |
              (!P27 ? KEY_OK_EVENT : 0);
    for (key_index = 0, mask = 1; key_index < 5; ++key_index, mask <<= 1)
    {
        if (pressed & mask)
        {
            if (key_count[key_index] < TRACK_KEY_DEBOUNCE_MS)
            {
                ++key_count[key_index];
            }
            if (key_count[key_index] == TRACK_KEY_DEBOUNCE_MS && !key_stable[key_index])
            {
                key_stable[key_index] = 1;
                key_hold[key_index] = 0;
                track_key_events |= mask;
            }
            if (key_stable[key_index])
            {
                if (key_hold[key_index] < 600)
                {
                    ++key_hold[key_index];
                }
                else if (key_index < 2)
                {
                    track_key_events |= mask;
                    key_hold[key_index] = 500;
                }
            }
        }
        else
        {
            if (key_count[key_index])
            {
                --key_count[key_index];
            }
            if (!key_count[key_index])
            {
                key_stable[key_index] = 0;
                key_hold[key_index] = 0;
            }
        }
    }
}

uint32_t TrackHW_Now(void)
{
    uint32_t now;
    uint8_t previous_irq = EA;

    /* 251 上多字节读取可能被时基中断打断，短暂关中断保证快照一致。 */
    EA = 0;
    now = track_milliseconds;
    EA = previous_irq;
    return now;
}

uint8_t TrackHW_Keys(void)
{
    uint8_t events;
    uint8_t previous_irq = EA;

    EA = 0;
    events = track_key_events;
    track_key_events = 0;
    EA = previous_irq;
    return events;
}

void TrackHW_Init(void)
{
    uint8_t channel;

    /* 在改变端口模式及 OLED 延时之前，先写入安全电平。 */
    P60 = 1;
    P62 = 1;
    P64 = 1;
    P66 = 1;
    P74 = 0;
    GPIO_Init(GPIO_P6, GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_6, GPIO_OUT_PP);
    GPIO_Init(GPIO_P7, GPIO_Pin_4, GPIO_OUT_PP);
    PWM_Init(PWMA_CH1P_P60, TRACK_MOTOR_FREQUENCY, 10000);
    PWM_Init(PWMA_CH2P_P62, TRACK_MOTOR_FREQUENCY, 10000);
    PWM_Init(PWMA_CH3P_P64, TRACK_MOTOR_FREQUENCY, 10000);
    PWM_Init(PWMA_CH4P_P66, TRACK_MOTOR_FREQUENCY, 10000);
    PWM_Init(PWMB_CH1_P74, TRACK_SERVO_FREQUENCY, 0);
    motor_period = ((uint16_t)PWMA_ARRH << 8) | PWMA_ARRL;
    servo_period = ((uint16_t)PWMB_ARRH << 8) | PWMB_ARRL;
    TrackHW_Stop();

    P42 = 1;
    P46 = 1;
    P45 = 1;
    P41 = 1;
    P27 = 1;
    GPIO_Init(GPIO_P4, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6, GPIO_PullUp);
    GPIO_Init(GPIO_P2, GPIO_Pin_7, GPIO_PullUp);
    GPIO_PinPullConfig(GPIO_P4, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6, GPIO_Pull_Up);
    GPIO_PinPullConfig(GPIO_P2, GPIO_Pin_7, GPIO_Pull_Up);
    for (channel = 0; channel < TRACK_SENSOR_COUNT; ++channel)
    {
        ADC_Init(sensor_pins[channel], ADC_SPEED_2X16T);
    }
    UART_Init(UART_1, UART1_RX_P43, UART1_TX_P44, TRACK_UART_BAUD, TIM1);
    /* 库的 TIM0 清标志实现存在 TF1 操作，本工程中断入口显式清 TF0。 */
    PIT_Timer_Ms(TIM0, 1);
    PT0 = 1;
}

/** @brief 完成一次有超时上限的 ADC 转换；返回 0 表示超时或结果越界。 */
static uint8_t adc_once(ADC_PIN_ENUM pin, uint16_t *value)
{
    uint16_t remaining = TRACK_ADC_TIMEOUT;

    ADC_CONTR = (ADC_CONTR & 0x90) | pin;
    ADC_CONTR |= 0x40;
    while (!(ADC_CONTR & 0x20))
    {
        if (!--remaining)
        {
            return 0;
        }
    }
    ADC_CONTR &= ~0x20;
    *value = ((uint16_t)ADC_RES << 8) | ADC_RESL;
    return *value <= 4095;
}

uint8_t TrackHW_Sample(uint16_t *values)
{
    uint8_t channel;
    uint8_t sample_index;
    uint16_t value;
    uint16_t minimum;
    uint16_t maximum;
    uint16_t discarded;
    uint32_t total;

    for (channel = 0; channel < TRACK_SENSOR_COUNT; ++channel)
    {
        minimum = 4095;
        maximum = 0;
        total = 0;
        /* 切换复用通道后丢弃首个读数，留出模拟输入建立时间。 */
        if (!adc_once(sensor_pins[channel], &discarded))
        {
            return 0;
        }
        for (sample_index = 0; sample_index < 5; ++sample_index)
        {
            if (!adc_once(sensor_pins[channel], &value))
            {
                return 0;
            }
            if (value < minimum)
            {
                minimum = value;
            }
            if (value > maximum)
            {
                maximum = value;
            }
            total += value;
        }
        values[channel] = (uint16_t)((total - minimum - maximum) / 3);
    }
    return 1;
}

/** @brief 万分比占空比转定时器比较值；满占空使用 ARR+1 保持恒高。 */
static uint16_t motor_compare(uint16_t duty)
{
    if (duty == 10000)
    {
        return motor_period + 1;
    }
    return (uint16_t)((uint32_t)motor_period * duty / 10000UL);
}

void TrackHW_Set(uint16_t servo, uint16_t left, uint16_t right, uint8_t motor_enable)
{
    uint16_t left_in1;
    uint16_t left_in2;
    uint16_t right_in1;
    uint16_t right_in2;
    uint16_t servo_compare;
    uint16_t swap_value;
    uint8_t previous_irq;
    uint8_t left_reversed;
    uint8_t right_reversed;

    if (servo < TRACK_SERVO_ABS_MIN || servo > TRACK_SERVO_ABS_MAX ||
        left > TRACK_DUTY_MAX || right > TRACK_DUTY_MAX)
    {
        TrackHW_Stop();
        return;
    }
    left_reversed = motor_left_reverse;
    right_reversed = motor_right_reverse;
    if (TRACK_MOTOR_SWAP)
    {
        swap_value = left;
        left = right;
        right = swap_value;
        left_reversed = motor_right_reverse;
        right_reversed = motor_left_reverse;
    }
    /* 电机默认安装方向相反；运行时反向设置跟随逻辑左右轮，交换通道时一起交换。 */
    left_in1 = motor_compare(left_reversed ? 10000 : 10000 - left);
    left_in2 = motor_compare(left_reversed ? 10000 - left : 10000);
    right_in1 = motor_compare(right_reversed ? 10000 - right : 10000);
    right_in2 = motor_compare(right_reversed ? 10000 : 10000 - right);
    servo_compare = (uint16_t)((uint32_t)servo_period * servo / 10000UL);

    /* 先在前台算完，再原子更新寄存器；截止中断锁定后绝不重新开启输出。 */
    previous_irq = EA;
    EA = 0;
    if (track_guard_armed && !track_guard_trip)
    {
        PWMB_CCR1H = (uint8_t)(servo_compare >> 8);
        PWMB_CCR1L = (uint8_t)servo_compare;
        PWMB_ENO |= 1;
        if (motor_enable)
        {
            PWMA_CCR1H = (uint8_t)(left_in1 >> 8);
            PWMA_CCR1L = (uint8_t)left_in1;
            PWMA_CCR2H = (uint8_t)(left_in2 >> 8);
            PWMA_CCR2L = (uint8_t)left_in2;
            PWMA_CCR3H = (uint8_t)(right_in1 >> 8);
            PWMA_CCR3L = (uint8_t)right_in1;
            PWMA_CCR4H = (uint8_t)(right_in2 >> 8);
            PWMA_CCR4L = (uint8_t)right_in2;
            PWMA_ENO |= 0x55;
        }
        else
        {
            PWMA_ENO &= ~0x55;
            P60 = 1;
            P62 = 1;
            P64 = 1;
            P66 = 1;
        }
        track_guard_age = 0;
    }
    EA = previous_irq;
}

void TrackHW_FeedWatchdog(void)
{
    WDT_CONTR |= 0x10;
}

uint8_t TrackUART_Queue(const char *text)
{
    uint16_t head;
    uint16_t tail;
    uint16_t length;
    uint16_t index;
    uint8_t previous_irq;

    length = (uint16_t)strlen(text);
    previous_irq = EA;
    EA = 0;
    head = tx_head;
    tail = tx_tail;
    EA = previous_irq;
    if (length >= TRACK_TX_SIZE || length > ((tail - head - 1) & (TRACK_TX_SIZE - 1)))
    {
        ++track_tx_dropped;
        return 0;
    }
    for (index = 0; index < length; ++index)
    {
        tx_data[head] = (uint8_t)text[index];
        head = (head + 1) & (TRACK_TX_SIZE - 1);
    }
    /* 数据全部写好后才发布 head，防止发送中断读到尚未填写的字节。 */
    previous_irq = EA;
    EA = 0;
    tx_head = head;
    if (!tx_active && tx_head != tx_tail)
    {
        tx_active = 1;
        SBUF = tx_data[tx_tail];
        tx_tail = (tx_tail + 1) & (TRACK_TX_SIZE - 1);
    }
    EA = previous_irq;
    return 1;
}

void TrackUART_TxISR(void)
{
    if (tx_tail != tx_head)
    {
        SBUF = tx_data[tx_tail];
        tx_tail = (tx_tail + 1) & (TRACK_TX_SIZE - 1);
    }
    else
    {
        tx_active = 0;
    }
}
