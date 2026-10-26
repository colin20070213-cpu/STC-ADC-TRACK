/** @file Encoder.c
 *  @brief 扩展用户原编码器封装：稳定快照、回绕差分，保留原三个接口。
 *  @note 按原库的 P04 计数/P52 方向接口使用，不改变为 AB 四倍频解码。
 */
#include "Encoder.h"
#include "CNU_PIE_TIMER.h"
#include "CNU_PIE_GPIO.h"

#define Encoder_Dir P52
#define Encoder_Tim P04
Encoder_TypeDef Encoder_X;

void Encoder_Init(void)
{
    Encoder_Tim = 1;
    Encoder_Dir = 1;
    GPIO_Init(GPIO_P0, GPIO_Pin_4, GPIO_PullUp);
    GPIO_Init(GPIO_P5, GPIO_Pin_2, GPIO_PullUp);
    Timer_Count_Init(TIMER3_P04);
    Encoder_X.pouse = 0;
    Encoder_X.pouse_last = 0;
    Encoder_X.pouse_t = 0;
    Encoder_X.pouse_read = 0;
    Encoder_X.Dir = Encoder_Dir;
}

uint8_t Encoder_Snapshot(uint16_t *count, uint8_t *direction)
{
    uint8_t high_before;
    uint8_t high_after;
    uint8_t low;
    uint8_t attempt;

    /* 外部计数器不受关中断影响；高-低-高一致才发布，最多尝试三次。 */
    for (attempt = 0; attempt < 3; ++attempt)
    {
        high_before = T3H;
        low = T3L;
        high_after = T3H;
        if (high_before == high_after)
        {
            *count = ((uint16_t)high_after << 8) | low;
            *direction = Encoder_Dir ? 1 : 0;
            return 1;
        }
    }
    return 0;
}

int Encoder_Count_Read(void)
{
    uint16_t count;
    uint16_t delta;
    uint8_t direction;

    if (!Encoder_Snapshot(&count, &direction)) return Encoder_X.pouse_t;
    delta = (uint16_t)(count - Encoder_X.pouse_last);
    Encoder_X.pouse = count;
    Encoder_X.pouse_last = count;
    Encoder_X.Dir = direction;
    /* 显式无符号模运算，避免旧有符号累计运算溢出。 */
    if (direction)
        Encoder_X.pouse_t = (int)(uint16_t)((uint16_t)Encoder_X.pouse_t + delta);
    else
        Encoder_X.pouse_t = (int)(uint16_t)((uint16_t)Encoder_X.pouse_t - delta);
    return Encoder_X.pouse_t;
}

void Encoder_Clear(void)
{
    Timer_Count_Clear(TIMER3_P04);
    Encoder_X.pouse = 0;
    Encoder_X.pouse_last = 0;
    Encoder_X.pouse_t = 0;
    Encoder_X.pouse_read = 0;
}
