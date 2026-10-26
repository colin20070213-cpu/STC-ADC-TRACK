/** @file Encoder.h
 *  @brief 原单路编码器库：P04/TIM3 脉冲，P52 方向。保留原有接口和字段。
 */
#ifndef _ENCODER_H
#define _ENCODER_H
#include "common.h"

typedef struct
{
    uint8_t TIMN_enum;       /**< 原计数器配置字段。 */
    uint8_t GPIN_enum;       /**< 原方向配置字段。 */
    uint8_t Dir;             /**< 最新 P52 方向电平。 */
    uint16_t pouse;          /**< 最新原始计数。 */
    uint16_t pouse_last;     /**< 上次原始计数。 */
    int pouse_t;             /**< 原累计接口，按 16 位回绕保持兼容。 */
    int pouse_read;          /**< 保留原字段。 */
} Encoder_TypeDef;

void Encoder_Init(void);
int Encoder_Count_Read(void);
void Encoder_Clear(void);
/** @brief 有界地读取 TIM3 稳定快照；成功返回 1，不停表、不清计数。
 *  @param count 返回 16 位原始脉冲计数，调用者以 uint16_t 差分处理回绕。
 *  @param direction 返回 P52 电平，沿用原库脉冲/方向接口定义。
 */
uint8_t Encoder_Snapshot(uint16_t *count, uint8_t *direction);
#endif
