#ifndef ENCODER_TEST_COMMON_H
#define ENCODER_TEST_COMMON_H
#include <stdint.h>
extern uint8_t P04,P52,encoder_low;
uint8_t EncoderTest_ReadHigh(void);
#define T3H EncoderTest_ReadHigh()
#define T3L encoder_low
#endif
