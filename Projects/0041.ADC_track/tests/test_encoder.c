/** @file test_encoder.c @brief 直接链接原库的稳定读取、回绕、方向兼容及清零。 */
#include "Encoder.h"
#include <assert.h>
#include <stdio.h>
uint8_t P04,P52,encoder_low;
static uint8_t high_value,read_index,read_mode;
extern Encoder_TypeDef Encoder_X;
uint8_t EncoderTest_ReadHigh(void)
{
    static const uint8_t torn[]={1,2,2,2};
    if(read_mode==1)return torn[read_index++];
    if(read_mode==2)return read_index++;
    return high_value;
}
void GPIO_Init(uint8_t port,uint8_t pin,uint8_t mode)
{
    assert((port==0 && pin==16)||(port==5 && pin==4));assert(mode==0);
}
void Timer_Count_Init(uint8_t pin){assert(pin==3);encoder_low=high_value=0;}
void Timer_Count_Clear(uint8_t pin){assert(pin==3);encoder_low=high_value=0;}
int main(void)
{
    uint16_t count=0xabcd;uint8_t direction=0;
    Encoder_Init();assert(P04 && P52);
    high_value=0xff;encoder_low=0xfa;
    assert(Encoder_Snapshot(&count,&direction) && count==65530 && direction==1);
    Encoder_X.pouse_last=count;Encoder_X.pouse_t=0;
    high_value=0;encoder_low=3;assert(Encoder_Count_Read()==9);
    P52=0;encoder_low=8;assert(Encoder_Count_Read()==4);
    read_mode=1;read_index=0;encoder_low=4;
    assert(Encoder_Snapshot(&count,&direction) && count==0x0204 && read_index==4);
    count=0xabcd;read_mode=2;read_index=0;
    assert(!Encoder_Snapshot(&count,&direction) && count==0xabcd && read_index==6);
    read_mode=0;Encoder_Clear();assert(!Encoder_X.pouse_t && !Encoder_X.pouse_last);
    puts("PASS: actual Encoder library P04/P52, coherent bounded snapshots, 65535 wrap, direction compatibility and clear");
    return 0;
}
