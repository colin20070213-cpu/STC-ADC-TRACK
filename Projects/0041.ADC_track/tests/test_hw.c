#include "track_hw.h"
#include "track_hw_test_device.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
uint8_t EA=1,P42=1,P46=1,P45=1,P41=1,P27=1,P60,P62,P64,P66,P74,PT0;
uint8_t PWMA_ENO,PWMB_ENO,PWMA_ARRH,PWMA_ARRL,PWMB_ARRH,PWMB_ARRL;
uint8_t PWMA_CCR1H,PWMA_CCR1L,PWMA_CCR2H,PWMA_CCR2L;
uint8_t PWMA_CCR3H,PWMA_CCR3L,PWMA_CCR4H,PWMA_CCR4L,PWMB_CCR1H,PWMB_CCR1L;
uint8_t ADC_CONTR,ADC_RES,ADC_RESL,WDT_CONTR,SBUF;
static uint32_t configured_baud;
void GPIO_Init(uint8_t p,uint8_t i,uint8_t m){(void)p;(void)i;(void)m;}
void GPIO_PinPullConfig(uint8_t p,uint8_t i,uint8_t m){(void)p;(void)i;(void)m;}
void PWM_Init(uint8_t pin,uint32_t freq,uint32_t duty)
{
    uint16_t period=freq==50?60321:3316;(void)duty;
    if(pin==PWMB_CH1_P74){PWMB_ARRH=period>>8;PWMB_ARRL=(uint8_t)period;}
    else {PWMA_ARRH=period>>8;PWMA_ARRL=(uint8_t)period;}
}
void ADC_Init(uint8_t pin,uint8_t speed){(void)pin;(void)speed;}
void UART_Init(uint8_t u,uint8_t rx,uint8_t tx,uint32_t b,uint8_t t)
{
    assert(u==UART_1 && rx==UART1_RX_P43 && tx==UART1_TX_P44 && t==TIM1);
    configured_baud=b;
}
void PIT_Timer_Ms(uint8_t t,uint16_t p){(void)t;(void)p;}

static void tick(unsigned ms){while(ms--)TrackHW_TickISR();}
static uint16_t word(uint8_t h,uint8_t l){return ((uint16_t)h<<8)|l;}
int main(void)
{
    uint16_t values[4];unsigned n;char chunk[101];
    TrackHW_Init();assert(!(PWMA_ENO&0x55) && !(PWMB_ENO&1) && P60 && P62 && P64 && P66 && !P74);
    assert(configured_baud==9600UL);
    TrackHW_Set(750,1000,1000,1);assert(!(PWMA_ENO&0x55));
    TrackHW_Arm(1);TrackHW_Set(750,1000,1000,0);
    assert(!(PWMA_ENO&0x55) && (PWMB_ENO&1));
    TrackHW_Set(750,1000,1000,1);assert((PWMA_ENO&0x55)==0x55);
    assert(word(PWMA_CCR2H,PWMA_CCR2L)==3317 && word(PWMA_CCR3H,PWMA_CCR3L)==3317);
    assert(word(PWMA_CCR1H,PWMA_CCR1L)<3317 && word(PWMA_CCR4H,PWMA_CCR4L)<3317);
    assert(!TrackHW_SetMotorDirections(1,1)); /* 运行使能期间拒绝修改方向。 */
    tick(TRACK_CONTROL_TIMEOUT-1);assert(!track_guard_trip);
    tick(1);assert(track_guard_trip && !(PWMA_ENO&0x55) && !(PWMB_ENO&1) && !P74);
    TrackHW_Set(750,1000,1000,1);assert(!(PWMA_ENO&0x55) && !(PWMB_ENO&1));
    TrackHW_Arm(0);tick(25);
    assert(!TrackHW_SetMotorDirections(2,0));
    assert(TrackHW_SetMotorDirections(1,0));
    TrackHW_Arm(1);TrackHW_Set(750,1000,1000,1);
    assert(word(PWMA_CCR1H,PWMA_CCR1L)==3317 && word(PWMA_CCR2H,PWMA_CCR2L)<3317);
    assert(word(PWMA_CCR3H,PWMA_CCR3L)==3317 && word(PWMA_CCR4H,PWMA_CCR4L)<3317);
    TrackHW_Arm(0);assert(TrackHW_SetMotorDirections(0,1));
    TrackHW_Arm(1);TrackHW_Set(750,1000,1000,1);
    assert(word(PWMA_CCR1H,PWMA_CCR1L)<3317 && word(PWMA_CCR2H,PWMA_CCR2L)==3317);
    assert(word(PWMA_CCR3H,PWMA_CCR3L)<3317 && word(PWMA_CCR4H,PWMA_CCR4L)==3317);
    TrackHW_Arm(0);assert(TrackHW_SetMotorDirections(1,1));
    TrackHW_Arm(1);TrackHW_Set(750,1000,1000,1);
    assert(word(PWMA_CCR1H,PWMA_CCR1L)==3317 && word(PWMA_CCR4H,PWMA_CCR4L)==3317);
    TrackHW_Arm(0);assert(TrackHW_SetMotorDirections(0,0));
    TrackHW_Arm(1);TrackHW_Set(750,4000,4000,1);
    assert((PWMA_ENO&0x55)==0x55 && word(PWMA_CCR1H,PWMA_CCR1L)==1989);
    assert(word(PWMA_CCR4H,PWMA_CCR4L)==1989);
    TrackHW_Set(750,4001,4000,1);assert(!(PWMA_ENO&0x55) && !(PWMB_ENO&1));
    TrackHW_Arm(0);
    P27=0;tick(19);assert(!TrackHW_Keys());tick(1);assert(TrackHW_Keys()==KEY_OK_EVENT);
    tick(1000);assert(!TrackHW_Keys());P27=1;tick(20);
    P42=0;tick(20);assert(TrackHW_Keys()==KEY_UP_EVENT);tick(700);assert(TrackHW_Keys()==KEY_UP_EVENT);
    P42=1;tick(20);assert(!TrackHW_Keys());
    assert(!TrackHW_Sample(values)); /* Stuck ADC cannot hang foreground. */
    assert(TrackUART_Queue("abc"));assert(SBUF=='a');TrackUART_TxISR();assert(SBUF=='b');
    TrackUART_TxISR();assert(SBUF=='c');TrackUART_TxISR();
    memset(chunk,'q',100);chunk[100]=0;
    for(n=0;n<5;n++)assert(TrackUART_Queue(chunk));
    assert(!TrackUART_Queue(chunk) && track_tx_dropped==1);
    for(n=0;n<501;n++)TrackUART_TxISR();assert(TrackUART_Queue("end"));
    puts("PASS: actual hardware layer, safe PWM, 20 ms cutoff, debounce, bounded ADC and UART backpressure");
    return 0;
}
