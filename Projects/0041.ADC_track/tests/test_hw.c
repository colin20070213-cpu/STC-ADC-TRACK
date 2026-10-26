#include "track_hw.h"
#include "track_hw_test_device.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
uint8_t EADC,PADC,PADCH;
uint8_t EA=1,P42=1,P46=1,P45=1,P41=1,P27=1,P60,P62,P64,P66,P74,PT0;
uint8_t PWMA_ENO,PWMB_ENO,PWMA_ARRH,PWMA_ARRL,PWMB_ARRH,PWMB_ARRL;
uint8_t PWMA_CCR1H,PWMA_CCR1L,PWMA_CCR2H,PWMA_CCR2L;
uint8_t PWMA_CCR3H,PWMA_CCR3L,PWMA_CCR4H,PWMA_CCR4L,PWMB_CCR1H,PWMB_CCR1L;
uint8_t ADC_CONTR,ADC_RES,ADC_RESL,WDT_CONTR,SBUF;
static uint32_t configured_baud;
static uint32_t configured_servo_hz;
static uint32_t servo_divisor;
void GPIO_Init(uint8_t p,uint8_t i,uint8_t m){(void)p;(void)i;(void)m;}
void GPIO_PinPullConfig(uint8_t p,uint8_t i,uint8_t m){(void)p;(void)i;(void)m;}
void PWM_Init(uint8_t pin,uint32_t freq,uint32_t duty)
{
    uint32_t divider=(FOSC/freq)>>16;
    uint16_t period=(uint16_t)((FOSC/freq)/(divider+1)-1);(void)duty;
    if(pin==PWMB_CH1_P74){
        configured_servo_hz=freq;servo_divisor=divider+1;
        PWMB_ARRH=period>>8;PWMB_ARRL=(uint8_t)period;
    }
    else {PWMA_ARRH=period>>8;PWMA_ARRL=(uint8_t)period;}
}
void Encoder_Init(void) {}
uint8_t Encoder_Snapshot(uint16_t *count,uint8_t *direction) { *count=123;*direction=1;return 1; }
void ADC_Init(uint8_t pin,uint8_t speed){(void)pin;(void)speed;}
void UART_Init(uint8_t u,uint8_t rx,uint8_t tx,uint32_t b,uint8_t t)
{
    assert(u==UART_1 && rx==UART1_RX_P43 && tx==UART1_TX_P44 && t==TIM1);
    configured_baud=b;
}
void PIT_Timer_Ms(uint8_t t,uint16_t p){(void)t;(void)p;}

/* 模拟每次 ADC 完成事件；包含首读异常值和有效样本的高低离群值。 */
static unsigned finish_scan(void)
{
    const uint16_t readings[6]={4095,1000,10,1002,3000,1004};
    unsigned n=0;
    while(ADC_CONTR&0x40) {
        uint16_t value=(uint16_t)(readings[n%6]+(n/6)*10);
        if(n%6==0)value=4095;
        ADC_RES=(uint8_t)(value>>8);ADC_RESL=(uint8_t)value;
        ADC_CONTR=(ADC_CONTR&~0x40)|0x20;
        TrackHW_AdcISR();
        assert(++n<=6U*TRACK_SENSOR_COUNT);
    }
    return n;
}
static void tick(unsigned ms){while(ms--){TrackHW_TickISR();finish_scan();}}
static uint16_t word(uint8_t h,uint8_t l){return ((uint16_t)h<<8)|l;}
/** @brief 按时钟和预分频独立换算实际高电平时长，检查全部允许舵机值。 */
static void servo_pulse_tests(void)
{
    uint16_t command;
    uint64_t measured_ns, expected_ns;

    assert(configured_servo_hz==TRACK_SERVO_FREQUENCY);
    TrackHW_Arm(1);
    for(command=TRACK_SERVO_ABS_MIN;command<=TRACK_SERVO_ABS_MAX;command++)
    {
        TrackHW_Set(command,0,0,0);
        assert(PWMB_ENO&1);
        measured_ns=(uint64_t)word(PWMB_CCR1H,PWMB_CCR1L)*servo_divisor*1000000000ULL/FOSC;
        expected_ns=(uint64_t)command*2000ULL;
        assert(measured_ns<=expected_ns+2000ULL && measured_ns+2000ULL>=expected_ns);
    }
    TrackHW_Arm(0);
    printf("PASS: servo %luHz, all 600..900 commands retain 2us/unit pulse width within 2us quantization\n",
           (unsigned long)configured_servo_hz);
}

int main(void)
{
    uint16_t values[4];unsigned n;char chunk[101];
    TrackHW_Init();assert(!(PWMA_ENO&0x55) && !(PWMB_ENO&1) && P60 && P62 && P64 && P66 && !P74);
    assert(configured_baud==9600UL && EADC && !PADC && !PADCH);
    assert(!TrackHW_Sample(values)); /* 未完成首帧时立即返回，不等待。 */
    for(n=0;n<100;n++) {
        unsigned k;
        tick(1);
        assert(TrackHW_Sample(values));
        for(k=0;k<TRACK_SENSOR_COUNT;k++)assert(values[k]==1002+k*10);
    }
    assert(track_adc_frames==100 && track_adc_overruns==0);
    /* 在第二帧尚未完成时，读取必须仍返回上一整帧，不混合两帧。 */
    TrackHW_TickISR();
    assert(TrackHW_Sample(values) && values[0]==1002);
    assert(finish_scan()==6U*TRACK_SENSOR_COUNT && track_adc_frames==101);
    servo_pulse_tests();
    TrackHW_Set(750,1000,1000,1);assert(!(PWMA_ENO&0x55));
    TrackHW_Arm(1);TrackHW_Set(750,1000,1000,0);
    assert(!(PWMA_ENO&0x55) && (PWMB_ENO&1));
    assert(word(PWMB_CCR1H,PWMB_CCR1L)==(uint32_t)word(PWMB_ARRH,PWMB_ARRL)*
           (750UL*TRACK_SERVO_UNIT_US*TRACK_SERVO_FREQUENCY/100UL)/10000UL);
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
    TrackHW_Arm(1);TrackHW_Set(750,1000,900,1);
    assert(!(PWMA_ENO&0x55) && !(PWMB_ENO&1)); /* 驱动层拒绝差速指令。 */
    TrackHW_Arm(0);
    P27=0;tick(19);assert(!TrackHW_Keys());tick(1);assert(TrackHW_Keys()==KEY_OK_EVENT);
    tick(1000);assert(!TrackHW_Keys());P27=1;tick(20);
    P42=0;tick(20);assert(TrackHW_Keys()==KEY_UP_EVENT);tick(700);assert(TrackHW_Keys()==KEY_UP_EVENT);
    P42=1;tick(20);assert(!TrackHW_Keys());
    /* ADC 卡住时，3ms 内拒绝旧帧并关闭输出，读取函数始终不阻塞。 */
    TrackHW_Arm(1);TrackHW_Set(750,1000,1000,1);
    TrackHW_TickISR();
    for(n=0;n<TRACK_ADC_STALE_MS;n++)TrackHW_TickISR();
    assert(!TrackHW_Sample(values) && !(PWMA_ENO&0x55) && !(PWMB_ENO&1));
    assert(track_adc_overruns>=TRACK_ADC_STALE_MS);
    finish_scan();tick(1);assert(TrackHW_Sample(values));TrackHW_Arm(0);
    assert(TrackUART_Queue("abc"));assert(SBUF=='a');TrackUART_TxISR();assert(SBUF=='b');
    TrackUART_TxISR();assert(SBUF=='c');TrackUART_TxISR();
    memset(chunk,'q',100);chunk[100]=0;
    for(n=0;n<5;n++)assert(TrackUART_Queue(chunk));
    assert(!TrackUART_Queue(chunk) && track_tx_dropped==1);
    for(n=0;n<501;n++)TrackUART_TxISR();assert(TrackUART_Queue("end"));
    puts("PASS: actual hardware layer, safe PWM, 20 ms cutoff, debounce, 1ms full-frame ADC/trim/atomic publish/stale cutoff and UART backpressure");
    return 0;
}
