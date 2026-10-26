#ifndef TRACK_HW_TEST_DEVICE_H
#define TRACK_HW_TEST_DEVICE_H
void Encoder_Init(void);
uint8_t Encoder_Snapshot(uint16_t *count,uint8_t *direction);
#define FOSC 33177600UL
extern uint8_t EADC,PADC,PADCH;
extern uint8_t EA,P42,P46,P45,P41,P27,P60,P62,P64,P66,P74,PT0;
extern uint8_t PWMA_ENO,PWMB_ENO,PWMA_ARRH,PWMA_ARRL,PWMB_ARRH,PWMB_ARRL;
extern uint8_t PWMA_CCR1H,PWMA_CCR1L,PWMA_CCR2H,PWMA_CCR2L;
extern uint8_t PWMA_CCR3H,PWMA_CCR3L,PWMA_CCR4H,PWMA_CCR4L,PWMB_CCR1H,PWMB_CCR1L;
extern uint8_t ADC_CONTR,ADC_RES,ADC_RESL,WDT_CONTR,SBUF;
typedef uint8_t ADC_PIN_ENUM;
enum {ADC_P10=0,ADC_P13=3,ADC_P01=9,ADC_P16=6,ADC_P05=13,ADC_SPEED_2X16T=15};
enum {GPIO_P6=6,GPIO_P7=7,GPIO_P4=4,GPIO_P2=2,GPIO_Pin_0=1,GPIO_Pin_1=2,GPIO_Pin_2=4,
      GPIO_Pin_4=16,GPIO_Pin_5=32,GPIO_Pin_6=64,GPIO_Pin_7=128,GPIO_OUT_PP=3,GPIO_PullUp=0,GPIO_Pull_Up=1};
enum {PWMA_CH1P_P60=4,PWMA_CH2P_P62=20,PWMA_CH3P_P64=36,PWMA_CH4P_P66=52,PWMB_CH1_P74=67};
enum {UART_1=0,UART1_RX_P43=6,UART1_TX_P44=7,TIM0=0,TIM1=1};
void GPIO_Init(uint8_t port,uint8_t pin,uint8_t mode);
void GPIO_PinPullConfig(uint8_t port,uint8_t pin,uint8_t mode);
void PWM_Init(uint8_t pin,uint32_t freq,uint32_t duty);
void ADC_Init(uint8_t pin,uint8_t speed);
void UART_Init(uint8_t uart,uint8_t rx,uint8_t tx,uint32_t baud,uint8_t timer);
void PIT_Timer_Ms(uint8_t timer,uint16_t period);
#endif
