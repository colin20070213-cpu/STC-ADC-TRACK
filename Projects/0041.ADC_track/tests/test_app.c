#include "track_core.h"
#include "track_hw.h"
#include "track_app_test_hw.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

unsigned char EA=1,P42=1,P46=1,P45=1,P41=1,P27=1,WDT_CONTR;
volatile uint32_t track_milliseconds;
volatile uint8_t track_guard_armed,track_guard_age,track_guard_trip,track_key_events;
volatile uint16_t track_tx_dropped;
static uint16_t hw_left,hw_right,hw_servo;
static uint8_t hw_left_reverse,hw_right_reverse;
static TrackParams flash_params;
static uint8_t store_fails, sample_ok=1, flash_valid;
static unsigned store_calls;
static unsigned uart_calls;
static uint16_t fake_raw[4]={500,500,500,500};
static char uart_line[512];
void TrackHW_Stop(void) { hw_left=hw_right=hw_servo=0; }
void TrackHW_Arm(uint8_t enable) { track_guard_armed=enable;track_guard_age=0;track_guard_trip=0;TrackHW_Stop(); }
uint8_t TrackHW_SetMotorDirections(uint8_t left,uint8_t right) { assert(!track_guard_armed);hw_left_reverse=left;hw_right_reverse=right;return 1; }
uint32_t TrackHW_Now(void) { return track_milliseconds; }
uint8_t TrackHW_Keys(void) { uint8_t k=track_key_events;track_key_events=0;return k; }
uint8_t TrackHW_Sample(uint16_t *v) { memcpy(v,fake_raw,sizeof(fake_raw));return sample_ok; }
void TrackHW_Set(uint16_t s,uint16_t l,uint16_t r,uint8_t motor) { hw_servo=s;hw_left=motor?l:0;hw_right=motor?r:0; }
void TrackHW_FeedWatchdog(void) {}
uint8_t TrackStore_Load(TrackParams *p) { if(flash_valid){*p=flash_params;return 1;}TrackParams_Default(p);flash_params=*p;return 0; }
uint8_t TrackStore_Save(const TrackParams *p) { ++store_calls;if(store_fails)return 0;flash_params=*p;flash_valid=1;return 1; }
TrackStoreError TrackStore_LastError(void) { return store_fails ? TRACK_STORE_ERASE : TRACK_STORE_OK; }
uint8_t TrackUART_Queue(const char *s) { assert(strlen(s)<sizeof(uart_line));strcpy(uart_line,s);++uart_calls;return 1; }
void Ms_Delay(uint16_t ms) { track_milliseconds+=ms; }
void LCD_Set_Pos(uint8_t x,uint8_t y) {(void)x;(void)y;}
void LCD_WrDat(uint8_t v) {(void)v;}
/* Include the real application: test its menu transitions, not a UI model. */
#include "../USER/src/track_app.c"

static void step(void) { track_milliseconds+=5;control_task(); }
/* 经由真实父/子菜单进入叶页面，避免直接指定业务模式绕过菜单分发。 */
static void enter(TrackMode target)
{
    uint8_t root_item, child_item=0, has_child=1;
    unsigned attempts;
    for(attempts=0;mode!=TRACK_MENU && attempts<4;attempts++)handle_keys(KEY_LEFT_EVENT);
    assert(mode==TRACK_MENU);
    if(menu_page!=MENU_ROOT)handle_keys(KEY_LEFT_EVENT);
    switch(target) {
        case TRACK_CAL: root_item=ROOT_CALIBRATION;child_item=0;break;
        case TRACK_BODY_CENTER: root_item=ROOT_CALIBRATION;child_item=1;break;
        case TRACK_PID: root_item=ROOT_PID;has_child=0;break;
        case TRACK_RUN: root_item=ROOT_RUN;has_child=0;break;
        case TRACK_CENTER: root_item=ROOT_SERVO;child_item=0;break;
        case TRACK_LIMIT_LEFT: root_item=ROOT_SERVO;child_item=1;break;
        case TRACK_LIMIT_RIGHT: root_item=ROOT_SERVO;child_item=2;break;
        case TRACK_SERVO_DIRECTION: root_item=ROOT_SERVO;child_item=3;break;
        case TRACK_SPEED: root_item=ROOT_MOTOR;child_item=0;break;
        case TRACK_LEFT_DIRECTION: root_item=ROOT_MOTOR;child_item=1;break;
        case TRACK_RIGHT_DIRECTION: root_item=ROOT_MOTOR;child_item=2;break;
        default:assert(0);return;
    }
    while(menu_item!=root_item)handle_keys(KEY_DOWN_EVENT);
    handle_keys(KEY_OK_EVENT);
    if(has_child) {
        assert(mode==TRACK_MENU && menu_page!=MENU_ROOT);
        while(menu_item!=child_item)handle_keys(KEY_DOWN_EVENT);
        handle_keys(KEY_OK_EVENT);
    }
}
static void dump_screen(const char *path)
{
    FILE *f;render_display();f=fopen(path,"wb");assert(f);
    assert(fwrite(framebuffer,1,1024,f)==1024);fclose(f);
}

/** @brief 解析真实 CSV，核对字段顺序、反馈符号与低波特率发送预算。 */
static void telemetry_tests(void)
{
    char copy[512];
    char *token;
    long fields[39];
    unsigned count, before;
    uint32_t now = 10000;

    mode=TRACK_PID;
    uart_snapshot_pending=0;
    last_telemetry=now;
    before=uart_calls;
    telemetry_task(now+TRACK_TELEMETRY_MS-1);
    assert(uart_calls==before);
    sensors.valid=1;sensors.center_valid=1;sensors.center_bias=150;
    sensors.norm[0]=600;sensors.norm[1]=400;
    control.error=50;
    telemetry_task(now+=TRACK_TELEMETRY_MS);
    assert(uart_calls==before+1 && TRACK_TELEMETRY_MS==250);
    strcpy(copy,uart_line);count=0;
    for(token=strtok(copy,",");token;token=strtok(NULL,","))
    {
        assert(count<39);fields[count++]=strtol(token,NULL,10);
    }
    assert(count==39 && fields[10]==50 && fields[37]==0 && fields[38]==50);
    sensors.norm[0]=400;sensors.norm[1]=600;
    control.error=-350;
    telemetry_task(now+=TRACK_TELEMETRY_MS);
    assert(strstr(uart_line,",0,-350\r\n"));

    /* 十字策略可以把控制误差置零，反馈仍必须保留本帧电感测量。 */
    control.error=0;control.crossing=1;
    sensors.norm[0]=1000;sensors.norm[1]=900;
    telemetry_task(now+=TRACK_TELEMETRY_MS);
    assert(strstr(uart_line,",0,-98\r\n"));

    /* 以各字段允许的最大打印宽度检查 8N1 的持续数据量，保留至少 20% 余量。 */
    params.kp100=TRACK_PID_KP_MAX_X100;params.ki100=params.kd100=2000;
    params.center=params.left=params.right=900;
    params.base_duty=params.curve_duty=4000;params.servo_left_sign=-1;
    params.left_reverse=params.right_reverse=1;
    for(count=0;count<4;++count){sensors.raw[count]=4095;sensors.norm[count]=1000;}
    sensors.norm[0]=0;sensors.center_bias=300;
    control.p_x10=control.i_x10=control.d_x10=control.unclamped_x10=-32768;
    control.servo=900;control.error=-1000;
    control.left_motor=control.right_motor=4000;
    control.signal_ok=control.saturated=1;track_tx_dropped=65535;fault=TRACK_NO_CENTER;
    telemetry_task(0xffffffffUL);
    assert(strlen(uart_line)*10000UL <= TRACK_UART_BAUD*TRACK_TELEMETRY_MS*4UL/5UL);
    printf("PASS: 39-column telemetry, explicit target/feedback and 9600-baud budget (%u bytes/frame)\n",(unsigned)strlen(uart_line));

    mode=TRACK_MENU;uart_snapshot_pending=0;before=uart_calls;
    telemetry_task(0xffffffffUL);
    assert(uart_calls==before);
}

/** @brief 验证居中页实际字模包含中间误差值和原始 ADC，防止只测计算不测显示。 */
static void center_display_test(void)
{
    const char *lines[2]={"E:  333 REF:    0", "ADC L:  40 R:  20"};
    unsigned line, character, column;

    mode=TRACK_BODY_CENTER;sensors.valid=1;sensors.center_bias=0;
    sensors.norm[0]=500;sensors.norm[1]=250;sensors.raw[0]=40;sensors.raw[1]=20;
    render_display();
    for(line=0;line<2;line++)
    {
        for(character=0;lines[line][character];character++)
        {
            for(column=0;column<6;column++)
            {
                assert(framebuffer[(3+line*2)*128+character*6+column]==
                       F6x8[(unsigned char)lines[line][character]-32][column]);
            }
        }
    }
    dump_screen("MDK/ui-body-low-signal.bin");
}
int main(void)
{
    unsigned n, old_store_calls;uint16_t old;TrackParams before_restore, before_direction;
    TrackApp_Init();assert(mode==TRACK_MENU && !sensors.valid);
    assert(sensor_math_ok);
    assert(strstr(uart_line,",target1000,feedback1000\r\n"));
    dump_screen("MDK/ui-menu-1.bin");menu_item=ROOT_RUN;dump_screen("MDK/ui-menu-2.bin");
    handle_keys(KEY_DOWN_EVENT);assert(menu_item==0);
    handle_keys(KEY_UP_EVENT);assert(menu_item==ROOT_RUN);
    menu_item=ROOT_SERVO;handle_keys(KEY_OK_EVENT);assert(menu_page==MENU_SERVO && menu_count()==4);
    dump_screen("MDK/ui-servo-menu-1.bin");
    handle_keys(KEY_UP_EVENT);assert(menu_item==3);dump_screen("MDK/ui-servo-menu-2.bin");
    handle_keys(KEY_LEFT_EVENT|KEY_DOWN_EVENT);assert(menu_page==MENU_ROOT && menu_item==ROOT_SERVO);
    menu_item=ROOT_MOTOR;handle_keys(KEY_OK_EVENT);assert(menu_page==MENU_MOTOR && menu_count()==3);
    dump_screen("MDK/ui-menu-3.bin");handle_keys(KEY_LEFT_EVENT);
    menu_item=ROOT_CALIBRATION;handle_keys(KEY_OK_EVENT);assert(menu_count()==2);
    dump_screen("MDK/ui-cal-menu.bin");handle_keys(KEY_LEFT_EVENT);
    menu_item=ROOT_SERVO;handle_keys(KEY_OK_EVENT);assert(menu_item==3);handle_keys(KEY_LEFT_EVENT);
    assert(params.center==760 && params.left==840 && params.right==670);
    enter(TRACK_BODY_CENTER);assert(mode==TRACK_BODY_CENTER && !hw_servo && !track_guard_armed);
    handle_keys(KEY_RIGHT_EVENT);step();
    assert(sensors.center_status==TRACK_BODY_IDLE && !sensors.center_valid);
    dump_screen("MDK/ui-body-need-cal.bin");
    handle_keys(KEY_OK_EVENT);assert(mode==TRACK_CAL && sensors.samples==0 && !hw_left && !hw_servo);
    handle_keys(KEY_LEFT_EVENT);assert(mode==TRACK_MENU && menu_page==MENU_CALIBRATION && menu_item==0);
    sensor_math_ok=0;
    enter(TRACK_PID);assert(mode==TRACK_MENU && fault==TRACK_MATH_ERROR && !hw_left && !hw_servo);
    sensor_math_ok=1;
    enter(TRACK_RUN);assert(mode==TRACK_MENU && fault==TRACK_NO_CAL && !hw_left && !hw_servo);
    enter(TRACK_CAL);assert(mode==TRACK_CAL);
    for(n=0;n<120;n++) {fake_raw[0]=fake_raw[1]=(uint16_t)(200+n*10);step();}
    dump_screen("MDK/ui-cal.bin");
    handle_keys(KEY_OK_EVENT);assert(sensors.valid && mode==TRACK_MENU);
    enter(TRACK_RUN);assert(mode==TRACK_MENU && fault==TRACK_NO_CENTER && !hw_left && !hw_servo);
    enter(TRACK_BODY_CENTER);assert(mode==TRACK_BODY_CENTER && !track_guard_armed);
    handle_keys(KEY_OK_EVENT);
    fake_raw[0]=850;fake_raw[1]=750;
    for(n=0;n<100;n++){step();assert(!hw_left && !hw_right && !hw_servo);}
    dump_screen("MDK/ui-body-capture.bin");
    for(n=100;n<TRACK_BODY_SAMPLES;n++)step();
    assert(sensors.center_status==TRACK_BODY_READY && !sensors.center_valid);
    dump_screen("MDK/ui-body-ready.bin");
    handle_keys(KEY_OK_EVENT);assert(sensors.center_valid && sensors.center_bias>0 && !hw_servo);
    old=(uint16_t)sensors.center_bias;
    handle_keys(KEY_RIGHT_EVENT);step();handle_keys(KEY_LEFT_EVENT);
    assert(mode==TRACK_MENU && sensors.center_bias==(int16_t)old && sensors.center_valid);
    /* Failed center measurement cannot replace a previously committed reference. */
    enter(TRACK_BODY_CENTER);handle_keys(KEY_OK_EVENT);sample_ok=0;step();handle_keys(KEY_OK_EVENT);
    assert(sensors.center_valid && sensors.center_bias==(int16_t)old);
    sample_ok=1;handle_keys(KEY_LEFT_EVENT);
    enter(TRACK_PID);step();assert(mode==TRACK_PID && !hw_left && !hw_right && hw_servo);
    assert(control.error==0 && control.servo==params.center);
    old=params.kp100;handle_keys(KEY_RIGHT_EVENT);handle_keys(KEY_UP_EVENT);step();
    dump_screen("MDK/ui-pid.bin");
    assert(params.kp100==old+1 && !hw_left);
    handle_keys(KEY_LEFT_EVENT);assert(params.kp100==old && mode==TRACK_PID);
    handle_keys(KEY_RIGHT_EVENT);handle_keys(KEY_UP_EVENT);handle_keys(KEY_OK_EVENT);
    assert(flash_params.kp100==old+1 && !editing && !hw_left);
    handle_keys(KEY_LEFT_EVENT);assert(mode==TRACK_MENU && !hw_servo);
    for(n=3;n<=5;n++) {
        enter((TrackMode)(n+1));step();assert(!hw_left && !hw_right && hw_servo);
        old=n==3?params.center:n==4?params.left:params.right;
        handle_keys(KEY_UP_EVENT);step();assert(!hw_left && !hw_right);
        handle_keys(KEY_LEFT_EVENT);assert(mode==TRACK_MENU && !hw_servo);
        assert((n==3?params.center:n==4?params.left:params.right)==old);
    }
    enter(TRACK_CENTER);handle_keys(KEY_UP_EVENT);handle_keys(KEY_OK_EVENT);
    dump_screen("MDK/ui-servo.bin");
    assert(flash_params.center==TRACK_SERVO_DEFAULT_CENTER+1);handle_keys(KEY_LEFT_EVENT);assert(params.center==TRACK_SERVO_DEFAULT_CENTER+1);
    enter(TRACK_LIMIT_LEFT);old=params.left;handle_keys(KEY_UP_EVENT);store_fails=1;handle_keys(KEY_OK_EVENT);
    assert(!strcmp(message,"FLASH ERASE FAILED") && flash_params.left==old);handle_keys(KEY_LEFT_EVENT);assert(params.left==old);store_fails=0;
    /* 速度页不依赖电感；编辑、保存与退出均不可使能执行器。 */
    enter(TRACK_SPEED);step();assert(mode==TRACK_SPEED && !track_guard_armed && !hw_servo && !hw_left);
    old=params.base_duty;handle_keys(KEY_RIGHT_EVENT);handle_keys(KEY_UP_EVENT);
    assert(params.base_duty==old+TRACK_SPEED_FINE_STEP);
    handle_keys(KEY_LEFT_EVENT);assert(params.base_duty==old && !editing && mode==TRACK_SPEED);
    handle_keys(KEY_RIGHT_EVENT);handle_keys(KEY_RIGHT_EVENT);handle_keys(KEY_UP_EVENT);
    assert(params.base_duty==old+TRACK_SPEED_COARSE_STEP);
    store_fails=1;handle_keys(KEY_OK_EVENT);assert(editing && flash_params.base_duty==old);
    store_fails=0;handle_keys(KEY_OK_EVENT);assert(!editing && flash_params.base_duty==params.base_duty);
    handle_keys(KEY_DOWN_EVENT);assert(speed_item==1);handle_keys(KEY_RIGHT_EVENT);
    handle_keys(KEY_DOWN_EVENT);handle_keys(KEY_OK_EVENT);
    assert(flash_params.curve_duty==TRACK_CURVE_MIN_DUTY-TRACK_SPEED_COARSE_STEP);
    dump_screen("MDK/ui-speed.bin");
    handle_keys(KEY_UP_EVENT);assert(speed_item==0);handle_keys(KEY_RIGHT_EVENT);
    for(n=0;n<40;n++)handle_keys(KEY_DOWN_EVENT);
    assert(params.base_duty==params.curve_duty && strstr(message,"LIMIT"));handle_keys(KEY_LEFT_EVENT);
    handle_keys(KEY_RIGHT_EVENT);for(n=0;n<40;n++)handle_keys(KEY_UP_EVENT);
    assert(params.base_duty==TRACK_DUTY_MAX && strstr(message,"LIMIT"));handle_keys(KEY_LEFT_EVENT);
    step();assert(!hw_left && !hw_right && !hw_servo);
    handle_keys(KEY_LEFT_EVENT);assert(mode==TRACK_MENU);

    /* 方向设置：不输出执行器，舵机换向同步交换限位，取消或写失败保留旧值。 */
    before_direction=params;
    enter(TRACK_SERVO_DIRECTION);step();assert(!hw_left && !hw_servo && !track_guard_armed);
    handle_keys(KEY_DOWN_EVENT);step();assert(params.servo_left_sign==-1 && !hw_servo);
    assert(params.left==before_direction.right && params.right==before_direction.left && params.center==before_direction.center);
    dump_screen("MDK/ui-servo-direction.bin");
    handle_keys(KEY_LEFT_EVENT);assert(menu_page==MENU_SERVO && menu_item==3);
    assert(params.servo_left_sign==before_direction.servo_left_sign && params.left==before_direction.left);
    enter(TRACK_SERVO_DIRECTION);handle_keys(KEY_DOWN_EVENT);store_fails=1;handle_keys(KEY_OK_EVENT);
    assert(flash_params.servo_left_sign==1 && strstr(message,"FAILED") && !track_guard_armed);
    handle_keys(KEY_LEFT_EVENT);assert(params.servo_left_sign==1);store_fails=0;
    enter(TRACK_SERVO_DIRECTION);handle_keys(KEY_DOWN_EVENT);handle_keys(KEY_OK_EVENT);
    assert(flash_params.servo_left_sign==-1 && TrackParams_Valid(&params));
    handle_keys(KEY_LEFT_EVENT);assert(params.servo_left_sign==-1);
    enter(TRACK_LEFT_DIRECTION);handle_keys(KEY_DOWN_EVENT);step();
    assert(params.left_reverse==1 && !hw_left_reverse && !hw_left && !hw_servo);
    handle_keys(KEY_LEFT_EVENT);assert(!params.left_reverse);
    enter(TRACK_LEFT_DIRECTION);handle_keys(KEY_DOWN_EVENT);handle_keys(KEY_OK_EVENT);
    assert(hw_left_reverse && !hw_right_reverse && flash_params.left_reverse);
    dump_screen("MDK/ui-left-direction.bin");handle_keys(KEY_LEFT_EVENT);
    enter(TRACK_RIGHT_DIRECTION);handle_keys(KEY_DOWN_EVENT);store_fails=1;handle_keys(KEY_OK_EVENT);
    assert(!hw_right_reverse && !flash_params.right_reverse);store_fails=0;
    handle_keys(KEY_OK_EVENT);assert(hw_left_reverse && hw_right_reverse);
    dump_screen("MDK/ui-right-direction.bin");step();assert(!hw_left && !hw_right && !hw_servo);
    handle_keys(KEY_LEFT_EVENT);

    /* 恢复 PID：预览与取消不写 Flash；失败可撤销；成功不改变其他参数。 */
    before_restore=params;old_store_calls=store_calls;
    enter(TRACK_PID);handle_keys(KEY_UP_EVENT);assert(pid_item==PID_ITEM_RESTORE);
    dump_screen("MDK/ui-pid-restore.bin");
    handle_keys(KEY_OK_EVENT);assert(editing && params.kp100==(uint16_t)(TRACK_PID_DEFAULT_KP*100.0f+0.5f) && store_calls==old_store_calls);
    dump_screen("MDK/ui-pid-confirm.bin");
    handle_keys(KEY_LEFT_EVENT);assert(!editing && params.kp100==before_restore.kp100);
    handle_keys(KEY_RIGHT_EVENT);store_fails=1;handle_keys(KEY_OK_EVENT);
    assert(editing && flash_params.kp100==before_restore.kp100 && !hw_left);
    handle_keys(KEY_LEFT_EVENT);assert(params.kp100==before_restore.kp100);store_fails=0;
    handle_keys(KEY_OK_EVENT);handle_keys(KEY_OK_EVENT);
    assert(!editing && flash_params.kp100==(uint16_t)(TRACK_PID_DEFAULT_KP*100.0f+0.5f) && flash_params.ki100==0 && flash_params.kd100==15);
    assert(params.center==before_restore.center && params.left==before_restore.left && params.right==before_restore.right);
    assert(params.base_duty==before_restore.base_duty && params.curve_duty==before_restore.curve_duty && params.lost_sum==before_restore.lost_sum);
    assert(params.servo_left_sign==-1 && params.left_reverse && params.right_reverse);
    step();assert(!hw_left && !hw_right);handle_keys(KEY_LEFT_EVENT);
    enter(TRACK_RUN);step();assert(mode==TRACK_RUN && hw_left>0);
    for(n=0;n<200;n++)step();assert(hw_left==params.base_duty && hw_right==params.base_duty);
    dump_screen("MDK/ui-run.bin");
    old=hw_servo;fake_raw[0]=fake_raw[1]=0;step();
    assert(mode==TRACK_RUN && !control.signal_ok && hw_servo==old && hw_left>0 && hw_left<=600);
    fake_raw[0]=850;fake_raw[1]=750;step();
    assert(mode==TRACK_RUN && control.signal_ok && !control.weak_ms);
    handle_keys(KEY_LEFT_EVENT|KEY_UP_EVENT);assert(mode==TRACK_MENU && !hw_left && !hw_servo);
    enter(TRACK_RUN);track_guard_trip=1;step();assert(mode==TRACK_MENU && fault==TRACK_TIMEOUT && !hw_servo);
    enter(TRACK_RUN);fake_raw[0]=fake_raw[1]=0;
    for(n=5;n<TRACK_ZERO_CONFIRM_MS;n+=5){step();assert(mode==TRACK_RUN);}
    step();assert(mode==TRACK_MENU && fault==TRACK_LOST && !hw_left && !hw_servo);
    enter(TRACK_PID);step();assert(mode==TRACK_PID && !hw_left && !hw_servo);
    sample_ok=0;step();assert(mode==TRACK_MENU && fault==TRACK_ADC_ERROR);
    /* ADC fault during calibration must not allow a stale partial commit. */
    enter(TRACK_CAL);sample_ok=1;
    for(n=0;n<120;n++){fake_raw[0]=fake_raw[1]=(uint16_t)(200+n*10);step();}
    sample_ok=0;step();handle_keys(KEY_OK_EVENT);assert(mode==TRACK_CAL && cal_adc_error);
    handle_keys(KEY_RIGHT_EVENT);assert(!cal_adc_error && sensors.samples==0);
    handle_keys(KEY_LEFT_EVENT);sample_ok=1;
    assert(sensors.center_valid); /* Cancelled extrema calibration keeps reference. */
    enter(TRACK_CAL);
    for(n=0;n<120;n++){fake_raw[0]=fake_raw[1]=(uint16_t)(200+n*10);step();}
    handle_keys(KEY_OK_EVENT);assert(sensors.valid && !sensors.center_valid && sensors.center_bias==0);
    enter(TRACK_RUN);assert(mode==TRACK_MENU && fault==TRACK_NO_CENTER && !hw_left);
    /* Held startup key cannot enter any page, but menu still renders. */
    old=params.base_duty;
    TrackApp_Init();assert(!sensors.valid && !sensors.center_valid && !defaults_loaded);
    assert(params.base_duty==old && params.center==TRACK_SERVO_DEFAULT_CENTER+1);
    assert(params.servo_left_sign==-1 && hw_left_reverse && hw_right_reverse);
    P27=0;track_key_events=KEY_OK_EVENT;TrackApp_Task();assert(mode==TRACK_MENU);
    P27=1;track_milliseconds+=30;TrackApp_Task();assert(!startup_keys_locked && mode==TRACK_MENU);
    puts("PASS: grouped menus/back/cursor, direction save/cancel/failure/reboot, servo limit swap, speed/PID/calibration and motor interlocks");
    telemetry_tests();
    center_display_test();
    return 0;
}
