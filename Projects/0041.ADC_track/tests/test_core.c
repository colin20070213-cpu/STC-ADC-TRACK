#include "track_core.h"
#include "track_hw.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

volatile uint8_t track_guard_armed;
static uint8_t flash_memory[1024];
static int fail_after = -1, mutations, stops;
void TrackHW_Stop(void) { ++stops; }
void TrackHW_FeedWatchdog(void) {}
uint8_t TrackTest_IAP(uint32_t address, uint8_t command, uint8_t value, uint8_t *readback)
{
    uint32_t offset = address-TRACK_STORE_SLOT_A;
    assert(offset < sizeof(flash_memory));
    if (command == 1) { *readback = flash_memory[offset]; return 1; }
    if (fail_after >= 0 && mutations++ == fail_after) return 0;
    if (command == 3) memset(flash_memory+(offset & ~511U),0xFF,512);
    else { assert(command == 2); flash_memory[offset] &= value; }
    *readback = 0; return 1;
}
static void calibration_tests(void)
{
    TrackSensors s; uint16_t raw[4] = {0}; unsigned n,k;
    memset(&s,0,sizeof(s)); TrackCal_Begin(&s);
    assert(!TrackCal_Commit(&s) && !s.valid && s.bad_channel == 255);
    for (n=0;n<TRACK_CAL_MIN_SAMPLES+20;n++) {
        for (k=0;k<TRACK_SENSOR_COUNT;k++) raw[k]=(uint16_t)(200+k*50+n*10);
        TrackCal_Sample(&s,raw);
    }
    assert(TrackCal_Commit(&s) && s.valid);
    for(k=0;k<TRACK_SENSOR_COUNT;k++) {
        assert(s.low[k] == 200+k*50 && s.high[k] == 200+k*50+(TRACK_CAL_MIN_SAMPLES+19)*10);
        raw[k]=s.low[k]+(s.high[k]-s.low[k])/2;
    }
    TrackSensors_Update(&s,raw);
    for(k=0;k<TRACK_SENSOR_COUNT;k++) assert(s.norm[k]==500);
    TrackCal_Begin(&s);
    for(n=0;n<TRACK_CAL_MIN_SAMPLES;n++) TrackCal_Sample(&s,raw);
    assert(!TrackCal_Commit(&s) && s.valid && s.low[0]==200);
    assert(TrackNormalize(0,100,1100)==0);
    assert(TrackNormalize(600,100,1100)==500);
    assert(TrackNormalize(4095,100,1100)==1000);
    assert(TrackNormalize(1,100,100)==0);
    assert(TrackNormalize(65535,0,4095)==1000);
    TrackCal_Begin(&s); raw[0]=4095;
    for(n=0;n<TRACK_CAL_MIN_SAMPLES;n++) TrackCal_Sample(&s,raw);
    assert(!TrackCal_Commit(&s));
    /* 恢复正常放大板阈值后，0～10 不应通过；量程 0～100 可通过。 */
    memset(&s,0,sizeof(s));memset(raw,0,sizeof(raw));TrackCal_Begin(&s);
    TrackCal_Sample(&s,raw);
    for(k=0;k<TRACK_SENSOR_COUNT;k++)raw[k]=10;
    for(n=1;n<TRACK_CAL_MIN_SAMPLES;n++)TrackCal_Sample(&s,raw);
    assert(!TrackCal_Commit(&s));
    for(k=0;k<TRACK_SENSOR_COUNT;k++)raw[k]=100;
    TrackCal_Sample(&s,raw);
    assert(TrackCal_Commit(&s));
    for(k=0;k<TRACK_SENSOR_COUNT;k++)raw[k]=50;
    TrackSensors_Update(&s,raw);
    for(k=0;k<TRACK_SENSOR_COUNT;k++)assert(s.low[k]==0 && s.high[k]==100 && s.norm[k]==500);
}
static void pid_tests(void)
{
    TrackParams p; TrackSensors s; TrackControl c; unsigned n;
    TrackParams_Default(&p); assert(TrackParams_Valid(&p));
    assert(p.kp100==(uint16_t)(TRACK_PID_DEFAULT_KP*100.0f+0.5f));
    p.kp100=40000;assert(TrackParams_Valid(&p)); /* 允许已有 Kp=400 的保存记录。 */
    p.kp100=50001;assert(!TrackParams_Valid(&p));TrackParams_Default(&p);
    memset(&s,0,sizeof(s)); TrackControl_Reset(&c,&p);
    assert(TrackControl_Step(&c,&s,&p,1)==TRACK_NO_CAL);
    s.valid=1; s.norm[0]=500; s.norm[1]=500;
    assert(TrackControl_Step(&c,&s,&p,1)==TRACK_OK && c.servo==p.center && c.left_motor<=TRACK_MOTOR_RAMP);
    s.norm[0]=900; s.norm[1]=100;
    for(n=0;n<100;n++) {
        assert(TrackControl_Step(&c,&s,&p,0)==TRACK_OK);
        assert(c.left_motor==0 && c.right_motor==0);
        assert(c.servo>=p.right && c.servo<=p.left);
    }
    assert(c.servo>p.center);
    TrackControl_Reset(&c,&p); s.norm[0]=100;s.norm[1]=900;
    TrackControl_Step(&c,&s,&p,0); assert(c.servo<p.center);
    p.kp100=20000; p.ki100=2000; p.kd100=0;
    TrackControl_Reset(&c,&p);s.norm[0]=1000;s.norm[1]=100;
    for(n=0;n<2000;n++) TrackControl_Step(&c,&s,&p,0);
    assert(c.saturated && c.steering_pid.integral==0.0f && c.servo==p.left);
    s.norm[0]=0;s.norm[1]=0;
    for(n=TRACK_PERIOD_MS;n<TRACK_ZERO_CONFIRM_MS;n+=TRACK_PERIOD_MS)
        assert(TrackControl_Step(&c,&s,&p,1)==TRACK_OK);
    assert(TrackControl_Step(&c,&s,&p,1)==TRACK_LOST);
    p.lost_sum=150;TrackControl_Reset(&c,&p);s.norm[0]=40;s.norm[1]=40;
    for(n=TRACK_PERIOD_MS;n<TRACK_LOST_CONFIRM_MS;n+=TRACK_PERIOD_MS)
        assert(TrackControl_Step(&c,&s,&p,1)==TRACK_OK && !c.signal_ok && !c.left_motor);
    assert(TrackControl_Step(&c,&s,&p,1)==TRACK_LOST);
    TrackParams_Default(&p);TrackControl_Reset(&c,&p);s.norm[0]=500;s.norm[1]=500;
    TrackControl_Step(&c,&s,&p,1);s.norm[0]=900;s.norm[1]=900;
    TrackControl_Step(&c,&s,&p,1); assert(c.crossing);
    s.norm[0]=0;s.norm[1]=0;
    for(n=TRACK_PERIOD_MS;n<TRACK_ZERO_CONFIRM_MS;n+=TRACK_PERIOD_MS)
        assert(TrackControl_Step(&c,&s,&p,1)==TRACK_OK);
    assert(TrackControl_Step(&c,&s,&p,1)==TRACK_LOST);
    p.center=p.left;assert(!TrackParams_Valid(&p));
}
static void body_center_tests(void)
{
    TrackParams p; TrackSensors s; TrackControl c; unsigned n; int16_t bias;
    uint16_t raw[TRACK_SENSOR_MAX];
    TrackParams_Default(&p); memset(&s,0,sizeof(s)); s.valid=1;
    s.norm[0]=600; s.norm[1]=400;
    TrackBodyCenter_Begin(&s);
    for(n=0;n<TRACK_BODY_SAMPLES-1;n++) TrackBodyCenter_Sample(&s,&p);
    assert(!TrackBodyCenter_Commit(&s) && !s.center_valid);
    TrackBodyCenter_Sample(&s,&p);
    assert(TrackBodyCenter_Commit(&s) && s.center_valid && s.center_bias==200);
    assert(TrackSensors_Feedback(&s)==0);
    TrackControl_Reset(&c,&p);TrackControl_Step(&c,&s,&p,0);
    assert(c.error==0 && c.servo==p.center);
    s.norm[0]=650;s.norm[1]=350;TrackControl_Step(&c,&s,&p,0);assert(c.error==100);
    assert(TrackSensors_Feedback(&s)==100);
    s.norm[0]=400;s.norm[1]=600;
    TrackBodyCenter_Begin(&s);
    for(n=0;n<TRACK_BODY_SAMPLES;n++)TrackBodyCenter_Sample(&s,&p);
    assert(TrackBodyCenter_Commit(&s) && s.center_bias==-200);
    TrackControl_Reset(&c,&p);TrackControl_Step(&c,&s,&p,0);assert(c.error==0);
    s.norm[0]=1000;s.norm[1]=0;TrackControl_Step(&c,&s,&p,0);assert(c.error==1000);
    assert(TrackSensors_Feedback(&s)==1000);
    bias=s.center_bias;
    TrackBodyCenter_Begin(&s);s.norm[0]=s.norm[1]=20;TrackBodyCenter_Sample(&s,&p);
    assert(s.center_status==TRACK_BODY_WEAK && !TrackBodyCenter_Commit(&s));
    TrackBodyCenter_Begin(&s);s.norm[0]=900;s.norm[1]=100;TrackBodyCenter_Sample(&s,&p);
    assert(s.center_status==TRACK_BODY_OFFSET && !TrackBodyCenter_Commit(&s));
    TrackBodyCenter_Begin(&s);s.norm[0]=s.norm[1]=500;TrackBodyCenter_Sample(&s,&p);
    s.norm[0]=600;s.norm[1]=400;TrackBodyCenter_Sample(&s,&p);
    assert(s.center_status==TRACK_BODY_MOVING && !TrackBodyCenter_Commit(&s));
    TrackBodyCenter_Begin(&s);s.raw[TRACK_SENSOR_COUNT-1]=4090;TrackBodyCenter_Sample(&s,&p);
    assert(s.center_status==TRACK_BODY_ADC && s.center_valid && s.center_bias==bias);
    /* The reference cannot suppress genuine loss, and new normalization invalidates it. */
    s.norm[0]=s.norm[1]=0;
    for(n=TRACK_PERIOD_MS;n<TRACK_ZERO_CONFIRM_MS;n+=TRACK_PERIOD_MS)
        assert(TrackControl_Step(&c,&s,&p,1)==TRACK_OK);
    assert(TrackControl_Step(&c,&s,&p,1)==TRACK_LOST);
    TrackCal_Begin(&s);assert(s.center_valid);
    memset(raw,0,sizeof(raw));TrackCal_Sample(&s,raw);
    for(n=0;n<TRACK_SENSOR_MAX;n++)raw[n]=1000;
    for(n=1;n<TRACK_CAL_MIN_SAMPLES;n++)TrackCal_Sample(&s,raw);
    assert(TrackCal_Commit(&s) && !s.center_valid && s.center_bias==0);
}
static void direction_tests(void)
{
    TrackParams params, original, restored;
    TrackSensors sensors;
    TrackControl control;
    uint8_t record[32];
    uint32_t sequence;
    TrackParams_Default(&params);
    params.center=751;params.left=805;params.right=695;
    original=params;
    assert(TrackParams_SetServoDirection(&params,-1));
    assert(params.center==751 && params.left==695 && params.right==805);
    assert(TrackParams_Valid(&params));
    memset(&sensors,0,sizeof(sensors));sensors.valid=1;sensors.norm[0]=900;sensors.norm[1]=100;
    TrackControl_Reset(&control,&params);
    TrackControl_Step(&control,&sensors,&params,0);assert(control.servo<params.center);
    assert(TrackParams_SetServoDirection(&params,1));
    assert(params.left==original.left && params.right==original.right);
    assert(!TrackParams_SetServoDirection(&params,0));
    params.left_reverse=1;params.right_reverse=1;
    assert(TrackParams_SetServoDirection(&params,-1));
    TrackRecord_Encode(record,&params,123);
    assert(record[3]==2 && record[26]==7);
    assert(TrackRecord_Decode(record,&restored,&sequence) && sequence==123);
    assert(restored.servo_left_sign==-1 && restored.left_reverse && restored.right_reverse);
    record[26]|=0x80;record[28]=(uint8_t)TrackCRC16(record,28);record[29]=(uint8_t)(TrackCRC16(record,28)>>8);
    assert(!TrackRecord_Decode(record,&restored,&sequence));
    params.left_reverse=2;assert(!TrackParams_Valid(&params));
}
/** @brief 覆盖实际 0～80 原始信号的所有左右组合，验证误差具有中间值。 */
static void low_signal_tests(void)
{
    TrackSensors sensors;
    TrackParams params;
    TrackControl control;
    uint16_t raw[TRACK_SENSOR_MAX] = {0};
    unsigned left, right, i, intermediate = 0;
    int32_t expected;
    int16_t actual;

    assert(TrackSensors_MathCheck());
    TrackParams_Default(&params);
    memset(&sensors,0,sizeof(sensors));sensors.valid=1;
    for(i=0;i<TRACK_SENSOR_COUNT;i++)sensors.high[i]=80;
    for(left=0;left<=80;left++)
    {
        for(right=0;right<=80;right++)
        {
            raw[0]=(uint16_t)left;raw[1]=(uint16_t)right;
            TrackSensors_Update(&sensors,raw);
            assert(sensors.norm[0]==left*1000U/80 && sensors.norm[1]==right*1000U/80);
            expected=sensors.norm[0]+sensors.norm[1];
            if(expected)expected=((int32_t)sensors.norm[0]-sensors.norm[1])*1000L/expected;
            actual=TrackSensors_Error(&sensors);
            assert(actual==expected && actual>=-1000 && actual<=1000);
            if(actual && actual!=-1000 && actual!=1000)++intermediate;
        }
    }
    assert(intermediate>1000);
    raw[0]=40;raw[1]=20;TrackSensors_Update(&sensors,raw);
    assert(TrackSensors_Error(&sensors)==333);
    raw[0]=20;raw[1]=40;TrackSensors_Update(&sensors,raw);
    assert(TrackSensors_Error(&sensors)==-333);

    /* 低幅度算术覆盖仍保留，但归一化 50/50 不再满足正常信号门槛。 */
    raw[0]=raw[1]=4;TrackSensors_Update(&sensors,raw);
    TrackBodyCenter_Begin(&sensors);
    for(i=0;i<TRACK_BODY_SAMPLES;i++)TrackBodyCenter_Sample(&sensors,&params);
    assert(!TrackBodyCenter_Commit(&sensors) && sensors.center_status==TRACK_BODY_WEAK);
    raw[0]=raw[1]=8;TrackSensors_Update(&sensors,raw);TrackBodyCenter_Begin(&sensors);
    for(i=0;i<TRACK_BODY_SAMPLES;i++)TrackBodyCenter_Sample(&sensors,&params);
    assert(TrackBodyCenter_Commit(&sensors) && sensors.center_bias==0);
    TrackControl_Reset(&control,&params);
    assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK && control.signal_ok);
    raw[0]=raw[1]=0;TrackSensors_Update(&sensors,raw);
    for(i=TRACK_PERIOD_MS;i<TRACK_ZERO_CONFIRM_MS;i+=TRACK_PERIOD_MS)
        assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK);
    assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_LOST && !control.left_motor);

    /* 正常归一化强度下，一个 ADC 量化台阶造成的小波动允许通过。 */
    TrackBodyCenter_Begin(&sensors);
    for(i=0;i<TRACK_BODY_SAMPLES;i++)
    {
        raw[0]=(uint16_t)(40+(i&1));raw[1]=(uint16_t)(41-(i&1));
        TrackSensors_Update(&sensors,raw);TrackBodyCenter_Sample(&sensors,&params);
    }
    assert(TrackBodyCenter_Commit(&sensors));
    raw[0]=raw[1]=1;TrackSensors_Update(&sensors,raw);
    TrackBodyCenter_Begin(&sensors);TrackBodyCenter_Sample(&sensors,&params);
    assert(sensors.center_status==TRACK_BODY_WEAK);
    puts("PASS: all 6561 low-signal pairs, continuous signed errors, low-amplitude centering and genuine loss");
}
static void record_tests(void)
{
    TrackParams p,q; uint8_t record[32],copy[32]; uint32_t seq; unsigned i;
    TrackParams_Default(&p); TrackRecord_Encode(record,&p,42);
    assert(TrackRecord_Decode(record,&q,&seq) && seq==42 && q.kp100==p.kp100);
    for(i=0;i<32;i++) {
        memcpy(copy,record,32);copy[i]^=1;
        if(i!=30) assert(!TrackRecord_Decode(copy,&q,&seq));
    }
    assert(TrackSequence_Newer(0,0xFFFFFFFFUL));
    assert(!TrackSequence_Newer(0xFFFFFFFFUL,0));
    assert(TrackCRC16((const uint8_t*)"123456789",9)==0x29B1);
    p.lost_sum=100;TrackRecord_Encode(record,&p,43);
    assert(TrackRecord_Decode(record,&q,&seq) && seq==43 && q.lost_sum==TRACK_LOST_SUM);
    assert(q.kp100==p.kp100 && q.center==p.center && q.left==p.left && q.right==p.right);
    p.lost_sum=175;TrackRecord_Encode(record,&p,44);
    assert(TrackRecord_Decode(record,&q,&seq) && q.lost_sum==175);
}
/** @brief 临时失线可恢复；连续失线到期必停，舵机保持且动力不会超过确认上限。 */
static void loss_confirmation_tests(void)
{
    TrackParams params;
    TrackSensors sensors;
    TrackControl control;
    unsigned n, elapsed, index;
    uint16_t held_servo;
    const uint16_t cases[][3]={{0,0,50},
        {TRACK_HARD_LOST_EACH-1,TRACK_HARD_LOST_EACH-1,50},
        {TRACK_HARD_LOST_EACH,TRACK_HARD_LOST_EACH,100},{0,149,100}};

    TrackParams_Default(&params);params.base_duty=4000;params.curve_duty=1000;
    params.lost_sum=150; /* 此用例固定总强度，分别验证弱信号和双路近零。 */
    assert(TrackParams_Valid(&params));
    memset(&sensors,0,sizeof(sensors));sensors.valid=1;
    sensors.norm[0]=sensors.norm[1]=500;
    TrackControl_Reset(&control,&params);
    for(n=0;n<TRACK_DUTY_MAX/TRACK_MOTOR_RAMP+1;n++)assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK);
    assert(control.left_motor==4000 && control.right_motor==4000);
    sensors.norm[0]=800;sensors.norm[1]=200;
    assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK);
    held_servo=control.servo;
    sensors.norm[0]=20;sensors.norm[1]=60;
    assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK);
    assert(!control.signal_ok && control.weak_ms==TRACK_PERIOD_MS && !control.zero_ms);
    assert(control.servo==held_servo && control.left_motor==600 && control.right_motor==600);
    assert(!control.steering_pid.initialized);
    sensors.norm[0]=sensors.norm[1]=90; /* 总和 180 位于恢复滞回区内。 */
    assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK && control.weak_ms==2*TRACK_PERIOD_MS);
    sensors.norm[0]=sensors.norm[1]=100;
    assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK && control.signal_ok);
    assert(!control.weak_ms && !control.zero_ms && control.left_motor==600+TRACK_MOTOR_RAMP);

    for(index=0;index<sizeof(cases)/sizeof(cases[0]);index++)
    {
        TrackControl_Reset(&control,&params);control.left_motor=control.right_motor=4000;
        control.servo=params.left;control.crossing=1;
        sensors.norm[0]=cases[index][0];sensors.norm[1]=cases[index][1];
        for(elapsed=TRACK_PERIOD_MS;elapsed<cases[index][2];elapsed+=TRACK_PERIOD_MS)
        {
            assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK);
            assert(!control.signal_ok && control.left_motor==600 && control.right_motor==600);
            assert(control.servo==params.left && !control.crossing);
        }
        assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_LOST);
        assert(!control.left_motor && !control.right_motor);
    }
    TrackControl_Reset(&control,&params);sensors.norm[0]=sensors.norm[1]=0;
    assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK && !control.left_motor);
    sensors.norm[0]=sensors.norm[1]=500;
    assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK && !control.weak_ms && !control.zero_ms);
    TrackControl_Reset(&control,&params);sensors.norm[0]=sensors.norm[1]=0;
    params.curve_duty=300;
    control.left_motor=200;control.right_motor=400;
    assert(TrackControl_Step(&control,&sensors,&params,1)==TRACK_OK);
    assert(control.left_motor==200 && control.right_motor==300);
    assert(TrackControl_Step(&control,&sensors,&params,0)==TRACK_OK && !control.left_motor && !control.right_motor);
    puts("PASS: 4000 PWM, transient loss recovery, held steering, recovery hysteresis and 50/100 ms loss deadlines");
}
static void store_tests(void)
{
    TrackParams p,q; uint8_t baseline[1024]; int cut;
    memset(flash_memory,0xFF,sizeof(flash_memory));
    assert(!TrackStore_Load(&p)); assert(TrackStore_Save(&p));
    assert(TrackStore_Load(&q) && q.center==TRACK_SERVO_DEFAULT_CENTER);
    mutations=0;fail_after=0;
    assert(TrackStore_Save(&q)); assert(mutations==0); /* Unchanged: no erase. */
    fail_after=-1; memcpy(baseline,flash_memory,1024);
    for(cut=0;cut<33;cut++) {
        memcpy(flash_memory,baseline,1024);fail_after=-1;TrackStore_Load(&p);
        p.kp100=4100;fail_after=cut;mutations=0;
        assert(!TrackStore_Save(&p)); /* Failure at every erase/program boundary. */
        fail_after=-1;assert(TrackStore_Load(&q) && q.kp100==(uint16_t)(TRACK_PID_DEFAULT_KP*100.0f+0.5f));
    }
    p=q;p.kp100=4200;assert(TrackStore_Save(&p));
    assert(TrackStore_Load(&q) && q.kp100==4200);
    /* 回归：失线阈值 40 和动力 4000 必须能一起写入、重读且再次保存免擦写。 */
    p.base_duty=4000;p.curve_duty=3000;p.lost_sum=40;assert(TrackStore_Save(&p));
    assert(TrackStore_Load(&q) && q.base_duty==4000 && q.curve_duty==3000 && q.lost_sum==40);
    mutations=0;fail_after=0;
    assert(TrackStore_Save(&q) && mutations==0);
    fail_after=-1;
    p.lost_sum=0;assert(!TrackParams_Valid(&p));
    assert(!TrackStore_Save(&p) && TrackStore_LastError()==TRACK_STORE_PARAMS);
    p.lost_sum=501;assert(!TrackParams_Valid(&p));
    p.lost_sum=1;assert(TrackParams_Valid(&p));
    p.lost_sum=500;assert(TrackParams_Valid(&p));
    p.lost_sum=40;
    puts("PASS: loss threshold 40 and speed 4000 persist; invalid thresholds are rejected");
    track_guard_armed=1;p.kp100=4300;assert(!TrackStore_Save(&p));track_guard_armed=0;
    assert(stops>0);
}
int main(void)
{
    calibration_tests();pid_tests();body_center_tests();direction_tests();low_signal_tests();record_tests();loss_confirmation_tests();store_tests();
    printf("PASS: calibration (%d channels), body center/reference rejection/invalidation, PID, loss, CRC and interrupted saves\n",TRACK_SENSOR_COUNT);
    return 0;
}
