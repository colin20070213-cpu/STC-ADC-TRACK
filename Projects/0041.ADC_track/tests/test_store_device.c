/**
 * @file test_store_device.c
 * @brief 运行真实 IAP 寄存器路径，模拟 EEPROM 分区及 bit4 失败标志。
 * @note 不替换 iap_command，避免只验证理想化存储接口而漏掉地址和标志位错误。
 */
#include "track_hw.h"
#include "track_store_test_device.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

uint8_t EA = 1, IAP_CONTR, IAP_CMD, IAP_TRIG, IAP_DATA;
uint8_t IAP_TPS, IAP_ADDRE, IAP_ADDRH, IAP_ADDRL;
uint32_t system_clock = 33177600UL;
volatile uint8_t track_guard_armed;
static uint8_t memory[1024];
static unsigned configured_bytes = sizeof(memory);
static unsigned operations, nop_count, watchdog_feeds;
static uint8_t fail_command, corrupt_reads;

void TrackHW_Stop(void) {}
void TrackHW_FeedWatchdog(void) { ++watchdog_feeds; }

/** @brief 第一条 NOP 时执行触发命令；其余 NOP 只计数。 */
void TrackTest_IAPNop(void)
{
    uint32_t address;

    ++nop_count;
    if (IAP_TRIG != 0xa5) return;
    IAP_TRIG = 0;
    ++operations;
    assert(!EA && IAP_CONTR == 0x80);
    assert(IAP_TPS == (system_clock + 999999UL) / 1000000UL);
    address = ((uint32_t)IAP_ADDRE << 16) | ((uint16_t)IAP_ADDRH << 8) | IAP_ADDRL;
    if (address >= configured_bytes || IAP_CMD == fail_command)
    {
        IAP_CONTR |= 0x10; /* 真正的 CMD_FAIL 在 bit4。 */
        IAP_DATA = 0xff;
        return;
    }
    switch (IAP_CMD)
    {
        case 1:
            IAP_DATA = memory[address] ^ (corrupt_reads ? 1 : 0);
            break;
        case 2:
            memory[address] &= IAP_DATA;
            break;
        case 3:
            memset(memory + (address & ~511UL), 0xff, 512);
            break;
        default:
            assert(0);
    }
}

/* 包含实际实现，以检查底层地址防护和 IAP 结束后的寄存器状态。 */
#include "../USER/src/track_store.c"

int main(void)
{
    TrackParams params, loaded;
    uint8_t readback;
    unsigned before;

    assert(TRACK_STORE_SLOT_A == 0 && TRACK_STORE_SLOT_B == 0x200);
    memset(memory, 0xff, sizeof(memory));
    assert(!TrackStore_Load(&params));
    assert(params.center == 760 && params.left == 840 && params.right == 670);
    assert(TrackStore_Save(&params) && TrackStore_LastError() == TRACK_STORE_OK);
    assert(TrackStore_Load(&loaded) && loaded.center == 760);
    assert(EA && !IAP_CONTR && !IAP_CMD && !IAP_TRIG);
    assert(IAP_ADDRE == 0xff && IAP_ADDRH == 0xff && IAP_ADDRL == 0xff);
    assert(nop_count == 4 * operations && watchdog_feeds == operations);

    /* 使用实际 IAP 寄存器路径复现速度保存：阈值 40 不应被参数校验拒绝。 */
    params.speed_base=25;params.speed_curve=12;params.speed_kp100=3500;
    params.lost_sum = 40;
    params.base_duty = 4000;
    assert(TrackStore_Save(&params) && TrackStore_LastError() == TRACK_STORE_OK);
    assert(TrackStore_Load(&loaded) && loaded.lost_sum == 40 && loaded.base_duty == 4000 && loaded.speed_base==25 && loaded.speed_kp100==3500);

    before = operations;
    assert(!iap_command(0x01fc00UL, 3, 0xff, &readback));
    assert(!iap_command(1024, 2, 0, &readback));
    assert(!iap_command(0, 4, 0, &readback));
    assert(operations == before); /* 程序区及未预留地址不能触发硬件操作。 */

    configured_bytes = 0; /* 模拟 ISP 中没有分配用户 EEPROM。 */
    params.kp100 += 1;
    assert(!TrackStore_Save(&params) && TrackStore_LastError() == TRACK_STORE_ERASE);
    assert(!iap_command(0, 1, 0xff, &readback)); /* bit4 必须被真实路径识别。 */
    assert(EA && !IAP_CONTR);
    configured_bytes = sizeof(memory);
    assert(TrackStore_Load(&loaded) && loaded.kp100 == TRACK_PID_DEFAULT_KP * 100);

    fail_command = 2;
    assert(!TrackStore_Save(&params) && TrackStore_LastError() == TRACK_STORE_WRITE);
    fail_command = 0;
    assert(TrackStore_Load(&loaded) && loaded.kp100 != params.kp100);
    assert(TrackStore_Save(&params));
    assert(TrackStore_Load(&loaded) && loaded.kp100 == params.kp100);

    params.kp100 += 1;
    corrupt_reads = 1;
    assert(!TrackStore_Save(&params) && TrackStore_LastError() == TRACK_STORE_VERIFY);
    corrupt_reads = 0;
    assert(TrackStore_Load(&loaded));

    fail_command = 1;
    params.kp100 += 1;
    assert(!TrackStore_Save(&params) && TrackStore_LastError() == TRACK_STORE_READ);
    fail_command = 0;

    before = operations;
    track_guard_armed = 1;
    assert(!TrackStore_Save(&params) && TrackStore_LastError() == TRACK_STORE_BUSY);
    assert(operations == before);
    track_guard_armed = 0;

    EA = 0;
    system_clock = 24000000UL;
    assert(iap_command(0, 1, 0xff, &readback) && !EA && IAP_TPS == 24);
    assert(nop_count == 4 * operations && watchdog_feeds == operations);
    puts("PASS: real IAP register path, low EEPROM slots, bit4 failures, ISP partition, IRQ/clock and save diagnostics");
    return 0;
}
