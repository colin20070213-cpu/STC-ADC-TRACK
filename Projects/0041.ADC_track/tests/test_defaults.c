/** @file test_defaults.c
 *  @brief 使用不同编译宏和固定旧版记录验证方案 A：Flash 优先，手动只恢复 PID。
 */
#include "track_core.h"
#include "track_hw.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

volatile uint8_t track_guard_armed;
static uint8_t flash_memory[1024];
static unsigned write_count;
static int fail_after=-1;
static unsigned mutation_index;
/* 固定格式 v1 记录：Kp=60、Ki=0.25、Kd=0.30，包含独立计算的 CRC。 */
static const uint8_t legacy_record[32] = { 0x50, 0x49, 0x45, 0x01, 0x07, 0x00, 0x00, 0x00, 0x70, 0x17, 0x19, 0x00, 0x1e, 0x00, 0xef, 0x02, 0x25, 0x03, 0xb7, 0x02, 0x78, 0x05, 0x8a, 0x02, 0xaf, 0x00, 0xff, 0xff, 0xf0, 0x24, 0xff, 0xa5 };

void TrackHW_Stop(void) {}
void TrackHW_FeedWatchdog(void) {}

uint8_t TrackTest_IAP(uint32_t address, uint8_t command, uint8_t value, uint8_t *readback)
{
    uint32_t offset = address - TRACK_STORE_SLOT_A;
    assert(offset < sizeof(flash_memory));
    if (command == 1)
    {
        *readback = flash_memory[offset];
        return 1;
    }
    if(fail_after>=0 && mutation_index++==(unsigned)fail_after)
    {
        return 0;
    }
    ++write_count;
    if (command == 3)
    {
        memset(flash_memory + (offset & ~511U), 0xFF, 512);
    }
    else
    {
        assert(command == 2);
        flash_memory[offset] &= value;
    }
    *readback = 0;
    return 1;
}

int main(void)
{
    TrackParams params;
    TrackParams restored;
    TrackParams migrated;
    uint8_t legacy_flash[1024];
    int cut;

    memset(flash_memory, 0xFF, sizeof(flash_memory));
    assert(!TrackStore_Load(&params));
    assert(params.kp100 == 5275 && params.ki100 == 5 && params.kd100 == 32);
    assert(params.center == 734 && params.left == 820 && params.right == 650);
    memcpy(flash_memory, legacy_record, sizeof(legacy_record));
    assert(TrackStore_Load(&params));
    assert(params.kp100 == 6000 && params.ki100 == 25 && params.kd100 == 30);
    assert(params.center == 751 && params.left == 805 && params.right == 695);
    assert(params.base_duty == 1400 && params.curve_duty == 650 && params.lost_sum == 175);
    assert(!write_count);
    /* 旧记录升级到含方向的 v2 时，逐点断电也必须恢复旧参数。 */
    memcpy(legacy_flash,flash_memory,sizeof(legacy_flash));
    for(cut=0;cut<(int)TRACK_STORE_RECORD_BYTES+1;cut++)
    {
        memcpy(flash_memory,legacy_flash,sizeof(legacy_flash));
        fail_after=-1;
        assert(TrackStore_Load(&migrated));
        assert(TrackParams_SetServoDirection(&migrated,-1));
        migrated.left_reverse=1;migrated.right_reverse=1;
        mutation_index=0;fail_after=cut;
        assert(!TrackStore_Save(&migrated));
        fail_after=-1;
        assert(TrackStore_Load(&params) && params.kp100==6000);
        assert(params.servo_left_sign==1 && !params.left_reverse && !params.right_reverse);
    }
    memcpy(flash_memory,legacy_flash,sizeof(legacy_flash));
    assert(TrackStore_Load(&params));
    assert(params.servo_left_sign==1 && !params.left_reverse && !params.right_reverse);
    assert(TrackStore_Save(&params) && write_count>0); /* 第一次明确保存才升级 v2。 */
    {
        unsigned previous_writes=write_count;
        assert(TrackStore_Save(&params) && write_count==previous_writes);
    }

    restored = params;
    TrackParams_RestorePID(&restored);
    assert(restored.kp100 == 5275 && restored.ki100 == 5 && restored.kd100 == 32);
    assert(restored.center == params.center && restored.left == params.left && restored.right == params.right);
    assert(restored.base_duty == params.base_duty && restored.curve_duty == params.curve_duty && restored.lost_sum == params.lost_sum);
    assert(TrackStore_Load(&params) && params.kp100 == 6000); /* 预览尚未保存。 */
    assert(TrackStore_Save(&restored));
    assert(TrackStore_Load(&params) && params.kp100 == 5275 && params.center == 751);
    assert(params.base_duty == 1400 && params.curve_duty == 650);
    assert(params.servo_left_sign==1 && !params.left_reverse && !params.right_reverse);
    assert(TrackParams_SetServoDirection(&params,-1));
    params.left_reverse=1;params.right_reverse=1;
    assert(TrackStore_Save(&params));
    memset(&params,0,sizeof(params));
    assert(TrackStore_Load(&params));
    assert(params.servo_left_sign==-1 && params.left_reverse && params.right_reverse);
    assert(params.center==751 && params.left==695 && params.right==805);
    puts("PASS: code defaults/Flash priority, legacy migration interrupted at 81 points, PID-only restore persistence");
    return 0;
}
