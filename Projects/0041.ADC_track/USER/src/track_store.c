/**
 * @file    track_store.c
 * @brief   双槽 Flash 参数保存：CRC 校验、交替写入、最后提交、写后回读。
 * @note    32 字节 v2 记录兼容读取旧版 v1。PID、方向、舵机、动力统一保存；
 *          电感量程和车身居中参考不写 Flash。
 */
#include "track_hw.h"
#include <string.h>
#ifdef TRACK_IAP_REGISTER_TEST
#include "track_store_test_device.h"
#endif

#define STORE_SLOT_NONE 255U
#define IAP_READ        1U
#define IAP_PROGRAM     2U
#define IAP_ERASE       3U
#define IAP_CMD_FAIL    0x10U /* IAP_CONTR 的 bit4；bit0 不是命令失败位。 */

static uint8_t store_active = STORE_SLOT_NONE;
static uint32_t store_sequence;
static uint8_t record_a[TRACK_RECORD_BYTES];
static uint8_t record_b[TRACK_RECORD_BYTES];
static TrackStoreError store_error;

#if defined(TRACK_HOST_TEST) && !defined(TRACK_IAP_REGISTER_TEST)
extern uint8_t TrackTest_IAP(uint32_t address, uint8_t command,
                             uint8_t value, uint8_t *readback);
#define iap_command TrackTest_IAP
#else
/** @brief 操作结束后关闭 IAP，并将地址寄存器停在无效地址。 */
static void iap_idle(void)
{
    IAP_CONTR = 0;
    IAP_CMD = 0;
    IAP_TRIG = 0;
    IAP_ADDRE = 0xFF;
    IAP_ADDRH = 0xFF;
    IAP_ADDRL = 0xFF;
}

/**
 * @brief 执行一个底层 IAP 操作，只允许访问两个预留扇区。
 * @param address  IAP 偏移地址，不是 CPU 的 FFxxxx 地址。
 * @param command  1=读，2=编程，3=擦除。
 * @param value    编程时写入的字节；其他命令忽略。
 * @param readback 返回 IAP_DATA；读命令时为读到的数据。
 * @return 硬件未报告失败则返回 1。
 */
static uint8_t iap_command(uint32_t address, uint8_t command,
                           uint8_t value, uint8_t *readback)
{
    uint8_t previous_irq;
    uint8_t succeeded;

    if ((address - TRACK_STORE_SLOT_A) >= 2UL * TRACK_STORE_PAGE_SIZE ||
        command < IAP_READ || command > IAP_ERASE || track_guard_armed)
    {
        return 0;
    }
    previous_irq = EA;
    EA = 0;
    IAP_CONTR = 0x80;
    /* 使用初始化后实际系统时钟，向上取整 MHz，避免等待时间不足。 */
    IAP_TPS = (uint8_t)((system_clock + 999999UL) / 1000000UL);
    IAP_CMD = command;
    IAP_ADDRE = (uint8_t)(address >> 16);
    IAP_ADDRH = (uint8_t)(address >> 8);
    IAP_ADDRL = (uint8_t)address;
    IAP_DATA = value;
    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    succeeded = !(IAP_CONTR & IAP_CMD_FAIL);
    *readback = IAP_DATA;
    iap_idle();
    EA = previous_irq;
    TrackHW_FeedWatchdog();
    return succeeded;
}
#endif

TrackStoreError TrackStore_LastError(void)
{
    return store_error;
}

/** @brief 读取完整记录，任意字节读失败立即返回 0。 */
static uint8_t read_record(uint32_t address, uint8_t *buffer)
{
    uint8_t index;

    for (index = 0; index < TRACK_RECORD_BYTES; ++index)
    {
        if (!iap_command(address + index, IAP_READ, 0xFF, buffer + index))
        {
            store_error = TRACK_STORE_READ;
            return 0;
        }
    }
    return 1;
}

uint8_t TrackStore_Load(TrackParams *params)
{
    TrackParams params_a;
    TrackParams params_b;
    uint32_t sequence_a = 0;
    uint32_t sequence_b = 0;
    uint8_t valid_a;
    uint8_t valid_b;

    store_active = STORE_SLOT_NONE;
    store_sequence = 0;
    valid_a = read_record(TRACK_STORE_SLOT_A, record_a) &&
              TrackRecord_Decode(record_a, &params_a, &sequence_a);
    valid_b = read_record(TRACK_STORE_SLOT_B, record_b) &&
              TrackRecord_Decode(record_b, &params_b, &sequence_b);
    if (valid_b && (!valid_a || TrackSequence_Newer(sequence_b, sequence_a)))
    {
        *params = params_b;
        store_active = 1;
        store_sequence = sequence_b;
        return 1;
    }
    if (valid_a)
    {
        *params = params_a;
        store_active = 0;
        store_sequence = sequence_a;
        return 1;
    }
    /* 方案 A：仅两槽均无有效参数时，自动采用代码宏初值。 */
    TrackParams_Default(params);
    return 0;
}

uint8_t TrackStore_Save(const TrackParams *params)
{
    uint32_t address;
    uint32_t verified_sequence;
    uint8_t target_slot;
    uint8_t index;
    uint8_t ignored;
    TrackParams verified_params;

    store_error = TRACK_STORE_OK;
    if (!TrackParams_Valid(params))
    {
        store_error = TRACK_STORE_PARAMS;
        return 0;
    }
    if (track_guard_armed)
    {
        store_error = TRACK_STORE_BUSY;
        return 0;
    }
    if (store_active != STORE_SLOT_NONE)
    {
        TrackRecord_Encode(record_a, params, store_sequence);
        address = store_active ? TRACK_STORE_SLOT_B : TRACK_STORE_SLOT_A;
        if (read_record(address, record_b) &&
            !memcmp(record_a, record_b, TRACK_RECORD_BYTES))
        {
            return 1; /* 参数没有改变，不消耗一次 Flash 擦写。 */
        }
    }
    TrackHW_Stop();
    target_slot = store_active == 0 ? 1 : 0;
    address = target_slot ? TRACK_STORE_SLOT_B : TRACK_STORE_SLOT_A;
    TrackRecord_Encode(record_a, params, store_sequence + 1);
    if (!iap_command(address, IAP_ERASE, 0xFF, &ignored))
    {
        store_error = TRACK_STORE_ERASE;
        return 0;
    }
    /* 只擦写另一槽；即使中途掉电，之前的有效记录仍然保留。 */
    for (index = 0; index < TRACK_RECORD_BYTES - 1; ++index)
    {
        if (!iap_command(address + index, IAP_PROGRAM, record_a[index], &ignored))
        {
            store_error = TRACK_STORE_WRITE;
            return 0;
        }
    }
    if (!iap_command(address + 31, IAP_PROGRAM, TRACK_RECORD_COMMIT, &ignored))
    {
        store_error = TRACK_STORE_WRITE;
        return 0;
    }
    if (!read_record(address, record_b))
    {
        return 0;
    }
    if (memcmp(record_a, record_b, TRACK_RECORD_BYTES) ||
        !TrackRecord_Decode(record_b, &verified_params, &verified_sequence))
    {
        store_error = TRACK_STORE_VERIFY;
        return 0;
    }
    store_active = target_slot;
    store_sequence = verified_sequence;
    store_error = TRACK_STORE_OK;
    return 1;
}
