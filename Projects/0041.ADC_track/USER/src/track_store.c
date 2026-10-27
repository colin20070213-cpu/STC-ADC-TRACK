/**
 * @file    track_store.c
 * @brief   双槽 Flash 参数保存：CRC 校验、交替写入、最后提交、写后回读。
 * @note    32 字节 v2 基础记录后追加 48 字节 SPD2 扩展；兼容旧版 v1/v2/SPD1。PID、权重、方向、舵机、动力统一保存；
 *          电感量程和车身居中参考不写 Flash。
 */
#include "track_hw.h"
#include <string.h>
#ifdef TRACK_IAP_REGISTER_TEST
#include "track_store_test_device.h"
#endif

#define STORE_SLOT_NONE 255U
#define STORE_TOTAL_BYTES TRACK_STORE_RECORD_BYTES /* 80B，扩展支持 32 位速度增益。 */
#define IAP_READ        1U
#define IAP_PROGRAM     2U
#define IAP_ERASE       3U
#define IAP_CMD_FAIL    0x10U /* IAP_CONTR 的 bit4；bit0 不是命令失败位。 */

static uint8_t store_active = STORE_SLOT_NONE;
static uint32_t store_sequence;
static uint8_t record_a[STORE_TOTAL_BYTES];
static uint8_t record_b[STORE_TOTAL_BYTES];
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

static void store_u16(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8);
}

static uint16_t load_u16(const uint8_t *buffer)
{
    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

/** @brief 明确编码 32 位小端字段，不直接保存 C 结构体内存。 */
static void store_u32(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8);
    buffer[2] = (uint8_t)(value >> 16);
    buffer[3] = (uint8_t)(value >> 24);
}

static uint32_t load_u32(const uint8_t *buffer)
{
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

/** @brief SPD2 支持 32 位速度增益；与基础序号/CRC 绑定并最后提交基础标记。 */
static void encode_store_record(uint8_t *record, const TrackParams *params, uint32_t sequence)
{
    TrackRecord_Encode(record, params, sequence);
    memset(record + 32, 0xff, STORE_TOTAL_BYTES - 32);
    memcpy(record + 32, "SPD2", 4);
    memcpy(record + 36, record + 4, 4);
    store_u16(record + 40, params->legacy_speed_base);
    store_u16(record + 42, params->legacy_speed_curve);
    store_u32(record + 44, params->speed_kp100);
    store_u32(record + 48, params->speed_ki100);
    store_u32(record + 52, params->speed_kd100);
    memcpy(record + 56, record + 28, 2);
    store_u16(record + 58, params->c4_a);
    store_u16(record + 60, params->c4_b);
    store_u16(record + 62, params->c4_c);
    memcpy(record + 64, "PWM1", 4); /* offset20/22 为 PWM 刻度指令，保留原数字。 */
    store_u16(record + 76, TrackCRC16(record + 32, 44));
    record[79] = 0x5a;
}

/** @brief 兼容读取旧 SPD1 的 16 位增益及可选 C4 预留区，不改旧数据标度。 */
static uint8_t decode_spd1(const uint8_t *record, TrackParams *candidate)
{
    uint8_t index;
    uint8_t legacy_weights = 1;

    if (memcmp(record + 36, record + 4, 4) || memcmp(record + 50, record + 28, 2) ||
        record[63] != 0x5a || load_u16(record + 60) != TrackCRC16(record + 32, 28)) return 0;
    candidate->legacy_speed_base = load_u16(record + 40);
    candidate->legacy_speed_curve = load_u16(record + 42);
    candidate->speed_kp100 = load_u16(record + 44);
    candidate->speed_ki100 = load_u16(record + 46);
    candidate->speed_kd100 = load_u16(record + 48);
    for (index = 52; index < 60; ++index)
    {
        if (record[index] != 0xff) legacy_weights = 0;
    }
    if (!legacy_weights)
    {
        if (record[58] != 'C' || record[59] != '4') return 0;
        candidate->c4_a = load_u16(record + 52);
        candidate->c4_b = load_u16(record + 54);
        candidate->c4_c = load_u16(record + 56);
    }
    return 1;
}

/** @brief 识别旧基础记录、SPD1、SPD2；损坏扩展不能当作缺失参数。 */
static uint8_t decode_store_record(const uint8_t *record, TrackParams *params, uint32_t *sequence)
{
    TrackParams candidate;
    uint32_t candidate_sequence;
    uint8_t index;
    uint8_t legacy = 1;

    if (!TrackRecord_Decode(record, &candidate, &candidate_sequence)) return 0;
    for (index = 32; index < STORE_TOTAL_BYTES; ++index)
    {
        if (record[index] != 0xff) legacy = 0;
    }
    if (!legacy)
    {
        if (!memcmp(record + 32, "SPD1", 4))
        {
            if (!decode_spd1(record, &candidate)) return 0;
        }
        else if (!memcmp(record + 32, "SPD2", 4))
        {
            if (memcmp(record + 36, record + 4, 4) || memcmp(record + 56, record + 28, 2) ||
                record[79] != 0x5a || load_u16(record + 76) != TrackCRC16(record + 32, 44)) return 0;
            if (memcmp(record + 64, "PWM1", 4) && memcmp(record + 64, "PPS1", 4))
            {
                /* 按用户要求沿用旧 PPS1/无标记记录的数字；未知标记仍拒绝。 */
                for (index = 64; index < 76; ++index)
                {
                    if (record[index] != 0xff) return 0;
                }
            }
            candidate.legacy_speed_base = load_u16(record + 40);
            candidate.legacy_speed_curve = load_u16(record + 42);
            candidate.speed_kp100 = load_u32(record + 44);
            candidate.speed_ki100 = load_u32(record + 48);
            candidate.speed_kd100 = load_u32(record + 52);
            candidate.c4_a = load_u16(record + 58);
            candidate.c4_b = load_u16(record + 60);
            candidate.c4_c = load_u16(record + 62);
        }
        else return 0;
        if (!TrackParams_Valid(&candidate)) return 0;
    }
    *params = candidate;
    *sequence = candidate_sequence;
    return 1;
}

/** @brief 读取完整记录，任意字节读失败立即返回 0。 */
static uint8_t read_record(uint32_t address, uint8_t *buffer)
{
    uint8_t index;

    for (index = 0; index < STORE_TOTAL_BYTES; ++index)
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
              decode_store_record(record_a, &params_a, &sequence_a);
    valid_b = read_record(TRACK_STORE_SLOT_B, record_b) &&
              decode_store_record(record_b, &params_b, &sequence_b);
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
        encode_store_record(record_a, params, store_sequence);
        address = store_active ? TRACK_STORE_SLOT_B : TRACK_STORE_SLOT_A;
        if (read_record(address, record_b) &&
            !memcmp(record_a, record_b, STORE_TOTAL_BYTES))
        {
            return 1; /* 参数没有改变，不消耗一次 Flash 擦写。 */
        }
    }
    TrackHW_Stop();
    target_slot = store_active == 0 ? 1 : 0;
    address = target_slot ? TRACK_STORE_SLOT_B : TRACK_STORE_SLOT_A;
    encode_store_record(record_a, params, store_sequence + 1);
    if (!iap_command(address, IAP_ERASE, 0xFF, &ignored))
    {
        store_error = TRACK_STORE_ERASE;
        return 0;
    }
    /* 只擦写另一槽；即使中途掉电，之前的有效记录仍然保留。 */
    for (index = 0; index < STORE_TOTAL_BYTES; ++index)
    {
        if (index == 31) continue; /* 原有效标记仍最后写，保证基础参数和扩展同次提交。 */
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
    if (memcmp(record_a, record_b, STORE_TOTAL_BYTES) ||
        !decode_store_record(record_b, &verified_params, &verified_sequence))
    {
        store_error = TRACK_STORE_VERIFY;
        return 0;
    }
    store_active = target_slot;
    store_sequence = verified_sequence;
    store_error = TRACK_STORE_OK;
    return 1;
}
