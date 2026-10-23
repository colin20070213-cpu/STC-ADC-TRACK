/*
 * OLED 总线回归测试：编译真实 LCD.c，采集 GPIO 时钟上升沿并还原命令。
 * 按 SSD1306 页寻址规则检查显存；不替代实机的接线、电平和时序验证。
 */
#include "LCD.h"
#include "Font.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

extern const unsigned char gImage_Wpie[1024];

static uint8_t ram[8][128];
static uint8_t pins = 0xff;
static uint8_t shift_byte, bit_count, transfer_dc;
static uint8_t mode = 2, page, column, pending_command, pending_parameters;
static unsigned command_count, logo_checks;

/** @brief 模拟控制器命令解码，只实现本库使用的命令及其参数长度。 */
static void receive_command(uint8_t value)
{
    ++command_count;
    if (pending_parameters)
    {
        if (pending_command == 0x20)
        {
            mode = value & 3;
        }
        --pending_parameters;
        return;
    }
    switch (value)
    {
        case 0x20: case 0x81: case 0xa8: case 0xd3:
        case 0xd5: case 0xd9: case 0xda: case 0xdb: case 0x8d:
            pending_command = value;
            pending_parameters = 1;
            return;
        default:
            break;
    }
    /* 页地址及列高低位命令仅在页寻址方式下定义。 */
    if (mode == 2)
    {
        if ((value & 0xf8) == 0xb0) page = value & 7;
        else if ((value & 0xf0) == 0x10) column = (column & 0x0f) | ((value & 7) << 4);
        else if ((value & 0xf0) == 0) column = (column & 0x70) | (value & 0x0f);
    }
}

void GPIO_Init(uint8_t port, uint8_t pin, uint8_t output_mode)
{
    assert(port == GPIO_P2 && output_mode == GPIO_OUT_PP);
    assert(pin & (GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6));
}

/** @brief 在真实驱动的 GPIO 写操作中捕获四线 SPI，校验每帧完整的八位。 */
void GPIO_Write_Bit(uint8_t port, uint8_t pin, uint8_t value)
{
    uint8_t previous = pins;
    assert(port == GPIO_P2);
    if (value) pins |= pin;
    else pins &= (uint8_t)~pin;

    if (!(pins & GPIO_Pin_4)) return;
    if (!(previous & GPIO_Pin_4))
    {
        mode = 2;
        page = column = pending_parameters = bit_count = 0;
    }
    if ((pins & GPIO_Pin_2) && !(previous & GPIO_Pin_2))
    {
        assert(bit_count == 8);
        bit_count = 0;
    }
    if (!(pins & GPIO_Pin_2) && (pins & GPIO_Pin_5) && !(previous & GPIO_Pin_5))
    {
        if (!bit_count)
        {
            shift_byte = 0;
            transfer_dc = pins & GPIO_Pin_6;
        }
        assert(bit_count < 8 && transfer_dc == (pins & GPIO_Pin_6));
        shift_byte = (uint8_t)((shift_byte << 1) | !!(pins & GPIO_Pin_3));
        if (++bit_count == 8)
        {
            if (transfer_dc)
            {
                assert(!pending_parameters);
                ram[page][column] = shift_byte;
                column = (column + 1) & 127;
                if (mode == 0 && !column) page = (page + 1) & 7;
            }
            else receive_command(shift_byte);
        }
    }
}

void Ms_Delay(uint16_t ms)
{
    if (ms == 1000)
    {
        assert(mode == 2);
        assert(memcmp(ram, gImage_Wpie, sizeof(ram)) == 0);
        ++logo_checks;
    }
}

static uint8_t pattern(unsigned index)
{
    return (uint8_t)((index * 29u) ^ (index >> 3) ^ 0xa5u);
}

int main(void)
{
    unsigned i, before;
    uint8_t expected[1024];
    char text[] = "Hello";

    memset(ram, 0xa5, sizeof(ram));
    LCD_Init();
    assert(logo_checks == 1 && mode == 2 && !pending_parameters);
    memset(expected, 0, sizeof(expected));
    assert(memcmp(ram, expected, sizeof(ram)) == 0);

    /* 每列单独定位：覆盖偶数列、奇数列和所有页，防止列地址重叠。 */
    for (i = 0; i < sizeof(expected); ++i)
    {
        LCD_Set_Pos((uint8_t)(i % 128), (uint8_t)(i / 128));
        LCD_WrDat(expected[i] = pattern(i));
    }
    assert(memcmp(ram, expected, sizeof(ram)) == 0);

    LCD_Fill(0xa5);
    memset(expected, 0xa5, sizeof(expected));
    assert(memcmp(ram, expected, sizeof(ram)) == 0);
    LCD_CLS();
    memset(expected, 0, sizeof(expected));
    LCD_P6x8Str(24, 2, text);
    for (i = 0; i < strlen(text); ++i)
    {
        memcpy(expected + 2 * 128 + 24 + i * 6, F6x8[text[i] - 32], 6);
    }
    assert(memcmp(ram, expected, sizeof(ram)) == 0);

    /* 模拟应用的八字节分片刷新，验证完整 1024 字节的实际串行输出。 */
    for (i = 0; i < sizeof(expected); ++i) expected[i] = pattern(i + 17);
    for (i = 0; i < sizeof(expected); i += 8)
    {
        unsigned j;
        LCD_Set_Pos((uint8_t)(i % 128), (uint8_t)(i / 128));
        for (j = 0; j < 8; ++j) LCD_WrDat(expected[i + j]);
    }
    assert(memcmp(ram, expected, sizeof(ram)) == 0);
    before = command_count;
    LCD_Set_Pos(128, 0);
    LCD_Set_Pos(0, 8);
    assert(command_count == before);
    puts("OLED GPIO/SPI: startup logo, addressing, fill, text and sliced refresh passed");
    return 0;
}
