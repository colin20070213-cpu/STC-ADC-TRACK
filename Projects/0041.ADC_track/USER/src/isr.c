/*********************************************************************************************************************

 *     COPYRIGHT NOTICE

 *     Copyright (c) 2023,CNU_W.PIE

 *     All rights reserved.

 *

 *     除注明出处外，以下所有内容版权均属胖胖个人所有，未经允许，不得用于商业用途，

 *     修改内容时必须保留PP的版权声明。

 *     Except where indicated, the copyright of all the contents below is owned by PP

 *     and can not be used for commercial purposes without permission.

 *     The copyright notice of PP must be preserved when modifying the content.

 *

 * @file       isr.c

 * @brief      中断服务

 * @author     胖胖

 * @version    v1.0

 * @note       NULL

 * @date       2023-07-26

 ********************************************************************************************************************/

#include "track_hw.h"

/** @brief 1ms 时基：只处理计时、按键和截止保护。 */
void TM0_Isr(void) interrupt 1
{
    TF0 = 0;
    TrackHW_TickISR();
}
/** @brief 串口发送队列推进；接收字节只读走，不接受远程控制。 */
void UART1_Isr(void) interrupt 4
{
    unsigned char ignored;
    if (TI)
    {
        TI = 0;
        TrackUART_TxISR();
    }
    if (RI)
    {
        ignored = SBUF;
        RI = 0;
    }
}
