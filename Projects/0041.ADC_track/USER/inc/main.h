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
 * @file       main.h
 * @brief      主函数
 * @author     胖胖
 * @version    v1.0
 * @note       NULL
 * @date       2023-07-26
 ********************************************************************************************************************/

#ifndef TRACK_MAIN_H
#define TRACK_MAIN_H

/* 保留原模板的常用库入口；库文件及工程引用不可因暂未使用而删除。 */
#include <stdio.h>
#include <string.h>
#include "isr.h"
#include "common.h"
#include "STC32Gxx.h"

#include "CNU_PIE_GPIO.h"
#include "CNU_PIE_TIMER.h"
#include "CNU_PIE_EXTI.h"
#include "CNU_PIE_ADC.h"
#include "CNU_PIE_I2C.h"
#include "CNU_PIE_SPI.h"
#include "CNU_PIE_PWM.h"
#include "CNU_PIE_WDog.h"
#include "CNU_PIE_UART.h"
#include "CNU_PIE_FIFO.h"

#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "LCD.h"
#include "OLED.h"
#include "Font.h"
#include "Encoder.h"
#include "remote_control.h"
#include "nrf24l01.h"

/* Ci24R1.h 与 nrf24l01.h 为接口重叠的可选实现，均保留在 boards 分组中。
 * 原模板选用 nrf24l01，不在此同时包含 Ci24R1，避免同名接口类型冲突。
 */
#endif
