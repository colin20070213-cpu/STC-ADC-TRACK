#ifndef __LCD_H_
#define __LCD_H_

#include "CNU_PIE_GPIO.h"

//--------------OLED参数定义---------------------
#define PAGE_SIZE    8
#define XLevelL		0x00
#define XLevelH		0x10
#define YLevel       0xB0
#define	Brightness	 0xFF 
#define WIDTH 	     128
#define HEIGHT 	     64	


#define OLED_CMD     0	
#define OLED_DATA    1	

#define SET   1
#define RESET 0

#define OLED_CS_PORT  GPIO_P2
#define OLED_DC_PORT  GPIO_P2
#define OLED_RST_PORT GPIO_P2
#define OLED_CLK_PORT GPIO_P2
#define OLED_D1_PORT  GPIO_P2

#define OLED_CS_PIN  GPIO_Pin_2
#define OLED_DC_PIN  GPIO_Pin_6
#define OLED_RST_PIN GPIO_Pin_4
#define OLED_CLK_PIN GPIO_Pin_5
#define OLED_D1_PIN  GPIO_Pin_3

//-----------------OLED引脚操作---------------- 
#define OLED_CS_Clear()  GPIO_Write_Bit(OLED_CS_PORT , OLED_CS_PIN , RESET)
#define OLED_CS_Set()  GPIO_Write_Bit(OLED_CS_PORT , OLED_CS_PIN , SET)

#define OLED_DC_Clear()  GPIO_Write_Bit(OLED_DC_PORT , OLED_DC_PIN , RESET)
#define OLED_DC_Set()  GPIO_Write_Bit(OLED_DC_PORT , OLED_DC_PIN , SET)
 					   
#define OLED_RST_Clear()  GPIO_Write_Bit(OLED_RST_PORT , OLED_RST_PIN , RESET)
#define OLED_RST_Set()  GPIO_Write_Bit(OLED_RST_PORT , OLED_RST_PIN , SET)

#define OLED_CLK_Clear()  GPIO_Write_Bit(OLED_CLK_PORT , OLED_CLK_PIN , RESET)
#define OLED_CLK_Set()  GPIO_Write_Bit(OLED_CLK_PORT , OLED_CLK_PIN , SET)

#define OLED_D1_Clear()  GPIO_Write_Bit(OLED_D1_PORT , OLED_D1_PIN , RESET)
#define OLED_D1_Set()  GPIO_Write_Bit(OLED_D1_PORT , OLED_D1_PIN , SET)

/** @brief 使用页寻址初始化 OLED，并显示 W.PIE 一秒后清屏。 */
void LCD_Init(void);
/** @brief 定位到列 column(0~127)、页 page(0~7)，不是像素行坐标。 */
void LCD_Set_Pos(uint8_t column, uint8_t page);
/** @brief 发送一个数据字节，控制当前列的竖直 8 个像素。 */
void LCD_WrDat(unsigned char value);
/** @brief 发送一个控制命令，前台调用。 */
void LCD_WrCmd(unsigned char command);
/** @brief 全屏填充，同一个字节重复写入全部 8 页。 */
void LCD_Fill(unsigned char value);
/** @brief 清空显存。 */
void LCD_CLS(void);
/** @brief 使用原库点阵写入 W.PIE 图案，不执行延时。 */
void Draw_WPIELogo(void);
void LCD_P6x8Str(unsigned char x,unsigned char y,char ch[]);
void LCD_P8x16Str(unsigned char x,unsigned char y,char ch[]);
void LCD_PrintU16(unsigned char x,unsigned char y,unsigned int num);
void LCD_PrintFloat(unsigned char x,unsigned char y,float num,unsigned int N);
#endif