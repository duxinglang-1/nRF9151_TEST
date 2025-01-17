/****************************************Copyright (c)************************************************
** File Name:			    LCD_VG6432TSWPG28_SSD1315.h
** Descriptions:			The LCD_VG6432TSWPG28_SSD1315 screen drive head file
** Created By:				xie biao
** Created Date:			2025-01-16
** Modified Date:      		2025-01-16
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr\types.h>
#include "lcd.h"

#ifdef LCD_VG6432TSWPG28_SSD1315

//------------------------------------------------------
#define COL 		64			//��
#define ROW 		32			//��
#define PAGE_H		8			//ÿ��page���ص��Ϊ8
#define PAGE_MAX	4			//�ܹ�4��page

#define LCD_TYPE_SPI			//SPI

//LCD�Ļ�����ɫ�ͱ���ɫ	   
extern uint16_t  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern uint16_t  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ

#define LCD_I2C_ADDR	(0x78 >> 1)
#define	LCD_RST		13

//LEDK(LED����)
#define LEDK	31
#define LEDA	14

#define X_min 0x0043		 //TP���Է�Χ��������
#define X_max 0x07AE
#define Y_min 0x00A1
#define Y_max 0x0759

#define LCD_DATA_LEN ((COL*ROW)/8)

//------------------------------------------------------

extern uint8_t lcd_data_buffer[2*LCD_DATA_LEN];

extern void BlockWrite(unsigned int x,unsigned int y,unsigned int w,unsigned int h);
extern void WriteOneDot(uint16_t color);
extern void Write_Data(uint8_t i);
extern void LCD_Init(void);
extern void LCD_Clear(uint16_t color);
extern void LCD_SleepIn(void);
extern void LCD_SleepOut(void);
extern void LCD_ResetBL_Timer(void);
extern void LCD_Set_BL_Mode(LCD_BL_MODE mode);
extern LCD_BL_MODE LCD_Get_BL_Mode(void);
#endif/*LCD_R108101_GC9307*/
