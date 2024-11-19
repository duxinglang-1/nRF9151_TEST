/****************************************Copyright (c)************************************************
** File Name:			    dps368.h
** Descriptions:			dps368 interface process head file
** Created By:				xie biao
** Created Date:			2024-06-18
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __DPS368_H__
#define __DPS368_H__

#define GPIO_ACT_I2C

#define PRESSURE_EINT		18
#define PRESSURE_SCL		17
#define PRESSURE_SDA		16

#define DPS368_CHIP_ID		0x10
#define DPS368_I2C_ADDRESS	0x77

#define DPS368_NO_ERROR   0
#define DPS368_ERROR     -1

typedef enum
{
  	//RegisterName    Addr. 	  bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 				Reset State
	REG_PSR_B2 		= 0x00,		//PSR[23:16]																0x00
	REG_PSR_B1 		= 0x01,		//PSR[15:8]																	0x00
	REG_PSR_B0 		= 0x02,		//PSR[7:0]																	0x00
	REG_TMP_B2 		= 0x03,		//TMP[23:16]																0x00
	REG_TMP_B1 		= 0x04,		//TMP[15:8]																	0x00
	REG_TMP_B0		= 0x05,		//TMP[7:0]																	0x00
	REG_PRS_CFG 	= 0x06,		// - | PM_RATE[2:0] | PM_PRC [3:0]											0x00
	REG_TMP_CFG 	= 0x07,		//TMP_EXT | TMP_RATE[2:0] | TM_PRC [3:0]  									0x00
	REG_MEAS_CFG 	= 0x08,		//COEF_RDY | SENSOR_RDY | TMP_RDY | PRS_RDY | - | MEAS_CRTL[2:0]			0xC0
	REG_CFG		 	= 0x09,		//INT_HL | INT_SEL[2:0] | TMP_SHIFT_EN | PRS_SHIFT_EN | FIFO_EN | SPI_MODE 	0x00
	REG_INT_STS 	= 0x0A,		// ----- | INT_FIFO_FULL | INT_TMP | INT_PRS								0x00
	REG_FIFO_STS 	= 0x0B, 	// ------ | FIFO_FULL | FIFO_EMPTY 											0x00
	REG_RESET 		= 0x0C,		//FIFO_FLUSH | --- | SOFT_RST[3:0]											0x00
	REG_ID  		= 0x0D, 	//REV_ID[3:0] | PROD_I[3:0]													0x10
	REG_COEF 		= 0x10,		//From 0x10 to 0x21, < see register description > 							0xXX
	REG_COEF_SRCE 	= 0x28,		//TMP_COEF_SRCE | -------													0xXX
	REG_MAX 		= 0xFF
}DPS368_REG;

typedef enum
{
	//Standby Mode
	MEAS_STANDBY		= 0b000, 	//- Idle / Stop background measurement
	
	//Command Mode
	MEAS_CMD_PSR		= 0b001,	// - Pressure measurement
	MEAS_CMD_TMP		= 0b010,	// - Temperature measurement
	
	//Background Mode
	MEAS_CONTI_PRS		= 0b101,	// - Continous pressure measurement
	MEAS_CONTI_TMP		= 0b110,	// - Continous temperature measurement
	MEAS_CONTI_PRS_TMP	= 0b111,	// - Continous pressure and temperature measurement
}DPS368_MEAS_CTRL;

typedef struct
{
	uint8_t prc		:4;
	uint8_t rate	:3;
	uint8_t ext		:1;
}dps368_tmp_t;

typedef struct
{
	uint8_t prc		:4;
	uint8_t rate	:3;
	uint8_t na		:1;
}dps368_psr_t;

typedef struct
{
	uint8_t ctrl	:3;
	uint8_t na		:5;
}dps368_meas_t;

typedef struct
{
	uint8_t spi_mode	:1;	
	uint8_t fifo_en		:1;
	uint8_t p_shift		:1;
	uint8_t t_shift		:1;
	uint8_t int_prs		:1;
	uint8_t int_tmp		:1;
	uint8_t	int_fifo	:1;
	uint8_t int_hl		:1;
}dps368_int_t;

typedef struct
{
	dps368_tmp_t tmp_cfg;
	dps368_psr_t prs_cfg;
	dps368_meas_t meas_cfg;
	dps368_int_t int_cfg;
}dps368_settings_t;

extern dps368_settings_t dps368_settings;

extern bool DPS368_Init(void);
extern void DPS368_Start(DPS368_MEAS_CTRL work_mode);
extern void DPS368_Stop(void);
extern void pressure_start(void);
extern void pressure_stop(void);
extern bool GetPressure(float *prs);

#endif/*__DPS368_H__*/
