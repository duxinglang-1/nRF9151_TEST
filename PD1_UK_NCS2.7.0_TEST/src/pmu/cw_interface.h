/****************************************Copyright (c)************************************************
** File Name:			    cw_interface.h
** Descriptions:			cw sensor drive interface head file
** Created By:				xie biao
** Created Date:			2024-09-12
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __CW_INTERFACE_H__
#define __CW_INTERFACE_H__

#ifdef PMU_SENSOR_CW221X_CW630X

#include <stdint.h>
#include <zephyr/kernel.h>
#include "cw2215.h"
#include "cw6307.h"

//#define GPIO_ACT_I2C

#define CW_NO_ERROR  	0
#define CW_ERROR      	-1

extern bool CW_I2C_Init(void);
extern void CW_Interface_Init(void);

extern void CW_Delay_ms(unsigned int dly);
extern void CW_Delay_us(unsigned int dly);

extern int CW2215_WriteReg(CW2215_REG reg, uint8_t value);
extern int CW2215_WriteRegMulti(CW2215_REG reg, uint8_t *value, uint8_t len);
extern int CW2215_ReadReg(CW2215_REG reg, uint8_t *value);
extern int CW2215_ReadRegMulti(CW2215_REG reg, uint8_t *value, uint8_t len);

extern int CW6307_WriteReg(CW2215_REG reg, uint8_t value);
extern int CW6307_WriteRegMulti(CW2215_REG reg, uint8_t *value, uint8_t len);
extern int CW6307_ReadReg(CW2215_REG reg, uint8_t *value);
extern int CW6307_ReadRegMulti(CW2215_REG reg, uint8_t *value, uint8_t len);

#endif/*PMU_SENSOR_CW221X_CW630X*/
#endif/*__CW_INTERFACE_H__*/