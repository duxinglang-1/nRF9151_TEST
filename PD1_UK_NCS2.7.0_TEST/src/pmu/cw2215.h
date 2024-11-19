/****************************************Copyright (c)************************************************
** File Name:			    cw2215.h
** Descriptions:			cw2215 sensor message process head file
** Created By:				xie biao
** Created Date:			2024-09-10
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __CW2215_H__
#define __CW2215_H__

#include <stdint.h>
#include <zephyr/kernel.h>
#include "pmu.h"

#ifdef PMU_SENSOR_CW221X_CW630X

#define BATTERY_SOC_GAUGE
#define BATTERY_NTC_CHECK

#define CW2215_I2C_ADDR		0x64
#define CW2215_CHIP_ID		0xA0

#define CW2215_MARK			0x80
#define CW2217_MARK			0x40
#define CW2218_MARK			0x00

#define CW2215_SLEEP_COUNTS		50

#define IC_INITIALIZING			0x00
#define IC_VOL_CUR_READY        0x04
#define IC_TEMP_READY           0x08
#define IC_SOC_READY           	0x0C

#define CW2215_SOC_IRQ_VALUE		0x7F
#define CW2215_PROFILE_UPDATE_FLAG	0x80

#define USER_RSENSE 			(10 * 1000)  //rsense * 1000
#define SIZE_OF_PROFILE			80

typedef enum
{
	REG_CHIP_ID		= 0x00,
	REG_VCELL_H		= 0x02,
	REG_VCELL_L		= 0x03,
	REG_SOC_INT		= 0x04,
	REG_SOC_DECIMAL	= 0x05,
	REG_TEMP		= 0x06,
	REG_MODE_CONFIG	= 0x08,
	REG_GPIO_CONFIG	= 0x0A,
	REG_SOC_ALERT	= 0x0B,
	REG_TEMP_MAX	= 0x0C,
	REG_TEMP_MIN	= 0x0D,
	REG_CURRENT_H	= 0x0E,
	REG_CURRENT_L	= 0x0F,
	REG_T_HOST_H	= 0xA0,
	REG_T_HOST_L	= 0xA1,
	REG_USER_CONF	= 0xA2,
	REG_CYCLE_H		= 0xA4,
	REG_CYCLE_L		= 0xA5,
	REG_SOH			= 0xA6,
	REG_IC_STATE	= 0xA7,
	REG_STB_CUR_H	= 0xA8,
	REG_STB_CUR_L	= 0xA9,
	REG_FW_VERSION	= 0xAB,
	REG_BAT_PROFILE	= 0x10,
	REG_MAX			= 0xFF
}CW2215_REG;

typedef enum
{
	CONFIG_RESTART		= 0x30,
	CONFIG_ACTIVE		= 0x00,
	CONFIG_SLEEP		= 0xF0,
	CONFIG_MAX			= 0xFF
}CW2215_CONFIG_MODE;

typedef enum
{
	STATE_NORMAL,
	STATE_NOT_ACTIVE,
	STATE_PROFILE_NOT_READY,
	STATE_PROFILE_NEED_UPDATE
}CW2215_STATE;

#endif/*PMU_SENSOR_CW2215*/
#endif/*__CW2215_H__*/
