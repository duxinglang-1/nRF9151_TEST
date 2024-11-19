/****************************************Copyright (c)************************************************
** File Name:			    cw2215.h
** Descriptions:			cw2215 sensor message process head file
** Created By:				xie biao
** Created Date:			2024-09-10
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __CW6307_H__
#define __CW6307_H__

#include <stdint.h>
#include <zephyr/kernel.h>
#include "pmu.h"

#ifdef PMU_SENSOR_CW221X_CW630X

#define CW6307_I2C_ADDR	0x0B
#define CW6307_CHIP_ID	0x34

#define CW_SET_ENABLE  1
#define CW_SET_DISABLE 0

#define VBUS_INPUT_VOL_LIMIT_OFFSET 4200
#define VBUS_INPUT_VOL_LIMIT_MAX    4950
#define VBUS_INPUT_VOL_LIMIT_STEP   50

#define VBUS_REGUALATION_VOL_OFFSET 4200
#define VBUS_REGUALATION_VOL_MAX    4950
#define VBUS_REGUALATION_VOL_STEP   50

#define VBUS_INPUT_CUR_LIMIT_OFFSET 50
#define VBUS_INPUT_CUR_LIMIT_MAX    500
#define VBUS_INPUT_CUR_LIMIT_STEP   30

#define CV_OFFSET  3850*10
#define CV_MAX     4600*10
#define CV_STEP    125

#define RECHG_VOL_200MV 1
#define RECHG_VOL_100MV 0

#define PRECHG_TO_CCCHG_VOL_3000 1
#define PRECHG_TO_CCCHG_VOL_2800 0

#define IC_INTERNAL_MAX_THERMAL_OFFSET 60
#define IC_INTERNAL_MAX_THERMAL_MAX    120
#define IC_INTERNAL_MAX_THERMAL_STEP   20

#define FCCCHG_CUR_OFFSET 10*100
#define FCCCHG_CUR_MAX    470*100
#define FCCCHG_CUR_STEP1  125

#define PRECHG_AND_TCC_CUR_OFFSET 25 /*2.5% of fast charge current*/
#define PRECHG_AND_TCC_CUR_MAX    200
#define PRECHG_AND_TCC_CUR_STEP   25

#define CHG_TIME_OUT_TIMER_OFFSET 0
#define CHG_TIME_OUT_TIMER_MAX    12
#define CHG_TIME_OUT_TIMER_STEP   4

#define CW_NTC_SELECT_10K		0
#define CW_NTC_SELECT_100K		1
#define CW_NTC_INTERNAL_PULL_UP	0
#define CW_NTC_EXTERNAL_PULL_UP	1

#define BAT_OFF_DELAY_TIME_MIN    1
#define BAT_OFF_DELAY_TIME_MAX    8

#define CW_VSYS_RESET_DELAY_TIME_2S	0
#define CW_VSYS_RESET_DELAY_TIME_4S	1

#define INT_BTN_HOLD_TIME_OFFSET  8
#define INT_BTN_HOLD_TIME_MAX     20
#define INT_BTN_HOLD_TIME_STEP    4

#define WATCHDOG_TIMER_OFFSET  30
#define WATCHDOG_TIMER_MAX     120
#define WATCHDOG_TIMER_STEP    30

#define UVLO_VOL_OFFSET        2500
#define UVLO_VOL_MAX           3200
#define UVLO_VOL_STEP          100

#define VSYS_OVER_CUR_PROTECTION_CUR_OFFSET 100
#define VSYS_OVER_CUR_PROTECTION_CUR_MAX    1600
#define VSYS_OVER_CUR_PROTECTION_CUR_STEP   100

typedef enum
{
	REG_VERSION		= 0x00,
	REG_VBUS_VOLT	= 0x01,
	REG_VBUS_CUR	= 0x02,
	REG_CHG_VOLT	= 0x03,
	REG_CHG_CUR1	= 0x04,
	REG_CHG_CUR2	= 0x05,
	REG_SAFETY		= 0x06,
	REG_CONFIG		= 0x07,
	REG_SAFETY_CFG	= 0x08,
	REG_SAFETY_CFG2	= 0x09,
	REG_INT_SET		= 0x0A,
	REG_INT_SRC		= 0x0B,
	REG_IC_STATUS	= 0x0D,
	REG_IC_STATUS2	= 0x0E,
	REG_END			= 0xFF
}CW6307_REG;

#endif/*PMU_SENSOR_CW221X_CW630X*/
#endif/*__CW6307_H__*/