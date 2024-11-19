#ifndef __MAX20353_H__
#define __MAX20353_H__

#include "pmu.h"

#ifdef PMU_SENSOR_MAX20353

#define BATTERY_VOLTAGE_LOW_NOTIFY	(3.55)
#define BATTERY_VOLTAGE_SHUTDOWN	(3.40)

#define MOTOR_TYPE_ERM	//转子马达
//#define MOTOR_TYPE_LRA	//线性马达

#define BATTERY_SOC_GAUGE	//xb add 20201124 电量计功能的代码
#define BATTERT_NTC_CHECK	//xb add 20210106 电池增加NTC温度检测

#define GPIO_ACT_I2C

#ifdef BATTERY_SOC_GAUGE
#define VERIFY_AND_FIX 1
#define LOAD_MODEL !(VERIFY_AND_FIX)
#define EMPTY_ADJUSTMENT		0
#define FULL_ADJUSTMENT			100
#define RCOMP0					64
#define TEMP_COUP				(-1.96875)
#define TEMP_CODOWN				(-7.875)
#define TEMP_CODOWNN10			(-3.90625)
#define OCVTEST					57984
#define SOCCHECKA				113
#define SOCCHECKB				115
#define BITS					18
#define RCOMPSEG				0x0100
#define INI_OCVTEST_HIGH_BYTE 	(OCVTEST>>8)
#define INI_OCVTEST_LOW_BYTE	(OCVTEST&0x00ff)
#endif

#endif/*PMU_SENSOR_MAX20353*/
#endif/*__MAX20353_H__*/
