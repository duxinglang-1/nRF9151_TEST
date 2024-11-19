/****************************************Copyright (c)************************************************
** File Name:			    cw6307.c
** Descriptions:			cw6307 sensor message process source file
** Created By:				xie biao
** Created Date:			2024-09-12
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "pmu.h"
#include "cw_interface.h"
#include "logger.h"

#ifdef PMU_SENSOR_CW221X_CW630X

#define BIT(num) (uint8_t)num

static uint8_t HardwareID;
static uint8_t FirmwareID;

static bool cw_crg_check_ok = false;


/*static function, PointReg: Register location, bit_start: The start bit you want set, bit_num : How many bit do you want set, val : value */
static void CW6307_SetRegBit(uint8_t Reg, uint8_t bit_start, uint8_t bit_num, uint8_t val)
{
    uint8_t reg_val,clear_zero = 0;
	
    CW6307_ReadReg(Reg, &reg_val);
    
    val = val<<(bit_start - (bit_num - 1));
    for(;bit_num>0;bit_num--)
	{
        clear_zero = clear_zero | (1<<bit_start);
        bit_start--;
    }
	
    clear_zero = ~clear_zero;
    reg_val = reg_val & clear_zero;
    reg_val = reg_val | val;
    
    CW6307_WriteReg(Reg, reg_val);
}

void CW6307_SetHiZ(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_VBUS_VOLT, BIT(7), 1, enable);
}

void CW6307_SetChgSwitch(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

    CW6307_SetRegBit(REG_VBUS_VOLT, BIT(6), 1, enable);
}

void CW6307_SetVBUSInVolLimit(uint16_t vol)
{
    uint8_t step;
	
    if((vol < VBUS_INPUT_VOL_LIMIT_OFFSET) || (vol > VBUS_INPUT_VOL_LIMIT_MAX))
        return;

    step = (vol - VBUS_INPUT_VOL_LIMIT_OFFSET) / VBUS_INPUT_VOL_LIMIT_STEP;
    CW6307_SetRegBit(REG_VBUS_VOLT, BIT(3), 4, step);
}

void CW6307_SetVSYSVol(uint16_t vol)
{
    uint8_t step;

    if((vol < VBUS_REGUALATION_VOL_OFFSET) || (vol > VBUS_REGUALATION_VOL_MAX))
        return;

    step = (vol - VBUS_REGUALATION_VOL_OFFSET) / VBUS_REGUALATION_VOL_STEP;
	CW6307_SetRegBit(REG_VBUS_CUR, BIT(7), 4, step);
}

void CW6307_SetVBUSInCurLimit(uint16_t cur)
{
    uint8_t step;
	
    if((cur < VBUS_INPUT_CUR_LIMIT_OFFSET) || (cur > VBUS_INPUT_CUR_LIMIT_MAX))
        return;

    step = (cur - VBUS_INPUT_CUR_LIMIT_OFFSET) / VBUS_INPUT_CUR_LIMIT_STEP;
	CW6307_SetRegBit(REG_VBUS_CUR, BIT(3), 4, step);
}

void CW6307_SetCVVol(uint16_t vol)
{
    uint8_t step;

    vol = vol * 10;
    if((vol < CV_OFFSET) || (vol > CV_MAX))
        return;

    step = (vol - CV_OFFSET) / CV_STEP;
	CW6307_SetRegBit(REG_CHG_VOLT, BIT(7), 6, step);
}

void CW6307_SetRechgVol(uint8_t vol)
{
    if((vol != RECHG_VOL_200MV) && (vol != RECHG_VOL_100MV))
        return;

	CW6307_SetRegBit(REG_CHG_VOLT, BIT(1), 1, vol);
}

void CW6307_SetPrechgVol(uint8_t vol)
{
    if((vol != PRECHG_TO_CCCHG_VOL_3000) && (vol != PRECHG_TO_CCCHG_VOL_2800))
        return;

	CW6307_SetRegBit(REG_CHG_VOLT, BIT(0), 1, vol);
}

void CW6307_SetInternalThermal(uint8_t thermal)
{
    uint8_t step;

    if((thermal < IC_INTERNAL_MAX_THERMAL_OFFSET) || (thermal > IC_INTERNAL_MAX_THERMAL_MAX))
        return;

    step = (thermal - IC_INTERNAL_MAX_THERMAL_OFFSET) / IC_INTERNAL_MAX_THERMAL_STEP;
	CW6307_SetRegBit(REG_CHG_CUR1, BIT(7), 2, step);  
}

void CW6307_SetFCCCur(uint16_t cur)
{
    uint8_t step;

    cur = cur * 100;
    if((cur < FCCCHG_CUR_OFFSET) || (cur > FCCCHG_CUR_MAX))
        return;

    if(cur <= FCCCHG_CUR_OFFSET * 2)  
	{	
		/*10 ~ 20*/
		step = (cur - FCCCHG_CUR_OFFSET) / FCCCHG_CUR_STEP1;
    }
    else if(cur <= FCCCHG_CUR_OFFSET * 2 * 2)
	{
		/*20 ~ 40*/
		step = (FCCCHG_CUR_OFFSET * 2 - FCCCHG_CUR_OFFSET) / FCCCHG_CUR_STEP1 
				+ (cur - FCCCHG_CUR_OFFSET * 2) / (FCCCHG_CUR_STEP1 * 2);
    }
    else if(cur <= FCCCHG_CUR_OFFSET * 2 * 2 * 2)
	{
		/*40 ~ 80*/
		step = (FCCCHG_CUR_OFFSET * 2 - FCCCHG_CUR_OFFSET) / FCCCHG_CUR_STEP1 
                + (FCCCHG_CUR_OFFSET * 2 * 2 - FCCCHG_CUR_OFFSET * 2) / (FCCCHG_CUR_STEP1 * 2)
                + (cur - FCCCHG_CUR_OFFSET * 2 * 2) / (FCCCHG_CUR_STEP1 * 2 * 2);
    }
	else
	{  
		/*80 ~ 470*/
        step = (FCCCHG_CUR_OFFSET * 2 - FCCCHG_CUR_OFFSET) / FCCCHG_CUR_STEP1 
                + (FCCCHG_CUR_OFFSET * 2 * 2 - FCCCHG_CUR_OFFSET * 2) / (FCCCHG_CUR_STEP1 * 2)
                + (FCCCHG_CUR_OFFSET * 2 * 2 * 2 - FCCCHG_CUR_OFFSET * 2 * 2) / (FCCCHG_CUR_STEP1 * 2 * 2)    
                + (cur - FCCCHG_CUR_OFFSET * 2 * 2 * 2) / (FCCCHG_CUR_STEP1 * 2 * 2 * 2);
    }
	
    CW6307_SetRegBit(REG_CHG_CUR1, BIT(5), 6, step);   
}

void CW6307_SetForcePrechg(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_CHG_CUR2, BIT(7), 1, enable);    
}

void CW6307_SetPrechgCur(uint8_t curX10)
{
    uint8_t step;
    
    if((curX10 < PRECHG_AND_TCC_CUR_OFFSET) || (curX10 > PRECHG_AND_TCC_CUR_MAX))
        return;

    step = (curX10 - PRECHG_AND_TCC_CUR_OFFSET) / PRECHG_AND_TCC_CUR_STEP;
	CW6307_SetRegBit(REG_CHG_CUR2, BIT(6), 3, step);
}

void CW6307_SetTerminateCur(uint8_t curX10)
{
    uint8_t step;
    
    if((curX10 < PRECHG_AND_TCC_CUR_OFFSET) || (curX10 > PRECHG_AND_TCC_CUR_MAX))
        return;

    step = (curX10 - PRECHG_AND_TCC_CUR_OFFSET) / PRECHG_AND_TCC_CUR_STEP;
	CW6307_SetRegBit(REG_CHG_CUR2, BIT(2), 3, step);
}

void CW6307_ResetAllReg(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_SAFETY, BIT(7), 1, enable);    
}

void CW6307_SetChgTimer(uint8_t hours)
{
    uint8_t step;
    
    if((hours < CHG_TIME_OUT_TIMER_OFFSET) || (hours > CHG_TIME_OUT_TIMER_MAX))
        return;

    step = (hours - CHG_TIME_OUT_TIMER_OFFSET) / CHG_TIME_OUT_TIMER_STEP;
	CW6307_SetRegBit(REG_SAFETY, BIT(6), 2, step);
}

void CW6307_SetChgAutoTerminate(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_SAFETY, BIT(4), 1, enable);    
}

void CW6307_SetPreChgTimer(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_SAFETY, BIT(3), 1, enable);    
}

void CW6307_SetNTCEnable(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

    CW6307_SetRegBit(REG_SAFETY, BIT(2), 1, enable);    
}

void CW6307_SetNTCSelect(uint8_t r)
{
    if((r != CW_NTC_SELECT_10K) && (r != CW_NTC_SELECT_100K))
        return;

	CW6307_SetRegBit(REG_SAFETY, BIT(1), 1, r);    
}

void CW6307_SetNTCPullUpNet(uint8_t net)
{
    if((net != CW_NTC_INTERNAL_PULL_UP) && (net != CW_NTC_EXTERNAL_PULL_UP))
        return;

	CW6307_SetRegBit(REG_SAFETY, BIT(0), 1, net);    
}

void CW6307_SetBatOffEnable(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_CONFIG, BIT(7), 1, enable);    
}

void CW6307_SetBatOffDelayTime(uint8_t sec)
{
    uint8_t step = 0;

    switch(sec)
	{
    case 1:
        step = 0;
        break;
    case 2:
        step = 1;
        break;
    case 4:
        step = 2;
        break;
    case 8:
        step = 3;
        break;
    default:    
        /*Did not change*/
        return;
    }

	CW6307_SetRegBit(REG_CONFIG, BIT(6), 2, step);
}

void CW6307_SetIntBtnEnable(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_CONFIG, BIT(4), 1, enable);    
}

void CW6307_SetBtnFuncSelect(uint8_t vsys_reset)
{
    if((vsys_reset != CW_SET_ENABLE) && (vsys_reset != CW_SET_DISABLE))
        return;

	CW6307_SetRegBit(REG_CONFIG, BIT(3), 1, vsys_reset);        
}

void CW6307_SetVSYSResetDelayTime(uint8_t sec)
{
    if((sec != CW_VSYS_RESET_DELAY_TIME_2S) && (sec != CW_VSYS_RESET_DELAY_TIME_4S))
        return;

	CW6307_SetRegBit(REG_CONFIG, BIT(2), 1, sec);        
}

void CW6307_SetIntBtnHoldTime(uint8_t sec)
{
    uint8_t step;
    
    if((sec < INT_BTN_HOLD_TIME_OFFSET) || (sec > INT_BTN_HOLD_TIME_MAX))
        return;

    step = (sec - INT_BTN_HOLD_TIME_OFFSET) / INT_BTN_HOLD_TIME_STEP;
	CW6307_SetRegBit(REG_CONFIG, BIT(1), 2, step);
}

void CW6307_SetChgWatchdogEnable(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_SAFETY_CFG, BIT(7), 1, enable);            
}

void CW6307_SetDisChgWatchdogEnable(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_SAFETY_CFG, BIT(6), 1, enable);        
}

void CW6307_SetWatchdogTime(uint8_t sec)
{
    uint8_t step;
    
    if((sec < WATCHDOG_TIMER_OFFSET) || (sec > WATCHDOG_TIMER_MAX))
        return;

    step = (sec - WATCHDOG_TIMER_OFFSET) / WATCHDOG_TIMER_STEP;
	CW6307_SetRegBit(REG_SAFETY_CFG, BIT(5), 2, step);
}

void CW6307_SetUVLOVol(uint16_t vol)
{
    uint8_t step;
    
    if((vol < UVLO_VOL_OFFSET) || (vol > UVLO_VOL_MAX))
        return;

    step = (vol - UVLO_VOL_OFFSET) / UVLO_VOL_STEP;
	CW6307_SetRegBit(REG_SAFETY_CFG, BIT(2), 3, step);
}

void CW6307_SetTimer2X(uint8_t enable)
{
	if((enable != CW_SET_ENABLE) && (enable != CW_SET_DISABLE))
		return;

	CW6307_SetRegBit(REG_SAFETY_CFG2, BIT(7), 1, enable);        
}

void CW6307_SetVSYSOverProtectionCur(uint16_t cur)
{
    uint8_t step;
    
    if((cur < VSYS_OVER_CUR_PROTECTION_CUR_OFFSET) || (cur > VSYS_OVER_CUR_PROTECTION_CUR_MAX))
        return;

    step = (cur - VSYS_OVER_CUR_PROTECTION_CUR_OFFSET) / VSYS_OVER_CUR_PROTECTION_CUR_STEP;
	CW6307_SetRegBit(REG_SAFETY_CFG2, BIT(3), 4, step);
}

/*
* 0x0A INT_SET: Interrupt Enable
* 0x0B INT_SRC: Interrupt Source
*
* b7		b6			b5			b4		b3		b2		b1				b0
* CHG_DET	CHG_REMOVE	CHG_FINISH	CHG_OV	BAT_OT	BAT_UT	SAFETY_TIMER	PRECHG_TIMER
*
* CHG_DET: Valid Input Source Detected on VBUS
* CHG_REMOVE: Adapter Unplug
* CHG_FINISH: Charging finished
* CHG_OV: Input Source Over-Voltage
* BAT_OT: Battery Over Temperature
* BAT_UT: Battery Under Temperature
* SAFETY_TIMER: Safety Timer Expiration
* PRECHG_TIMER: Pre-Charge Timer Expiration
*/
void CW6307_SetChgInterrupt(uint8_t interrupt_val)
{
    CW6307_SetRegBit(REG_INT_SET, BIT(7), 8, interrupt_val);
}

void CW6307_GetIntSource(uint8_t *data)
{
	CW6307_ReadReg(REG_INT_SRC, data);
}

void CW6307_CleanIntSource(uint8_t count, ...)
{
    int i;
    int res = 0;
    va_list v1;
    unsigned char clean_value = 0xFF;
    unsigned char reg_value = 0;
	
    if(count < 1 || count > 8)
        return;

    if(count == 8)
        CW6307_SetRegBit(REG_INT_SRC, BIT(7), 8, 0x00);

    va_start(v1, count);
    for(i=0;i<count;i++)
    {
        res = va_arg(v1, int); 
        if(res < 0 || res > 7)
            return;

        clean_value &= ~(1<<res);
    } 
    va_end(v1);
	
    CW6307_ReadReg(REG_INT_SRC, &reg_value);
    reg_value = reg_value & clean_value;

	CW6307_SetRegBit(REG_INT_SRC, BIT(7), 8, reg_value);
}

/*
* 0x0D IC_STATUS: IC Condition Report
*
* b7			b6			b5			b4			b3			b2			b1			b0
* THERM_STAT	PRE_CHG		CC_STAT		CV_STAT		CHG_FINISH	VBUS_VLIM	VBUS_ILIM	PPMG_STAT
* 
* THERM_STAT:	0 - not in thermal regulation, 1 - in thermal regulation
* PRE_CHG:		0 - not in pre-charge state, 1 - in pre-charge state
* CC_STAT: 		0 - not in CC charging state, 1 - in CC charging state
* CV_STAT: 		0 - not in CV charging state, 1 - in CV charging state
* CHG_FINISH: 	0 - not finish charging, 1 - charging finished
* VBUS_VLIM: 	0 - not in VBUS voltage limit regulation, 1 - in VBUS voltage limit regulation
* VBUS_ILIM: 	0 - not in VBUS current limit regulation, 1 - in VBUS current limit regulation
* PPMG_STAT: 	0 - not in Power Path Management, 1 - in Power Path Management
*/
void CW6307_GetStatus(uint8_t *data)
{
	CW6307_ReadReg(REG_IC_STATUS, data);
}

/*
* 0x0E IC_STATUS II: IC Condition Report II
*
* b7		b6			b5			b4		b3		b2		b1						b0
* VBUS_PG	VBUS_OV		CHRG_STAT	IC_OT	BAT_OT	BAT_UT	SAFETY_TIMER_EXPIRATION	PRECHG_TIMER_EXPIRATION
*
* VBUS_PG:					0 - the VBUS is not in power good state, 1 - the VBUS is in power good state
* VBUS_OV:					0 - the VBUS is not over voltage, 1 - the VBUS is over voltage
* CHRG_STAT:				0 - the device is not in charging state, 1 - the device is in charging state
* IC_OT:					0 - the device is not over temperature£¬1 - the device is over temperature
* BAT_OT:					0 - Battery is not over temperature£¬1 - Battery is over temperature
* BAT_UT:					0 - Battery is not under temperature£¬1 - Battery is under temperature
* SAFETY_TIMER_EXPIRATION:	0 - Fast charge safety timer is not in expiration status£¬1 - Fast charge safety timer is in expiration status
* PRECHG_TIMER_EXPIRATION:	0 - Pre-charge timer is not in expiration status£¬1 - Pre-charge timer is in expiration status
*/
void CW6307_GetStatus2(uint8_t *data)
{
	CW6307_ReadReg(REG_IC_STATUS2, data);
}

void CW6307_GetDeviceID(uint8_t *Device_ID)
{
	CW6307_ReadReg(REG_VERSION, Device_ID);
}

void CW6307_GetChargeStatus(void)
{
	uint8_t status1,status2;
	
	CW6307_GetStatus(&status1);
	CW6307_GetStatus2(&status2);
	if(status1 != 0x00)
	{
		if((status1&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("in Power Path Management.");
		#endif
		}
		if((status1&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("in VBUS current limit regulation.");
		#endif
		}
		if((status1&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("in VBUS current limit regulation.");
		#endif
		}
		if((status1&0x08) == 0x08)
		{
		#ifdef PMU_DEBUG
			LOGD("charging finished.");
		#endif

			g_chg_status = BAT_CHARGING_FINISHED;
		}
		if((status1&0x10) == 0x10)
		{
		#ifdef PMU_DEBUG
			LOGD("in CV charging state.");
		#endif

			g_chg_status = BAT_CHARGING_PROGRESS;
		}
		if((status1&0x20) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("in CC charging state.");
		#endif

			g_chg_status = BAT_CHARGING_PROGRESS;
		}
		if((status1&0x40) == 0x40)
		{
		#ifdef PMU_DEBUG
			LOGD("in pre-charge state.");
		#endif

			g_chg_status = BAT_CHARGING_PROGRESS;
		}
		if((status1&0x80) == 0x80)
		{
		#ifdef PMU_DEBUG
			LOGD("in thermal regulation.");
		#endif
		}
	}

	if(status2 != 0x00)
	{
		if((status2&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("Pre-charge timer is in expiration status.");
		#endif
		}
		if((status2&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("Fast charge safety timer is in expiration status.");
		#endif
		}
		if((status2&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery is under temperature.");
		#endif
		}
		if((status2&0x08) == 0x08)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery is over temperature.");
		#endif
		}
		if((status2&0x10) == 0x10)
		{
		#ifdef PMU_DEBUG
			LOGD("the device is over temperature.");
		#endif
		}
		if((status2&0x20) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("the device is in charging state.");
		#endif

			charger_is_connected = true;
		}
		if((status2&0x40) == 0x40)
		{
		#ifdef PMU_DEBUG
			LOGD("the VBUS is over voltage.");
		#endif
		}
		if((status2&0x80) == 0x80)
		{
		#ifdef PMU_DEBUG
			LOGD("the VBUS is in power good state.");
		#endif

			charger_is_connected = true;
		}	
	}

	if(charger_is_connected)
	{
		if(g_chg_status != BAT_CHARGING_PROGRESS)
		{
			if(g_bat_soc < 4)
			{
				g_bat_level = BAT_LEVEL_VERY_LOW;
			}
			else if(g_bat_soc < 7)
			{
				g_bat_level = BAT_LEVEL_LOW;
			}
			else if(g_bat_soc < 80)
			{
				g_bat_level = BAT_LEVEL_NORMAL;
			}
			else
			{
				g_bat_level = BAT_LEVEL_GOOD;
			}
		}
		else
		{
			g_bat_level = BAT_LEVEL_NORMAL;
		}
	}
	else
	{			
		charger_is_connected = false;
		
		g_chg_status = BAT_CHARGING_NO;
		
		//CW2215_GetSoc(&g_bat_soc);
		//if(g_bat_soc>100)
		//	g_bat_soc = 100;

		if(g_bat_soc < 4)
		{
			g_bat_level = BAT_LEVEL_VERY_LOW;
			pmu_battery_low_shutdown();
		}
		else if(g_bat_soc < 7)
		{
			g_bat_level = BAT_LEVEL_LOW;
		}
		else if(g_bat_soc < 80)
		{
			g_bat_level = BAT_LEVEL_NORMAL;
		}
		else
		{
			g_bat_level = BAT_LEVEL_GOOD;
		}
	}
}

bool CW6307_Init(void)
{
	CW6307_GetDeviceID(&HardwareID);
	if(HardwareID != CW6307_CHIP_ID)
		return false;

	return true;
}

void pmu_charge_update(void)
{
	CW6307_GetChargeStatus();
}

void Pmu_ChargeInit(void)
{
	//charge enable
	CW6307_SetChgSwitch(CW_SET_ENABLE);
	
	//Battery Charge Finish Voltage (CV): 4.35v
	CW6307_SetCVVol(4230);
	//Battery Fast Charge Constant Current (CC): 200ma
	CW6307_SetFCCCur(200);
	//Battery Charge Finish termination Current: 0.025C
	CW6307_SetTerminateCur(PRECHG_AND_TCC_CUR_OFFSET);
	//Battery Re-Charge Threshold: 100mv
	CW6307_SetRechgVol(RECHG_VOL_100MV);
	//Battery Low Battery Threshold: 3.0v
	CW6307_SetPrechgVol(PRECHG_TO_CCCHG_VOL_3000);
	
	//Enable all intterrupt of charge
	CW6307_SetChgInterrupt(0xFF);
	
	//Update charge status
	CW6307_GetChargeStatus();
}

void pmu_interrupt_proc(void)
{
	uint8_t data;
			
	CW6307_GetIntSource(&data);
#ifdef PMU_DEBUG
	LOGD("int source:0x%02X", data);
#endif
	if(data != 0x00)
	{
		uint8_t status1, status2;
		
		CW6307_GetStatus(&status1);
		CW6307_GetStatus2(&status2);
	#ifdef PMU_DEBUG
		LOGD("status1:0x%02X, status2:0x%02X", status1,status2);
	#endif	
		
		if((data&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("Pre-Charge Timer Expiration.");
		#endif
		}
		if((data&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("Safety Timer Expiration.");
		#endif
		}
		if((data&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery Under Temperature.");
		#endif
		}
		if((data&0x08) == 0x08)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery Over Temperature.");
		#endif
		}
		if((data&0x10) == 0x10)
		{
		#ifdef PMU_DEBUG
			LOGD("Input Source Over-Voltage.");
		#endif
		}
		if((data&0x20) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("Charging finished.");
		#endif
		
			if(g_chg_status != BAT_CHARGING_FINISHED)
			{
				g_chg_status = BAT_CHARGING_FINISHED;

				g_bat_soc = CW2215_GetSoc(&g_bat_soc);
			#ifdef PMU_DEBUG
				LOGD("g_bat_soc:%d", g_bat_soc);
			#endif
				if(g_bat_soc >= 95)
					g_bat_soc = 100;
			}
		}
		if((data&0x40) == 0x40)
		{
		#ifdef PMU_DEBUG
			LOGD("Adapter Unplug.");
		#endif
		
			charger_is_connected = false;
			g_chg_status = BAT_CHARGING_NO;

			CW2215_GetSoc(&g_bat_soc);
			if(g_bat_soc > 100)
				g_bat_soc = 100;
			
			if(g_bat_soc < 4)
			{
				g_bat_level = BAT_LEVEL_VERY_LOW;
				pmu_battery_low_shutdown();
			}
			else if(g_bat_soc < 7)
			{
				g_bat_level = BAT_LEVEL_LOW;
			}
			else if(g_bat_soc < 80)
			{
				g_bat_level = BAT_LEVEL_NORMAL;
			}
			else
			{
				g_bat_level = BAT_LEVEL_GOOD;
			}

		#ifdef CONFIG_FACTORY_TEST_SUPPORT
			FTPMUStatusUpdate(2);
		#endif
		}
		if((data&0x80) == 0x80)
		{
		#ifdef PMU_DEBUG
			LOGD("Adapter plug.");
		#endif

			charger_is_connected = true;
		
		#ifdef CONFIG_FACTORY_TEST_SUPPORT
			FTPMUStatusUpdate(2);
		#endif	

			CW6307_GetChargeStatus();

			if((status2&0x20) == 0x20)
			{
				g_chg_status = BAT_CHARGING_PROGRESS;
				pmu_battery_stop_shutdown();
			}
		}
	}

	CW6307_CleanIntSource(8);
}

#endif/*#ifdef PMU_SENSOR_CW221X_CW630X*/
