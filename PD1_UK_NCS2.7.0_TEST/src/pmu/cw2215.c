/****************************************Copyright (c)************************************************
** File Name:			    cw2215.c
** Descriptions:			cw2215 sensor message process source file
** Created By:				xie biao
** Created Date:			2024-09-10
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include "pmu.h"
#include "cw_interface.h"
#include "datetime.h"
#include "settings.h"
#include "external_flash.h"
#include "logger.h"

#ifdef PMU_SENSOR_CW221X_CW630X

//#define SHOW_LOG_IN_SCREEN

#define PMU_ALRTB		5
#define PMU_EINT		8

static uint8_t HardwareID;
static uint8_t FirmwareID;

static unsigned char config_profile_info[SIZE_OF_PROFILE] = 
{
#if 1	//xb test 2024.09.11 200mAh_profile3_TSZH30X_10mohm_20240104
	0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xAD,0xC0,0xB5,0xCE,0xC0,0xC4,0x98,0x61,
	0x49,0xFF,0xFF,0xD8,0x9A,0x81,0x60,0x50,
	0x44,0x38,0x27,0x6F,0x20,0xD2,0x7D,0xEA,
	0xD3,0xBA,0xC3,0xC4,0x92,0xA2,0xA1,0x99,
	0x94,0x94,0x93,0x8F,0x78,0x5E,0x4F,0x41,
	0x32,0x4A,0x68,0x8B,0xA4,0x8C,0x75,0xA0,
	0x20,0x00,0xAB,0x10,0x00,0x72,0x9C,0x00,
	0x00,0x00,0x64,0x12,0x81,0x8B,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA2,

#else
	0x5A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xB9,0xB6,0xC2,0xBD,0xC4,0xC1,0x95,0x5B,
	0x2C,0xFF,0xFF,0xE1,0xBF,0x7F,0x6A,0x5B,
	0x51,0x4C,0x46,0x84,0xC2,0xD9,0x9E,0xD5,
	0xCF,0xCE,0xCD,0xCA,0xC9,0xB1,0xE0,0xAE,
	0xBD,0xC6,0xAB,0x98,0x8C,0x83,0x7C,0x6C,
	0x63,0x65,0x80,0x91,0xA2,0x73,0x61,0x53,
	0x00,0x00,0x57,0x10,0x00,0x40,0xF6,0x00,
	0x00,0x00,0x64,0x1F,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF4,
#endif	
};

static bool pmu_check_ok = false;
static struct device *gpio_pmu;
static struct gpio_callback gpio_cb1,gpio_cb2;

static void test_soc_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(soc_timer, test_soc_timerout, NULL);
static void pmu_battery_low_shutdown_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(soc_pwroff, pmu_battery_low_shutdown_timerout, NULL);
static void sys_pwr_off_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(sys_pwroff, sys_pwr_off_timerout, NULL);
static void vibrate_start_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(vib_start_timer, vibrate_start_timerout, NULL);
static void vibrate_stop_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(vib_stop_timer, vibrate_stop_timerout, NULL);
#ifdef BATTERY_NTC_CHECK
static void CW2215_CheckTemp(struct k_timer *timer_id);
K_TIMER_DEFINE(ntc_check_timer, CW2215_CheckTemp, NULL);
#endif

bool vibrate_start_flag = false;
bool vibrate_stop_flag = false;
bool pmu_trige_flag = false;
bool pmu_alert_flag = false;
bool pmu_bat_flag = false;
bool pmu_check_temp_flag = false;
bool pmu_redraw_bat_flag = true;
bool lowbat_pwr_off_flag = false;
bool sys_pwr_off_flag = false;
bool read_soc_status = false;
bool pmu_bat_has_notify = false;
bool sys_shutdown_is_running = false;

vibrate_msg_t g_vib = {0};

extern bool key_pwroff_flag;

#ifdef BATTERY_NTC_CHECK
static void CW2215_CheckTemp(struct k_timer *timer_id)
{
	pmu_check_temp_flag = true;
}
#endif/*BATTERY_NTC_CHECK*/


void CW2215_Buck1Disable(void)
{
}

void CW2215_Buck1Config(void)
{
}

void CW2215_Buck2Disable(void)
{
}

void CW2215_Buck2Config(void)
{
}

void CW2215_LDO1Disable(void)
{
}

void CW2215_LDO1Config(void)
{
}

void CW2215_LDO2Disable(void)
{
}

void CW2215_LDO2Config(void)
{
}

void CW2215_GetDeviceID(uint8_t *Device_ID)
{
	CW2215_ReadReg(REG_CHIP_ID, Device_ID);
}

void CW2215_GetFWVersion(uint8_t *FW_Ver)
{
	CW2215_ReadReg(REG_FW_VERSION, FW_Ver);
}

static void CW2215_Sleep(void)
{
	uint8_t reg_val = CONFIG_RESTART;

	CW2215_WriteReg(REG_MODE_CONFIG, reg_val);
	CW_Delay_ms(20);

	reg_val = CONFIG_SLEEP;
	CW2215_WriteReg(REG_MODE_CONFIG, reg_val);
	CW_Delay_ms(10);
}

static int CW2215_Active(void)
{
	uint8_t reg_val = CONFIG_RESTART;

	CW2215_WriteReg(REG_MODE_CONFIG, reg_val);
	CW_Delay_ms(20);

	reg_val = CONFIG_ACTIVE;
	CW2215_WriteReg(REG_MODE_CONFIG, reg_val);
	CW_Delay_ms(10);
}

static void CW2215_WriteTemp(int temperature)
{
	uint8_t A0_value,A1_value;
	
	if(temperature < -40 || temperature > 87)
	{
		return;
	}
	
	A0_value = (temperature + 40) * 2;
	
	CW2215_ReadReg(REG_T_HOST_L, &A1_value);
	A1_value = ~A1_value;

	CW2215_WriteReg(REG_T_HOST_H, A0_value);
	CW2215_WriteReg(REG_T_HOST_L, A1_value);
}

static void CW2215_WriteProfile(uint8_t *data)
{
	CW2215_WriteRegMulti(REG_BAT_PROFILE, data, SIZE_OF_PROFILE);
}

static int CW2215_GetVoltage(float *lp_vol)
{
	uint8_t data[2] = {0};
	float ad_value = 0.0;

	CW2215_ReadRegMulti(REG_VCELL_H, data, sizeof(data));
	//V(uV) = value(0x02 0x03 DEC) * 312.5
	ad_value = ((data[0]*256 + data[1])*312.5)/(1000*1000);
#ifdef PMU_DEBUG
	LOGD("voltage:%fV", ad_value);
#endif
	*lp_vol = ad_value;
}

#define UI_FULL     100
void CW2215_GetSoc(uint8_t *lp_uisoc)
{
	//The high byte(0x04) contains the SOC in 1% unit which can be directly used if this resolution is good enough for the application. 
	//The low byte(0x05) provides more accurate fractional part of the SOC and its LSB is (1/256) %.
	//soc = value(0x04 DEC) + value(0x05 DEC)/256;
	uint8_t data[2] = {0};
	uint8_t UI_SOC = 0;

	CW2215_ReadRegMulti(REG_SOC_INT, data, sizeof(data));
	UI_SOC = data[0];
#ifdef PMU_DEBUG
	LOGD("soc:%d", UI_SOC);
#endif

	if(UI_SOC > 100)
		UI_SOC = 100;
	
	*lp_uisoc = UI_SOC;
}

void CW2215_SetTempThreshold(float t_max, float t_min)
{
	uint8_t data[2];

	if(t_max > 87.5 || t_min < 40.0)
		return;
	
	data[0] = (t_max + 40)*2;
	CW2215_WriteReg(REG_TEMP_MAX, data[0]);
	
	data[1] = (t_min + 40)*2;
	CW2215_WriteReg(REG_TEMP_MIN, data[1]);
}

void CW2215_GetTemp(float *lp_temp)
{
	uint8_t reg_val;

	CW2215_ReadReg(REG_TEMP, &reg_val);

	*lp_temp = (float)reg_val/2.0 - 40;
#ifdef PMU_DEBUG
	LOGD("temp:%f", *lp_temp);
#endif	
}

long get_complement_code(unsigned short raw_code)
{
	long complement_code = 0;
	int dir = 0;

	if (0 != (raw_code & 0x8000)){
		dir = -1;
		raw_code =  (0XFFFF - raw_code) + 1;
	}
	else{
		dir = 1;
	}

	complement_code = (long)raw_code * dir;

	return complement_code;
}

void CW2215_GetTwosComplement(int32_t *raw, uint8_t length)
{
	if(*raw & ((uint32_t)1<<(length - 1)))
	{
		*raw -= (uint32_t)1<<length;
	}
}

static void CW2215_GetCurrent(float *lp_current)
{
	uint8_t fw_version,data[2] = {0};
	int32_t cw_current = 0;
	float cur = 0.0;
	
	CW2215_ReadReg(REG_FW_VERSION, &fw_version);
	CW2215_ReadRegMulti(REG_CURRENT_H, data, sizeof(data));

	cw_current = (uint32_t)data[0]<<8 | (uint32_t)data[1];
	CW2215_GetTwosComplement(&cw_current, 16);
	//cw_current = get_complement_code(data[0]*256+data[1]);
	if(((fw_version & CW2215_MARK) != 0) || ((fw_version & CW2217_MARK) != 0))
	{
		cur = (float)(cw_current*1600)/USER_RSENSE;
	}
	else if((fw_version != 0) && ((fw_version & 0xC0) == CW2218_MARK))
	{
		cur = (float)(cw_current*3815)/USER_RSENSE;
	}
	else
	{
		cur = 0.0;
	}

#ifdef PMU_DEBUG
	LOGD("cur:%fma", cur);
#endif

	*lp_current = cur;
}

static void CW2215_GetCycleCount(uint16_t *lp_count)
{
	uint8_t data[2] = {0};

	CW2215_ReadRegMulti(REG_CYCLE_H, data, sizeof(data));

	*lp_count = ((uint16_t)data[0]<<8 | data[1]) / 16;
#ifdef PMU_DEBUG
	LOGD("count:%d", *lp_count);
#endif	
}

static void CW2215_GetBatHealth(uint8_t *lp_soh)
{
	uint8_t reg_val = 0;
	uint8_t SOH = 0;

	CW2215_ReadReg(REG_SOH, &reg_val);
#ifdef PMU_DEBUG
	LOGD("soh:%d", reg_val);
#endif
	if(reg_val > 100)
		reg_val = 100;
	*lp_soh = reg_val;
}

static void CW2215_GetInterrupt(uint8_t *data)
{	//INT_CONF Register
	//b7		b6			b5			b4			b3			b2		b1		b0
	//Reserved 	EN_SOC_INT 	EN_TMX_INT 	EN_TMN_INT 	Reserved 	SOC_INT TMX_INT TMN_INT
	uint8_t reg_val;

	CW2215_ReadReg(REG_GPIO_CONFIG, &reg_val);

	*data = reg_val;
}

static void CW2215_SetInterrupt(uint8_t data)
{	//INT_CONF Register
	//b7		b6			b5			b4			b3			b2		b1		b0
	//Reserved 	EN_SOC_INT 	EN_TMX_INT 	EN_TMN_INT 	Reserved 	SOC_INT TMX_INT TMN_INT
	uint8_t reg_val;

	CW2215_ReadReg(REG_GPIO_CONFIG, &reg_val);
	reg_val = (reg_val | (data&0x70));
	CW2215_WriteReg(REG_GPIO_CONFIG, reg_val); 
}

static void CW2215_DumpReg(void)
{
	uint8_t i,reg_val;

	for(i=REG_CHIP_ID;i<REG_MAX;i++)
	{
		CW2215_ReadReg(i, &reg_val);
	#ifdef PMU_DEBUG
		LOGD("REG:0x%02X, VALUE:0x%02X", i, reg_val);
	#endif
	}
}

static void CW2215_Config_Start(void)
{
	uint8_t reg_val;
	uint16_t count = 0;

	CW2215_Sleep();

	//update new battery info
	CW2215_WriteProfile(config_profile_info);

	//set UPDATE_FLAG AND SOC INTTERRUP VALUE
	reg_val = CW2215_PROFILE_UPDATE_FLAG | CW2215_SOC_IRQ_VALUE;
	CW2215_WriteReg(REG_SOC_ALERT, reg_val);

	/*close all interruptes*/
	reg_val = 0; 
	CW2215_SetInterrupt(reg_val); 

	CW2215_Active();
		
	while(1)
	{
		CW_Delay_ms(100);
		CW2215_ReadReg(REG_IC_STATE, &reg_val);
	#ifdef PMU_DEBUG
		LOGD("IC_STATE:0x%02X", reg_val);
	#endif	
		if(IC_SOC_READY == (reg_val & IC_SOC_READY))
		{
		#ifdef PMU_DEBUG
			LOGD("config success!");
		#endif
			break;
		}
		
		count++;
		if(count >= CW2215_SLEEP_COUNTS)
		{
		#ifdef PMU_DEBUG
			LOGD("config fail!");
		#endif
			CW2215_Sleep();
			return;
		}
	}
}

static CW2215_STATE CW2215_GetState(void)
{
	int i;
	uint8_t reg_val;
	uint8_t data[SIZE_OF_PROFILE] = {0};

	CW2215_ReadReg(REG_MODE_CONFIG, &reg_val);
	if(reg_val != CONFIG_ACTIVE)
		return STATE_NOT_ACTIVE;
	
	CW2215_ReadReg(REG_SOC_ALERT, &reg_val);
	if(0x00 == (reg_val & 0x80))
		return STATE_PROFILE_NOT_READY;

	CW2215_ReadRegMulti(REG_BAT_PROFILE, data, SIZE_OF_PROFILE);
	for(i=0;i<SIZE_OF_PROFILE;i++)
	{
		if(config_profile_info[i] != data[i])
			break;
	}
	
	if(i != SIZE_OF_PROFILE)
	{
	#ifdef PMU_DEBUG
		LOGD("profile need update!");
	#endif
		return STATE_PROFILE_NEED_UPDATE;
	}

#ifdef PMU_DEBUG
	LOGD("state is normal!");
#endif	
	return STATE_NORMAL;
}

bool CW2215_Init(void)
{
	CW2215_GetDeviceID(&HardwareID);
	if(HardwareID != CW2215_CHIP_ID)
		return false;

	switch(CW2215_GetState())
	{
	case STATE_NORMAL:
		break;
		
	case STATE_NOT_ACTIVE:
	case STATE_PROFILE_NOT_READY:
	case STATE_PROFILE_NEED_UPDATE:
		CW2215_Config_Start();
		break;
	}

	return true;
}

void PPG_Power_On(void)
{}

void PPG_Power_Off(void)
{}

void Set_Screen_Backlight_Level(BACKLIGHT_LEVEL level)
{}

void Set_Screen_Backlight_On(void)
{}

void Set_Screen_Backlight_Off(void)
{}

void sys_pwr_off_timerout(struct k_timer *timer_id)
{
	sys_pwr_off_flag = true;
}

void VibrateStart(void)
{}

void VibrateStop(void)
{}

void vibrate_start_timerout(struct k_timer *timer_id)
{
	vibrate_stop_flag = true;
}

void vibrate_stop_timerout(struct k_timer *timer_id)
{
	vibrate_start_flag = true;
}

void vibrate_off(void)
{
	k_timer_stop(&vib_start_timer);
	k_timer_stop(&vib_stop_timer);
	memset(&g_vib, 0, sizeof(g_vib));
	
	vibrate_stop_flag = true;
}

void vibrate_on(VIBRATE_MODE mode, uint32_t mSec1, uint32_t mSec2)
{
	g_vib.work_mode = mode;
	g_vib.on_time = mSec1;
	g_vib.off_time = mSec2;
	
	switch(g_vib.work_mode)
	{
	case VIB_ONCE:
	case VIB_RHYTHMIC:
		k_timer_start(&vib_start_timer, K_MSEC(g_vib.on_time), K_NO_WAIT);
		break;

	case VIB_CONTINUITY:
		break;
	}

	vibrate_start_flag = true;
}

void system_power_off(uint8_t flag)
{
	if(!sys_shutdown_is_running)
	{
	#ifdef PMU_DEBUG
		LOGD("begin");
	#endif
		sys_shutdown_is_running = true;
		
		SaveSystemDateTime();
		if(1)//(nb_is_connected())
		{
			SendPowerOffData(flag);
		}

		k_timer_start(&sys_pwroff, K_MSEC(5*1000), K_NO_WAIT);
	}
}

void SystemShutDown(void)
{	
#ifdef PMU_DEBUG
	LOGD("begin");
#endif

	//CW2215_WriteReg(REG_TASKENTERSHIPMODE, 0x01);
}

void pmu_battery_low_shutdown_timerout(struct k_timer *timer_id)
{
	lowbat_pwr_off_flag = true;
}

void pmu_battery_stop_shutdown(void)
{
	if(k_timer_remaining_get(&soc_pwroff) > 0)
		k_timer_stop(&soc_pwroff);
}

void pmu_battery_low_shutdown(void)
{
	k_timer_start(&soc_pwroff, K_MSEC(10*1000), K_NO_WAIT);
}

void pmu_battery_update(void)
{
	uint16_t data;
	float value;

	CW2215_GetTemp(&value);
	CW2215_GetVoltage(&value);
	CW2215_GetCurrent(&value);
	CW2215_GetSoc(&data);
	CW2215_GetCycleCount(&data);
	CW2215_GetBatHealth(&data);
}

void pmu_status_update(void)
{
	uint8_t status0,status1;
	uint16_t data;
	float value;
	
	static BAT_CHARGER_STATUS chg_status_bk = BAT_CHARGING_MAX;

	if(!pmu_check_ok)
		return;

	pmu_battery_update();
	pmu_charge_update();
	return;
	
	if(charger_is_connected)
	{
		CW2215_GetSoc(&g_bat_soc);
		if(g_bat_soc > 100)
			g_bat_soc = 100;

		if(g_chg_status != BAT_CHARGING_PROGRESS)
		{
			if(g_bat_soc < 4)
				g_bat_level = BAT_LEVEL_VERY_LOW;
			else if(g_bat_soc < 7)
				g_bat_level = BAT_LEVEL_LOW;
			else if(g_bat_soc < 80)
				g_bat_level = BAT_LEVEL_NORMAL;
			else
				g_bat_level = BAT_LEVEL_GOOD;
		}
	}
	else
	{			
		g_chg_status = BAT_CHARGING_NO;
		
		CW2215_GetSoc(&g_bat_soc);
		if(g_bat_soc>100)
			g_bat_soc = 100;

		if(g_bat_soc < 4)
		{
			g_bat_level = BAT_LEVEL_VERY_LOW;
			pmu_battery_low_shutdown();
		}
		else if(g_bat_soc < 7)
			g_bat_level = BAT_LEVEL_LOW;
		else if(g_bat_soc < 80)
			g_bat_level = BAT_LEVEL_NORMAL;
		else
			g_bat_level = BAT_LEVEL_GOOD;
	}

#ifdef PMU_DEBUG
	LOGD("chg_bk:%d, chg:%d, soc:%d, bat_level:%d", chg_status_bk, g_chg_status, g_bat_soc, g_bat_level);
#endif

	if(chg_status_bk != g_chg_status)
	{
		chg_status_bk = g_chg_status;
	}
}

void PmuInterruptHandle(void)
{
	pmu_trige_flag = true;
}

void pmu_alert_proc(void)
{
	uint8_t data;

	CW2215_GetInterrupt(&data);
#ifdef PMU_DEBUG
	LOGD("data:0x%02X", data);
#endif
	if((data&0x04) == 0x04)
	{
		pmu_battery_update();
		pmu_charge_update();
	}
}

void PmuAlertHandle(void)
{
	pmu_alert_flag = true;
}

void Pmu_SOCInit(void)
{
	uint8_t reg_val;

	//Set Soc trigger value: %1 of soc. (any value from 0x65 to 0x7F, it will generate as long as the integer part of the SOC(1%))
	CW2215_ReadReg(REG_SOC_ALERT, &reg_val);
	reg_val = reg_val | CW2215_SOC_IRQ_VALUE;
	CW2215_WriteReg(REG_SOC_ALERT, reg_val);
	
	//Set battery temperature alert threshlod: MAX 45.0, MIN 0.0
	CW2215_SetTempThreshold(45.0, 0.0);
	
	//Set interrupt source: EN_SOC, EN_TMX, EN_TMN
	CW2215_SetInterrupt(0x70);

	CW2215_GetSoc(&g_bat_soc);
}

void Pmu_InitData(void)
{
	Pmu_SOCInit();
	Pmu_ChargeInit();
	
#ifdef PMU_DEBUG
	LOGD("usb:%d, chg:%d, soc:%d", charger_is_connected, g_chg_status, g_bat_soc);
#endif
	if(!charger_is_connected && g_bat_soc == 0)
	{
	#ifdef PMU_DEBUG
		LOGD("g_bat_soc=0, Can't startup!");
	#endif
		SystemShutDown();
	}
}

void pmu_init(void)
{
	bool rst;

#ifdef PMU_DEBUG
	LOGD("pmu_init");
#endif
  	gpio_pmu = DEVICE_DT_GET(PMU_PORT);
	if(!gpio_pmu)
	{
	#ifdef PMU_DEBUG
		LOGD("Cannot bind gpio device");
	#endif
		return;
	}

	//charger interrupt
#if 0 //xb add 2024-10-23	充电中断与SOS按键共用一个GPIO
	gpio_pin_configure(gpio_pmu, PMU_EINT, GPIO_INPUT|GPIO_PULL_UP);
	gpio_pin_interrupt_configure(gpio_pmu, PMU_EINT, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb1, PmuInterruptHandle, BIT(PMU_EINT));
	gpio_add_callback(gpio_pmu, &gpio_cb1);
	gpio_pin_interrupt_configure(gpio_pmu, PMU_EINT, GPIO_INT_ENABLE|GPIO_INT_EDGE_FALLING);
#endif

	//alert interrupt
	gpio_pin_configure(gpio_pmu, PMU_ALRTB, GPIO_INPUT|GPIO_PULL_UP);
	gpio_pin_interrupt_configure(gpio_pmu, PMU_ALRTB, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb2, PmuAlertHandle, BIT(PMU_ALRTB));
	gpio_add_callback(gpio_pmu, &gpio_cb2);
	gpio_pin_interrupt_configure(gpio_pmu, PMU_ALRTB, GPIO_INT_ENABLE|GPIO_INT_EDGE_FALLING);

	rst = CW_I2C_Init();
	if(!rst)
		return;

	CW_Interface_Init();
	
	pmu_check_ok = CW2215_Init();
	if(!pmu_check_ok)
		return;
	pmu_check_ok = CW6307_Init();
	if(!pmu_check_ok)
		return;
	
	Pmu_InitData();

#ifdef PMU_DEBUG
	LOGD("pmu_init done!");
#endif
}

void test_pmu(void)
{
    pmu_init();
}

#ifdef BATTERY_SOC_GAUGE
void test_soc_status(void)
{
	uint8_t bat_soc = 0;
	
	CW2215_GetSoc(&bat_soc);
}

void test_soc_timerout(struct k_timer *timer_id)
{
	read_soc_status = true;
}

void test_soc(void)
{
	k_timer_start(&soc_timer, K_MSEC(1*1000), K_MSEC(1*1000));
}
#endif/*BATTERY_SOC_GAUGE*/

void GetBatterySocString(uint8_t *str_utc)
{
	if(str_utc == NULL)
		return;

	sprintf(str_utc, "%d", g_bat_soc);
}

void PMUMsgProcess(void)
{
	bool ret = false;
	uint8_t val;

	if(pmu_trige_flag)
	{
	#ifdef PMU_DEBUG
		LOGD("chg int");
	#endif
		pmu_interrupt_proc();
		pmu_trige_flag = false;
	}
	
	if(pmu_alert_flag)
	{
	#ifdef PMU_DEBUG
		LOGD("alert");
	#endif
		pmu_alert_proc();
		pmu_alert_flag = false;
	}

	if(lowbat_pwr_off_flag)
	{
		system_power_off(1);
		lowbat_pwr_off_flag = false;
	}
	
	if(key_pwroff_flag)
	{
	#ifdef PMU_DEBUG
		LOGD("key_pwroff_flag");
	#endif
		system_power_off(2);
		key_pwroff_flag = false;
	}
	
	if(sys_pwr_off_flag)
	{
	#ifdef PMU_DEBUG
		LOGD("pmu_check_ok:%d", pmu_check_ok);
	#endif
		if(pmu_check_ok)
			SystemShutDown();
		
		sys_pwr_off_flag = false;
	}
	
	if(vibrate_start_flag)
	{
		if(pmu_check_ok)
			VibrateStart();

		vibrate_start_flag = false;

		if(g_vib.work_mode == VIB_RHYTHMIC)
		{
			k_timer_start(&vib_start_timer, K_MSEC(g_vib.on_time), K_NO_WAIT);
		}
		else
		{
			memset(&g_vib, 0, sizeof(g_vib));
		}
	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		FTVibrateStatusUpdate(true);
	#endif
	}
	
	if(vibrate_stop_flag)
	{
		if(pmu_check_ok)
			VibrateStop();

		vibrate_stop_flag = false;

		if(g_vib.work_mode == VIB_RHYTHMIC)
		{
			k_timer_start(&vib_stop_timer, K_MSEC(g_vib.off_time), K_NO_WAIT);
		}
		else
		{
			memset(&g_vib, 0, sizeof(g_vib));
		}
	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		FTVibrateStatusUpdate(false);
	#endif
	}

#ifdef BATTERY_SOC_GAUGE
	if(read_soc_status)
	{
		if(pmu_check_ok)
			test_soc_status();
		
		read_soc_status = false;
	}
#endif
}

void test_bat_soc(void)
{
#ifdef SHOW_LOG_IN_SCREEN
	sprintf(tmpbuf, "SOC:%d\n", g_bat_soc);
	show_infor1(tmpbuf);
#endif
}

#endif/*PMU_SENSOR_MAX20353*/

