/****************************************Copyright (c)************************************************
** File Name:			    npm1300.c
** Descriptions:			npm1300 sensor message process head file
** Created By:				xie biao
** Created Date:			2024-02-28
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __NPM1300_H__
#define __NPM1300_H__

#include <stdint.h>
#include <zephyr/kernel.h>
#include "pmu.h"

#ifdef PMU_SENSOR_NPM1300

#define BATTERY_SOC_GAUGE
#define BATTERY_NTC_CHECK

//#define GPIO_ACT_I2C

#define NPM1300_I2C_ADDR	0x6b
#define NPM1300_NO_ERROR   0
#define NPM1300_ERROR      -1


/* ADC result masks */
#define ADC_MSB_SHIFT      2U
#define ADC_LSB_MASK       0x03U
#define ADC_LSB_VBAT_SHIFT 0U
#define ADC_LSB_NTC_SHIFT  2U
#define ADC_LSB_IBAT_SHIFT 4U

/* NTC temp masks */
#define NTCTEMP_MSB_SHIFT 2U
#define NTCTEMP_LSB_MASK  0x03U

/* Ibat status */
#define IBAT_STAT_DISCHARGE      0x04U
#define IBAT_STAT_CHARGE_TRICKLE 0x0CU
#define IBAT_STAT_CHARGE_COOL    0x0DU
#define IBAT_STAT_CHARGE_NORMAL  0x0FU

typedef enum
{
	//MAIN
	REG_TASKSWRESET				= 0x0001,	//Task Force a full reboot power-cycle
	REG_EVENTSADCSET			= 0x0002,	//ADC Events Event Set
	REG_EVENTSADCCLR			= 0x0003,	//ADC Events Event Clear
	REG_INTENEVENTSADCSET 		= 0x0004, 	//ADC Events Interrupt Enable Set
	REG_INTENEVENTSADCCLR 		= 0x0005,	//ADC Events Interrupt Enable Clear
	REG_EVENTSBCHARGER0SET 		= 0x0006,	//Battery Charger Temperature Events Event Set
	REG_EVENTSBCHARGER0CLR 		= 0x0007, 	//Battery Charger Temperature Events Event Clear
	REG_INTENEVENTSBCHARGER0SET = 0x0008, 	//Battery Charger Temperature Events Interrupt Enable Set
	REG_INTENEVENTSBCHARGER0CLR = 0x0009,	//Battery Charger Temperature Events Interrupt Enable Clear
	REG_EVENTSBCHARGER1SET 		= 0x000A, 	//Battery Charger Status Events Event Set
	REG_EVENTSBCHARGER1CLR 		= 0x000B, 	//Battery Charger Status Events Event Clear
	REG_INTENEVENTSBCHARGER1SET = 0x000C, 	//Battery Charger Status Events Interrupt Enable Set
	REG_INTENEVENTSBCHARGER1CLR = 0x000D, 	//Battery Charger Status Events Interrupt Enable Clear
	REG_EVENTSBCHARGER2SET 		= 0x000E, 	//Battery Charger Battery Events Event Set
	REG_EVENTSBCHARGER2CLR 		= 0x000F, 	//Battery Charger Battery Events Event Clear
	REG_INTENEVENTSBCHARGER2SET = 0x0010, 	//Battery Charger Battery Events Interrupt Enable Set
	REG_INTENEVENTSBCHARGER2CLR = 0x0011, 	//Battery Charger Battery Events Interrupt Enable Clear
	REG_EVENTSSHPHLDSET			= 0x0012, 	//ShipHold pin Events Event Set
	REG_EVENTSSHPHLDCLR 		= 0x0013, 	//ShipHold pin Events Event Clear
	REG_INTENEVENTSSHPHLDSET 	= 0x0014, 	//ShipHold pin Events Interrupt Enable Set
	REG_INTENEVENTSSHPHLDCLR 	= 0x0015, 	//ShipHold pin Events Interrupt Enable Clear
	REG_EVENTSVBUSIN0SET 		= 0x0016, 	//VBUSIN Voltage Detection Events Event Set
	REG_EVENTSVBUSIN0CLR 		= 0x0017, 	//VBUSIN Voltage Detection Events Event Clear
	REG_INTENEVENTSVBUSIN0SET 	= 0x0018, 	//VBUSIN Voltage Detection Events Interrupt Enable Set
	REG_INTENEVENTSVBUSIN0CLR 	= 0x0019, 	//VBUSIN Voltage Detection Events Interrupt Enable Clear
	REG_EVENTSVBUSIN1SET 		= 0x001A, 	//VBUSIN Thermal and USB Events Event Set
	REG_EVENTSVBUSIN1CLR 		= 0x001B, 	//VBUSIN Thermal and USB Events Event Clear
	REG_INTENEVENTSVBUSIN1SET 	= 0x001C, 	//VBUSIN Thermal and USB Events Interrupt Enable Set
	REG_INTENEVENTSVBUSIN1CLR 	= 0x001D, 	//VBUSIN Thermal and USB Events Interrupt Enable Clear
	REG_EVENTSGPIOSET 			= 0x0022, 	//GPIO Event Event Set
	REG_EVENTSGPIOCLR 			= 0x0023, 	//GPIO Event Event Clear
	REG_INTENEVENTSGPIOSET 		= 0x0024, 	//GPIO Event Interrupt Enable Set
	REG_INTENEVENTSGPIOCLR 		= 0x0025, 	//GPIO Event Interrupt Enable Clear
	//VBUSIN
	REG_TASKUPDATEILIMSW 		= 0x0200, 	//Select Input Current limit for VBUS
	REG_VBUSINILIM0 			= 0x0201, 	//Select Input Current limit for VBUS NOTE: Reset value from OTP, value listed in this table may not be correct.
	REG_VBUSINILIMSTARTUP		= 0x0202,	//
	REG_VBUSSUSPEND 			= 0x0203, 	//Suspend mode enable
	REG_USBCDETECTSTATUS 		= 0x0205, 	//VBUS CC comparator status flags
	REG_VBUSINSTATUS 			= 0x0207, 	//VBUS status flags
	//BCHARGER
	REG_TASKRELEASEERR 			= 0x0300, 	//Release Charger from Error
	REG_TASKCLEARCHGERR 		= 0x0301, 	//Clear error registers
	REG_TASKCLEARSAFETYTIMER 	= 0x0302, 	//Clear safety timers
	REG_BCHGENABLESET 			= 0x0304, 	//Charger Enable Set
	REG_BCHGENABLECLR 			= 0x0305, 	//Charger Enable Clear
	REG_BCHGDISABLESET 			= 0x0306, 	//Charger Disable Recharge Set
	REG_BCHGDISABLECLR 			= 0x0307, 	//Charger Disable Recharge Clear
	REG_BCHGISETMSB 			= 0x0308, 	//Battery Charger Current Configuration
	REG_BCHGISETLSB 			= 0x0309, 	//Battery Charger Current Configuration
	REG_BCHGISETDISCHARGEMSB 	= 0x030A, 	//Battery Charger Discharge Configuration
	REG_BCHGISETDISCHARGELSB 	= 0x030B, 	//Battery Charger Discharge Configuration
	REG_BCHGVTERM 				= 0x030C, 	//Battery Charger Termination Voltage Normal temp
	REG_BCHGVTERMR 				= 0x030D, 	//Battery Charger Termination Voltage Warm temp
	REG_BCHGVTRICKLESEL 		= 0x030E, 	//Battery Charger Trickle Level Select
	REG_BCHGITERMSEL 			= 0x030F, 	//Battery Charger ITERM Level Select
	REG_NTCCOLD 				= 0x0310, 	//NTC thermistor threshold for COLD temperature region
	REG_NTCCOLDLSB 				= 0x0311, 	//NTC thermistor threshold for COLD temperature region
	REG_NTCCOOL 				= 0x0312, 	//NTC thermistor threshold for COOL temperature region
	REG_NTCCOOLLSB 				= 0x0313, 	//NTC thermistor threshold for COOL temperature region
	REG_NTCWARM 				= 0x0314, 	//NTC thermistor threshold for WARM temperature region
	REG_NTCWARMLSB 				= 0x0315, 	//NTC thermistor threshold for WARM temperature region
	REG_NTCHOT 					= 0x0316, 	//NTC thermistor threshold for HOT temperature region
	REG_NTCHOTLSB 				= 0x0317, 	//NTC thermistor threshold for HOT temperature region
	REG_DIETEMPSTOP 			= 0x0318, 	//DIE TEMP threshold for stop charging
	REG_DIETEMPSTOPLSB 			= 0x0319, 	//DIE TEMP threshold for stop charging lsb
	REG_DIETEMPRESUME 			= 0x031A, 	//DIE TEMP threshold for resuming charging
	REG_DIETEMPRESUMELSB 		= 0x031B, 	//DIE TEMP threshold for resuming charging lsb
	REG_BCHGILIMSTATUS 			= 0x032D, 	//BCHARGER Ilim Status
	REG_NTCSTATUS 				= 0x0332, 	//NTC Comparator Status
	REG_DIETEMPSTATUS 			= 0x0333, 	//DieTemp Comparator Status
	REG_BCHGCHARGESTATUS 		= 0x0334, 	//Charging Status
	REG_BCHGERRREASON 			= 0x0336, 	//Charger-FSM Error. Latched error reasons. Cleared with TASKS_CLEAR_CHG_ERR
	REG_BCHGERRSENSOR 			= 0x0337, 	//Charger-FSM Error. Latched sensor values. Cleared with TASKS_CLEAR_CHG_ERR
	REG_BCHGCONFIG 				= 0x033C, 	//Charger configuration
	REG_BCHGVBATLOWEN			= 0x0350,	//Charger Enable Set When Voltage of Battery is low.
	//BUCk
	REG_BUCK1ENASET 			= 0x0400, 	//BUCK1 Enable pulse
	REG_BUCK1ENACLR 			= 0x0401, 	//BUCK1 Disable pulse
	REG_BUCK2ENASET 			= 0x0402, 	//BUCK2 Enable pulse
	REG_BUCK2ENACLR 			= 0x0403, 	//BUCK2 Disable pulse
	REG_BUCK1PWMSET 			= 0x0404, 	//BUCK1 PWM mode enable pulse
	REG_BUCK1PWMCLR 			= 0x0405, 	//BUCK1 PWM mode disable pulse
	REG_BUCK2PWMSET 			= 0x0406, 	//BUCK2 PWM mode enable pulse
	REG_BUCK2PWMCLR 			= 0x0407, 	//BUCK2 PWM mode disable pulse
	REG_BUCK1NORMVOUT 			= 0x0408, 	//BUCK1 Output voltage Normal mode
	REG_BUCK1RETVOUT 			= 0x0409, 	//BUCK1 Output voltage Retention mode
	REG_BUCK2NORMVOUT 			= 0x040A, 	//BUCK2 Output voltage Normal mode
	REG_BUCK2RETVOUT 			= 0x040B, 	//BUCK2 Output voltage Retention mode
	REG_BUCKENCTRL 				= 0x040C, 	//BUCK Enable GPIO Select
	REG_BUCKVRETCTRL 			= 0x040D, 	//BUCK Retention Voltage select
	REG_BUCKPWMCTRL 			= 0x040E, 	//BUCK Forced PWM mode GPIO select
	REG_BUCKSWCTRLSEL 			= 0x040F, 	//BUCK Software Control select
	REG_BUCK1VOUTSTATUS 		= 0x0410, 	//BUCK1 VOUT Status register. Lets software read the Vout value in case its driven by the FSM.
	REG_BUCK2VOUTSTATUS 		= 0x0411, 	//BUCK2 VOUT Status register. Lets software read the Vout value in case its driven by the FSM.
	REG_BUCKCTRL0 				= 0x0415, 	//BUCK Auto PFM to PWM Control select
	REG_BUCKSTATUS 				= 0x0434, 	//BUCK status register
	//ADC
	REG_TASKVBATMEASURE 		= 0x0500, 	//Task Take VBAT measurement
	REG_TASKNTCMEASURE 			= 0x0501, 	//Task Take NTC measurement
	REG_TASKTEMPMEASURE 		= 0x0502, 	//Task Take Die Temperature measurement
	REG_TASKVSYSMEASURE 		= 0x0503, 	//Task Take VSYS measurement
	REG_TASKIBATMEASURE 		= 0x0506, 	//Task Take IBATmeasurement
	REG_TASKVBUS7MEASURE 		= 0x0507, 	//Task Take VBUS 7V range measurement
	REG_TASKDELAYEDVBATMEASURE 	= 0x0508, 	//Task Take delayed VBAT measurement
	REG_ADCCONFIG				= 0x0509, 	//ADC Configuration
	REG_ADCNTCRSEL 				= 0x050A, 	//Select Battery NTC register
	REG_ADCAUTOTIMCONF 			= 0x050B, 	//Auto measurement intervals
	REG_TASKAUTOTIMUPDATE 		= 0x050C, 	//update toggle for NTC and Die temp AutoTime register bits
	REG_ADCDELTIMCONF 			= 0x050D, 	//Vbat Delay timer control
	REG_ADCIBATMEASSTATUS 		= 0x0510, 	//Battery current measurement status
	REG_ADCVBATRESULTMSB 		= 0x0511, 	//ADC VBAT measurement result MSB
	REG_ADCNTCRESULTMSB 		= 0x0512, 	//ADC NTC measurement result MSB
	REG_ADCTEMPRESULTMSB 		= 0x0513, 	//ADC DIE TEMP measurement result MSB
	REG_ADCVSYSRESULTMSB 		= 0x0514, 	//ADC VSYS measurement result MSB
	REG_ADCGP0RESULTLSBS 		= 0x0515, 	//ADC result LSB's (Vbat, Ntc, Temp and Vsys)
	REG_ADCVBAT0RESULTMSB 		= 0x0516, 	//ADC VBAT0 Burst measurement result MSB
	REG_ADCVBAT1RESULTMSB 		= 0x0517, 	//ADC VBAT1 Burst measurement result MSB
	REG_ADCVBAT2RESULTMSB 		= 0x0518, 	//ADC VBAT2 Burst measurement result MSB
	REG_ADCVBAT3RESULTMSB 		= 0x0519, 	//ADC VBAT3 Burst or VBUS measurement result MSB
	REG_ADCGP1RESULTLSBS 		= 0x051A, 	//ADC result LSB's (Vbat_burst0, 1, 2 and 3)
	REG_ADCIBATMEASEN 			= 0x0524, 	//Enable auto IBAT measurement
	//GPIOS
	REG_GPIO_0_MODE				= 0x0600, 	//GPIO Mode Configuration
	REG_GPIO_1_MODE				= 0x0601,	//GPIO Mode Configuration
	REG_GPIO_2_MODE 			= 0x0602, 	//GPIO Mode Configuration
	REG_GPIO_3_MODE 			= 0x0603, 	//GPIO Mode Configuration
	REG_GPIO_4_MODE 			= 0x0604, 	//GPIO Mode Configuration
	REG_GPIO_0_DRIVE 			= 0x0605, 	//GPIO Drive strength Configuration
	REG_GPIO_1_DRIVE 			= 0x0606, 	//GPIO Drive strength Configuration
	REG_GPIO_2_DRIVE 			= 0x0607, 	//GPIO Drive strength Configuration
	REG_GPIO_3_DRIVE 			= 0x0608, 	//GPIO Drive strength Configuration
	REG_GPIO_4_DRIVE 			= 0x0609, 	//GPIO Drive strength Configuration
	REG_GPIO_0_PUEN 			= 0x060A, 	//GPIO Pull-up Enable Configuration
	REG_GPIO_1_PUEN 			= 0x060B, 	//GPIO Pull-up Enable Configuration
	REG_GPIO_2_PUEN 			= 0x060C, 	//GPIO Pull-up Enable Configuration
	REG_GPIO_3_PUEN 			= 0x060D, 	//GPIO Pull-up Enable Configuration
	REG_GPIO_4_PUEN 			= 0x060E, 	//GPIO Pull-up Enable Configuration
	REG_GPIO_0_PDEN 			= 0x060F, 	//GPIO Pull-down Enable Configuration
	REG_GPIO_1_PDEN 			= 0x0610, 	//GPIO Pull-down Enable Configuration
	REG_GPIO_2_PDEN 			= 0x0611, 	//GPIO Pull-down Enable Configuration
	REG_GPIO_3_PDEN 			= 0x0612, 	//GPIO Pull-down Enable Configuration
	REG_GPIO_4_PDEN 			= 0x0613, 	//GPIO Pull-down Enable Configuration
	REG_GPIO_0_OPENDRAIN		= 0x0614, 	//GPIO Open Drain Configuration
	REG_GPIO_1_OPENDRAIN 		= 0x0615, 	//GPIO Open Drain Configuration
	REG_GPIO_2_OPENDRAIN 		= 0x0616, 	//GPIO Open Drain Configuration
	REG_GPIO_3_OPENDRAIN 		= 0x0617, 	//GPIO Open Drain Configuration
	REG_GPIO_4_OPENDRAIN 		= 0x0618, 	//GPIO Open Drain Configuration
	REG_GPIO_0_DEBOUNCE			= 0x0619, 	//GPIO Debounce Configuration
	REG_GPIO_1_DEBOUNCE 		= 0x061A, 	//GPIO Debounce Configuration
	REG_GPIO_2_DEBOUNCE 		= 0x061B, 	//GPIO Debounce Configuration
	REG_GPIO_3_DEBOUNCE 		= 0x061C, 	//GPIO Debounce Configuration
	REG_GPIO_4_DEBOUNCE 		= 0x061D, 	//GPIO Debounce Configuration
	REG_GPIOSTATUS 				= 0x061E, 	//GPIO Status from GPIO Pads
	//TIMER
	REG_TIMERSET 				= 0x0700, 	//Start Timer
	REG_TIMERCLR 				= 0x0701, 	//Stop Timer
	REG_TIMERTARGETSTROBE 		= 0x0703, 	//Strobe for timer Target
	REG_WATCHDOGKICK 			= 0x0704, 	//Watchdog kick
	REG_TIMERCONFIG 			= 0x0705, 	//Timer mode selection
	REG_TIMERSTATUS 			= 0x0706, 	//Timers Status
	REG_TIMERHIBYTE 			= 0x0708, 	//Timer Most Significant Byte
	REG_TIMERMIDBYTE 			= 0x0709, 	//Timer Middle Byte
	REG_TIMERLOBYTE 			= 0x070A, 	//Timer Least Significant Byte
	//LDSW
	REG_TASKLDSW1SET 			= 0x0800, 	//Enable LDSW1
	REG_TASKLDSW1CLR 			= 0x0801, 	//Disable LDSW1
	REG_TASKLDSW2SET 			= 0x0802, 	//Enable LDSW2
	REG_TASKLDSW2CLR 			= 0x0803, 	//Disable LDSW2
	REG_LDSWSTATUS 				= 0x0804, 	//Load Switch Status
	REG_LDSW1GPISEL 			= 0x0805, 	//Load Switch1 GPIO Control Select
	REG_LDSW2GPISEL 			= 0x0806, 	//Load Switch2 GPIO Control Select
	REG_LDSWCONFIG 				= 0x0807, 	//Load Switch Configuration
	REG_LDSW1LDOSEL 			= 0x0808, 	//Load Switch1 / LDO Select
	REG_LDSW2LDOSEL 			= 0x0809, 	//Load Switch2 / LDO Select
	REG_LDSW1VOUTSEL 			= 0x080C, 	//LDO1 programmable output voltage
	REG_LDSW2VOUTSEL 			= 0x080D, 	//LDO2 programmable output voltage
	//POF
	REG_POFCONFIG 				= 0x0900, 	//Power Failure Detection block configuration
	//LEDDRV
	REG_LEDDRV0MODESEL 			= 0x0A00, 	//Select for LED_0 mode
	REG_LEDDRV1MODESEL 			= 0x0A01, 	//Select for LED_1 mode
	REG_LEDDRV2MODESEL 			= 0x0A02, 	//Select for LED_2 mode
	REG_LEDDRV0SET 				= 0x0A03, 	//Set LED_0 to be On
	REG_LEDDRV0CLR 				= 0x0A04, 	//Clear LED_0 to be Off
	REG_LEDDRV1SET 				= 0x0A05, 	//Set LED_1 to be On
	REG_LEDDRV1CLR 				= 0x0A06, 	//Clear LED_1 to be Off
	REG_LEDDRV2SET 				= 0x0A07, 	//Set LED_2 to be On
	REG_LEDDRV2CLR 				= 0x0A08, 	//Clear LED_2 to be Off
	//SHIP
	REG_TASKENTERHIBERNATE 		= 0x0B00, 	//Task Enter Hibernate
	REG_TASKSHPHLDCFGSTROBE 	= 0x0B01, 	//Task Ship Hold config
	REG_TASKENTERSHIPMODE 		= 0x0B02, 	//Task enter ShipMode
	REG_TASKRESETCFG 			= 0x0B03, 	//Request reset config
	REG_SHPHLDCONFIG 			= 0x0B04, 	//Ship Hold button press timer config
	REG_SHPHLDSTATUS 			= 0x0B05, 	//Status of the SHPHLD pin
	REG_LPRESETCONFIG 			= 0x0B06, 	//Long press reset config register
	//ERR
	REG_TASKCLRERRLOG 			= 0x0E00, 	//task to clear the Errlog registers
	REG_SCRATCH0				= 0x0E01, 	//Scratch register 0
	REG_SCRATCH1 				= 0x0E02, 	//Scratch register 1
	REG_RSTCAUSE 				= 0x0E03, 	//Error log for internal reset causes. Cleared withTASK_CLR_ERRLOG
	REG_CHARGERERRREASON 		= 0x0E04, 	//Error log for slowDomain. Cleared with TASK_CLR_ERRLOG
	REG_CHARGERERRSENSOR 		= 0x0E05, 	//Bcharger Fsm sensor error. Cleared with TASK_CLR_ERRLOG
}NPM1300_REG;

typedef enum
{
	LED_0,
	LED_1,
	LED_2,
	LED_MAX
}NPM1300_LED;

typedef enum
{
	NTC_TEMP_COLD,
	NTC_TEMP_COOL,
	NTC_TEMP_NORMAL,
	NTC_TEMP_WARM,
	NTC_TEMP_HOT,
	NTC_TEMP_MAX
}NPM1300_NTC_TEMP;

typedef enum
{
	NTC_TYPE_NONE,
	NTC_TYPE_10K,
	NTC_TYPE_47K,
	NTC_TYPE_100K,
	NTC_TYPE_MAX
}NPM1300_NTC_TYPE;

typedef enum
{
	CHANNEL_VOLTAGE,
	CHANNEL_TEMP,
	CHANNEL_CURRENT,
	CHANNEL_CHARGER_STATUS,
	CHANNEL_CHARGER_ERROR,
	CHANNEL_MAX_LOAD_CURRENT,
	CHANNEL_DESIGN_VOLTAGE,
	CHANNEL_DESIRED_VOLTAGE,
	CHANNEL_DESIRED_CHARGING_CURRENT,
	CHANNEL_MAX
}NPM1300_CHANNEL_TYPE;

typedef struct
{
	/** Integer part of the value. */
	int32_t val1;
	/** Fractional part of the value (in one-millionth parts). */
	int32_t val2;
}npm1300_sensor_value_t;

typedef struct
{
	uint8_t ibat_stat;
	uint8_t msb_vbat;
	uint8_t msb_ntc;
	uint8_t msb_die;
	uint8_t msb_vsys;
	uint8_t lsb_a;
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t msb_ibat;
	uint8_t msb_vbus;
	uint8_t lsb_b;
}npm1300_adc_result_t;

typedef struct
{
	uint16_t voltage;
	uint16_t current;
	uint16_t temp;
	uint8_t status;
	uint8_t error;
	uint8_t ibat_stat;
	uint8_t vbus_stat;
}npm1300_charger_data_t;

typedef struct
{
	int32_t term_microvolt;			//常温下充电截止电压, 序号0~13 对应电压3.50V~4.45, 0.05v/step
	int32_t term_warm_microvolt;	//热温下充电截止电压，序号0~13 对应电压3.50V~4.45, 0.05v/step
	int32_t current_microamp;		//恒流充电电流限制
	int32_t dischg_limit_microamp;	//放电最大电流限制
	int32_t vbus_limit_microamp;	//VBUS最大电流限制
	int32_t temp_thresholds[4U];	//NTCCold,NTCCool,NTCWarm,NTCHot
	uint32_t thermistor_ohms;		//NTC热敏电阻阻值,单位欧姆(10kΩ47kΩ100kΩ)
	uint16_t thermistor_beta;		//NTC热敏电阻B值,单位K(一般查询供应商表格，这里选择默认值3380K)
	uint8_t thermistor_idx;			//NTC热敏电阻类型序号, 0:No thermistor, 1 NTC10K, 2 NTC47K, 3 NTC100K
	uint8_t trickle_sel;			//涓流电压设置, 0:2.9V(default), 1:2.5V
	uint8_t iterm_sel;				//充电完成截止电流,单位C 0:10%(default), 1:20%
	bool charging_enable;			//充电使能
	bool vbatlow_charge_enable;		//电池低电压时充电使能
	bool disable_recharge;			//重新复充电禁止
}npm1300_charger_config_t;

#endif/*PMU_SENSOR_NPM1300*/
#endif/*__NPM1300_H__*/
