/****************************************Copyright (c)************************************************
** File Name:			    npm1300.c
** Descriptions:			npm1300 sensor message process source file
** Created By:				xie biao
** Created Date:			2024-02-28
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/linear_range.h>
#include "..\..\nrfxlib\nrf_fuel_gauge\include\nrf_fuel_gauge.h"
#include "pmu.h"
#include "npm1300.h"
#include "datetime.h"
#include "settings.h"
#include "external_flash.h"
#include "logger.h"

static const struct battery_model battery_model = {
#include "battery_model.inc"
};

#ifdef PMU_SENSOR_NPM1300

//#define SHOW_LOG_IN_SCREEN

#ifdef GPIO_ACT_I2C
#define PMU_SCL		0
#define PMU_SDA		1

#else/*GPIO_ACT_I2C*/

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
#define PMU_DEV DT_NODELABEL(i2c1)
#else
#error "i2c1 devicetree node is disabled"
#define PMU_DEV	""
#endif

#define PMU_SCL			0
#define PMU_SDA			1

#endif/*GPIO_ACT_I2C*/

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio1), okay)
#define PMU_PORT DT_NODELABEL(gpio1)
#else
#error "gpio0 devicetree node is disabled"
#define PMU_PORT	""
#endif

#define PMU_ALRTB		8
#define PMU_EINT		9

static uint8_t HardwareID;
static uint8_t FirmwareID;

static bool pmu_check_ok = false;
static uint8_t PMICStatus[4], PMICInts[3];
static struct device *i2c_pmu;
static struct device *gpio_pmu;
static struct gpio_callback gpio_cb1,gpio_cb2;

/* Linear range for charger terminal voltage */
static const struct linear_range charger_volt_ranges[] = {
	LINEAR_RANGE_INIT(3500000, 50000, 0U, 3U), LINEAR_RANGE_INIT(4000000, 50000, 4U, 13U)};

/* Linear range for charger current */
static const struct linear_range charger_current_range = LINEAR_RANGE_INIT(32000, 2000, 16U, 400U);

/* Linear range for Discharge limit */
static const struct linear_range discharge_limit_range = LINEAR_RANGE_INIT(268090, 3230, 83U, 415U);

/* Linear range for vbusin current limit */
static const struct linear_range vbus_current_ranges[] = {
	LINEAR_RANGE_INIT(100000, 0, 1U, 1U), LINEAR_RANGE_INIT(500000, 100000, 5U, 15U)};

static float max_charge_current;
static float term_charge_current;
static int64_t ref_time;

static npm1300_charger_data_t charger_data = {0};
static npm1300_charger_config_t charger_config = 
									{
										4350000,
										4000000,
										150000,
										1000000,
										500000,
										{
											0x7fffffff,
											0x7fffffff,
											0x7fffffff,
											0x7fffffff
										},
										10000,
										3380,
										1,
										0,
										0,
										true,
										true,
										false
									};

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
static void nPM1300_CheckTemp(struct k_timer *timer_id);
K_TIMER_DEFINE(ntc_check_timer, nPM1300_CheckTemp, NULL);
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

pmudev_ctx_t pmu_dev_ctx;

extern bool key_pwroff_flag;
extern int nrf_fuel_gauge_init(const struct nrf_fuel_gauge_init_parameters *parameters, float *v0);

void Delay_ms(unsigned int dly)
{
	k_sleep(K_MSEC(dly));
}

void Delay_us(unsigned int dly)
{
	k_usleep(dly);
}

#ifdef GPIO_ACT_I2C
void I2C_INIT(void)
{
	if(gpio_pmu == NULL)
		gpio_pmu = DEVICE_DT_GET(PMU_PORT);

	gpio_pin_configure(gpio_pmu, PMU_SCL, GPIO_OUTPUT);
	gpio_pin_configure(gpio_pmu, PMU_SDA, GPIO_OUTPUT);
	gpio_pin_set(gpio_pmu, PMU_SCL, 0);
	gpio_pin_set(gpio_pmu, PMU_SCL, 1);
	gpio_pin_set(gpio_pmu, PMU_SDA, 0);
	gpio_pin_set(gpio_pmu, PMU_SDA, 1);
}

void I2C_SDA_OUT(void)
{
	gpio_pin_configure(gpio_pmu, PMU_SDA, GPIO_OUTPUT);
}

void I2C_SDA_IN(void)
{
	gpio_pin_configure(gpio_pmu, PMU_SDA, GPIO_INPUT);
}

void I2C_SDA_H(void)
{
	gpio_pin_set(gpio_pmu, PMU_SDA, 1);
}

void I2C_SDA_L(void)
{
	gpio_pin_set(gpio_pmu, PMU_SDA, 0);
}

void I2C_SCL_H(void)
{
	gpio_pin_set(gpio_pmu, PMU_SCL, 1);
}

void I2C_SCL_L(void)
{
	gpio_pin_set(gpio_pmu, PMU_SCL, 0);
}

//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½Åºï¿½
void I2C_Start(void)
{
	I2C_SDA_OUT();

	I2C_SDA_H();
	I2C_SCL_H();
	I2C_SDA_L();
	I2C_SCL_L();
}

//ï¿½ï¿½ï¿½ï¿½Í£Ö¹ï¿½Åºï¿½
void I2C_Stop(void)
{
	I2C_SDA_OUT();

	I2C_SCL_L();
	I2C_SDA_L();
	I2C_SCL_H();
	I2C_SDA_H();
}

//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ï¿½Åºï¿½ACK
void I2C_Ack(void)
{
	I2C_SDA_OUT();
	
	I2C_SDA_L();
	I2C_SCL_L();
	I2C_SCL_H();
	I2C_SCL_L();
}

//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ï¿½Åºï¿½NACK
void I2C_NAck(void)
{
	I2C_SDA_OUT();
	
	I2C_SDA_H();
	I2C_SCL_L();
	I2C_SCL_H();
	I2C_SCL_L();
}

//ï¿½È´ï¿½ï¿½Ó»ï¿½Ó¦ï¿½ï¿½ï¿½Åºï¿½
//ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½1 ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½Ê§ï¿½ï¿½
//		  0 ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½É¹ï¿½
uint8_t I2C_Wait_Ack(void)
{
	uint8_t val,tempTime=0;

	I2C_SDA_IN();
	I2C_SCL_H();

	while(1)
	{
		val = gpio_pin_get_raw(gpio_pmu, PMU_SDA);
		if(val == 0)
			break;
		
		tempTime++;
		if(tempTime>250)
		{
			I2C_Stop();
			return 1;
		}	 
	}

	I2C_SCL_L();
	return 0;
}

//I2C ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½Ö½ï¿½
uint8_t I2C_Write_Byte(uint8_t txd)
{
	uint8_t i=0;

	I2C_SDA_OUT();
	I2C_SCL_L();//ï¿½ï¿½ï¿½ï¿½Ê±ï¿½Ó¿ï¿½Ê¼ï¿½ï¿½ï¿½Ý´ï¿½ï¿½ï¿½

	for(i=0;i<8;i++)
	{
		if((txd&0x80)>0) //0x80  1000 0000
			I2C_SDA_H();
		else
			I2C_SDA_L();

		txd<<=1;
		I2C_SCL_H();
		I2C_SCL_L();
	}

	return I2C_Wait_Ack();
}

//I2C ï¿½ï¿½È¡Ò»ï¿½ï¿½ï¿½Ö½ï¿½
void I2C_Read_Byte(bool ack, uint8_t *data)
{
	uint8_t i=0,receive=0,val=0;

	I2C_SDA_IN();
	I2C_SCL_L();

	for(i=0;i<8;i++)
	{
		I2C_SCL_H();
		receive<<=1;
		val = gpio_pin_get_raw(gpio_pmu, PMU_SDA);
		if(val == 1)
			receive++;
		I2C_SCL_L();
	}

	if(ack == false)
		I2C_NAck();
	else
		I2C_Ack();

	*data = receive;
}

uint8_t I2C_write_data(uint8_t addr, uint8_t *databuf, uint32_t len)
{
	uint32_t i;

	addr = (addr<<1);

	I2C_Start();
	if(I2C_Write_Byte(addr))
		goto err;

	for(i=0;i<len;i++)
	{
		if(I2C_Write_Byte(databuf[i]))
			goto err;
	}

	I2C_Stop();
	return 0;
	
err:
	return -1;
}

uint8_t I2C_read_data(uint8_t addr, uint8_t *databuf, uint32_t len)
{
	uint32_t i;

	addr = (addr<<1)|1;

	I2C_Start();
	if(I2C_Write_Byte(addr))
		goto err;

	for(i=0;i<len;i++)
	{
		if(i == len-1)
			I2C_Read_Byte(false, &databuf[i]);
		else
			I2C_Read_Byte(true, &databuf[i]);
	}
	I2C_Stop();
	return 0;
	
err:
	return -1;
}
#endif

static bool init_i2c(void)
{
#ifdef GPIO_ACT_I2C
	I2C_INIT();
	return true;
#else
	i2c_pmu = DEVICE_DT_GET(PMU_DEV);
	if(!i2c_pmu)
	{
	#ifdef PMU_DEBUG
		LOGD("ERROR SETTING UP I2C");
	#endif
		return false;
	} 
	else
	{
		i2c_configure(i2c_pmu, I2C_SPEED_SET(I2C_SPEED_STANDARD));
		return true;
	}
#endif	
}

static int32_t platform_write(struct device *handle, uint8_t addr, uint16_t reg, uint8_t *bufp, uint16_t len)
{
	uint32_t rslt = 0;
	uint8_t data[len+2];

	data[0] = (uint8_t)(reg>>8);
	data[1] = (uint8_t)(reg&0x00ff);
	memcpy(&data[2], bufp, len);

#ifdef GPIO_ACT_I2C
	rslt = I2C_write_data(addr, data, sizeof(data));
#else
	rslt = i2c_write(handle, data, sizeof(data), addr);
#endif

	return rslt;
}

static int32_t platform_read(struct device *handle, uint8_t addr, uint16_t reg, uint8_t *bufp, uint16_t len)
{
	uint32_t rslt = 0;
	uint8_t data[2] = {0};

	data[0] = (uint8_t)(reg>>8);
	data[1] = (uint8_t)(reg&0x00ff);

#ifdef GPIO_ACT_I2C
	rslt = I2C_write_data(addr, data, sizeof(data));
	if(rslt == 0)
	{
		rslt = I2C_read_data(addr, bufp, len);
	}
#else
	rslt = i2c_write(handle, data, sizeof(data), addr);
	if(rslt == 0)
	{
		Delay_ms(10);
		rslt = i2c_read(handle, bufp, len, addr);
	}
#endif
	return rslt;
}

static int nPM1300_WriteRegMulti(NPM1300_REG reg, uint8_t *value, uint8_t len)
{
	int32_t ret;

	ret = pmu_dev_ctx.write_reg(pmu_dev_ctx.handle, NPM1300_I2C_ADDR, reg, value, len);
	if(ret != 0)
	{
		ret = NPM1300_ERROR; 
	}
	else
	{ 
		ret = NPM1300_NO_ERROR;
	}

	return ret;
}

static int nPM1300_WriteReg(NPM1300_REG reg, uint8_t value)
{ 
    int32_t ret;

	ret = pmu_dev_ctx.write_reg(pmu_dev_ctx.handle, NPM1300_I2C_ADDR, reg, &value, 1);
	if(ret != 0)
	{
		ret = NPM1300_ERROR;  
	}
	else
	{
		ret = NPM1300_NO_ERROR; 
	}

	return ret;
}

static int nPM1300_ReadReg(NPM1300_REG reg, uint8_t *value)
{
    int32_t ret;

	ret = pmu_dev_ctx.read_reg(pmu_dev_ctx.handle, NPM1300_I2C_ADDR, reg, value, 1);
    if(ret != 0)
    {
        ret = NPM1300_ERROR;
    }
    else
    {
        ret = NPM1300_NO_ERROR;
    }

    return ret;
}

static int nPM1300_ReadRegMulti(NPM1300_REG reg, uint8_t *value, uint8_t len)
{
    int32_t ret;

	ret = pmu_dev_ctx.read_reg(pmu_dev_ctx.handle, NPM1300_I2C_ADDR, reg, value, len);
    if(ret != 0)
        ret = NPM1300_ERROR;
    else
        ret = NPM1300_NO_ERROR;

    return ret;
}

static uint32_t calc_ntc_res(npm1300_charger_config_t config, int32_t temp_mdegc)
{
	float inv_t0 = 1.f / 298.15f;
	float temp = (float)temp_mdegc / 1000000.f;
	float inv_temp_k = 1.f / (temp + 273.15f);

	return config.thermistor_ohms * exp((float)config.thermistor_beta * (inv_temp_k - inv_t0));
}

static int set_ntc_thresholds(npm1300_charger_config_t config)
{
	int ret;
	uint8_t idx,data[5];
	uint16_t code;
	uint32_t res;
	
	for(idx=0;idx<4;idx++)
	{
		if(config.temp_thresholds[idx] < INT32_MAX)
		{
			res = calc_ntc_res(config, config.temp_thresholds[idx]);

			/* Ref: Datasheet Figure 14: Equation for battery temperature */
			code = (1024 * res) / (res + config.thermistor_ohms);
			data[0] = code>>NTCTEMP_MSB_SHIFT;
			data[1] = code&NTCTEMP_LSB_MASK;
			
			ret = nPM1300_WriteRegMulti(REG_NTCCOLD+(2*idx), &data[0], 2);
			if(ret != NPM1300_NO_ERROR)
				return ret;
		}
	}

	return 0;
}

static uint16_t adc_get_res(uint8_t msb, uint8_t lsb, uint16_t lsb_shift)
{
	return ((uint16_t)msb << ADC_MSB_SHIFT) | ((lsb >> lsb_shift) & ADC_LSB_MASK);
}

static void calc_temp(npm1300_charger_config_t config, uint16_t code, npm1300_sensor_value_t *valp)
{
	/* Ref: Datasheet Figure 42: Battery temperature (Kelvin) */
	float log_result = log((1024.f / (float)code) - 1);
	float inv_temp_k = (1.f / 298.15f) - (log_result / (float)config.thermistor_beta);

	float temp = (1.f / inv_temp_k) - 273.15f;

	valp->val1 = (int32_t)temp;
	valp->val2 = (int32_t)(fmodf(temp, 1.f) * 1000000.f);
}

static void calc_current(npm1300_charger_config_t config, npm1300_charger_data_t data, npm1300_sensor_value_t *valp)
{
	int32_t full_scale_ma;
	int32_t current;

	switch(data.ibat_stat)
	{
	case IBAT_STAT_DISCHARGE:
		full_scale_ma = config.dischg_limit_microamp / 1000;
		break;
	case IBAT_STAT_CHARGE_TRICKLE:
		full_scale_ma = -config.current_microamp / 10000;
		break;
	case IBAT_STAT_CHARGE_COOL:
		full_scale_ma = -config.current_microamp / 2000;
		break;
	case IBAT_STAT_CHARGE_NORMAL:
		full_scale_ma = -config.current_microamp / 1000;
		break;
	default:
		full_scale_ma = 0;
		break;
	}

	current = (data.current * full_scale_ma) / 1024;

	valp->val1 = current / 1000;
	valp->val2 = (current % 1000) * 1000;
}

static int nPM1300_ReadSonsor(npm1300_charger_data_t *data)
{
	int ret;
	npm1300_adc_result_t adc_ret = {0};
	
	/* Read charge status and error reason */
	ret = nPM1300_ReadReg(REG_BCHGCHARGESTATUS, &data->status);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	ret = nPM1300_ReadReg(REG_BCHGERRREASON, &data->error);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Read adc results */
	ret = nPM1300_ReadRegMulti(REG_ADCIBATMEASSTATUS, &adc_ret, sizeof(adc_ret));
	if(ret != NPM1300_NO_ERROR)
		return ret;

	data->voltage = adc_get_res(adc_ret.msb_vbat, adc_ret.lsb_a, ADC_LSB_VBAT_SHIFT);
	data->temp = adc_get_res(adc_ret.msb_ntc, adc_ret.lsb_a, ADC_LSB_NTC_SHIFT);
	data->current = adc_get_res(adc_ret.msb_ibat, adc_ret.lsb_b, ADC_LSB_IBAT_SHIFT);
	data->ibat_stat = adc_ret.ibat_stat;

	/* Trigger temperature measurement */
	ret = nPM1300_WriteReg(REG_TASKNTCMEASURE, 0x01);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Trigger current and voltage measurement */
	ret = nPM1300_WriteReg(REG_TASKVBATMEASURE, 0x01);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Read vbus status */
	ret = nPM1300_ReadReg(REG_VBUSINSTATUS, &data->vbus_stat);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	return ret;
}

static void nPM1300_ReadChannel(npm1300_charger_config_t config, npm1300_charger_data_t data, NPM1300_CHANNEL_TYPE chan, npm1300_sensor_value_t *valp)
{
	int32_t tmp;

	switch(chan)
	{
	case CHANNEL_VOLTAGE:
		tmp = data.voltage * 5000 / 1024;
		valp->val1 = tmp / 1000;
		valp->val2 = (tmp % 1000) * 1000;
		break;
	case CHANNEL_TEMP:
		calc_temp(config, data.temp, valp);
		break;
	case CHANNEL_CURRENT:
		calc_current(config, data, valp);
		break;
	case CHANNEL_CHARGER_STATUS:
		valp->val1 = data.status;
		valp->val2 = 0;
		break;
	case CHANNEL_CHARGER_ERROR:
		valp->val1 = data.error;
		valp->val2 = 0;
		break;
	case CHANNEL_DESIRED_CHARGING_CURRENT:
		valp->val1 = config.current_microamp / 1000000;
		valp->val2 = config.current_microamp % 1000000;
		break;
	case CHANNEL_MAX_LOAD_CURRENT:
		valp->val1 = config.dischg_limit_microamp / 1000000;
		valp->val2 = config.dischg_limit_microamp % 1000000;
		break;		
	}
}

static int nPM1300_ReadData(float *voltage, float *current, float *temp)
{
	npm1300_sensor_value_t value;
	int ret;

	ret = nPM1300_ReadSonsor(&charger_data);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	nPM1300_ReadChannel(charger_config, charger_data, CHANNEL_VOLTAGE, &value);
	*voltage = (float)value.val1 + ((float)value.val2 / 1000000);

	nPM1300_ReadChannel(charger_config, charger_data, CHANNEL_TEMP, &value);
	*temp = (float)value.val1 + ((float)value.val2 / 1000000);

	nPM1300_ReadChannel(charger_config, charger_data, CHANNEL_CURRENT, &value);
	*current = (float)value.val1 + ((float)value.val2 / 1000000);

	return 0;
}

#ifdef BATTERY_NTC_CHECK
uint16_t nPM1300_GetNTCTemp(void)
{
	uint8_t data = 0;
	uint16_t ntc_temp,die_temp;

	nPM1300_ReadReg(REG_ADCNTCRESULTMSB, &data);
	//ADC NTC measurement result MSB
	//0b A A A A A A A A
	//A ADC NTC thermistor Battery measurement result upper 8-bits
	ntc_temp = (uint16_t)(data<<8);

	nPM1300_ReadReg(REG_ADCTEMPRESULTMSB, &data);
	//ADC DIE TEMP measurement result MSB
	//0b A A A A A A A A
	//ADC Die Temperature measurement result upper 8-bits
	die_temp = (uint16_t)(data<<8);
	
	nPM1300_ReadReg(REG_ADCGP0RESULTLSBS, &data);
	//ADC result LSB's (Vbat, Ntc, Temp and Vsys)
	//0b D D C C B B A A
	//A VBAT measurement result LSBs
	//B Battery NTC thermistor measurement result LSBs
	//C Die Temperature measurement result LSBs
	//D VSYS measurement result LSBs
	ntc_temp |= ((data>>2)&0x03);
	die_temp |= ((data>>4)&0x03);

#ifdef PMU_DEBUG
	LOGD("ntc_t:%d, die_t:%d", ntc_temp, die_temp);
#endif
	return ntc_temp;
}

uint16_t nPM1300_GetDieTemp(void)
{
	uint8_t data;
	uint16_t die_temp;

	nPM1300_ReadReg(REG_ADCTEMPRESULTMSB, &data);
	//ADC DIE TEMP measurement result MSB
	//0b A A A A A A A A
	//ADC Die Temperature measurement result upper 8-bits
	die_temp = (uint16_t)(data<<8);
	
	nPM1300_ReadReg(REG_ADCGP0RESULTLSBS, &data);
	//ADC result LSB's (Vbat, Ntc, Temp and Vsys)
	//0b D D C C B B A A
	//A VBAT measurement result LSBs
	//B Battery NTC thermistor measurement result LSBs
	//C Die Temperature measurement result LSBs
	//D VSYS measurement result LSBs
	die_temp |= (data&0x30);

#ifdef PMU_DEBUG
	LOGD("die_t:%d", die_temp);
#endif
	return die_temp;
}

static void nPM1300_CheckTemp(struct k_timer *timer_id)
{
	pmu_check_temp_flag = true;
}

#endif/*BATTERY_NTC_CHECK*/

void nPM1300_Reset(void)
{
	nPM1300_WriteReg(REG_TASKSWRESET, 0x01);		//Turn off all Supplies and apply internal reset
	k_sleep(K_MSEC(50));
}

void nPM1300_Buck1Disable(void)
{
	nPM1300_WriteReg(REG_BUCK1ENACLR, 0x01);		//Disable buck1 output
}

void nPM1300_Buck1Config(void)
{
	nPM1300_WriteReg(REG_BUCK1ENASET, 0x01);		//Enable buck1 output
	nPM1300_WriteReg(REG_BUCK1NORMVOUT, 0x08);		//1.8v value:0~23, voltage:1.0v~3.3V(0.1v per step)
	nPM1300_WriteReg(REG_BUCKSWCTRLSEL, 0x01);		//Allow SW to override VSET1 pin
}

void nPM1300_Buck2Disable(void)
{
	nPM1300_WriteReg(REG_BUCK2ENACLR, 0x01);
}

void nPM1300_Buck2Config(void)
{
	nPM1300_WriteReg(REG_BUCK2ENASET, 0x01);		//Enable buck1 output
	nPM1300_WriteReg(REG_BUCK2NORMVOUT, 0x17);		//3.3v value:0~23, voltage:1.0v~3.3V(0.1v per step)
	nPM1300_WriteReg(REG_BUCKSWCTRLSEL, 0x02);		//Allow SW to override VSET2 pin
}

void nPM1300_LDO1Disable(void)
{
	nPM1300_WriteReg(REG_TASKLDSW1CLR, 0x01);		//Disable LDO1 output
}

void nPM1300_LDO1Config(void)
{
	nPM1300_WriteReg(REG_TASKLDSW1SET, 0x01);		//Enable LOD1 output
	nPM1300_WriteReg(REG_LDSW1LDOSEL, 0x01);		//Select LDSW1 or LDO1, 0:load switch 1:LDO
	nPM1300_WriteReg(REG_LDSW1VOUTSEL, 0x14);		//3.0v value:0~23, voltage:1.0v~3.3V(0.1v per step)
}

void nPM1300_LDO2Disable(void)
{
	nPM1300_WriteReg(REG_TASKLDSW2CLR, 0x01);		//Disable ldo2 output
}

void nPM1300_LDO2Config(void)
{
	nPM1300_WriteReg(REG_TASKLDSW2SET, 0x01);		//Enable LDO2 output
	nPM1300_WriteReg(REG_LDSW2LDOSEL, 0x01);		//Select LDSW2 or LDO2, 0:load switch 1:LDO
	nPM1300_WriteReg(REG_LDSW2VOUTSEL, 0x08);		//1.8v value:0~23, voltage:1.0v~3.3V(0.1v per step)
}

void nPM1300_LEDConfig(NPM1300_LED index, bool flag)
{
	static bool init_flag = false;

	if(!init_flag)
	{
		//Select for LED0 mode
		//0: Error condition from Charger;
		//1: Charging indicator (On during charging);
		//2: Driven from register LEDDRV_0_SET/CLR
		init_flag = true;
		nPM1300_WriteReg(REG_LEDDRV0MODESEL, 0x02);
		nPM1300_WriteReg(REG_LEDDRV1MODESEL, 0x02);
		nPM1300_WriteReg(REG_LEDDRV2MODESEL, 0x02);
	}
	
	if(flag)
		nPM1300_WriteReg(REG_LEDDRV0SET+2*index, 0x01);
	else
		nPM1300_WriteReg(REG_LEDDRV0CLR+2*index, 0x01);
}


void nPM1300_GetNTCStatus(void)
{
	uint8_t data[5] = {0};

	nPM1300_ReadReg(REG_ADCNTCRSEL, &data[0]);
	switch(data[0]&0x03)
	{
	case 0://No thermistor
	#ifdef PMU_DEBUG
		LOGD("No thermistor");
	#endif
		break;
	case 1://NTC10K
	#ifdef PMU_DEBUG
		LOGD("NTC10K");
	#endif
		break;
	case 2://NTC47K
	#ifdef PMU_DEBUG
		LOGD("NTC47K");
	#endif
		break;
	case 3://NTC100K
	#ifdef PMU_DEBUG
		LOGD("NTC100K");
	#endif
		break;
	}

	nPM1300_ReadRegMulti(REG_NTCCOLD, &data, 2);
#ifdef PMU_DEBUG
	LOGD("NTCCOLD:%X %X, temp:%d", data[0], data[1], 0);
#endif
	nPM1300_ReadRegMulti(REG_NTCCOOL, &data, 2);
#ifdef PMU_DEBUG
	LOGD("NTCCOOL:%X %X, temp:%d", data[0], data[1], 0);
#endif
	nPM1300_ReadRegMulti(REG_NTCWARM, &data, 2);
#ifdef PMU_DEBUG
	LOGD("NTCWARM:%X %X, temp:%d", data[0], data[1], 0);
#endif
	nPM1300_ReadRegMulti(REG_NTCHOT, &data, 2);
#ifdef PMU_DEBUG
	LOGD("NTCHOT:%X %X, temp:%d", data[0], data[1], 0);
#endif
	
	
	nPM1300_ReadReg(REG_NTCSTATUS, &data[0]);	//0b D C B A, A NTCCOLD, B NTCCOOL, C NTCWARM, D NTCHOT
	switch(data[0]&0x0f)
	{
	case 0:
	#ifdef PMU_DEBUG
		LOGD("NTC_NO-RMAL");
	#endif
		break;
	case 1:
	#ifdef PMU_DEBUG
		LOGD("NTC_COLD");
	#endif
		break;
	case 2:
	#ifdef PMU_DEBUG
		LOGD("NTC_COOL");
	#endif
		break;
	case 4:
	#ifdef PMU_DEBUG
		LOGD("NTC_WARM");
	#endif
		break;
	case 8:
	#ifdef PMU_DEBUG
		LOGD("NTC_HOT");
	#endif
		break;
	}
		
	nPM1300_ReadReg(REG_DIETEMPSTATUS, &data[0]);
	switch(data[0]&0x01)
	{
	case 0:
	#ifdef PMU_DEBUG
		LOGD("Die below high threshold");
	#endif
		break;
	case 1:
	#ifdef PMU_DEBUG
		LOGD("Die above high threshold");
	#endif
		break;
	}
}

void nPM1300_GetUSBStatus(bool *status)
{
	uint8_t data;
	
	nPM1300_ReadReg(REG_USBCDETECTSTATUS, &data);
	if(data != 0x00)
	{
		*status = true;
		
		switch(data&0x03)//CC1 Charger detection comparator output
		{
		case 0:
		#ifdef PMU_DEBUG
			LOGD("cc1 no connection");
		#endif
			break;
		case 1:
		#ifdef PMU_DEBUG	
			LOGD("cc1 Default USB 100/500mA");
		#endif
			break;
		case 2:
		#ifdef PMU_DEBUG	
			LOGD("cc1 1.5A High Power");
		#endif
			break;
		case 3:
		#ifdef PMU_DEBUG	
			LOGD("cc1 3A High Power");
		#endif
			break;
		}
		switch((data&0x0c)>>2)//CC2 Charger detection comparator output
		{
		case 0:
		#ifdef PMU_DEBUG	
			LOGD("cc2 no connection");
		#endif
			break;
		case 1:
		#ifdef PMU_DEBUG	
			LOGD("cc2 Default USB 100/500mA");
		#endif
			break;
		case 2:
		#ifdef PMU_DEBUG	
			LOGD("cc2 1.5A High Power");
		#endif
			break;
		case 3:
		#ifdef PMU_DEBUG	
			LOGD("cc2 3A High Power");
		#endif
			break;
		}
	}
	else
	{
		*status = false;
	}
}

void nPM1300_GetChargeStatus(BAT_CHARGER_STATUS *status)
{
	uint8_t data;
	
	nPM1300_ReadReg(REG_BCHGCHARGESTATUS, &data);
#ifdef PMU_DEBUG
	LOGD("BCHGCHARGESTATUS:%0X", data);
#endif
	if(data != 0x00)
	{
	#ifdef PMU_DEBUG
		LOGD("it is charging!");
	#endif
		if((data&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery is connected.");
		#endif
		}
		if((data&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("Charging completed (Battery Full).");
		#endif
			*status = BAT_CHARGING_FINISHED;
		}
		if((data&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("Trickle charge.");
		#endif
			*status = BAT_CHARGING_PROGRESS;
		}
		if((data&0x08) == 0x08)
		{
		#ifdef PMU_DEBUG
			LOGD("Constant Current charging.");
		#endif
			*status = BAT_CHARGING_PROGRESS;
		}
		if((data&0x10) == 0x10)
		{
		#ifdef PMU_DEBUG
			LOGD("Constant Voltage charging.");
		#endif
			*status = BAT_CHARGING_PROGRESS;
		}
		if((data&0x20) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery re-charge is needed.");
		#endif
		}
		if((data&0x40) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("Charging stopped due Die Temp high.");
		#endif
			*status = BAT_CHARGING_NO;
		}
		if((data&0x80) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("Supplement Mode Active.");
		#endif
		}
	}
	else
	{
	#ifdef PMU_DEBUG
		LOGD("it is not charging!");
	#endif
		*status = BAT_CHARGING_NO;
	}
}

void nPM1300_GetBatStatus(void)
{
	float voltage, current, temp;
	
	nPM1300_ReadData(&voltage, &current, &temp);
#ifdef PMU_DEBUG
	LOGD("vol:%f, current:%f, temp:%f", voltage, current, temp);
#endif
}

uint8_t nPM1300_GetSocStatus(void)
{
	float voltage,current,temp;
	float soc,tte,ttf;
	float delta;
	uint8_t bat_soc = 0;
	int ret;

	ret = nPM1300_ReadData(&voltage, &current, &temp);
	if(ret != NPM1300_NO_ERROR)
		return 0;

	delta = (float) k_uptime_delta(&ref_time) / 1000.f;

	soc = nrf_fuel_gauge_process(voltage, current, temp, delta, NULL);
	//Get predicted "time-to-empty" discharge duration.[s]
	tte = nrf_fuel_gauge_tte_get();
	//Get predicted "time-to-full" charging duration.[s]
	ttf = nrf_fuel_gauge_ttf_get(-max_charge_current, -term_charge_current);

	if(soc > 0.0)
		bat_soc = (uint8_t)round(soc);
	else
		bat_soc = 0;

#ifdef PMU_DEBUG
	LOGD("V: %.3f, I: %.3f, T: %.2f", voltage, current, temp);
	LOGD("bat_soc: %d, SoC: %.2f, TTE: %.0f, TTF: %.0f", bat_soc, soc, tte, ttf);
#endif

	return bat_soc;
}

void nPM1300_SetBatChargeConfig(void)
{
	nPM1300_WriteReg(REG_BCHGCONFIG, 0x00);//Disable charging if battery is warm. 0:enable 1:disable
}

void nPM1300_SetNTCTypeSel(NPM1300_NTC_TYPE data)
{
	if(data >= NTC_TYPE_MAX)
		return;

	nPM1300_WriteReg(REG_ADCNTCRSEL, 0x00+data);
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
{
}

void VibrateStop(void)
{
}

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

	nPM1300_WriteReg(REG_TASKENTERSHIPMODE, 0x01);
}

void pmu_charger_status_indicate(BAT_CHARGER_STATUS chg_status)
{
	BAT_CHARGER_STATUS status_bk = BAT_CHARGING_MAX;
	
	switch(chg_status)
	{
	case BAT_CHARGING_NO:
		nPM1300_LEDConfig(LED_0, false);
		nPM1300_LEDConfig(LED_1, false);
		break;
		
	case BAT_CHARGING_PROGRESS:
		nPM1300_LEDConfig(LED_0, true);
		nPM1300_LEDConfig(LED_1, false);
		break;
		
	case BAT_CHARGING_FINISHED:
		nPM1300_LEDConfig(LED_0, false);
		nPM1300_LEDConfig(LED_1, true);
		break;
	}

	status_bk = chg_status;
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
	uint8_t tmpbuf[8] = {0};
	
	if(!pmu_check_ok)
		return;

	g_bat_soc = nPM1300_GetSocStatus();
#ifdef PMU_DEBUG
	LOGD("SOC:%d", g_bat_soc);
#endif	

	if(g_bat_soc > 100)
		g_bat_soc = 100;

	if(charger_is_connected)
	{
		g_bat_level = BAT_LEVEL_NORMAL;
	}
	else
	{
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
	
#ifdef CONFIG_FACTORY_TEST_SUPPORT
	FTPMUStatusUpdate(1);
#endif	
}

void pmu_status_update(void)
{
	uint8_t status0,status1;
	static BAT_CHARGER_STATUS chg_status_bk = BAT_CHARGING_MAX;

	if(!pmu_check_ok)
		return;
	
	nPM1300_GetUSBStatus(&charger_is_connected);	
	if(charger_is_connected)
	{
		nPM1300_GetChargeStatus(&g_chg_status);
		
	#ifdef BATTERY_SOC_GAUGE	
		g_bat_soc = nPM1300_GetSocStatus();
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
	#endif
	}
	else
	{			
		charger_is_connected = false;
		
		g_chg_status = BAT_CHARGING_NO;
		
	#ifdef BATTERY_SOC_GAUGE	
		g_bat_soc = nPM1300_GetSocStatus();
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
	#endif
	}

#ifdef PMU_DEBUG
	LOGD("chg_bk:%d, chg:%d, soc:%d, bat_level:%d", chg_status_bk, g_chg_status, g_bat_soc, g_bat_level);
#endif

	if(chg_status_bk != g_chg_status)
	{
		chg_status_bk = g_chg_status;
		pmu_charger_status_indicate(g_chg_status);
	}
}

bool pmu_interrupt_proc(void)
{
	uint8_t data[5] = {0};
	int ret;

	Delay_ms(10);
			
	nPM1300_ReadReg(REG_EVENTSBCHARGER0SET, &data[0]);
#ifdef PMU_DEBUG
	LOGD("EVENTSBCHARGER0SET:%0X", data[0]);
#endif
	if(data[0] != 0x00)
	{
		nPM1300_WriteReg(REG_EVENTSBCHARGER0CLR, data[0]);
		if((data[0]&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("Cold Battery detected from NTC measure.");
		#endif
		}
		if((data[0]&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("Cool Battery detected from NTC measure.");
		#endif
		}
		if((data[0]&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("Warm Battery detected from NTC measure.");
		#endif
		}
		if((data[0]&0x08) == 0x08)
		{
		#ifdef PMU_DEBUG
			LOGD("Hot Battery detected from NTC measure.");
		#endif
		}
		if((data[0]&0x10) == 0x10)
		{
		#ifdef PMU_DEBUG
			LOGD("die high temperature detected from Die Temp measure.");
		#endif
		}
		if((data[0]&0x20) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("die resume temperature detected from Die Temp measure.");
		#endif
		}
	}
	
	nPM1300_ReadReg(REG_EVENTSBCHARGER1SET, &data[0]);
#ifdef PMU_DEBUG
	LOGD("EVENTSBCHARGER1SET:%0X", data[0]);
#endif
	if(data[0] != 0x00)
	{
		nPM1300_WriteReg(REG_EVENTSBCHARGER1CLR, data[0]);
		if((data[0]&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("supplement mode activated.");
		#endif
		}
		if((data[0]&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("Trickle Charge started.");
		#endif
		}
		if((data[0]&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("Constant Current charging started.");
		#endif
		}
		if((data[0]&0x08) == 0x08)
		{
		#ifdef PMU_DEBUG
			LOGD("Constant Voltage charging started.");
		#endif
		}
		if((data[0]&0x10) == 0x10)
		{
		#ifdef PMU_DEBUG
			LOGD("charging completed (Battery Full).");
		#endif
		}
		if((data[0]&0x20) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("charging error.");
		#endif
		}
	}
	
	nPM1300_ReadReg(REG_EVENTSBCHARGER2SET, &data[0]);
#ifdef PMU_DEBUG
	LOGD("EVENTSBCHARGER2SET:%0X", data[0]);
#endif
	if(data[0] != 0x00)
	{
		nPM1300_WriteReg(REG_EVENTSBCHARGER2CLR, data[0]);
		if((data[0]&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery Detected.");
		#endif
		}
		if((data[0]&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery Lost.");
		#endif
		}
		if((data[0]&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery re-charge needed.");
		#endif
		}
	}
	
	nPM1300_ReadReg(REG_EVENTSVBUSIN0SET, &data[0]);
#ifdef PMU_DEBUG
	LOGD("EVENTSVBUSIN0SET:%0X", data[0]);
#endif
	if(data[0] != 0x00)
	{
		nPM1300_WriteReg(REG_EVENTSVBUSIN0CLR, data[0]);
		if((data[0]&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("VBUS input detected.");
		#endif
		}
		if((data[0]&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("VBUS input removed.");
		#endif
		}
		if((data[0]&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("VBUS Over Voltage Detected.");
		#endif
		}
		if((data[0]&0x08) == 0x08)
		{
		#ifdef PMU_DEBUG
			LOGD("VBUS Over Removed.");
		#endif
		}
		if((data[0]&0x10) == 0x10)
		{
		#ifdef PMU_DEBUG
			LOGD("VBUS Under Voltage Detected.");
		#endif
		}
		if((data[0]&0x20) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("VBUS Under Removed.");
		#endif
		}
	}

	nPM1300_ReadReg(REG_EVENTSVBUSIN1SET, &data[0]);
#ifdef PMU_DEBUG
	LOGD("EVENTSVBUSIN1SET:%0X", data[0]);
#endif
	if(data[0] != 0x00)
	{
		nPM1300_WriteReg(REG_EVENTSVBUSIN1CLR, data[0]);
		if((data[0]&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("Thermal Warning detected.");
		#endif
		}
		if((data[0]&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("Thermal Warning removed.");
		#endif
		}
		if((data[0]&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("Thermal Shutown detected.");
		#endif
		}		
		if((data[0]&0x08) == 0x08)
		{
		#ifdef PMU_DEBUG
			LOGD("Thermal Shutdown removed.");
		#endif
		}
		if((data[0]&0x10) == 0x10)
		{
		#ifdef PMU_DEBUG
			LOGD("when Voltage on CC1 changes.");
		#endif
		}
		if((data[0]&0x20) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("when Voltage on CC2 changes.");
		#endif
		}
	}

	nPM1300_ReadReg(REG_USBCDETECTSTATUS, &data[0]);
#ifdef PMU_DEBUG
	LOGD("USBCDETECTSTATUS:%0X", data[0]);
#endif
	if(data[0] != 0x00)
	{
	#ifdef PMU_DEBUG
		LOGD("charger has been inserted!");
	#endif
		charger_is_connected = true;
	
		switch(data[0]&0x03)//CC1 Charger detection comparator output
		{
		case 0:
		#ifdef PMU_DEBUG
			LOGD("cc1 no connection");
		#endif
			break;
		case 1:
		#ifdef PMU_DEBUG
			LOGD("c1 Default USB 100/500mA");
		#endif
			break;
		case 2:
		#ifdef PMU_DEBUG
			LOGD("cc1 1.5A High Power");
		#endif
			break;
		case 3:
		#ifdef PMU_DEBUG
			LOGD("cc1 3A High Power");
		#endif
			break;
		}
		switch((data[0]&0x0c)>>2)//CC2 Charger detection comparator output
		{
		case 0:
		#ifdef PMU_DEBUG
			LOGD("cc2 no connection");
		#endif
			break;
		case 1:
		#ifdef PMU_DEBUG
			LOGD("cc2 Default USB 100/500mA");
		#endif
			break;
		case 2:
		#ifdef PMU_DEBUG
			LOGD("cc2 1.5A High Power");
		#endif
			break;
		case 3:
		#ifdef PMU_DEBUG
			LOGD("cc2 3A High Power");
		#endif
			break;
		}

		pmu_battery_stop_shutdown();
	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		FTPMUStatusUpdate(2);
	#endif	
	}
	else
	{
	#ifdef PMU_DEBUG
		LOGD("charger has been removed!");
	#endif
		charger_is_connected = false;
		g_chg_status = BAT_CHARGING_NO;

	#ifdef BATTERY_SOC_GAUGE	
		g_bat_soc = nPM1300_GetSocStatus();
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
	#endif

	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		FTPMUStatusUpdate(2);
	#endif
	}

	nPM1300_ReadReg(REG_NTCSTATUS, &data[0]);	//0b D C B A, A NTCCOLD, B NTCCOOL, C NTCWARM, D NTCHOT
	switch(data[0]&0x0f)
	{
	case 0:
	#ifdef PMU_DEBUG
		LOGD("NTC_NORMAL");
	#endif
		break;
	case 1:
	#ifdef PMU_DEBUG
		LOGD("NTC_COLD");
	#endif
		break;
	case 2:
	#ifdef PMU_DEBUG
		LOGD("NTC_COOL");
	#endif
		break;
	case 4:
	#ifdef PMU_DEBUG
		LOGD("NTC_WARM");
	#endif
		break;
	case 8:
	#ifdef PMU_DEBUG
		LOGD("NTC_HOT");
	#endif
		break;
	}
	
	nPM1300_ReadReg(REG_BCHGCHARGESTATUS, &data[0]);
#ifdef PMU_DEBUG
	LOGD("BCHGCHARGESTATUS:%0X", data[0]);
#endif
	if(data[0] != 0x00)
	{
	#ifdef PMU_DEBUG
		LOGD("it is charging!");
	#endif
		if((data[0]&0x01) == 0x01)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery is connected.");
		#endif
			g_chg_status = BAT_CHARGING_NO;
		}
		if((data[0]&0x02) == 0x02)
		{
		#ifdef PMU_DEBUG
			LOGD("Charging completed (Battery Full).");
		#endif
			g_chg_status = BAT_CHARGING_FINISHED;
		}
		if((data[0]&0x04) == 0x04)
		{
		#ifdef PMU_DEBUG
			LOGD("Trickle charge.");
		#endif
			g_chg_status = BAT_CHARGING_PROGRESS;
		}
		if((data[0]&0x08) == 0x08)
		{
		#ifdef PMU_DEBUG
			LOGD("Constant Current charging.");
		#endif
			g_chg_status = BAT_CHARGING_PROGRESS;
		}
		if((data[0]&0x10) == 0x10)
		{
		#ifdef PMU_DEBUG
			LOGD("Constant Voltage charging.");
		#endif
			g_chg_status = BAT_CHARGING_PROGRESS;
		}
		if((data[0]&0x20) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("Battery re-charge is needed.");
		#endif
			g_chg_status = BAT_CHARGING_PROGRESS;
		}
		if((data[0]&0x40) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("Charging stopped due Die Temp high.");
		#endif
			g_chg_status = BAT_CHARGING_NO;
		}
		if((data[0]&0x80) == 0x20)
		{
		#ifdef PMU_DEBUG
			LOGD("Supplement Mode Active.");
		#endif
			g_chg_status = BAT_CHARGING_PROGRESS;
		}

		pmu_charger_status_indicate(g_chg_status);
	}
	else
	{
	#ifdef PMU_DEBUG
		LOGD("it is not charging!");
	#endif
		g_chg_status = BAT_CHARGING_NO;
		pmu_charger_status_indicate(g_chg_status);
	}

	//ISetMsb = floor(Ichg(mA)/4), ISetLsb = (Ichg(mA)%2 == 1 ? 1 : 0), from 32 mA to 800 mA, in 2 mA steps
	nPM1300_ReadRegMulti(REG_BCHGISETMSB, &data, 2);
#ifdef PMU_DEBUG
	LOGD("BCHGISETMSB:%0X %0X, Ichg=%dmA", data[0],data[1], 4*data[0]+2*(data[1]%2));
#endif	
	nPM1300_ReadReg(REG_BCHGITERMSEL, &data[0]);
	switch(data[0])
	{
	case 0:
	#ifdef PMU_DEBUG	
		LOGD("10%(default)");
	#endif
		break;
	case 1:
	#ifdef PMU_DEBUG
		LOGD("20%(default)");
	#endif
		break;
	}

	nPM1300_WriteReg(REG_VBUSINILIM0, 0x05);
	nPM1300_WriteReg(REG_TASKUPDATEILIMSW, 0x01);
}

void PmuInterruptHandle(void)
{
	pmu_trige_flag = true;
}

//An alert can indicate many different conditions. The
//STATUS register identifies which alert condition was met.
//Clear the corresponding bit after servicing the alert
bool pmu_alert_proc(void)
{
	uint8_t i;
	int ret;
	uint8_t MSB=0,LSB=0;

#if 0
	if(!pmu_check_ok)
		return true;

#ifdef PMU_DEBUG
	LOGD("begin");
#endif

#ifdef BATTERY_SOC_GAUGE
	ret = MAX20353_SOCReadReg(0x1A, &MSB, &LSB);
	if(ret == NPM1300_ERROR)
		return false;
	
#ifdef PMU_DEBUG
	LOGD("status:%02X", MSB);
#endif
	if(MSB&0x40)
	{
	#ifdef PMU_DEBUG
		LOGD("enable voltage reset alert!");
	#endif
		//EnVr (enable voltage reset alert)
		MSB = MSB&0xBF;
	}
	if(MSB&0x20)
	{
	#ifdef PMU_DEBUG
		LOGD("SOC change alert!");
	#endif
		//SC (1% SOC change) is set when SOC changes by at least 1% if CONFIG.ALSC is set
		MSB = MSB&0xDF;

		pmu_battery_update();
	}
	if(MSB&0x10)
	{
	#ifdef PMU_DEBUG
		LOGD("SOC low alert!");
	#endif
		//HD (SOC low) is set when SOC crosses the value in CONFIG.ATHD
		MSB = MSB&0xEF;
	}
	if(MSB&0x08)
	{
	#ifdef PMU_DEBUG
		LOGD("voltage reset alert!");
	#endif
		//VR (voltage reset) is set after the device has been reset if EnVr is set.
		MSB = MSB&0xF7;
	}
	if(MSB&0x04)
	{
	#ifdef PMU_DEBUG
		LOGD("voltage low alert!");
	#endif
		//VL (voltage low) is set when VCELL has been below ALRT.VALRTMIN
		MSB = MSB&0xFB;
	}
	if(MSB&0x02)
	{
	#ifdef PMU_DEBUG
		LOGD("voltage high alert!");
	#endif
		//VH (voltage high) is set when VCELL has been above ALRT.VALRTMAX
		MSB = MSB&0xFD;
	}
	if(MSB&0x01)
	{
	#ifdef PMU_DEBUG
		LOGD("reset indicator alert!");
	#endif
		//RI (reset indicator) is set when the device powers up.
		//Any time this bit is set, the IC is not configured, so the
		//model should be loaded and the bit should be cleared
		MSB = MSB&0xFE;
		MAX20353_SOCWriteReg(0x1A, MSB, LSB);

		handle_model(LOAD_MODEL);
		MAX20353_QuickStart();
		delay_ms(150);

		goto SOC_RESET;
	}

	ret = MAX20353_SOCWriteReg(0x1A, MSB, LSB);
	if(ret == NPM1300_ERROR)
		return false;

SOC_RESET:	
	ret = MAX20353_SOCWriteReg(0x0C, RCOMP0, 0x5C);
	if(ret == NPM1300_ERROR)
		return false;

	return true;
#endif
#endif
}

void PmuAlertHandle(void)
{
	pmu_alert_flag = true;
}

void nPM1300_InitData(void)
{
	nPM1300_GetUSBStatus(&charger_is_connected);
	if(charger_is_connected)
	{
		nPM1300_GetChargeStatus(&g_chg_status);
		
	#ifdef BATTERY_SOC_GAUGE	
		g_bat_soc = nPM1300_GetSocStatus();
		if(g_bat_soc > 100)
			g_bat_soc = 100;

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
	#endif
	}
	else
	{			
		charger_is_connected = false;
		
		g_chg_status = BAT_CHARGING_NO;
		
	#ifdef BATTERY_SOC_GAUGE	
		g_bat_soc = nPM1300_GetSocStatus();
		if(g_bat_soc>100)
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
	#endif
	}

	pmu_charger_status_indicate(g_chg_status);
	
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

void nPM1300_PowerSupply(void)
{
	nPM1300_Buck1Config();	//1.8v
	nPM1300_Buck2Config();	//3.3v
	nPM1300_LDO1Config();	//3.0v
	nPM1300_LDO2Config();	//1.8v

	//LED¿ØÖÆ
	nPM1300_LEDConfig(LED_0, true);
	nPM1300_LEDConfig(LED_1, true);
	nPM1300_LEDConfig(LED_2, true);
	nPM1300_LEDConfig(LED_0, false);
	nPM1300_LEDConfig(LED_1, false);
	nPM1300_LEDConfig(LED_2, false);
}

void nPM1300_SOCInit(void)
{
	npm1300_sensor_value_t value;
	struct nrf_fuel_gauge_init_parameters parameters = { .model = &battery_model };
	int ret;

	ret = nPM1300_ReadData(&parameters.v0, &parameters.i0, &parameters.t0);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Store charge nominal and termination current, needed for ttf calculation */
	nPM1300_ReadChannel(charger_config, charger_data, CHANNEL_DESIRED_CHARGING_CURRENT, &value);
	max_charge_current = (float)value.val1 + ((float)value.val2 / 1000000);
	term_charge_current = max_charge_current / 10.f;

	nrf_fuel_gauge_init(&parameters, NULL);

	ref_time = k_uptime_get();
}

int nPM1300_ChargerInit(void)
{
	uint8_t data[5] = {0};
	uint16_t idx;
	int ret;
	
	//Set GPIO_0 mode for GPO Interrupt
	nPM1300_WriteReg(REG_GPIO_0_MODE, 0x05);

	//Select Battery NTC
	nPM1300_WriteReg(REG_ADCNTCRSEL, charger_config.thermistor_idx);

	ret = set_ntc_thresholds(charger_config);
	if(ret != 0)
		return ret;

	/* Configure termination voltages */
	ret = linear_range_group_get_win_index(charger_volt_ranges, ARRAY_SIZE(charger_volt_ranges),
					       					charger_config.term_microvolt, charger_config.term_microvolt,
					       					&idx);
	if(ret == -EINVAL)
		return ret;

	ret = nPM1300_WriteReg(REG_BCHGVTERM, idx);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	ret = linear_range_group_get_win_index(charger_volt_ranges, ARRAY_SIZE(charger_volt_ranges),
					       					charger_config.term_warm_microvolt,
					       					charger_config.term_warm_microvolt, &idx);
	if(ret == -EINVAL)
		return ret;

	ret = nPM1300_WriteReg(REG_BCHGVTERMR, idx);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Set current, allow rounding down to closest value */
	ret = linear_range_get_win_index(&charger_current_range,
					 					charger_config.current_microamp - charger_current_range.step,
					 					charger_config.current_microamp, &idx);
	if(ret == -EINVAL)
		return ret;

	data[0] = idx/2;
	data[1] = idx&0x01;
	ret = nPM1300_WriteRegMulti(REG_BCHGISETMSB, data ,2);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Set discharge limit, allow rounding down to closest value */
	ret = linear_range_get_win_index(&discharge_limit_range,
					 					charger_config.dischg_limit_microamp - discharge_limit_range.step,
					 					charger_config.dischg_limit_microamp, &idx);
	if(ret == -EINVAL)
		return ret;

	data[0] = idx/2;
	data[1] = idx&0x01;
	ret = nPM1300_WriteRegMulti(REG_BCHGISETDISCHARGEMSB, data ,2);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Configure vbus current limit */
	ret = linear_range_group_get_win_index(vbus_current_ranges, ARRAY_SIZE(vbus_current_ranges),
					       charger_config.vbus_limit_microamp,
					       charger_config.vbus_limit_microamp, &idx);
	if (ret == -EINVAL)
		return ret;

	ret = nPM1300_WriteReg(REG_VBUSINILIMSTARTUP, idx);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Configure trickle voltage threshold */
	ret = nPM1300_WriteReg(REG_BCHGVTRICKLESEL, charger_config.trickle_sel);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Configure termination current */
	ret = nPM1300_WriteReg(REG_BCHGITERMSEL, charger_config.iterm_sel);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Enable current measurement */
	ret = nPM1300_WriteReg(REG_ADCIBATMEASEN, 0x01);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Trigger current and voltage measurement */
	ret = nPM1300_WriteReg(REG_TASKVBATMEASURE, 0x01);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Trigger temperature measurement */
	ret = nPM1300_WriteReg(REG_TASKNTCMEASURE, 0x01);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Enable automatic temperature measurements during charging */
	ret = nPM1300_WriteReg(REG_TASKAUTOTIMUPDATE, 0x01);
	if(ret != NPM1300_NO_ERROR)
		return ret;

	/* Enable charging at low battery if configured */
	if(charger_config.vbatlow_charge_enable)
	{
		ret = nPM1300_WriteReg(REG_BCHGVBATLOWEN, 0x01);
		if(ret != NPM1300_NO_ERROR)
			return ret;
	}

	/* Disable automatic recharging if configured */
	if(charger_config.disable_recharge)
	{
		ret = nPM1300_WriteReg(REG_BCHGDISABLESET, 0x01);
		if(ret != NPM1300_NO_ERROR)
			return ret;
	}

	/* Enable charging if configured */
	if(charger_config.charging_enable)
	{
		ret = nPM1300_WriteReg(REG_BCHGENABLESET, 0x01);
		if(ret != NPM1300_NO_ERROR)
			return ret;
	}	

	return 0;
}

bool nPM1300_Init(void)
{
	//³äµçÉèÖÃ
	nPM1300_ChargerInit();
	//SOCµçÁ¿
	nPM1300_SOCInit();
	//¹©µçÊä³ö
	nPM1300_PowerSupply();

	return true;
}

void pmu_init(void)
{
	bool rst;
	gpio_flags_t flag = GPIO_INPUT|GPIO_PULL_DOWN;

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

#if 1
	//charger interrupt
	gpio_pin_configure(gpio_pmu, PMU_EINT, flag);
	gpio_pin_interrupt_configure(gpio_pmu, PMU_EINT, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb1, PmuInterruptHandle, BIT(PMU_EINT));
	gpio_add_callback(gpio_pmu, &gpio_cb1);
	gpio_pin_interrupt_configure(gpio_pmu, PMU_EINT, GPIO_INT_ENABLE|GPIO_INT_EDGE_RISING);

	//alert interrupt
	gpio_pin_configure(gpio_pmu, PMU_ALRTB, flag);
	gpio_pin_interrupt_configure(gpio_pmu, PMU_ALRTB, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb2, PmuAlertHandle, BIT(PMU_ALRTB));
	gpio_add_callback(gpio_pmu, &gpio_cb2);
	gpio_pin_interrupt_configure(gpio_pmu, PMU_ALRTB, GPIO_INT_ENABLE|GPIO_INT_EDGE_RISING);
#endif
	rst = init_i2c();
	if(!rst)
		return;

	pmu_dev_ctx.write_reg = platform_write;
	pmu_dev_ctx.read_reg  = platform_read;
#ifdef GPIO_ACT_I2C
	pmu_dev_ctx.handle    = NULL;
#else
	pmu_dev_ctx.handle    = i2c_pmu;
#endif

	pmu_check_ok = nPM1300_Init();
	if(!pmu_check_ok)
		return;
	
	nPM1300_InitData();

#ifdef PMU_DEBUG
	LOGD("pmu_init done!");
#endif
}

#ifndef GPIO_ACT_I2C
void test_i2c(void)
{
	struct device *i2c_dev;

	LOGD("Starting i2c scanner...");
	i2c_dev = DEVICE_DT_GET(PMU_DEV);
	if(!i2c_dev)
	{
		LOGD("I2C: Device driver not found.");
		return;
	}
	i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_FAST));

	LOGD("Value of NRF_TWIM1_NS->PSEL.SCL: %ld",NRF_TWIM1_NS->PSEL.SCL);
	LOGD("Value of NRF_TWIM1_NS->PSEL.SDA: %ld",NRF_TWIM1_NS->PSEL.SDA);
	LOGD("Value of NRF_TWIM1_NS->FREQUENCY: %ld",NRF_TWIM1_NS->FREQUENCY);
	LOGD("26738688 -> 100k");
	LOGD("67108864 -> 250k");
	LOGD("104857600 -> 400k");

	for (uint8_t i = 0; i < 0x7f; i++)
	{
		struct i2c_msg msgs[1];
		uint8_t dst = 1;
		uint8_t error = 0u;

		/* Send the address to read from */
		msgs[0].buf = &dst;
		msgs[0].len = 1U;
		msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

		error = i2c_transfer(i2c_dev, &msgs[0], 1, i);
		if(error == 0)
		{
			LOGD("0x%2x device address found on I2C Bus", i);
		}
		else
		{
			//LOGD("error %d", error);
		}
	}
}
#endif

void test_pmu(void)
{
    pmu_init();
}

#ifdef BATTERY_SOC_GAUGE
void test_soc_status(void)
{
	//nPM1300_GetBatStatus();
	nPM1300_GetSocStatus();
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
		LOGD("usb int");
	#endif
		pmu_trige_flag = false;

		pmu_interrupt_proc();
	}
	
	if(pmu_alert_flag)
	{
	#ifdef PMU_DEBUG
		LOGD("alert");
	#endif
		//ret = pmu_alert_proc();
		//if(ret)
		{
			pmu_alert_flag = false;
		}
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
