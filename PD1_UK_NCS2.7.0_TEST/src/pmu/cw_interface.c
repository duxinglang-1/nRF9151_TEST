/****************************************Copyright (c)************************************************
** File Name:			    cw_interface.c
** Descriptions:			cw sensor drive interface soyrce file
** Created By:				xie biao
** Created Date:			2024-09-12
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

#ifdef PMU_SENSOR_CW221X_CW630X

#ifdef GPIO_ACT_I2C
#define PMU_SCL		7
#define PMU_SDA		6

#else/*GPIO_ACT_I2C*/

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay)
#define PMU_DEV DT_NODELABEL(i2c3)
#else
#error "i2c3 devicetree node is disabled"
#define PMU_DEV	""
#endif

#define PMU_SCL			7
#define PMU_SDA			6

#endif/*GPIO_ACT_I2C*/

static struct device *gpio_pmu;
static struct device *i2c_pmu;

pmudev_ctx_t pmu_dev_ctx;

void CW_Delay_ms(unsigned int dly)
{
	k_sleep(K_MSEC(dly));
}

void CW_Delay_us(unsigned int dly)
{
	k_usleep(dly);
}

#ifdef GPIO_ACT_I2C
static void I2C_INIT(void)
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

static void I2C_SDA_OUT(void)
{
	gpio_pin_configure(gpio_pmu, PMU_SDA, GPIO_OUTPUT);
}

static void I2C_SDA_IN(void)
{
	gpio_pin_configure(gpio_pmu, PMU_SDA, GPIO_INPUT);
}

static void I2C_SDA_H(void)
{
	gpio_pin_set(gpio_pmu, PMU_SDA, 1);
}

static void I2C_SDA_L(void)
{
	gpio_pin_set(gpio_pmu, PMU_SDA, 0);
}

static void I2C_SCL_H(void)
{
	gpio_pin_set(gpio_pmu, PMU_SCL, 1);
}

static void I2C_SCL_L(void)
{
	gpio_pin_set(gpio_pmu, PMU_SCL, 0);
}

static void I2C_Start(void)
{
	I2C_SDA_OUT();

	I2C_SDA_H();
	I2C_SCL_H();
	I2C_SDA_L();
	I2C_SCL_L();
}

static void I2C_Stop(void)
{
	I2C_SDA_OUT();

	I2C_SCL_L();
	I2C_SDA_L();
	I2C_SCL_H();
	I2C_SDA_H();
}

static void I2C_Ack(void)
{
	I2C_SDA_OUT();
	
	I2C_SDA_L();
	I2C_SCL_L();
	I2C_SCL_H();
	I2C_SCL_L();
}

static void I2C_NAck(void)
{
	I2C_SDA_OUT();
	
	I2C_SDA_H();
	I2C_SCL_L();
	I2C_SCL_H();
	I2C_SCL_L();
}

static uint8_t I2C_Wait_Ack(void)
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

static uint8_t I2C_Write_Byte(uint8_t txd)
{
	uint8_t i=0;

	I2C_SDA_OUT();
	I2C_SCL_L();

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

static void I2C_Read_Byte(bool ack, uint8_t *data)
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

static uint8_t I2C_write_data(uint8_t addr, uint8_t *databuf, uint32_t len)
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

static uint8_t I2C_read_data(uint8_t addr, uint8_t *databuf, uint32_t len)
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

static int32_t platform_write(struct device *handle, uint8_t addr, uint16_t reg, uint8_t *bufp, uint16_t len)
{
	uint32_t rslt = 0;
	uint8_t data[len+1];

	data[0] = (uint8_t)(reg&0x00ff);
	memcpy(&data[1], bufp, len);

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
	uint8_t data[1] = {0};

	data[0] = (uint8_t)(reg&0x00ff);

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
		CW_Delay_ms(10);
		rslt = i2c_read(handle, bufp, len, addr);
	}
#endif
	return rslt;
}

bool CW_I2C_Init(void)
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

void CW_Interface_Init(void)
{
	pmu_dev_ctx.write_reg = platform_write;
	pmu_dev_ctx.read_reg  = platform_read;
#ifdef GPIO_ACT_I2C
	pmu_dev_ctx.handle	  = NULL;
#else
	pmu_dev_ctx.handle	  = i2c_pmu;
#endif

	CW6307_Init();
}

int CW2215_WriteReg(CW2215_REG reg, uint8_t value)
{ 
	int32_t ret;

	ret = pmu_dev_ctx.write_reg(pmu_dev_ctx.handle, CW2215_I2C_ADDR, reg, &value, 1);
	if(ret != 0)
	{
		ret = CW_ERROR;  
	}
	else
	{
		ret = CW_NO_ERROR; 
	}

	return ret;
}

int CW2215_WriteRegMulti(CW2215_REG reg, uint8_t *value, uint8_t len)
{
	int32_t ret;

	ret = pmu_dev_ctx.write_reg(pmu_dev_ctx.handle, CW2215_I2C_ADDR, reg, value, len);
	if(ret != 0)
	{
		ret = CW_ERROR; 
	}
	else
	{ 
		ret = CW_NO_ERROR;
	}

	return ret;
}

int CW2215_ReadReg(CW2215_REG reg, uint8_t *value)
{
	int32_t ret;

	ret = pmu_dev_ctx.read_reg(pmu_dev_ctx.handle, CW2215_I2C_ADDR, reg, value, 1);
	if(ret != 0)
	{
		ret = CW_ERROR;
	}
	else
	{
		ret = CW_NO_ERROR;
	}

	return ret;
}

int CW2215_ReadRegMulti(CW2215_REG reg, uint8_t *value, uint8_t len)
{
	int32_t ret;

	ret = pmu_dev_ctx.read_reg(pmu_dev_ctx.handle, CW2215_I2C_ADDR, reg, value, len);
	if(ret != 0)
		ret = CW_ERROR;
	else
		ret = CW_NO_ERROR;

	return ret;
}


int CW6307_WriteReg(CW2215_REG reg, uint8_t value)
{ 
	int32_t ret;

	ret = pmu_dev_ctx.write_reg(pmu_dev_ctx.handle, CW6307_I2C_ADDR, reg, &value, 1);
	if(ret != 0)
	{
		ret = CW_ERROR;  
	}
	else
	{
		ret = CW_NO_ERROR; 
	}

	return ret;
}

int CW6307_WriteRegMulti(CW2215_REG reg, uint8_t *value, uint8_t len)
{
	int32_t ret;

	ret = pmu_dev_ctx.write_reg(pmu_dev_ctx.handle, CW6307_I2C_ADDR, reg, value, len);
	if(ret != 0)
	{
		ret = CW_ERROR; 
	}
	else
	{ 
		ret = CW_NO_ERROR;
	}

	return ret;
}

int CW6307_ReadReg(CW2215_REG reg, uint8_t *value)
{
	int32_t ret;

	ret = pmu_dev_ctx.read_reg(pmu_dev_ctx.handle, CW6307_I2C_ADDR, reg, value, 1);
	if(ret != 0)
	{
		ret = CW_ERROR;
	}
	else
	{
		ret = CW_NO_ERROR;
	}

	return ret;
}

int CW6307_ReadRegMulti(CW2215_REG reg, uint8_t *value, uint8_t len)
{
	int32_t ret;

	ret = pmu_dev_ctx.read_reg(pmu_dev_ctx.handle, CW6307_I2C_ADDR, reg, value, len);
	if(ret != 0)
		ret = CW_ERROR;
	else
		ret = CW_NO_ERROR;

	return ret;
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

	LOGD("Value of NRF_TWIM1_NS->PSEL.SCL: %ld",NRF_TWIM1->PSEL.SCL);
	LOGD("Value of NRF_TWIM1_NS->PSEL.SDA: %ld",NRF_TWIM1->PSEL.SDA);
	LOGD("Value of NRF_TWIM1_NS->FREQUENCY: %ld",NRF_TWIM1->FREQUENCY);
	LOGD("26738688 -> 100k");
	LOGD("67108864 -> 250k");
	LOGD("104857600 -> 400k");

	for(uint8_t i = 0; i < 0x7f; i++)
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

#endif/*PMU_SENSOR_CW221X_CW630X*/
