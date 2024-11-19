/****************************************Copyright (c)************************************************
** File Name:			    audio.c
** Descriptions:			audio process source file
** Created By:				xie biao
** Created Date:			2021-03-04
** Modified Date:      		2021-05-08 
** Version:			    	V1.1
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include "audio.h"
#include "logger.h"

#if 0
#define I2S_MCK		25
#define I2S_LRCK	13
#define I2S_BCK		14
#define I2S_DI		16
#define I2S_DO		17

#define I2S_DATA_BLOCK_WORDS    512
static uint32_t m_buffer_rx[2][I2S_DATA_BLOCK_WORDS];
static uint32_t m_buffer_tx[2][I2S_DATA_BLOCK_WORDS];

// Delay time between consecutive I2S transfers performed in the main loop
// (in milliseconds).
#define PAUSE_TIME          500
// Number of blocks of data to be contained in each transfer.
#define BLOCKS_TO_TRANSFER  20

static uint8_t volatile m_blocks_transferred     = 0;
static uint8_t          m_zero_samples_to_ignore = 0;
static uint16_t         m_sample_value_to_send;
static uint16_t         m_sample_value_expected;
static bool             m_error_encountered;

static uint32_t       * volatile mp_block_to_fill  = NULL;
static uint32_t const * volatile mp_block_to_check = NULL;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
#define AUDIO_PORT DT_NODELABEL(gpio0)
#else
#error "gpio0 devicetree node is disabled"
#define AUDIO_PORT	""
#endif

#define WTN_DATA	20
#define WTN_BUSY	21

static bool audio_trige_flag = false;

static struct device *gpio_audio;
static struct gpio_callback gpio_cb;

//延时函数
static void Delay_ms(unsigned int dly)
{
	k_sleep(K_MSEC(dly));
}

static void Delay_us(unsigned int dly)
{
	k_sleep(K_USEC(dly));
}

//发送一个字节数据
void Audio_Send_ByteData(uint8_t data)
{
	uint8_t j;
	
	gpio_pin_set(gpio_audio, WTN_DATA, 0);
	Delay_ms(5);
	
	for(j=0;j<8;j++)
	{
		if(data&0x01)
		{
			gpio_pin_set(gpio_audio, WTN_DATA, 1);
			Delay_us(600);
			gpio_pin_set(gpio_audio, WTN_DATA, 0);
			Delay_us(200);
		}
		else
		{
			gpio_pin_set(gpio_audio, WTN_DATA, 1);
			Delay_us(200);
			gpio_pin_set(gpio_audio, WTN_DATA, 0);
			Delay_us(600);
			
		}
		data >>= 1;
	}
	
	gpio_pin_set(gpio_audio, WTN_DATA, 1);
}

//控制音量
void Volume_Control(unsigned char vol)  //E0  ------  EF
{
	Audio_Send_ByteData(vol);
	Delay_us(400);
}

//播放语音
void Voice_Start(uint8_t voice_addr)  //0  -----------  6
{
	Audio_Send_ByteData(voice_addr);
}

//停止播放
void Voice_Stop(void)
{
	Audio_Send_ByteData(0xFE);
}

//循环播放当前语音
void Voice_Loop(void)
{	
	Delay_us(400);
	Audio_Send_ByteData(0xF2);
}

//播放120报警声
void audio_play_alarm(void)
{
	Voice_Start(3);
}

//播放中文语音提示
void audio_play_chn_voice(void)
{
	Voice_Start(1);
}

//播放英文语音提示
void audio_play_en_voice(void)
{
	Voice_Start(2);
}

//停止播放
void audio_stop(void)
{
	Voice_Stop();
}

//SOS停止播放报警
void SOSStopAlarm(void)
{
	Voice_Stop();
}

//SOS播放报警
void SOSPlayAlarm(void)
{
	Voice_Start(3);
}

//摔倒停止播放报警
void FallStopAlarm(void)
{
	Voice_Stop();
}

//摔倒播放中文报警
void FallPlayAlarmCn(void)
{
	Voice_Start(1);
}

//摔倒播放英文报警
void FallPlayAlarmEn(void)
{
	Voice_Start(2);
}

void AudioInterruptHandle(void)
{
	LOGD("begin");
	audio_trige_flag = true;
}

//io口初始化 
void audio_init(void)
{
	gpio_flags_t flag = GPIO_INPUT|GPIO_PULL_UP;
	
	gpio_audio = DEVICE_DT_GET(AUDIO_PORT);
	if(gpio_audio == NULL)
		return;

	gpio_pin_configure(gpio_audio, WTN_DATA, GPIO_OUTPUT);
	gpio_pin_set(gpio_audio, WTN_DATA, 1);

	//busy interrupt
	gpio_pin_configure(gpio_audio, WTN_BUSY, flag);
    gpio_pin_interrupt_configure(gpio_audio, WTN_BUSY, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb, AudioInterruptHandle, BIT(WTN_BUSY));
	gpio_add_callback(gpio_audio, &gpio_cb);
    gpio_pin_interrupt_configure(gpio_audio, WTN_BUSY, GPIO_INT_ENABLE|GPIO_INT_EDGE_RISING);

	Delay_ms(100);
}

void AudioMsgProcess(void)
{
	if(audio_trige_flag)
	{
		audio_trige_flag = false;
	}
}
