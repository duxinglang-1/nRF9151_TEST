/*
* Copyright (c) 2019 Nordic Semiconductor ASA
*
* SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
*/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <stdio.h>
#include <string.h>
#include "logger.h"
#include "transfer_cache.h"
#include "datetime.h"
#include "Settings.h"
#include "Uart.h"
#include "inner_flash.h"

//#define UART_DEBUG

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
#define WIFI_DEV DT_NODELABEL(uart0)
#else
#error "uart0 devicetree node is disabled"
#define WIFI_DEV	""
#endif

#define BUF_MAXSIZE	2048

#ifdef CONFIG_PM_DEVICE
bool uart_wifi_sleep_flag = false;
bool uart_wifi_wake_flag = false;
bool uart_wifi_is_waked = true;
#define UART_WIFI_WAKE_HOLD_TIME_SEC		(5)
#endif

static bool uart_wifi_send_data_flag = false;
static bool uart_wifi_rece_data_flag = false;
static bool uart_wifi_rece_frame_flag = false;

static CacheInfo uart_wifi_send_cache = {0};
static CacheInfo uart_wifi_rece_cache = {0};

static uint32_t wifi_rece_len=0;
static uint32_t wifi_send_len=0;

static uint8_t wifi_rx_buf[BUF_MAXSIZE]={0};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static struct device *uart_wifi = NULL;
static struct gpio_callback gpio_cb;

static struct uart_data_t
{
	void  *fifo_reserved;
	uint8_t    data[BUF_MAXSIZE];
	uint16_t   len;
};

static void UartWifiSendDataCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_wifi_send_data_timer, UartWifiSendDataCallBack, NULL);
static void UartWifiReceDataCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_wifi_rece_data_timer, UartWifiReceDataCallBack, NULL);
static void UartWifiReceFrameCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_wifi_rece_frame_timer, UartWifiReceFrameCallBack, NULL);
#ifdef CONFIG_PM_DEVICE
static void UartWifiSleepInCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_wifi_sleep_in_timer, UartWifiSleepInCallBack, NULL);
#endif

void UartSetWifiBaudRate(uint32_t baudrate)
{
	struct uart_config uart_cfg;

	uart_config_get(uart_wifi, &uart_cfg);
	uart_cfg.baudrate = baudrate;
	uart_configure(uart_wifi, &uart_cfg);
}

static void uart_send_data_handle(struct device *dev, uint8_t *buffer, uint32_t datalen)
{
#ifdef CONFIG_PM_DEVICE
	uart_sleep_out(dev);
#endif

#ifdef UART_DEBUG
	LOGD("len:%d, data:%s", datalen, buffer);
#endif

	uart_fifo_fill(dev, buffer, datalen);
	uart_irq_tx_enable(dev); 	
}

static void uart_receive_data_handle(struct device *dev, uint8_t *data, uint32_t datalen)
{
	uint32_t data_len = 0;

#ifdef UART_DEBUG
	LOGD("uart for wifi!");
#endif
#ifdef CONFIG_WIFI_SUPPORT
	wifi_receive_data_handle(data, datalen);
#endif
}

void UartWifiSendData(void)
{
	uint8_t data_type,*p_data;
	uint32_t data_len;
	int ret;

	ret = get_data_from_cache(&uart_wifi_send_cache, &p_data, &data_len, &data_type);
	if(ret)
	{
	#ifdef UART_DEBUG
		LOGD("begin");
	#endif
		uart_send_data_handle(uart_wifi, p_data, data_len);
		delete_data_from_cache(&uart_wifi_send_cache);
		k_timer_start(&uart_wifi_send_data_timer, K_MSEC(50), K_NO_WAIT);
	}
}

void UartWifiSendDataStart(void)
{
	k_timer_start(&uart_wifi_send_data_timer, K_MSEC(50), K_NO_WAIT);
}

bool WifiSendCacheIsEmpty(void)
{
	if(cache_is_empty(&uart_wifi_send_cache))
		return true;
	else
		return false;
}

void WifiSendData(uint8_t *data, uint32_t datalen)
{
	int ret;

	ret = add_data_into_cache(&uart_wifi_send_cache, data, datalen, DATA_TRANSFER);
	UartWifiSendDataStart();
}

void UartWifiReceData(void)
{
	uint8_t data_type,*p_data;
	uint32_t data_len;
	int ret;

	ret = get_data_from_cache(&uart_wifi_rece_cache, &p_data, &data_len, &data_type);
	if(ret)
	{
		uart_receive_data_handle(uart_wifi, p_data, data_len);
		delete_data_from_cache(&uart_wifi_rece_cache);
		k_timer_start(&uart_wifi_rece_data_timer, K_MSEC(50), K_NO_WAIT);
	}
}

void WifiReceDataStart(void)
{
	k_timer_start(&uart_wifi_rece_data_timer, K_MSEC(50), K_NO_WAIT);
}

bool WifiReceCacheIsEmpty(void)
{
	if(cache_is_empty(&uart_wifi_rece_cache))
		return true;
	else
		return false;
}

void WifiReceData(uint8_t *data, uint32_t datalen)
{
	int ret;

	ret = add_data_into_cache(&uart_wifi_rece_cache, data, datalen, DATA_TRANSFER);
	WifiReceDataStart();
}

static void uart_wifi_cb(struct device *x)
{
	uint8_t tmpbyte = 0;
	uint32_t len=0;

	uart_irq_update(x);

	if(uart_irq_rx_ready(x)) 
	{
		if(wifi_rece_len >= BUF_MAXSIZE)
			wifi_rece_len = 0;

		while((len = uart_fifo_read(x, &wifi_rx_buf[wifi_rece_len], BUF_MAXSIZE-wifi_rece_len)) > 0)
		{
			wifi_rece_len += len;
			k_timer_start(&uart_wifi_rece_frame_timer, K_MSEC(20), K_NO_WAIT);
		}
	}
	
	if(uart_irq_tx_ready(x))
	{
		struct uart_data_t *buf;
		uint16_t written = 0;

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		/* Nothing in the FIFO, nothing to send */
		if(!buf)
		{
			uart_irq_tx_disable(x);
			return;
		}

		while(buf->len > written)
		{
			written += uart_fifo_fill(x, &buf->data[written], buf->len - written);
		}

		while (!uart_irq_tx_complete(x))
		{
			/* Wait for the last byte to get
			* shifted out of the module
			*/
		}

		if (k_fifo_is_empty(&fifo_uart_tx_data))
		{
			uart_irq_tx_disable(x);
		}

		k_free(buf);
	}
}

#ifdef CONFIG_PM_DEVICE
void uart_sleep_out(struct device *dev)
{
	if(uart_wifi_is_waked)
		return;
	uart_wifi_is_waked = true;
	
	pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);

#ifdef UART_DEBUG
	LOGD("uart set active success!");
#endif
}

void uart_sleep_in(struct device *dev)
{	
	if(!uart_wifi_is_waked)
		return;
	uart_wifi_is_waked = false;
	
	pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);

#ifdef UART_DEBUG
	LOGD("uart set low power success!");
#endif
}

static void UartWifiSleepInCallBack(struct k_timer *timer_id)
{
#ifdef UART_DEBUG
	LOGD("begin");
#endif
	uart_wifi_sleep_flag = true;
}
#endif

static void UartWifiSendDataCallBack(struct k_timer *timer)
{
	uart_wifi_send_data_flag = true;
}

static void UartWifiReceDataCallBack(struct k_timer *timer_id)
{
	uart_wifi_rece_data_flag = true;
}

static void UartWifiReceFrameCallBack(struct k_timer *timer_id)
{
	uart_wifi_rece_frame_flag = true;
}

void UartWifiOff(void)
{
	if(k_timer_remaining_get(&uart_wifi_send_data_timer) > 0)
		k_timer_stop(&uart_wifi_send_data_timer);
	delete_all_from_cache(&uart_wifi_send_cache);
	
#ifdef CONFIG_PM_DEVICE
	uart_sleep_in(uart_wifi);
#endif
}

void uart_wifi_init(void)
{
#ifdef UART_DEBUG
	LOGD("begin");
#endif

	if(uart_wifi == NULL)
	{
		uart_wifi = DEVICE_DT_GET(WIFI_DEV);
		uart_irq_callback_set(uart_wifi, uart_wifi_cb);
		uart_irq_rx_enable(uart_wifi);

	#ifdef CONFIG_PM_DEVICE
		k_timer_start(&uart_wifi_sleep_in_timer, K_SECONDS(UART_WIFI_WAKE_HOLD_TIME_SEC), K_NO_WAIT);
	#endif
	}
}

void UartMsgProc(void)
{
#ifdef CONFIG_PM_DEVICE
	if(uart_wifi_wake_flag)
	{
		uart_wifi_wake_flag = false;
		uart_sleep_out(uart_wifi);
	}

	if(uart_wifi_sleep_flag)
	{
		uart_wifi_sleep_flag = false;
		uart_sleep_in(uart_wifi);
	}
#endif/*CONFIG_PM_DEVICE*/

	if(uart_wifi_send_data_flag)
	{
		UartWifiSendData();
		uart_wifi_send_data_flag = false;
	}

	if(uart_wifi_rece_data_flag)
	{
		UartWifiReceData();
		uart_wifi_rece_data_flag = false;
	}
	
	if(uart_wifi_rece_frame_flag)
	{
		WifiReceData(wifi_rx_buf, wifi_rece_len);
		memset(wifi_rx_buf, 0, sizeof(wifi_rx_buf));
		wifi_rece_len = 0;
		uart_wifi_rece_frame_flag = false;
	}
}
