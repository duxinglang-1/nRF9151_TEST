/****************************************Copyright (c)************************************************
** File Name:			    esp8266.c
** Descriptions:			wifi process source file
** Created By:				xie biao
** Created Date:			2021-03-29
** Modified Date:      		2021-03-29 
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>
#include "esp8266.h"
#include "uart.h"
#include "logger.h"
#include "transfer_cache.h"

//#define WIFI_DEBUG

#define WIFI_EN_PIN		1
#define WIFI_RST_PIN	2

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
#define WIFI_PORT DT_NODELABEL(gpio0)
#else
#error "gpio0 devicetree node is disabled"
#define WIFI_PORT	""
#endif

#define WIFI_RETRY_COUNT_MAX	5
#define WIFI_AUTO_OFF_TIME_SEC	(1)

uint8_t g_wifi_mac_addr[20] = {0};
uint8_t g_wifi_ver[20] = {0};

static uint8_t retry = 0;

static struct device *gpio_wifi = NULL;

bool sos_wait_wifi = false;
bool fall_wait_wifi = false;
bool location_wait_wifi = false;
bool wifi_is_on = false;

uint8_t wifi_test_info[256] = {0};

static bool app_wifi_on = false;
static bool wifi_on_flag = false;
static bool wifi_off_flag = false;
static bool test_wifi_flag = false;
static bool wifi_test_update_flag = false;
static bool wifi_rescanning_flag = false;
static bool wifi_wait_timerout_flag = false;
static bool wifi_off_retry_flag = false;
static bool wifi_off_ok_flag = false;
static bool wifi_get_infor_flag = false;

static wifi_infor wifi_data = {0};

static void WifiGetInforCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(wifi_get_infor_timer, WifiGetInforCallBack, NULL);
static void APP_Ask_wifi_Data_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(wifi_scan_timer, APP_Ask_wifi_Data_timerout, NULL);
static void wifi_rescan_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(wifi_rescan_timer, wifi_rescan_timerout, NULL);
static void wifi_off_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(wifi_off_retry_timer, wifi_off_timerout, NULL);
static void wifi_turn_off_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(wifi_turn_off_timer, wifi_turn_off_timerout, NULL);

static void APP_Ask_wifi_Data_timerout(struct k_timer *timer_id)
{
	wifi_wait_timerout_flag = true;
}

static void wifi_rescan_timerout(struct k_timer *timer_id)
{
	wifi_rescanning_flag = true;
}

static void wifi_off_timerout(struct k_timer *timer_id)
{
	wifi_off_retry_flag = true;
}

static void wifi_turn_off_timerout(struct k_timer *timer_id)
{
	wifi_off_flag = true;
}

void wifi_scanned_wait_timerout(void)
{
	retry++;
	if(retry < WIFI_RETRY_COUNT_MAX)
	{
	#ifdef WIFI_DEBUG
		LOGD("rescan!");
	#endif
		wifi_rescanning_flag = true;
		memset(&wifi_data, 0, sizeof(wifi_data));
		k_timer_start(&wifi_scan_timer, K_MSEC(5000), K_NO_WAIT);	
	}
	else
	{
		retry = 0;
		app_wifi_on = false;
		wifi_turn_off();

		if(sos_wait_wifi)
		{
			sos_get_wifi_data_reply(wifi_data);	
			sos_wait_wifi = false;
		}
	#if defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_FALL_DETECT_SUPPORT)
		if(fall_wait_wifi)
		{
			fall_get_wifi_data_reply(wifi_data);	
			fall_wait_wifi = false;
		}
	#endif
		if(location_wait_wifi)
		{
			location_get_wifi_data_reply(wifi_data);
			location_wait_wifi = false;
		}	
	}
}

void wifi_get_scanned_data(void)
{
	app_wifi_on = false;
	retry = 0;
	
	if(k_timer_remaining_get(&wifi_scan_timer) > 0)
		k_timer_stop(&wifi_scan_timer);
	
	if(sos_wait_wifi)
	{
		sos_get_wifi_data_reply(wifi_data);	
		sos_wait_wifi = false;
	}

#if defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_FALL_DETECT_SUPPORT)
	if(fall_wait_wifi)
	{
		fall_get_wifi_data_reply(wifi_data);	
		fall_wait_wifi = false;
	}
#endif

	if(location_wait_wifi)
	{
		location_get_wifi_data_reply(wifi_data);
		location_wait_wifi = false;
	}
}

void APP_Ask_wifi_data(void)
{
#ifdef WIFI_DEBUG
	LOGD("begin");
#endif
	if(!app_wifi_on)
	{
		if(k_timer_remaining_get(&wifi_turn_off_timer) > 0)
			k_timer_stop(&wifi_turn_off_timer);

		retry = 0;
		app_wifi_on = true;
		memset(&wifi_data, 0, sizeof(wifi_data));
		
		wifi_turn_on_and_scanning();
		k_timer_start(&wifi_scan_timer, K_MSEC(5*1000), K_NO_WAIT);	
	}
}

/*============================================================================
* Function Name  : Send_Cmd_To_Esp8285
* Description    : 向ESP8265送命令
* Input          : cmd:发送的命令字符串;waittime:等待时间(单位:10ms)
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void Send_Cmd_To_Esp8285(uint8_t *cmd, uint32_t WaitTime)
{
	WifiSendData(cmd, strlen(cmd));
}

/*============================================================================
* Function Name  : IsInWifiScreen
* Description    : 处于wifi测试界面
* Input          : None
* Output         : None
* Return         : bool
* CALL           : 可被外部调用
==============================================================================*/
bool IsInWifiScreen(void)
{
	//if(screen_id == SCREEN_ID_WIFI_TEST)
	//	return true;
	//else
		return false;
}

/*============================================================================
* Function Name  : wifi_is_working
* Description    : wifi功能正在运行
* Input          : None
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
bool wifi_is_working(void)
{
#ifdef WIFI_DEBUG
	LOGD("wifi_is_on:%d", wifi_is_on);
#endif

	if(wifi_is_on)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*============================================================================
* Function Name  : wifi_enable
* Description    : Esp8285_EN使能，高电平有效
* Input          : None
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void wifi_enable(void)
{
	gpio_pin_set(gpio_wifi, WIFI_RST_PIN, 0);
	k_sleep(K_MSEC(20));
	gpio_pin_set(gpio_wifi, WIFI_RST_PIN, 1);
	gpio_pin_set(gpio_wifi, WIFI_EN_PIN, 1);
}

/*============================================================================
* Function Name  : wifi_disable
* Description    : Esp8285_EN使能禁止，低电平有效
* Input          : None
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void wifi_disable(void)
{
	gpio_pin_set(gpio_wifi, WIFI_EN_PIN, 0);
}

/*============================================================================
* Function Name  : wifi_start_scanning
* Description    : ESP8285模块启动WiFi信号扫描
* Input          : None
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/ 	
void wifi_start_scanning(void)
{
	//设置工作模式 1:station模式 2:AP模式 3:兼容AP+station模式
	Send_Cmd_To_Esp8285(WIFI_SET_MODE,100);
	//设置AT+CWLAP信号的排序方式：按RSSI排序，只显示信号强度和MAC模式
	Send_Cmd_To_Esp8285(WIFI_SET_AP_SCAN_OPT,50);
	//启动扫描
	Send_Cmd_To_Esp8285(WIFI_SET_AP_SCAN_START,0);
}

/*============================================================================
* Function Name  : wifi_turn_on_and_scanning
* Description    : ESPP8285 init
* Input          : None
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void wifi_turn_on_and_scanning(void)
{
	wifi_is_on = true;

#ifdef WIFI_DEBUG	
	LOGD("begin");
#endif

	wifi_enable();
	wifi_start_scanning();
}

void wifi_turn_off_success(void)
{
#ifdef WIFI_DEBUG	
	LOGD("begin");
#endif

	gpio_pin_set(gpio_wifi, WIFI_EN_PIN, 0);
	wifi_off_retry_flag = false;
	k_timer_stop(&wifi_off_retry_timer);

	wifi_is_on = false;
	UartWifiOff();
}

void wifi_turn_off(void)
{
#ifdef WIFI_DEBUG
	LOGD("begin");
#endif

	if(!wifi_is_on)
		return;
	
	wifi_is_on = false;
	wifi_disable();
	UartWifiOff();
}

void wifi_rescanning(void)
{
	if(!wifi_is_on)
		return;

	//设置AT+CWLAP信号的排序方式：按RSSI排序，只显示信号强度和MAC模式
	Send_Cmd_To_Esp8285(WIFI_SET_AP_SCAN_OPT,50);
	Send_Cmd_To_Esp8285(WIFI_SET_AP_SCAN_START, 0);
}

/*============================================================================
* Function Name  : wifi_receive_data_handle
* Description    : NRF9160 接收 ESP8285发来的AP扫描信息进行处理
* Input          : buf:数据缓存 len:数据长度
* Output         : None
* Return         : None
* CALL           : 可被外部调用
==============================================================================*/
void wifi_receive_data_handle(uint8_t *buf, uint32_t len)
{
	uint8_t count = 0;
	uint8_t tmpbuf[256] = {0};
	uint8_t *ptr = NULL;
	uint8_t *ptr1 = NULL;
	uint8_t *ptr2 = NULL;
	bool flag = false;

#ifdef WIFI_DEBUG	
	LOGD("receive:%s", buf);
#endif

	if((ptr = strstr(buf, WIFI_SLEEP_REPLY)) != NULL)
	{
		wifi_off_ok_flag = true;
		return;
	}

	if((ptr = strstr(buf, WIFI_GET_MAC_REPLY)) != NULL)
	{
		//AT+CIFSR
		//+CIFSR:STAIP,"192.168.3.221"
		//+CIFSR:STAMAC,"70:03:9f:d3:54:58"
		//\r\n
		//OK
		//\r\n
		ptr1 = strstr(ptr, WIFI_DATA_MAC_BEGIN);
		if(ptr1)
		{
			ptr1++;
			ptr2 = strstr(ptr1, WIFI_DATA_MAC_BEGIN);
			if(ptr2)
			{
				memcpy(g_wifi_mac_addr, ptr1, ptr2-ptr1);
			}
		}
		return;
	}

	if((ptr = strstr(buf, WIFI_GET_VER)) != NULL)
	{
		//AT+GMR
		//AT version:1.6.2.0(Apr 13 2018 11:10:59)
		//SDK version:2.2.1(6ab97e9)
		//compile time:Jun  7 2018 19:34:26
		//Bin version(Wroom 02):1.6.2
		//OK
		//\r\n
		ptr1 = strstr(ptr, WIFI_DATA_VER_BIN);
		if(ptr1)
		{
			ptr1++;
			ptr1 = strstr(ptr1, WIFI_DATA_SEP_COLON);
			if(ptr1)
			{
				ptr1++;
				ptr2 = strstr(ptr1, WIFI_DATA_END);
				if(ptr2)
				{
					memcpy(g_wifi_ver, ptr1, ptr2-ptr1);
				}
			}
		}

		wifi_off_flag = true;
		return;
	}

	if((ptr = strstr(buf,WIFI_DATA_HEAD)) != NULL)
	{
		//+CWLAP:(-61,"f4:84:8d:8e:9f:eb")
		//+CWLAP:(-67,"da:f1:5b:ff:f2:bc")
		//+CWLAP:(-67,"e2:c1:13:2d:9e:47")
		//+CWLAP:(-73,"7c:94:2a:39:9f:50")
		//+CWLAP:(-76,"52:c2:e8:c6:fa:1e")
		//+CWLAP:(-80,"80:ea:07:73:96:1a")
		//\r\n
		//OK
		//\r\n 
		while(1)
		{
			uint8_t len;
		    uint8_t str_rssi[8]={0};
			uint8_t str_mac[32]={0};

			//head
			ptr1 = strstr(ptr,WIFI_DATA_HEAD);
			if(ptr1 == NULL)
			{
				ptr2 = ptr;
				goto loop;
			}

			//scaned data flag
			flag = true;
			
			//rssi
			ptr += strlen(WIFI_DATA_HEAD);
			ptr1 = strstr(ptr,WIFI_DATA_RSSI_BEGIN);         //取字符串中的,之后的字符
			if(ptr1 == NULL)
			{
				ptr2 = ptr;
				goto loop;
			}
			
			ptr2 = strstr(ptr1+1,WIFI_DATA_RSSI_END);
			if(ptr2 == NULL)
			{
				ptr2 = ptr1+1;
				goto loop;
			}

			len = ptr2 - (ptr1+1);
			if(len > 4)
			{
				goto loop;
			}
			
			memcpy(str_rssi, ptr1+1, len);

			//MAC
			ptr1 = strstr(ptr2,WIFI_DATA_MAC_BEGIN);
			if(ptr1 == NULL)
			{
				goto loop;
			}

			ptr2 = strstr(ptr1+1,WIFI_DATA_MAC_END);
			if(ptr2 == NULL)
			{
				ptr2 = ptr1+1;
				goto loop;
			}

			len = ptr2 - (ptr1+1);
			if(len != 17)
			{
				goto loop;
			}

			memcpy(str_mac, ptr1+1, len);
			
			if(test_wifi_flag)
			{
				uint8_t buf[128] = {0};

				count++;
				if(count<=6)
				{
				#if defined(LCD_VGM068A4W01_SH1106G)||defined(LCD_VGM096064A6W01_SP5090)
					sprintf(buf, "%02d|", -(atoi(str_rssi)));
				#else
					sprintf(buf, "%s|%02d\n", str_mac, -(atoi(str_rssi)));
				#endif
					strcat(tmpbuf, buf);
				}
			}
			else
			{
				strcpy(wifi_data.node[wifi_data.count].rssi, str_rssi);
				strcpy(wifi_data.node[wifi_data.count].mac, str_mac);
				
				wifi_data.count++;
				if(wifi_data.count == WIFI_NODE_MAX)
					break;
			}

		loop:
			ptr = ptr2+1;
			if(*ptr == 0x00)
				break;
		}
	}	
	
	if(test_wifi_flag)
	{
		if(count > 0)
		{
			memset(wifi_test_info,0,sizeof(wifi_test_info));
			sprintf(wifi_test_info, "%d\n", count);
			strcat(wifi_test_info, tmpbuf);
			wifi_test_update_flag = true;

		#ifdef CONFIG_FACTORY_TEST_SUPPORT
			FTWifiStatusUpdate(count);
		#endif
		}
	}
	else
	{
		if(flag && (wifi_data.count >= WIFI_LOCAL_MIN_COUNT))	//扫描有效数据
		{
			wifi_get_scanned_data();
			wifi_off_flag = true;
		}
	}
}

void MenuStartWifi(void)
{
	wifi_on_flag = true;
	test_wifi_flag = true;
}

void MenuStopWifi(void)
{
	wifi_off_flag = true;
	test_wifi_flag = false;	
}

#ifdef CONFIG_FACTORY_TEST_SUPPORT
void FTStartWifi(void)
{
	wifi_on_flag = true;
	test_wifi_flag = true;
}

void FTStopWifi(void)
{
	wifi_off_flag = true;
	test_wifi_flag = false;	
}
#endif

void wifi_test_update(void)
{
	//if(screen_id == SCREEN_ID_WIFI_TEST)
	//{
	//	scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
	//}
}

void test_wifi(void)
{
	MenuStartWifi();
}

static void WifiGetInforCallBack(struct k_timer *timer_id)
{
	wifi_get_infor_flag = true;
}

void WifiMsgProcess(void)
{
	static uint8_t wifi_sleep_retry = 0;
	
	if(wifi_get_infor_flag)
	{
		wifi_get_infor_flag = false;
		wifi_get_infor();
	}
	
	if(wifi_on_flag)
	{
		wifi_on_flag = false;

		if(wifi_is_on)
			return;

		if(k_timer_remaining_get(&wifi_turn_off_timer) > 0)
			k_timer_stop(&wifi_turn_off_timer);
		
		memset(&wifi_data, 0, sizeof(wifi_data));
		wifi_turn_on_and_scanning();

		if(test_wifi_flag)
		{
			k_timer_start(&wifi_rescan_timer, K_MSEC(5000), K_MSEC(5000));	
		}
	}
	
	if(wifi_off_flag)
	{
		wifi_off_flag = false;

		if(!wifi_is_on)
			return;
		
		wifi_turn_off();
		
		if(k_timer_remaining_get(&wifi_rescan_timer) > 0)
			k_timer_stop(&wifi_rescan_timer);
		if(k_timer_remaining_get(&wifi_turn_off_timer) > 0)
			k_timer_stop(&wifi_turn_off_timer);
	}

	if(wifi_rescanning_flag)
	{
		wifi_rescanning_flag = false;
		wifi_rescanning();
	}

	if(wifi_off_ok_flag)
	{
		wifi_off_ok_flag = false;
		wifi_sleep_retry = 0;
		wifi_turn_off_success();
	}
	
	if(wifi_off_retry_flag)
	{
		wifi_off_retry_flag = false;
		wifi_sleep_retry++;
		if(wifi_sleep_retry > 3)
			wifi_off_ok_flag = true;
		else
			wifi_off_flag = true;
	}
	
	if(wifi_wait_timerout_flag)
	{
		wifi_wait_timerout_flag = false;
		wifi_scanned_wait_timerout();
	}
	
	if(wifi_test_update_flag)
	{
		wifi_test_update_flag = false;
		wifi_test_update();
	}	
}

void wifi_get_infor(void)
{
	//设置工作模式 1:station模式 2:AP模式 3:兼容AP+station模式
	Send_Cmd_To_Esp8285(WIFI_SET_MODE,50);
	//获取Mac地址
	Send_Cmd_To_Esp8285(WIFI_GET_MAC_CMD,50);
	//获取版本信息
	Send_Cmd_To_Esp8285(WIFI_GET_VER,50);

	k_timer_start(&wifi_turn_off_timer, K_SECONDS(5), K_NO_WAIT);
}

void wifi_init(void)
{
#ifdef WIFI_DEBUG
	LOGD("begin");
#endif

	uart_wifi_init();

	gpio_wifi = DEVICE_DT_GET(WIFI_PORT);
	if(!gpio_wifi)
	{
	#ifdef WIFI_DEBUG
		LOGD("Could not get gpio!");
	#endif
		return;
	}

	gpio_pin_configure(gpio_wifi, WIFI_RST_PIN, GPIO_OUTPUT);
	gpio_pin_configure(gpio_wifi, WIFI_EN_PIN, GPIO_OUTPUT);

	gpio_pin_set(gpio_wifi, WIFI_RST_PIN, 0);
	k_sleep(K_MSEC(20));
	gpio_pin_set(gpio_wifi, WIFI_RST_PIN, 1);
	gpio_pin_set(gpio_wifi, WIFI_EN_PIN, 1);

	k_timer_start(&wifi_get_infor_timer, K_SECONDS(3), K_NO_WAIT);
}
