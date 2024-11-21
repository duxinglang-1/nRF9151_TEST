/****************************************Copyright (c)************************************************
** File Name:			    communicate.c
** Descriptions:			communicate source file
** Created By:				xie biao
** Created Date:			2021-04-28
** Modified Date:      		2021-04-28 
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "settings.h"
#include "nb.h"
#include "gps.h"
#include "external_flash.h"
#ifdef CONFIG_WIFI_SUPPORT
#include "esp8266.h"
#endif
#include "datetime.h"
#ifdef CONFIG_IMU_SUPPORT
#include "Lsm6dso.h"
#endif
#include "communicate.h"
#include "logger.h"

extern uint16_t g_last_steps;

#ifdef CONFIG_WIFI_SUPPORT
/*****************************************************************************
 * FUNCTION
 *  location_get_wifi_data_reply
 * DESCRIPTION
 *  定位协议包获取WiFi数据之后的上传数据包处理
 * PARAMETERS
 *  wifi_data       [IN]       wifi数据结构体
 * RETURNS
 *  Nothing
 *****************************************************************************/
void location_get_wifi_data_reply(wifi_infor wifi_data)
{
	uint8_t reply[256] = {0};
	uint32_t i,count=3;

	if(wifi_data.count > 0)
		count = wifi_data.count;
		
	strcat(reply, "3,");
	for(i=0;i<count;i++)
	{
		strcat(reply, wifi_data.node[i].mac);
		strcat(reply, "&");
		strcat(reply, wifi_data.node[i].rssi);
		strcat(reply, "&");
		if(i < (count-1))
			strcat(reply, "|");
	}

	NBSendLocationData(reply, strlen(reply));
}
#endif

/*****************************************************************************
 * FUNCTION
 *  location_get_gps_data_reply
 * DESCRIPTION
 *  定位协议包获取GPS数据之后的上传数据包处理
 * PARAMETERS
 *	flag			[IN]		GPS数据获取标记, ture:成功 false:失败
 *  gps_data       	[IN]		GPS数据结构体
 * RETURNS
 *  Nothing
 *****************************************************************************/
void location_get_gps_data_reply(bool flag, struct nrf_modem_gnss_pvt_data_frame gps_data)
{
	uint8_t reply[128] = {0};
	uint8_t tmpbuf[8] = {0};
	uint32_t tmp1;
	double tmp2;

	if(!flag)
	{
	#ifdef CONFIG_WIFI_SUPPORT
		location_wait_wifi = true;
		APP_Ask_wifi_data();
	#endif
		return;
	}
	
	strcpy(reply, "4,");
	
	//latitude
	if(gps_data.latitude < 0)
	{
		strcat(reply, "-");
		gps_data.latitude = -gps_data.latitude;
	}

	tmp1 = (uint32_t)(gps_data.latitude);	//经度整数部分
	tmp2 = gps_data.latitude - tmp1;	//经度小数部分
	//integer
	sprintf(tmpbuf, "%d", tmp1);
	strcat(reply, tmpbuf);
	//dot
	strcat(reply, ".");
	//decimal
	tmp1 = (uint32_t)(tmp2*1000000);
	sprintf(tmpbuf, "%02d", (uint8_t)(tmp1/10000));
	strcat(reply, tmpbuf);
	tmp1 = tmp1%10000;
	sprintf(tmpbuf, "%02d", (uint8_t)(tmp1/100));
	strcat(reply, tmpbuf);
	tmp1 = tmp1%100;
	sprintf(tmpbuf, "%02d", (uint8_t)(tmp1));
	strcat(reply, tmpbuf);

	//semicolon
	strcat(reply, "|");
	
	//longitude
	if(gps_data.longitude < 0)
	{
		strcat(reply, "-");
		gps_data.longitude = -gps_data.longitude;
	}

	tmp1 = (uint32_t)(gps_data.longitude);	//经度整数部分
	tmp2 = gps_data.longitude - tmp1;	//经度小数部分
	//integer
	sprintf(tmpbuf, "%d", tmp1);
	strcat(reply, tmpbuf);
	//dot
	strcat(reply, ".");
	//decimal
	tmp1 = (uint32_t)(tmp2*1000000);
	sprintf(tmpbuf, "%02d", (uint8_t)(tmp1/10000));
	strcat(reply, tmpbuf);	
	tmp1 = tmp1%10000;
	sprintf(tmpbuf, "%02d", (uint8_t)(tmp1/100));
	strcat(reply, tmpbuf);	
	tmp1 = tmp1%100;
	sprintf(tmpbuf, "%02d", (uint8_t)(tmp1));
	strcat(reply, tmpbuf);

	NBSendLocationData(reply, strlen(reply));
}

void TimeCheckSendWristOffData(void)
{
	uint8_t reply[8] = {0};

	if(CheckSCC())
		strcpy(reply, "1");
	else
		strcpy(reply, "0");
	
	NBSendTimelyWristOffData(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  SendPowerOnData
 * DESCRIPTION
 *  发送开机数据包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendPowerOnData(void)
{
	uint8_t tmpbuf[10] = {0};
	uint8_t reply[256] = {0};

	//imsi
	strcpy(reply, g_imsi);
	strcat(reply, ",");
	
	//iccid
	strcat(reply, g_iccid);
	strcat(reply, ",");

	//nb rsrp
	sprintf(tmpbuf, "%d,", g_rsrp);
	strcat(reply, tmpbuf);
	
	//time zone
	strcat(reply, g_timezone);
	strcat(reply, ",");
	
	//battery
	GetBatterySocString(tmpbuf);
	strcat(reply, tmpbuf);
	strcat(reply, ",");

	//mcu fw version
	strcat(reply, g_fw_version);	
	strcat(reply, ",");

	//modem fw version
	strcat(reply, &g_modem[12]);	
	strcat(reply, ",");

	//ppg algo
#ifdef CONFIG_PPG_SUPPORT	
	strcat(reply, g_ppg_ver);
#else
	strcat(reply, "NO PPG");
#endif
	strcat(reply, ",");

	//wifi version
#ifdef CONFIG_WIFI_SUPPORT
	strcat(reply, g_wifi_ver);
#else
	strcat(reply, "NO WiFi");
#endif
	strcat(reply, ",");

	//wifi mac
#ifdef CONFIG_WIFI_SUPPORT	
	strcat(reply, g_wifi_mac_addr);
#else
	strcat(reply, "NO WiFi");
#endif
	strcat(reply, ",");

	//ble version
#ifdef CONFIG_BLE_SUPPORT	
	strcat(reply, &g_nrf52810_ver[15]);
#else
	strcat(reply, "NO BLE");
#endif
	strcat(reply, ",");

	//ble mac
#ifdef CONFIG_BLE_SUPPORT	
	strcat(reply, g_ble_mac_addr);
#else
	strcat(reply, "NO BLE");
#endif

	NBSendPowerOnInfor(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  SendPowerOffData
 * DESCRIPTION
 *  发送关机数据包
 * PARAMETERS
 *	pwroff_mode			[IN]		关机模式 1:低电关机 2:按键关机 3:重启关机 
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendPowerOffData(uint8_t pwroff_mode)
{
	uint8_t tmpbuf[10] = {0};
	uint8_t reply[128] = {0};

	//pwr off mode
	sprintf(reply, "%d,", pwroff_mode);
	
	//nb rsrp
	sprintf(tmpbuf, "%d,", g_rsrp);
	strcat(reply, tmpbuf);
	
	//battery
	GetBatterySocString(tmpbuf);
	strcat(reply, tmpbuf);
			
	NBSendPowerOffInfor(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  SendSettingsData
 * DESCRIPTION
 *  发送终端设置项数据包
 * PARAMETERS
 *	Nothing
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendSettingsData(void)
{
	uint8_t tmpbuf[10] = {0};
	uint8_t reply[128] = {0};

	//temp uint
	sprintf(reply, "%d,", global_settings.temp_unit);
	
	//language
	switch(global_settings.language)
	{
#ifndef FW_FOR_CN
	case LANGUAGE_EN:	//English
		strcat(reply, "en");
		break;
	case LANGUAGE_DE:	//Deutsch
		strcat(reply, "de");
		break;
	case LANGUAGE_FR:	//French
		strcat(reply, "fr");
		break;
	case LANGUAGE_ITA:	//Italian
		strcat(reply, "it");
		break;
	case LANGUAGE_ES:	//Spanish
		strcat(reply, "es");
		break;
	case LANGUAGE_PT:	//Portuguese
		strcat(reply, "pt");
		break;
#else
	case LANGUAGE_CHN:	//Chinese
		strcat(reply, "zh");
		break;
	case LANGUAGE_EN:	//English
		strcat(reply, "en");
		break;
#endif	
	}
	
	NBSendSettingsData(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  SendSosAlarmData
 * DESCRIPTION
 *  发送SOS报警包(无地址信息)
 * PARAMETERS
 *	
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendSosAlarmData(void)
{
	uint8_t reply[8] = {0};
	uint32_t i,count=1;

	strcpy(reply, "1");
	NBSendAlarmData(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  SendFallAlarmData
 * DESCRIPTION
 *  发送Fall报警包(无地址信息)
 * PARAMETERS
 *	
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendFallAlarmData(void)
{
	uint8_t reply[8] = {0};
	uint32_t i,count=1;

	strcpy(reply, "2");
	NBSendAlarmData(reply, strlen(reply));
}

/*****************************************************************************
 * FUNCTION
 *  SendVerCheckAskData
 * DESCRIPTION
 *  发送FOTA版本信息请求包
 * PARAMETERS
 *	
 * RETURNS
 *  Nothing
 *****************************************************************************/
void SendVerCheckAskData(void)
{
	uint8_t reply[8] = {0};
	uint32_t i,count=1;

	strcpy(reply, "1");
	NBSendVerCheckData(reply, strlen(reply));
}

