/****************************************Copyright (c)************************************************
** File name:			     datetime.c
** Last modified Date:          
** Last Version:		   
** Descriptions:		   ʹ�õ�ncs�汾-1.2.0
**						
** Created by:			л��
** Created date:		2019-12-31
** Version:			    1.0
** Descriptions:		ϵͳ����ʱ�����
******************************************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include "datetime.h"
#include "settings.h"
#include "max20353.h"
#ifdef CONFIG_BLE_SUPPORT
#include "ble.h"
#endif
#include "ucs2.h"
#include "logger.h"

//#define DATETIME_DEBUG

#define SEC_START_YEAR		1970
#define SEC_START_MONTH		1
#define SEC_START_DAY		1
#define SEC_START_HOUR		0
#define SEC_START_MINUTE	0
#define SEC_START_SECOND	0

#define SEC_PER_MINUTE		60
#define SEC_PER_HOUR		(SEC_PER_MINUTE*60)
#define SEC_PER_DAY			(SEC_PER_HOUR*24)
#define SEC_PER_SMALL_YEAR	(SEC_PER_DAY*365)
#define SEC_PER_BIG_YEAR	(SEC_PER_DAY*366)

sys_date_timer_t date_time = {0};
sys_date_timer_t last_date_time = {0};

bool update_time = false;
bool update_date = false;
bool update_week = false;
bool update_date_time = false;
bool sys_time_count = false;
bool show_date_time_first = true;

static bool send_timing_data_flag = false;
#ifdef CONFIG_IMU_SUPPORT
#ifdef CONFIG_STEP_SUPPORT
static bool save_step_data_flag = false;
#endif
#ifdef CONFIG_SLEEP_SUPPORT
static bool save_sleep_data_flag = false;
#endif
#endif

uint8_t date_time_changed = 0;//ͨ��λ���ж�����ʱ���Ƿ��б仯���ӵ�6λ���𣬷ֱ��ʾ������ʱ����
uint64_t laststamp = 0;

static void clock_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(clock_timer, clock_timer_handler, NULL);


uint8_t CheckYearIsLeap(uint32_t years)
{
	if(((years%4 == 0) && (years%100 != 0))||(years%400 == 0))
		return 1;
	else
		return 0;
}

uint8_t GetWeekDayByDate(sys_date_timer_t date)
{
	uint8_t flag=0,index=SYSTEM_STARTING_WEEK;//1900��1��1��������һ 0=sunday
	uint32_t i,count=0;
	
	if(date.year < SYSTEM_STARTING_YEAR)
		return 0xff;

	for(i=SYSTEM_STARTING_YEAR;i<date.year;i++)
	{
		if(((i%4 == 0)&&(i%100 != 0))||(i%400 == 0))	//����366��
			count += 366;
		else
			count += 365;
	}

	if(CheckYearIsLeap(date.year))
		flag = 1;
	
	switch(date.month)
	{
	case 1:
		count += 0;
		break;
	case 2:
		count += 31;
		break;
	case 3:
		count += (31+(28+flag));
		break;
	case 4:
		count += (2*31+(28+flag));
		break;
	case 5:
		count += (2*31+30+(28+flag));
		break;
	case 6:
		count += (3*31+30+(28+flag));
		break;
	case 7:
		count += (3*31+2*30+(28+flag));
		break;
	case 8:
		count += (4*31+2*30+(28+flag));
		break;
	case 9:
		count += (5*31+2*30+(28+flag));
		break;
	case 10:
		count += (5*31+3*30+(28+flag));
		break;
	case 11:
		count += (6*31+3*30+(28+flag));
		break;
	case 12:
		count += (6*31+4*30+(28+flag));
		break;			
	}

	count += (date.day-1);
	
	count = count%7;
	index = (index+count)%7;
	
	return index;
}

int DateCompare(sys_date_timer_t date1, sys_date_timer_t date2)
{
	uint8_t date1buf[128] = {0};
	uint8_t date2buf[128] = {0};

	sprintf(date1buf, "%04d%02d%02d%02d%02d%02d",
							date1.year,
							date1.month,
							date1.day,
							date1.hour,
							date1.minute,
							date1.second
							);
	
	sprintf(date2buf, "%04d%02d%02d%02d%02d%02d",
							date2.year,
							date2.month,
							date2.day,
							date2.hour,
							date2.minute,
							date2.second
							);

	return strcmp(date1buf, date2buf);
}

void DateIncrease(sys_date_timer_t *date, uint32_t days)
{
	(*date).day += days;

	while(1)
	{
		if((*date).month == 1 \
		|| (*date).month == 3 \
		|| (*date).month == 5 \
		|| (*date).month == 7 \
		|| (*date).month == 8 \
		|| (*date).month == 10 \
		|| (*date).month == 12)
		{
			if((*date).day > 31)
			{
				(*date).day -= 31;
				(*date).month++;
				if((*date).month > 12)
				{
					(*date).month = 1;
					(*date).year++;
				}
			}
			else
			{
				break;
			}
		}
		else if((*date).month == 4 \
			|| (*date).month == 6 \
			|| (*date).month == 9 \
			|| (*date).month == 11)
		{
			if((*date).day > 30)
			{
				(*date).day -= 30;
				(*date).month++;
				if((*date).month > 12)
				{
					(*date).month = 1;
					(*date).year++;
				}
			}
			else
			{
				break;
			}
		}
		else
		{
			uint8_t Leap = 0;

			if(CheckYearIsLeap((*date).year))
				Leap = 1;
			
			if((*date).day > (28+Leap))
			{
				(*date).day -= (28+Leap);
				(*date).month++;
				if((*date).month > 12)
				{
					(*date).month = 1;
					(*date).year++;
				}
			}
			else
			{
				break;
			}
		}	
	}

	(*date).week = GetWeekDayByDate((*date));
}

void DateDecrease(sys_date_timer_t *date, uint32_t days)
{
	if((*date).day > days)
	{
		(*date).day -= days;
	}
	else
	{
		while(1)
		{
			if((*date).month == 1 \
			|| (*date).month == 2 \
			|| (*date).month == 4 \
			|| (*date).month == 6 \
			|| (*date).month == 8 \
			|| (*date).month == 9 \
			|| (*date).month == 11)
			{
				if((*date).month == 1)
				{
					(*date).year--;
					(*date).month = 12;
				}
				else
				{
					(*date).month--;
				}

				(*date).day += 31;
				if((*date).day > days)
				{
					(*date).day -= days;
					break;
				}
				else
				{
					days -= (*date).day;
				}
			}
			else if((*date).month == 5 \
				|| (*date).month == 7 \
				|| (*date).month == 10 \
				|| (*date).month == 12)
			{
				(*date).month--;

				(*date).day += 30;
				if((*date).day > days)
				{
					(*date).day -= days;
					break;
				}
				else
				{
					days -= (*date).day;
				}
			}
			else
			{
				uint8_t Leap = 0;

				if(CheckYearIsLeap((*date).year))
					Leap = 1;

				(*date).month--;
				
				(*date).day += (28+Leap);
				if((*date).day > days)
				{
					(*date).day -= days;
					break;
				}
				else
				{
					days -= (*date).day;
				}
			}	
		}		
	}

	(*date).week = GetWeekDayByDate((*date));
}

void TimeIncrease(sys_date_timer_t *date, uint32_t minutes)
{
	uint8_t m_add,h_add,day_add;
	
	m_add = minutes%60;
	h_add = minutes/60;
	day_add = h_add/24;

#ifdef DATETIME_DEBUG
	LOGD("m_add:%d, h_add:%d", m_add, h_add);
#endif

	(*date).minute += m_add;
	if((*date).minute > 59)
	{
		(*date).minute = (*date).minute%60;
		h_add++;
	}
	
	(*date).hour += h_add;
	if((*date).hour > 23)
	{
		(*date).hour = (*date).hour%24;
		day_add++;
	}	
	
	if(day_add > 0)
	{
		DateIncrease(date, day_add);
	}

	(*date).week = GetWeekDayByDate((*date));
}

void TimeDecrease(sys_date_timer_t *date, uint32_t minutes)
{
	uint8_t m_dec,h_dec,day_dec;

	m_dec = minutes%60;
	h_dec = minutes/60;
	day_dec = h_dec/24;

	if((*date).minute >= m_dec)
	{
		(*date).minute -= m_dec;
	}
	else
	{
		(*date).minute = ((*date).minute+60)-m_dec;
		h_dec++;
	}

	if((*date).hour >= h_dec)
	{
		(*date).hour -= h_dec;
	}
	else
	{
		(*date).hour = ((*date).hour+24)-h_dec;
		day_dec++;
	}
	
	if(day_dec > 0)
	{
		DateDecrease(date, day_dec);
	}

	(*date).week = GetWeekDayByDate((*date));
}

void RedrawSystemTime(void)
{
}

void UpdateSystemTime(void)
{
	uint64_t timestamp,timeskip;
	static uint64_t timeoffset=0;

   	memcpy(&last_date_time, &date_time, sizeof(sys_date_timer_t));

	timestamp = k_uptime_get();
	timeskip = abs(timestamp-laststamp);
	laststamp = timestamp;

	timeoffset += (timeskip%1000);
	if(timeoffset >= 1000)
	{
		timeskip += 1000;
		timeoffset -= 1000;
	}

	date_time.second += (timeskip/1000);
	if(date_time.second > 59)
	{
		date_time.minute += date_time.second/60;
		date_time.second = date_time.second%60;
		date_time_changed = date_time_changed|0x02;
		//date_time_changed = date_time_changed|0x04;//�����ڱ䶯��ͬʱ��ʱ��Ҳ��ͬ�������䶯
		if(date_time.minute > 59)
		{
			date_time.hour += date_time.minute/60;
			date_time.minute = date_time.minute%60;
			date_time_changed = date_time_changed|0x04;
			if(date_time.hour > 23)
			{
				date_time.hour = 0;
				date_time.day++;
				date_time.week++;
				if(date_time.week > 6)
					date_time.week = 0;
				date_time_changed = date_time_changed|0x08;
				if(date_time.month == 1 \
				|| date_time.month == 3 \
				|| date_time.month == 5 \
				|| date_time.month == 7 \
				|| date_time.month == 8 \
				|| date_time.month == 10 \
				|| date_time.month == 12)
				{
					if(date_time.day > 31)
					{
						date_time.day = 1;
						date_time.month++;
						date_time_changed = date_time_changed|0x10;
						if(date_time.month > 12)
						{
							date_time.month = 1;
							date_time.year++;
							date_time_changed = date_time_changed|0x20;
						}
					}
				}
				else if(date_time.month == 4 \
					|| date_time.month == 6 \
					|| date_time.month == 9 \
					|| date_time.month == 11)
				{
					if(date_time.day > 30)
					{
						date_time.day = 1;
						date_time.month++;
						date_time_changed = date_time_changed|0x10;
						if(date_time.month > 12)
						{
							date_time.month = 1;
							date_time.year++;
							date_time_changed = date_time_changed|0x20;
						}
					}
				}
				else
				{
					uint8_t Leap = 0;

					if(CheckYearIsLeap(date_time.year))
						Leap = 1;
					
					if(date_time.day > (28+Leap))
					{
						date_time.day = 1;
						date_time.month++;
						date_time_changed = date_time_changed|0x10;
						if(date_time.month > 12)
						{
							date_time.month = 1;
							date_time.year++;
							date_time_changed = date_time_changed|0x20;
						}
					}
				}

				update_date_time = true;
			}
		}
	}
	date_time_changed = date_time_changed|0x01;
	
	//ÿ���ӱ���һ��ʱ��
	if((date_time_changed&0x02) != 0)
	{
		SaveSystemDateTime();
		date_time_changed = date_time_changed&0xFD;

	#if !defined(NB_SIGNAL_TEST)&&!defined(CONFIG_FACTORY_TEST_SUPPORT)
		if(1
		  #ifdef CONFIG_FOTA_DOWNLOAD
			&& (!fota_is_running())
		  #endif/*CONFIG_FOTA_DOWNLOAD*/
		  #ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
			&& (!dl_is_running())
		  #endif/*CONFIG_DATA_DOWNLOAD_SUPPORT*/
		  #ifdef CONFIG_FACTORY_TEST_SUPPORT
			&& (!FactryTestActived())
		  #endif/*CONFIG_FACTORY_TEST_SUPPORT*/
		)
		{
		#ifdef CONFIG_ALARM_SUPPORT	
			AlarmRemindCheck(date_time);
		#endif
			//TimeCheckSendLocationData();
		}
	#endif

		//pmu_status_update();
	}

	if((date_time_changed&0x04) != 0)
	{		
		date_time_changed = date_time_changed&0xFB;

	#if !defined(NB_SIGNAL_TEST)&&!defined(CONFIG_FACTORY_TEST_SUPPORT)
	 #ifdef CONFIG_IMU_SUPPORT
	  #ifdef CONFIG_STEP_SUPPORT
		save_step_data_flag = true;
	  #endif
	  #ifdef CONFIG_SLEEP_SUPPORT
		save_sleep_data_flag = true;
	  	if(date_time.hour == SLEEP_TIME_START)
	  	{
	  		reset_sleep_data = true;
	  	}
	  #endif
	 #endif
	#endif
	
		if(date_time.hour == 3)	//xb ddd 2024-03-14 Upload a synchronization packet at 03:00 for the backend to calibrate the watch's time.
		{
			SyncSendHealthData();
		}

		//if(date_time.hour == 4)	//xb add 2024-11-22 Start firmware FOTA upgrade at 4am.
		{
			//VerCheckStart();
		}
	}

	if((date_time_changed&0x08) != 0)
	{
		date_time_changed = date_time_changed&0xF7;

	#if defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_STEP_SUPPORT)
		g_steps = 0;
		reset_steps = true;
	#endif
	}
}

static void clock_timer_handler(struct k_timer *timer)
{
	sys_time_count = true;
}

void StartSystemDateTime(void)
{
	k_timer_start(&clock_timer, K_MSEC(1000), K_MSEC(1000));
}

void StopSystemDateTime(void)
{
	k_timer_stop(&clock_timer);
}

bool CheckSystemDateTimeIsValid(sys_date_timer_t systime)
{
	bool ret = true;

	if((systime.year<SYSTEM_STARTING_YEAR || systime.year>9999)
		|| ((systime.month==0)||(systime.month>12)) 
		|| (systime.day==0) 
		|| ((systime.day>31)&&((systime.month==1)||(systime.month==3)||(systime.month==5)||(systime.month==7)||(systime.month==8)||(systime.month==10)||(systime.month==12)))
		|| ((systime.day>30)&&((systime.month==4)||(systime.month==6)||(systime.month==9)||(systime.month==11)))
		|| ((systime.day>29)&&((systime.month==2)&&(systime.year%4==0)))
		|| ((systime.day>28)&&((systime.month==2)&&(systime.year%4!=0)))
		|| ((systime.hour>23)||(systime.minute>59)||(systime.second>59))
		|| (systime.week>6))
	{
		ret = false;
	}
	
	return ret;
}

void GetSystemTimeSecString(uint8_t *str_utc)
{
	uint32_t i;
	uint32_t total_sec,total_day=0;

	sprintf(str_utc, "%04d%02d%02d%02d%02d%02d", 
						date_time.year,
						date_time.month,
						date_time.day,
						date_time.hour,
						date_time.minute,
						date_time.second);
}

void GetSystemDateStrings(uint8_t *str_date)
{
	uint8_t tmpbuf[128] = {0};
	
	switch(global_settings.date_format)
	{
	case DATE_FORMAT_YYYYMMDD:
		sprintf((char*)str_date, "%04d/%02d/%02d", date_time.year, date_time.month, date_time.day);
		break;
	case DATE_FORMAT_MMDDYYYY:
		sprintf((char*)str_date, "%02d/%02d/%04d", date_time.month, date_time.day, date_time.year);
		break;
	case DATE_FORMAT_DDMMYYYY:
		sprintf((char*)str_date, "%02d/%02d/%04d", date_time.day, date_time.month, date_time.year);
		break;
	}

#ifdef FONTMAKER_UNICODE_FONT
	strcpy(tmpbuf, str_date);
	mmi_asc_to_ucs2(str_date, tmpbuf);
#endif
}

void GetSysteAmPmStrings(uint8_t *str_ampm)
{
	uint8_t flag = 0;
	uint8_t *am_pm[2] = {"am", "pm"};
	uint8_t tmpbuf[128] = {0};

	if(date_time.hour > 12)
		flag = 1;
	
	switch(global_settings.time_format)
	{
	case TIME_FORMAT_24:
		sprintf((char*)str_ampm, "  ");
		break;
	case TIME_FORMAT_12:
		sprintf((char*)str_ampm, "%s", am_pm[flag]);
		break;
	}

#ifdef FONTMAKER_UNICODE_FONT
	strcpy(tmpbuf, str_ampm);
	mmi_asc_to_ucs2(str_ampm, tmpbuf);
#endif

}

void GetSystemTimeStrings(uint8_t *str_time)
{
	uint8_t tmpbuf[128] = {0};
	
	switch(global_settings.time_format)
	{
	case TIME_FORMAT_24:
		sprintf((char*)str_time, "%02d:%02d:%02d", date_time.hour, date_time.minute, date_time.second);
		break;
	case TIME_FORMAT_12:
		sprintf((char*)str_time, "%02d:%02d:%02d", (date_time.hour>12 ? (date_time.hour-12):date_time.hour), date_time.minute, date_time.second);
		break;
	}
}

void GetSystemWeekStrings(uint8_t *str_week)
{
	uint8_t *week_en[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
	uint8_t *week_chn[7] = {"��", "һ", "��", "��", "��", "��", "��"};
	uint8_t tmpbuf[128] = {0};

	switch(global_settings.language)
	{
	#ifdef FW_FOR_CN
	case LANGUAGE_CHN:
		strcpy((char*)str_week, (const char*)week_chn[date_time.week]);
		break;
	#endif
	case LANGUAGE_EN:
		strcpy((char*)str_week, (const char*)week_en[date_time.week]);
		break;
	}
}

void TimeMsgProcess(void)
{
	if(sys_time_count)
	{
		sys_time_count = false;
		UpdateSystemTime();
	}

#ifdef CONFIG_IMU_SUPPORT
#ifdef CONFIG_STEP_SUPPORT
	if(save_step_data_flag)
	{
		SetCurDayStepRecData(g_steps);
		save_step_data_flag = false;
	}
#endif
#ifdef CONFIG_SLEEP_SUPPORT	
	if(save_sleep_data_flag)
	{
		sleep_data sleep = {0};
		
		sleep.deep = g_deep_sleep;
		sleep.light = g_light_sleep;
		SetCurDaySleepRecData(sleep);
		save_sleep_data_flag = false;
	}
#endif
#endif
}
