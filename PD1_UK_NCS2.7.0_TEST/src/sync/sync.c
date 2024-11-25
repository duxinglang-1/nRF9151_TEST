/****************************************Copyright (c)************************************************
** File Name:			    sync.c
** Descriptions:			data synchronism process source file
** Created By:				xie biao
** Created Date:			2021-12-16
** Modified Date:      		2021-12-16 
** Version:			    	V1.0
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <soc.h>
#include <nrf_socket.h>
#include <nrfx.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "sync.h"
#ifdef CONFIG_WIFI_SUPPORT
#include "esp8266.h"
#endif
#ifdef CONFIG_ANIMATION_SUPPORT
#include "animation.h"
#endif
#include "logger.h"

SYNC_STATUS sync_state = SYNC_STATUS_IDLE;

static bool sync_start_flag = false;
static bool sync_status_change_flag = false;
static bool sync_redraw_flag = false;

static void SyncTimerOutCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(sync_timer, SyncTimerOutCallBack, NULL);

void SyncTimerOutCallBack(struct k_timer *timer_id)
{
	sync_status_change_flag = true;
}

void SyncStatusUpdate(void)
{
	switch(sync_state)
	{
	case SYNC_STATUS_IDLE:
		break;

	case SYNC_STATUS_LINKING:
		SyncNetWorkCallBack(SYNC_STATUS_FAIL);
		break;

	case SYNC_STATUS_SENT:
	case SYNC_STATUS_FAIL:
		sync_state = SYNC_STATUS_IDLE;
		break;
	}
}

bool SyncIsRunning(void)
{
	if(sync_state > SYNC_STATUS_IDLE)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void SyncDataStop(void)
{
	k_timer_stop(&sync_timer);
#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
	sync_state = SYNC_STATUS_IDLE;
}

void MenuStartSync(void)
{
	sync_start_flag = true;
}

void SyncDataStart(void)
{
	uint8_t delay;
	
	if(sync_state != SYNC_STATUS_IDLE)
	{
		return;
	}

	ClearLeftKeyUpHandler();
	sync_state = SYNC_STATUS_LINKING;
	SyncSendHealthData();
	k_timer_start(&sync_timer, K_SECONDS(60), K_NO_WAIT);
}

void SyncNetWorkCallBack(SYNC_STATUS status)
{
	if((sync_state != SYNC_STATUS_IDLE) && (sync_state != status))
	{
		sync_state = status;

		switch(sync_state)
		{
		case SYNC_STATUS_IDLE:
			break;
			
		case SYNC_STATUS_LINKING:
			SyncSendHealthData();
			break;
			
		case SYNC_STATUS_SENT:
		case SYNC_STATUS_FAIL:
			k_timer_stop(&sync_timer);
		#ifdef CONFIG_ANIMATION_SUPPORT 
			AnimaStop();
		#endif

			if(k_timer_remaining_get(&sync_timer) == 0)
				k_timer_start(&sync_timer, K_SECONDS(3), K_NO_WAIT);
			break;
		}
	}
}

void SyncMsgProcess(void)
{
	if(sync_start_flag)
	{
		SyncDataStart();
		sync_start_flag = false;
	}

	if(sync_redraw_flag)
	{
		sync_redraw_flag = false;
	}
	
	if(sync_status_change_flag)
	{
		SyncStatusUpdate();
		sync_status_change_flag = false;
	}
}

