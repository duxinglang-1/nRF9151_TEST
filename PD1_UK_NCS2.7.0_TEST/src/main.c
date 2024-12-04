/*
* Copyright (c) 2019 Nordic Semiconductor ASA
*
* SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <modem/nrf_modem_lib.h>
#include <dk_buttons_and_leds.h>
#include "datetime.h"
#include "inner_flash.h"
#include "external_flash.h"
#ifdef CONFIG_BLE_SUPPORT
#include "ble.h"
#endif
#include "settings.h"
#ifdef CONFIG_IMU_SUPPORT
#include "lsm6dso.h"
#endif
#ifdef CONFIG_GPS_SUPPORT
#include "gps.h"
#endif
#include "pmu.h"
#include "codetrans.h"
#ifdef CONFIG_AUDIO_SUPPORT
#include "audio.h"
#endif
#ifdef CONFIG_WATCHDOG
#include "watchdog.h"
#endif
#ifdef CONFIG_PRESSURE_SUPPORT
#include "pressure.h"
#endif
#ifdef CONFIG_WIFI_SUPPORT
#include "esp8266.h"
#endif/*CONFIG_WIFI_SUPPORT*/
#include "logger.h"

static bool sys_pwron_completed_flag = false;

/* Stack definition for application workqueue */
K_THREAD_STACK_DEFINE(nb_stack_area,
		      8192);
static struct k_work_q nb_work_q;

#ifdef CONFIG_IMU_SUPPORT
K_THREAD_STACK_DEFINE(imu_stack_area,
              2048);
static struct k_work_q imu_work_q;
#endif

K_THREAD_STACK_DEFINE(gps_stack_area,
              2048);
static struct k_work_q gps_work_q;


static void modem_init(void)
{
	int ret,type;
	
	ret = nrf_modem_lib_init();
	if(ret)
	{
		LOGD("ret: %d", ret);
		if(ret != -EAGAIN && ret != -EIO)
		{
			return ret;
		}
		else if(ret == -EIO)
		{
			LOGD("Please program full modem firmware with the bootloader or external tools");
		}
	}

	type = mcuboot_swap_type();
	switch(type)
	{
	/** Attempt to boot the contents of slot 0. */
	case BOOT_SWAP_TYPE_NONE:
		/* Normal reset, nothing happened, do nothing. */
		LOGD("BOOT_SWAP_TYPE_NONE");
		return;

	/** Swap to slot 1. Absent a confirm command, revert back on next boot. */
	case BOOT_SWAP_TYPE_TEST:
		LOGD("BOOT_SWAP_TYPE_TEST");
		break;

	/** Swap to slot 1, and permanently switch to booting its contents. */
	case BOOT_SWAP_TYPE_PERM:
		LOGD("BOOT_SWAP_TYPE_PERM");
		break;
		
	/** Swap failed because image to be run is not valid. */
	case BOOT_SWAP_TYPE_FAIL:
		LOGD("BOOT_SWAP_TYPE_FAIL");
		break;

	/** Swap back to alternate slot. A confirm changes this state to NONE. */
	case BOOT_SWAP_TYPE_REVERT:
		/* Happens on a successful application FOTA. */
		LOGD("BOOT_SWAP_TYPE_REVERT");
		ret = boot_write_img_confirmed();
		LOGD("ret:%d", ret);
		break;
	}
}

void system_init(void)
{
	k_sleep(K_MSEC(500));//xb test 2022-03-11 启动时候延迟0.5S,等待其他外设完全启动

	modem_init();

#ifdef CONFIG_FOTA_DOWNLOAD
	fota_init();
#endif

	InitSystemSettings();

#ifdef CONFIG_IMU_SUPPORT
	init_imu_int1();//xb add 2022-05-27
#endif
	pmu_init();
	key_init();
	//flash_init();
	
#ifdef CONFIG_AUDIO_SUPPORT	
	audio_init();
#endif
#ifdef CONFIG_WIFI_SUPPORT
	wifi_init();
#endif
#ifdef CONFIG_IMU_SUPPORT
	IMU_init(&imu_work_q);
#endif
#ifdef CONFIG_PRESSURE_SUPPORT
	pressure_init();
#endif
	//LogInit();

	NB_init(&nb_work_q);
	GPS_init(&gps_work_q);
}

void work_init(void)
{
	k_work_queue_start(&nb_work_q, nb_stack_area,
					K_THREAD_STACK_SIZEOF(nb_stack_area),
					CONFIG_APPLICATION_WORKQUEUE_PRIORITY,NULL);
#ifdef CONFIG_IMU_SUPPORT	
	k_work_queue_start(&imu_work_q, imu_stack_area,
					K_THREAD_STACK_SIZEOF(imu_stack_area),
					K_HIGHEST_APPLICATION_THREAD_PRIO,NULL);
#endif
	k_work_queue_start(&gps_work_q, gps_stack_area,
					K_THREAD_STACK_SIZEOF(gps_stack_area),
					CONFIG_APPLICATION_WORKQUEUE_PRIORITY,NULL);	
	
	if(IS_ENABLED(CONFIG_WATCHDOG))
	{
		watchdog_init_and_start(&k_sys_work_q);
	}
}

bool system_is_completed(void)
{
	return sys_pwron_completed_flag;
}

void system_init_completed(void)
{
	if(!sys_pwron_completed_flag)
		sys_pwron_completed_flag = true;
}

/***************************************************************************
* 描  述 : main函数 
* 入  参 : 无 
* 返回值 : int 类型
**************************************************************************/
int main(void)
{
	LOGD("begin");

	work_init();
	system_init();
	
//	test_show_string();
//	test_show_image();
//	test_show_color();
//	test_show_stripe();
//	test_nvs();
//	test_flash();
//	test_uart_ble();
//	test_sensor();
//	test_show_digital_clock();
//	test_sensor();
//	test_pmu();
//	test_crypto();
//	test_imei_for_qr();
//	test_tp();
//	test_gps_on();
//	test_nb();
//	test_i2c();
//	test_bat_soc();
//	test_notify();
//	test_wifi();
//	LogInit();

	while(1)
	{
		KeyMsgProcess();
		TimeMsgProcess();
		NBMsgProcess();
		GPSMsgProcess();
		PMUMsgProcess();
	#ifdef CONFIG_IMU_SUPPORT	
		IMUMsgProcess();
	#ifdef CONFIG_FALL_DETECT_SUPPORT
		FallMsgProcess();
	#endif
	#endif
		SettingsMsgPorcess();
		SOSMsgProc();
	#ifdef CONFIG_WIFI_SUPPORT	
		WifiMsgProcess();
	#endif
		UartMsgProc();
	#ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
		DlMsgProc();
	#endif
	#ifdef CONFIG_FOTA_DOWNLOAD
		VerCheckMsgProc();
		FotaMsgProc();
	#endif
	#ifdef CONFIG_AUDIO_SUPPORT
		AudioMsgProcess();
	#endif
	#ifdef CONFIG_PRESSURE_SUPPORT
		PressureMsgProcess();
	#endif
	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		FactoryTestProccess();
	#endif
	#ifdef CONFIG_LOG
		LogMsgProcess();
	#endif
		system_init_completed();
		k_cpu_idle();
	}
}
