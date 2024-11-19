/****************************************Copyright (c)************************************************
** File Name:			    pmu.c
** Descriptions:			power mode unit message process source file
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
#include "pmu.h"

uint8_t g_bat_soc = 0;

bool charger_is_connected = false;

BAT_CHARGER_STATUS g_chg_status = BAT_CHARGING_NO;
BAT_LEVEL_STATUS g_bat_level = BAT_LEVEL_NORMAL;

