/****************************************Copyright (c)************************************************
** File Name:			    pmu.h
** Descriptions:			power mode unit message process headsss file
** Created By:				xie biao
** Created Date:			2024-02-28
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __PMU_H__
#define __PMU_H__

#include <stdint.h>
#include <zephyr/kernel.h>

#define PMU_DEBUG

//#define PMU_SENSOR_MAX20353
//#define PMU_SENSOR_NPM1300
#define PMU_SENSOR_CW221X_CW630X

typedef int32_t (*pmu_write_ptr)(struct device *handle, uint8_t addr, uint16_t reg, uint8_t *bufp, uint16_t len);
typedef int32_t (*pmu_read_ptr)(struct device *handle, uint8_t addr, uint16_t reg, uint8_t *bufp, uint16_t len);

typedef struct {
  /** Component mandatory fields **/
  pmu_write_ptr  write_reg;
  pmu_read_ptr   read_reg;
  /** Customizable optional pointer **/
  struct device *handle;
} pmudev_ctx_t;

typedef enum
{
	BAT_CHARGING_NO,
	BAT_CHARGING_PROGRESS,
	BAT_CHARGING_FINISHED,
	BAT_CHARGING_MAX
}BAT_CHARGER_STATUS;

typedef enum
{
	BAT_LEVEL_VERY_LOW,
	BAT_LEVEL_LOW,
	BAT_LEVEL_NORMAL,
	BAT_LEVEL_GOOD,
	BAT_LEVEL_MAX
}BAT_LEVEL_STATUS;

typedef enum
{
	VIB_ONCE,
	VIB_CONTINUITY,
	VIB_RHYTHMIC,
	VIB_MAX
}VIBRATE_MODE;

typedef struct
{
	VIBRATE_MODE work_mode;
	uint32_t on_time;
	uint32_t off_time;
}vibrate_msg_t;

extern bool pmu_trige_flag;
extern bool pmu_alert_flag;
extern bool pmu_check_temp_flag;
extern bool vibrate_start_flag;
extern bool vibrate_stop_flag;
extern bool charger_is_connected;

extern uint8_t g_bat_soc;
extern BAT_CHARGER_STATUS g_chg_status;
extern BAT_LEVEL_STATUS g_bat_level;

extern void test_pmu(void);
extern void pmu_init(void);
extern void Set_Screen_Backlight_On(void);
extern void Set_Screen_Backlight_Off(void);
extern void Set_PPG_Power_On(void);
extern void Set_PPG_Power_Off(void);
extern void SystemShutDown(void);
extern void PMUMsgProcess(void);
extern void GetBatterySocString(uint8_t *str_utc);
extern void system_power_off(uint8_t flag);
extern void PmuInterruptHandle(void);
#endif/*__PMU_H__*/