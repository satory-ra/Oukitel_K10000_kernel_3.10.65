#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
#if defined(CONFIG_T823X_WH_PROJ)
 #define PS_HIGH 6000
 #define PS_LOW  5600
#elif defined(CONFIG_T875C_DG_PROJ)
	 #define PS_HIGH 250
	 #define PS_LOW  150
#elif defined(CONFIG_T875A_T92_PROJ)
	 #define PS_HIGH 1500
	 #define PS_LOW  1400
#elif defined(CONFIG_T875F_T95_PROJ)
	 #define PS_HIGH 2000
	 #define PS_LOW  1900
#elif defined(CONFIG_T823T_HS_PROJ)
 #define PS_HIGH 2000
 #define PS_LOW  1600
#else
 #define PS_HIGH 800
 #define PS_LOW  500
#endif

static struct alsps_hw cust_alsps_hw = {
  	.i2c_num    = 2,
	.polling_mode_ps = 0,		          /* not work, define in epl8882.c */
	.polling_mode_als = 1,		          /* not work, define in epl8882.c */
	.power_id   = MT65XX_POWER_NONE,      /* LDO is not used */
	.power_vol  = VOL_DEFAULT,            /* LDO is not used */
	.i2c_addr   = {0x92, 0x48, 0x78, 0x00},
	.als_level	= {20, 45, 70, 90, 150, 300, 500, 700, 1150, 2250, 4500, 8000, 15000, 30000, 50000},
	.als_value	= {10, 30, 60, 80, 100, 200, 400, 600, 800, 1500, 3000, 6000, 10000, 20000, 40000, 60000},
	.ps_threshold_low = PS_LOW,
	.ps_threshold_high = PS_HIGH,
	.als_threshold_low = 1000,
	.als_threshold_high = 1500,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

