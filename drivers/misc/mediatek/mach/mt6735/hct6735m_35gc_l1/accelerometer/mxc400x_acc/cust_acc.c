#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>


#if defined(CONFIG_T525_PROJ)
	#define MXC400X_ACC_I2C_BUS_NUM    2
	#define MXC400X_ACC_DIRECTION      7//6
#else
	#define MXC400X_ACC_I2C_BUS_NUM    2
	#define MXC400X_ACC_DIRECTION      1
#endif

/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = MXC400X_ACC_I2C_BUS_NUM,
    .direction = MXC400X_ACC_DIRECTION,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
    .is_batch_supported = false,
};
/*---------------------------------------------------------------------------*/
struct acc_hw* mxc400x_get_cust_acc_hw(void)
{
    return &cust_acc_hw;
}
