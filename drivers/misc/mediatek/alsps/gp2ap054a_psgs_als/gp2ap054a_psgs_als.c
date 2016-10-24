/* 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <linux/gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "gp2ap054a_psgs_als.h"
#include <linux/hwmsen_helper.h>
#include <alsps.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE


/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define gp2ap054a_PSGS_DEV_NAME     "gp2ap054a00F_PSGS"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                 
/******************************************************************************
* extern functions 
*******************************************************************************/
/*extern unsigned int mt_eint_ack(unsigned int eint_num);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern int module_check(void);
*/
 #include <mach/eint.h>

/*----------------------------------------------------------------------------*/
static int gp2ap054a_psgs_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int gp2ap054a_psgs_i2c_remove(struct i2c_client *client);
//static int gp2ap054a_psgs_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int gp2ap054a_psgs_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int gp2ap054a_psgs_i2c_resume(struct i2c_client *client);

static int alsps_local_init(void);
static int alsps_remove(void);
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id gp2ap054a_psgs_i2c_id[] = {{gp2ap054a_PSGS_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_gp2ap054a_psgs={ I2C_BOARD_INFO(gp2ap054a_PSGS_DEV_NAME, 0x39)};
static int debug=1;

static int ps_para_num  = 0;          //the value must select early
/*----------------------------------------------------------------------------*/
struct gs_params{
	//// GS Wakeup/Shutdown flag////
	int gs_enabled;
	//// Clear PROX & Interrupt FLAG registers for ONOFF Func////
	int clear_int;

	//// GS <-> PS mode switch function ////
	int switch_mode_on;
	short int  switch_counts_th;
	//// PS Mode ////
	int ps_only_on;

	//// Temporal parameters for Direction Judgement ////
	/*
	float max_x1, min_x1, max_y1, min_y1;
	float max_x2, min_x2, max_y2, min_y2, diff_max_x, diff_max_y;
	*/
	int max_x1, min_x1, max_y1, min_y1;
	int max_x2, min_x2, max_y2, min_y2, diff_max_x, diff_max_y;
	
	unsigned int x_plus, x_minus, y_plus, y_minus;
	unsigned char gs_state;
	int speed_counts;

	//// Thresholds depending on the performance ////
	signed short int  ignore_diff_th;
	unsigned short int  ignore_z_th;
	//float ratio_th;
	int ratio_th;

	//// Parameters for active offset cancelation ////
	int active_osc_on;
	int allowable_variation;
	int acquisition_num;

	//// Parameters for Zoom function ////
	int zoomFuncOn;
	int zoom_counts_th;
	//// Saturation notification ////
	int saturated_data[4];
};

#define MOVING_AVE_NUM		32
#define PULSE_AVE_NUM		5
#define FAST_PULSE_RANGE	1
#define SLOW_PULSE_RANGE	0

struct pulse_params{
	//int  pulse_state;
	//int data12[MOVING_AVE_NUM];		//[moving_ave_num]
	//int data12_ave[MOVING_AVE_NUM];	//[moving_ave_num]
	//int data12_ave_now;				//0
	//int data12_lhpf;				//0
	
	int d12[48];//[ <moving_ave_num]
	int d12_ave[48];//[ <moving_ave_num]
	int d12_ave_now;//0
	int d12_now;//0
	int d12_lhpf;//0	
	int mult_data;
	int  hpf_num;//16
	int	moving_ave_num;//16
	int  pulse_ave_num;//3
	int pulse[PULSE_AVE_NUM];		//[pulse_ave_num]


	int data12pre;
	int q;
	int p;
	
	int pulse_ave;
	int pulse_ave_pre;
	int count;
	int  analog_offset_reg;
	int  pulse_filer_range;	
	int min;
	int min_pre;
	int ao_flag;
	int pulse_count_start_flag;
	//int func_ave_result_temp;
};

/*----------------------------------------------------------------------------*/

struct gp2ap054a_psgs_als_priv
{
	struct mutex				mutex ;
	struct alsps_hw  *hw;
	struct i2c_client *client;
	//u8					regData[12] ;
	struct work_struct	eint_work;
	u8							regData[20] ;
	/*
	struct input_dev		   *input_dev ;
	struct input_dev*			proximity_input_dev;
	struct input_dev* 			gesture_input_dev;
	*/
	int							mode ;
	
	/*misc*/
	u16 		als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on; 	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	
	/*data*/
	u16			als;
	//u16			ps;
	u16			als_level_num;
	u16			als_value_num;
	u32			als_level[C_CUST_ALS_LEVEL-1];
	u32			als_value[C_CUST_ALS_LEVEL];
	int			als_mode ;
	int			als_lux_prev ;
	
	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val; 	/*the cmd value can't be read, stored in ram*/
	//atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	//atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;

	struct workqueue_struct 	*gs_polling_workqueue;
	struct work_struct			gs_polling_work ; 
	struct hrtimer				gs_polling_timer ;
	int							gs_delay ;
	ktime_t						gs_polling_delay ;

	int							ps_gpio ;
	int							ps_irq ;
	struct work_struct			ps_int_work ;
	int							ps_distance ;
	int 						ps;

	struct delayed_work			ps_avg_timer_work ;
	int							ps_avg_enabled ;
	int							ps_avg_delay ;
	ktime_t						ps_avg_read_delay ;
	struct hrtimer				ps_avg_read_timer ;
	struct work_struct			ps_avg_work ;
	int							ps_avg[3] ;
	int							ps_avg_tmp[3] ;
	int							ps_avg_read_cnt ;
	int							enable_proximity;
	int 						enable_gesture;
	int 						current_ps_data;
	int 						last_gesture;
	int 						delay_proximity;
	int 						delay_gesture;
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/
    //atomic_t 						ps_thd_val_low;
    //atomic_t 						ps_thd_val_high;
	atomic_t	ps_in_threshold;          //the threshold used now
    atomic_t    ps_out_threshold;
     atomic_t    ps_base_value;            //the base value is from app  
    atomic_t						trace;
	int 						status;
	unsigned char write_register[24];

	int 		analog_offset_reg;

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
	#endif     
} ;
/*----------------------------------------------------------------------------*/
static 	struct gs_params st_gs;

static struct i2c_driver gp2ap054a_psgs_i2c_driver = {	
	.probe      = gp2ap054a_psgs_i2c_probe,
	.remove     = gp2ap054a_psgs_i2c_remove,
	//.detect     = gp2ap054a_psgs_i2c_detect,
	.suspend    = gp2ap054a_psgs_i2c_suspend,
	.resume     = gp2ap054a_psgs_i2c_resume,
	.id_table   = gp2ap054a_psgs_i2c_id,
	.driver = {
		.name = gp2ap054a_PSGS_DEV_NAME,
	},
};

/*----------------------------------------------------------------------------*/
struct PS_CALI_DATA_STRUCT
{
	int close;
	int far_away;
	int valid;
};

/*----------------------------------------------------------------------------*/

static int alsps_init_flag =-1; // 0<==>OK -1 <==> fail

static struct alsps_init_info gp2ap054a_init_info = {
		.name = gp2ap054a_PSGS_DEV_NAME,
		.init = alsps_local_init,
		.uninit = alsps_remove,
	
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *gp2ap054a_psgs_i2c_client = NULL;
//static struct gp2ap054a_psgs_als_priv *g_gp2ap054a_psgs_ptr = NULL;
//static struct gp2ap054a_psgs_als_priv *gp2ap054a_als_obj = NULL;
static struct gp2ap054a_psgs_als_priv *gp2ap054a_psgs_als_obj = NULL;
static struct platform_driver gp2ap054a_psgs_driver;
static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS    = 1,
	CMC_BIT_PS	   = 2,
	CMC_BIT_GS	   = 4,
	CMC_BIT_PULSE  = 8,
}CMC_BIT;
/*-----------------------------CMC for debugging-------------------------------*/
typedef enum {
    CMC_TRC_ALS_DATA= 0x0001,
    CMC_TRC_PS_DATA = 0x0002,
    CMC_TRC_EINT    = 0x0004,
    CMC_TRC_IOCTL   = 0x0008,
    CMC_TRC_I2C     = 0x0010,
    CMC_TRC_CVT_ALS = 0x0020,
    CMC_TRC_CVT_PS  = 0x0040,
    CMC_TRC_DEBUG   = 0x8000,
} CMC_TRC;

/*following GIPO should be set by DCT TOOL*/
#if 0
#define GPIO_ALS_EINT_PIN         GPIO190
#define GPIO_ALS_EINT_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_ALS_EINT_PIN_M_EINT  GPIO_MODE_01
#define GPIO_ALS_EINT_PIN_M_PWM  GPIO_MODE_04
#define CUST_EINT_ALS_NUM              3
#define CUST_EINT_ALS_DEBOUNCE_CN      0
#define CUST_EINT_ALS_POLARITY         CUST_EINT_POLARITY_LOW
#define CUST_EINT_ALS_SENSITIVE        CUST_EINT_LEVEL_SENSITIVE
#define CUST_EINT_ALS_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE
#endif

/*----------------------------------------------------------------------------*/
static u8 gp2ap_ps_th[4] = {
	/* Reg.4 INT_LT[7:0]:0xE8 */
	0xA0,
	/* Reg.5 INT_LT[15:8]:0x03 */
	0x16,
	/* Reg.6 INT_HT[7:0]:0xDC */
	0xAA,
	/* Reg.7 INT_HT[15:8]:0x05 */
	0x19
} ;

static u8 gp2ap_init_data[20] = {
    /* COMMAND1(00H) */
    COMMAND1_SD,
    /* COMMAND2(01H) */
    0x00,
    /* COMMAND3(02H) */
    (COMMAND3_INT_PS | COMMAND3_INT_GS_PULSE | COMMAND3_INT_ALS_PULSE | COMMAND3_INT_PS_PULSE),
    /* ALS1(03H) */
    (ALS1_RES18 | MIDDLE_LUX_RANGE | 0x40),
    /* ALS2(04H) */
    (ALS2_ALS_INTVAL1P56),
    /* PS1(05H) */
    (PS1_PRST4 | PS1_RANGEX2 | PS1_RES12),
    /* PS2(06H) */
    (PS2_IS150 | PS2_SUM16 | 0x01),
    /* PS3(07H) */
    (0xC8 | PS3_GS_INT3| PS3_GS_INTVAL6P25),
    /* PS_LT_LSB(08H) */
    0x00,
    /* PS_LT_MSB(09H) */
    0x00,
    /* PS_HT_LSB(0AH) */
    0xFF,
    /* PS_HT_MSB(0BH) */
    0xFF,
    /* OS_DATA0_LSB(0CH) */
    0x00,
    /* OS_DATA0_MSB(0DH) */
    0x00,
    /* OS_DATA1_LSB(0EH) */
    0x00,
    /* OS_DATA1_MSB(0FH) */
    0x00,
    /* OS_DATA2_LSB(10H) */
    0x00,
    /* OS_DATA2_MSB(11H) */
    0x00,
    /* OS_DATA3_LSB(12H) */
    0x00,
    /* OS_DATA3_MSB(13H) */
    0x00
} ;


static void clearGSparams(struct gs_params *p_gs);
static void initGSparams(struct gs_params *p_gs);
static void getActiveOffset(short int  *raw_data, short int  *act_os_d, struct gs_params *p_gs);
static int change_work_mode(int mode,struct gp2ap054a_psgs_als_priv* data);
static short int  getDirection(short int  *sub_os_data, struct gs_params *p_gs);
static void gp2ap_init_device( u8 mode, struct gp2ap054a_psgs_als_priv *data );

/*----------------------------------------------------------------------------*/
/*void set_ps_para_num(int para)
{
	if( para >= 0 && para<= (sizeof(gp2ap054a_ps_para)/sizeof(struct ps_para) -1))
		ps_para_num=para;
	else
		printk(KERN_ERR"%d is wrong ps index,[0,%d]\n",para,sizeof(gp2ap054a_ps_para)/sizeof(struct ps_para) -1);
}

static int gp2ap054a_ps_select_para(struct gp2ap054a_psgs_als_priv *obj)         //
{
    int out_threshold, in_threshold;   
    int base = atomic_read(&obj->ps_base_value);
    int step_num;
    struct ps_step *step;
    struct ps_para *para;
    
    //TODO: get different parameter here
    para = &gp2ap054a_ps_para[ps_para_num];
    step = &para->step[0];
    
    //count the the threshold from base value
    if(!(base > para->base_value_min && base <= para->base_value_max))
        base = para->base_value;
    
    for(step_num = 0; step_num < para->step_num; step_num++, step++)
    {
        if(base > step->min && base <= step->max)
            break;
    }

#if PS_OPERATION
    in_threshold  = base + step->in_coefficient;
    out_threshold = base + step->out_coefficient;
#else
    in_threshold  = base * (step->in_coefficient) / 10;
    out_threshold = base * (step->out_coefficient) / 10;
#endif
    printk("select_threshold in_coefficient %d out_coefficient %d\n", step->in_coefficient, step->out_coefficient);

	atomic_set(&obj->ps_in_threshold,  in_threshold);
    atomic_set(&obj->ps_out_threshold, out_threshold);
    atomic_set(&obj->ps_base_value,    base);           //reset base value
	APS_LOG("ps_in_threshold %d ps_out_threshold %d\n", atomic_read(&obj->ps_in_threshold), atomic_read(&obj->ps_out_threshold));
	return 0; 
}*/
/*----------------------------------------------------------------------------*/


static int gp2ap_i2c_write( u8 reg, u8 *wbuf, struct i2c_client *client )
{
	int 		err = 0 ;

	if( client == NULL )
	{
		return -ENODEV ;
	}
	
	err = hwmsen_write_byte(client,reg,*wbuf);
	//APS_DBG("PS:  %05d => %05d =>%05d [M] \n", reg, *wbuf,err);
	return err ;
}
static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
   u8 buf;
    int ret = 0;
    client->addr = client->addr&(I2C_MASK_FLAG | I2C_WR_FLAG |I2C_RS_FLAG);

    buf = addr;
	ret = i2c_master_send(client, (const char*)&buf, 1<<8 | 1);
    if (ret < 0) {
        HWM_ERR("send command error!!\n");
        return -EFAULT;
    }

    *data = buf;
	client->addr = client->addr& I2C_MASK_FLAG;
    return 0;
}

//for gesture judgement 
static void initGSparams(struct gs_params *p_gs){
	int num;
	clearGSparams(p_gs);
	p_gs->clear_int		 		= 1;
	p_gs->switch_mode_on 		= 0;
	p_gs->switch_counts_th 		= 300;
	p_gs->ps_only_on			= 0;
	p_gs->ignore_z_th 			= 20;
	p_gs->ignore_diff_th 		= 10;
	//p_gs->ratio_th 				= 0.1;
	p_gs->ratio_th 				= 100;// kernel space has no float type
	p_gs->active_osc_on 		= 1;
	p_gs->allowable_variation 	= 30;
	p_gs->acquisition_num		= 10;
	p_gs->zoomFuncOn 			= 1;
	p_gs->zoom_counts_th		= 25;
	for(num=0;num<4;num++){
		p_gs->saturated_data[num]	= 0;
	}
}

// *******************************************************************
// * Name	: clearGSparams(Input)
// * Input	: struct gs_params pointer
// * Output	:
// * Note	: clear temporal variables for direction judgement
// *******************************************************************
static void clearGSparams(struct gs_params *p_gs) {
	p_gs->x_plus  = 0;
	p_gs->x_minus = 0;
	p_gs->y_plus  = 0;
	p_gs->y_minus = 0;
	p_gs->max_x1  = 0;
	p_gs->min_x1  = 0;
	p_gs->max_y1  = 0;
	p_gs->min_y1  = 0;
	p_gs->max_x2  = 0;
	p_gs->min_x2  = 0;
	p_gs->max_y2  = 0;
	p_gs->min_y2  = 0;
	p_gs->diff_max_x = 0;
	p_gs->diff_max_y = 0;
	p_gs->speed_counts= 0;
	p_gs->gs_state = STATE_ONE;

	return;
}


// *******************************************************************
// * Name	: getActiveOffset(Input1, Output, Input2)
// * Input	: Input1 = raw_data[5], Input2 = struct gs_params pointer
// * Output	: act_os_d[4]
// * Note	: to get offset value from acquisition times of raw data.
// *******************************************************************
static void getActiveOffset(short int  *raw_data, short int  *act_os_d, struct gs_params *p_gs){
	static int act_os_counts=0;
	static short int  max_d[4]={0};
	static short int  min_d[4]={0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
	static int avg_d[4]={0};
	int num;

	if( (p_gs->saturated_data[0]) || (p_gs->saturated_data[1]) ||
		(p_gs->saturated_data[2]) || (p_gs->saturated_data[3]) ){
		act_os_counts = 0;
		return;
	}
	
	if( *(raw_data+4) < max_aoc_counts){  
		//// The last measurement of offset
		if(act_os_counts == p_gs->acquisition_num -1){
			for(num=0;num<4;num++){
				avg_d[num] = avg_d[num] + *(raw_data+num);
				if(*(raw_data+num) > max_d[num]){
					max_d[num] = *(raw_data+num);
				}
				if(*(raw_data+num) < min_d[num]){
					min_d[num] = *(raw_data+num);
				}
			}
			for(num=0;num<4;num++){
				avg_d[num] = avg_d[num] - max_d[num] - min_d[num];
				avg_d[num] = avg_d[num]/(p_gs->acquisition_num-2);
				//// If data variation is large, offset data doesn't update and uses the former data.
				if( (max_d[num] < avg_d[num] + p_gs->allowable_variation) &&
					(min_d[num] + p_gs->allowable_variation > avg_d[num])){
					/*
					if(max_d[num] < (p_gs->ignore_z_th/4)){
						*(act_os_d+num) = max_d[num];
					}else{
						*(act_os_d+num) = max_d[num] -(p_gs->ignore_z_th/4);
					}
					*/
					*(act_os_d+num) = avg_d[num] + p_gs->allowable_variation;
				}
			}
			act_os_counts = 0;
		//// The first measurement of offset
		}else if(act_os_counts == 0){
			for(num=0;num<4;num++){
				avg_d[num] = *(raw_data+num);
				max_d[num] = *(raw_data+num);
				min_d[num] = *(raw_data+num);
			}
			act_os_counts++;
		}else{
			for(num=0;num<4;num++){
				avg_d[num] = avg_d[num] + *(raw_data+num);
				if(*(raw_data+num) > max_d[num]){
					max_d[num] = *(raw_data+num);
				}
				if(*(raw_data+num) < min_d[num]){
					min_d[num] = *(raw_data+num);
				}
			}
			act_os_counts++;
		}

	}
	return;
}

// *******************************************************************
// * Name	: output_gs_debug1(Input)
// * Input	: Input = getDirection() results
// * Output	: 
// * Note	: display the direction results on the terminal window
// *******************************************************************
void output_gs_debug(int  res){
	
	switch(res){
		case (DIR_RIGHT|((short int )SPEED_HIGH<<4)):
			printk(KERN_INFO "DDD GESTURE R\n");
		break;
		case (DIR_RIGHT|((short int )SPEED_MID<<4)):
			printk(KERN_INFO "DDD GESTURE R\n");
		break;
		case 0x8://(DIR_RIGHT):
			printk(KERN_INFO "DDD GESTURE R\n");
		break;
		case (DIR_LEFT|((short int )SPEED_HIGH<<4)):
			printk(KERN_INFO "DDD GESTURE L\n");
		break;
		case (DIR_LEFT|((short int )SPEED_MID<<4)):
			printk(KERN_INFO "DDD GESTURE L\n");
		break;						
		case 0x4://(DIR_LEFT):
			printk(KERN_INFO "DDD GESTURE L\n");
		break;
		case (DIR_TOP|((short int )SPEED_HIGH<<4)):
			printk(KERN_INFO "DDD GESTURE T\n");
		break;
		case (DIR_TOP|((short int )SPEED_MID<<4)):
			printk(KERN_INFO "DDD GESTURE T\n");
		break;						
		case 0x2://(DIR_TOP):
			printk(KERN_INFO "DDD GESTURE T\n");
		break;
		case (DIR_BOTTOM|((short int )SPEED_HIGH<<4)):
			printk(KERN_INFO "DDD GESTURE B\n");
		break;
		case (DIR_BOTTOM|((short int )SPEED_MID<<4)):
			printk(KERN_INFO "DDD GESTURE B\n");
		break;						
		case 0x1://(DIR_BOTTOM):
			printk(KERN_INFO "DDD GESTURE B\n");
		break;

		case ((short int )ZOOM_LEVEL5<<8):
			printk(KERN_INFO "DDD GESTURE ZOOM LEVEL5\n");
		break;
		case ((short int )ZOOM_LEVEL4<<8):
			printk(KERN_INFO "DDD GESTURE ZOOM LEVEL4\n");
		break;
		case ((short int )ZOOM_LEVEL3<<8):
			printk(KERN_INFO "DDD GESTURE ZOOM LEVEL3\n");
		break;
		case ((short int )ZOOM_LEVEL2<<8):
			printk(KERN_INFO "DDD GESTURE ZOOM LEVEL2\n");
		break;
		case ((short int )ZOOM_LEVEL1<<8):
			printk(KERN_INFO "DDD GESTURE ZOOM LEVEL1\n");
		break;
		
		default:
		break;
	}
}

// *******************************************************************
// * Name	: getDirection(Input1, Input2)
// * Input	: Input1 = sub_os_data[4], Input2 = struct gs_params pointer
// * Output	: Output return value is unsigned 16bits integer = 0x0ZSD
//            Z(4bits from the higer 5th to 8th bit) ; Zoom results
//            S(4bits from the lower 5th to 8th bit) ; Speed results
//            D(lower 4bits) ; Direction results
// * Note	: get the result of Direction or Zoom
// *******************************************************************
short int  getDirection(short int  *sub_os_data, struct gs_params *p_gs){
	signed short int  data_x, data_y;
	unsigned short int  data_z=0;
	int ratio_x, ratio_y;
	short int  direction_res = 0;
	const int COUNTS_HIGH_SP = 6;
	const int COUNTS_MID_SP	= 15;
	int num;

	static int now_level=ZOOM_LEVEL_DEF, temp_level=ZOOM_LEVEL_DEF, prev_level=ZOOM_LEVEL_DEF;
	static int change_zoom_counts = 0;

	//// Diff calculation ////
	data_y = *(sub_os_data+2) + *(sub_os_data+3) - *sub_os_data - *(sub_os_data+1);
	if( ((data_y > -(p_gs->ignore_diff_th)) && (data_y < (p_gs->ignore_diff_th))) ||
		(*(sub_os_data  ) == OVER_FLOW_DATA) || (*(sub_os_data+1) == OVER_FLOW_DATA) ||
		(*(sub_os_data+2) == OVER_FLOW_DATA) || (*(sub_os_data+3) == OVER_FLOW_DATA) )
	{
		data_y = 0;
	}
	data_x = *(sub_os_data+1) + *(sub_os_data+2) - *(sub_os_data) - *(sub_os_data+3);
	if( ((data_x > -(p_gs->ignore_diff_th)) && (data_x < (p_gs->ignore_diff_th))) ||
		(*(sub_os_data  ) == OVER_FLOW_DATA) || (*(sub_os_data+1) == OVER_FLOW_DATA) ||
		(*(sub_os_data+2) == OVER_FLOW_DATA) || (*(sub_os_data+3) == OVER_FLOW_DATA) )
	{
		data_x = 0;
	}
	for(num=0; num<4; num++){
		data_z += *(sub_os_data+num);
	}

	//// Ratio calculation ////
	if(data_z == 0){
		ratio_y = 0;
		ratio_x = 0;
	}else{
		/*
		ratio_y = (float)data_y / (float)data_z;
		ratio_x = (float)data_x / (float)data_z;
		*/
		ratio_y = (data_y*1000) / (data_z);//modify *10
		ratio_x = (data_x*1000) / (data_z);
		//TODO add log
		if(debug){
			printk(KERN_INFO"ratio x %d y % data x %d y %d z %d\n",ratio_x,ratio_y,data_x,data_y,data_z);
		}
	}

	//// Judgement FSM start ////
	switch (p_gs->gs_state)
	{
		case 1://STATE_ONE:
			if(data_z >= p_gs->ignore_z_th){
				if(ratio_x > (p_gs->ratio_th)){
					p_gs->x_plus = 1;
					p_gs->max_x1 = ratio_x;
				}else{
					p_gs->x_plus = 0;
					p_gs->max_x1 = 0;
				}

				if(ratio_x < -(p_gs->ratio_th)){
					p_gs->x_minus = 1 ;
					p_gs->min_x1 = ratio_x;
				}else{
					p_gs->x_minus = 0;
					p_gs->min_x1 = 0;
				}

				if(ratio_y > (p_gs->ratio_th)){
					p_gs->y_plus = 1;
					p_gs->max_y1 = ratio_y;
				}else{
					p_gs->y_plus = 0;
					p_gs->max_y1 = 0;
				}

				if(ratio_y < -(p_gs->ratio_th)){
					p_gs->y_minus = 1;
					p_gs->min_y1 = ratio_y;
				}else{
					p_gs->y_minus = 0;
					p_gs->min_y1 = 0;
				}
			}

			if( (p_gs->x_plus > 0) | (p_gs->x_minus > 0) |
				(p_gs->y_plus > 0) | (p_gs->y_minus > 0) )
			{
				p_gs->gs_state = STATE_TWO;
			}else{
				p_gs->gs_state = STATE_ONE;
			}

		break;

		case 2://STATE_TWO:
			if( (data_z < p_gs->ignore_z_th) )
			{
				clearGSparams(p_gs);
			}else if(
				      ((p_gs->x_plus ) && (ratio_x < -(p_gs->ratio_th))) ||
				      ((p_gs->x_minus) && (ratio_x >  (p_gs->ratio_th))) ||
				      ((p_gs->y_plus ) && (ratio_y < -(p_gs->ratio_th))) ||
				      ((p_gs->y_minus) && (ratio_y >  (p_gs->ratio_th))) )
			{
				if(ratio_x > (p_gs->ratio_th)){
					p_gs->max_x2 = ratio_x;
				}else{
					p_gs->max_x2 = 0;
				}

				if(ratio_x < -(p_gs->ratio_th)){
					p_gs->min_x2 = ratio_x;
				}else{
					p_gs->min_x2 = 0;
				}

				if(ratio_y > (p_gs->ratio_th)){
					p_gs->max_y2 = ratio_y;
				}else{
					p_gs->max_y2 = 0;
				}

				if(ratio_y < -(p_gs->ratio_th)){
					p_gs->min_y2 = ratio_y;
				}else{
					p_gs->min_y2 = 0;
				}
				p_gs->gs_state = STATE_THREE;

			}else {
				if( (ratio_x > (p_gs->max_x1)) && (ratio_x > (p_gs->ratio_th))){
					p_gs->max_x1 = ratio_x;
					p_gs->x_plus = 1;
				}else if( (ratio_x < (p_gs->min_x1)) & (ratio_x < -(p_gs->ratio_th)) ){
					p_gs->min_x1 = ratio_x;
					p_gs->x_minus = 1;
				}
				if( (ratio_y > (p_gs->max_y1)) & (ratio_y > (p_gs->ratio_th)) ){
					p_gs->max_y1 = ratio_y;
					p_gs->y_plus = 1;
				}else if( (ratio_y < (p_gs->min_y1)) & (ratio_y < -(p_gs->ratio_th)) ){
					p_gs->min_y1 = ratio_y;
					p_gs->y_minus =1;
				}
				if( p_gs->x_plus && p_gs->x_minus){
					if((p_gs->max_x1) > -(p_gs->min_x1)) {
						p_gs->x_plus  = 1;
						p_gs->x_minus = 0;
					}else {
						p_gs->x_plus  = 0;
						p_gs->x_minus = 1;
					}
				}
				if( p_gs->y_plus && p_gs->y_minus){
					if((p_gs->max_y1) > -(p_gs->min_y1)) {
						p_gs->y_plus  = 1;
						p_gs->y_minus = 0;
					}else {
						p_gs->y_plus  = 0;
						p_gs->y_minus = 1;
					}
				}
				p_gs->gs_state = STATE_TWO;
			}
		break;

		case 3://STATE_THREE:
			if( data_z < (p_gs->ignore_z_th) )
			{
				if( (p_gs->x_plus) & (p_gs->min_x2 < -(p_gs->ratio_th))){
					p_gs->diff_max_x = p_gs->max_x1 - p_gs->min_x2;
				}else if( (p_gs->x_minus) & (p_gs->max_x2 > p_gs->ratio_th) ){
					p_gs->diff_max_x = p_gs->max_x2 - p_gs->min_x1;
				}else {
					p_gs->diff_max_x = 0;
				}

				if( (p_gs->y_plus) & (p_gs->min_y2 < -(p_gs->ratio_th)) ){
					p_gs->diff_max_y = p_gs->max_y1 - p_gs->min_y2;
				}else if( (p_gs->y_minus) & (p_gs->max_y2 > p_gs->ratio_th) ){
					p_gs->diff_max_y = p_gs->max_y2 - p_gs->min_y1;
				}else{
					p_gs->diff_max_y = 0;
				}

				//// Final direction Judgement ////
				if( p_gs->diff_max_x >= p_gs->diff_max_y){
					if(p_gs->x_plus == 1){
						direction_res = DIR_RIGHT;
						printk(KERN_INFO"dir: >>>>>>>>>>>>>>>>>>>>>>>>>> \n");
					}else {
						direction_res = DIR_LEFT;
						printk(KERN_INFO"dir: <<<<<<<<<<<<<<<<<<<<<<<<<<<< \n");
					}
				}else{
					if(p_gs->y_plus == 1){
						direction_res = DIR_TOP;
						printk(KERN_INFO"dir: AAAAAAAAAAAAAAAA \n");
					}else {
						direction_res = DIR_BOTTOM;
						printk(KERN_INFO"dir: VVVVVVVVVVVVVVV\n");
					}
				}
				if(debug){
					printk(KERN_INFO"diff_max_x %d diff_max_y %d x_plus %d y_plus %d direction_res 0x%x \n",
							p_gs->diff_max_x,p_gs->diff_max_y,p_gs->x_plus,p_gs->y_plus,direction_res);
				}
				//del speed data,no use 20141028
				/*
				if(p_gs->speed_counts < (short int )COUNTS_HIGH_SP*(short int )sampling_rate/(short int )STANDARD_SAMPLING_RATE){
					direction_res |=((short int )SPEED_HIGH<<4);
				}else if(p_gs->speed_counts < (short int )COUNTS_MID_SP*(short int )sampling_rate/(short int )STANDARD_SAMPLING_RATE){
					direction_res |=((short int )SPEED_MID<<4);
				}
				*/
				if(debug){
					printk(KERN_INFO"p_gs->speed_counts %d direction_res 0x%x\n",p_gs->speed_counts,direction_res);
				}
				clearGSparams(p_gs);
				change_zoom_counts = 0;

			}else {
				if( (ratio_x > p_gs->max_x2) & (ratio_x > p_gs->ratio_th) ){
					p_gs->max_x2 = ratio_x;
				}else if ( (ratio_x < (p_gs->min_x2)) & (ratio_x < -(p_gs->ratio_th))){
					p_gs->min_x2 = ratio_x;
				}
				if( (ratio_y > (p_gs->max_y2)) & (ratio_y > (p_gs->ratio_th)) ){
					p_gs->max_y2 = ratio_y;
				}else if( (ratio_y < (p_gs->min_y2)) & (ratio_y < -(p_gs->ratio_th))){
					p_gs->min_y2 = ratio_y;
				}
				p_gs->gs_state = STATE_THREE;
			}

		break;

		default:
		break;
	}
	if(debug){
		printk(KERN_INFO"after switch diff_max_x %d diff_max_y %d x_plus %d y_plus %d direction_res 0x%x state %d\n",
		p_gs->diff_max_x,p_gs->diff_max_y,p_gs->x_plus,p_gs->y_plus,direction_res,p_gs->gs_state);
	}

	//// Zoom function ////
	if((p_gs->zoomFuncOn == 1) && (data_z > zoom_z_th[0])){
		if(data_z > zoom_z_th[4]){
			temp_level = ZOOM_LEVEL5;
		}else if(data_z > zoom_z_th[3]){
			temp_level = ZOOM_LEVEL4;
		}else if(data_z > zoom_z_th[2]){
			temp_level = ZOOM_LEVEL3;
		}else if(data_z > zoom_z_th[1]){
			temp_level = ZOOM_LEVEL2;
		}else{
			temp_level = ZOOM_LEVEL1;
		}

		if(//(now_level!=temp_level) &&
			(temp_level==prev_level)){
			change_zoom_counts++;
		}else{
			change_zoom_counts = 0;
		}

		if(change_zoom_counts > (short int )(p_gs->zoom_counts_th)*(short int )sampling_rate/(short int )STANDARD_SAMPLING_RATE){
			now_level = temp_level;
			change_zoom_counts = 0;
			clearGSparams(p_gs);
			direction_res = ((short int )now_level<<8);
			if(debug){
				printk(KERN_INFO"change_zoom_counts %d direction_res 0x%x\n",change_zoom_counts,direction_res);
			}
		}
		prev_level = temp_level;

	}

	//// Speed Judgement counts////
	if(p_gs->gs_state > STATE_ONE){
		p_gs->speed_counts++;
	}else{
		p_gs->speed_counts = 0;
	}

	return direction_res;
}// End of getDirection()
//end fo judgemen


static char *strtok( char *s1, char *s2 )
{
	static char	   *str ;
	char		   *start ;

	if( s1 != NULL )
	{
		str = s1 ;
	}
	start = str ;

	if( str == NULL )
	{
		return NULL ;
	}

	if( s2 != NULL )
	{
		str = strstr( str, s2 ) ;
		if( str != NULL )
		{
			*str = 0x00 ;
			str += strlen( s2 ) ;
		}
	}
	else
	{
		str = NULL ;
	}
	return start ;
}
/////

int getGP2AP050Gesture(u16 *adc_data, struct gp2ap054a_psgs_als_priv *data){

	int				i ,num,err;
	int				gesture_ret;


		if(debug)
		{
			printk("raw222222 c0=%05d c1=%05d c2=%05d c3=%05d \n",
				adc_data[0],adc_data[1],adc_data[2],adc_data[3]);
		}

		adc_data[4] = 0;
		for(i=0;i<4;i++){	
		  adc_data[4] += adc_data[i];
		}
		//// Saturation function ////
		if( (adc_data[0] & 0x8000) || (adc_data[1] & 0x8000) ||
			(adc_data[2] & 0x8000) || (adc_data[3] & 0x8000) ){
			for(num=0;num<4;num++){
				if((adc_data[num] & 0x8000)==0x8000){
					st_gs.saturated_data[num] = 1;
				}else{
					st_gs.saturated_data[num] = 0;
				}
				adc_data[num] = 0;
			}
		}else{
			for(num=0;num<4;num++){
				st_gs.saturated_data[num] = 0;
			}
		}

		//// Active offset calibration ////
		if(st_gs.active_osc_on == 1){
			getActiveOffset(adc_data, act_os_d, &st_gs);
			//// Offset subtraction ////
			for(num=0;num<4;num++){
				if(adc_data[num] > act_os_d[num]){
					sub_os_data[num] = adc_data[num] - act_os_d[num];
				}else{
					sub_os_data[num] = 0;
				}
			}
		}else{
			for(num=0;num<4;num++){
				sub_os_data[num] = adc_data[num];
			}
		}
		if(debug){
			printk(KERN_INFO"sub_os_data c0=%05d c1=%05d c2=%05d c3=%05d\n",
				sub_os_data[0],sub_os_data[1],sub_os_data[2],sub_os_data[3]);
		}
		
		//// Data Clipping ////
		for(num=0;num<4;num++){
			if(sub_os_data[num] > OVER_FLOW_DATA){
				sub_os_data[num] = OVER_FLOW_DATA;
			}
		}
		//====Output sub_os_data
		
		//// Calculation & get direction results ////
		gesture_ret = getDirection(sub_os_data, &st_gs);
		printk(KERN_INFO"current gesture_ret=%d\n",gesture_ret);
		return gesture_ret;
		//if(data->last_gesture!=gesture_ret){
		//	
		//	data->last_gesture=gesture_ret;
		//	//TODO:  
		//	//input_report_abs(data->gesture_input_dev,ABS_X,gesture_ret);
		//	//input_sync( data->gesture_input_dev ) ;
		//	gs_sensor_data.values[0] = gesture_ret;
		//	gs_sensor_data.values[1] = 0;
		//	gs_sensor_data.values[2] = 0;
		//	gs_sensor_data.value_divide = 1;
		//	gs_sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		//    if((err = hwmsen_get_interrupt_data(ID_TEMPRERATURE/*ID_GESTURE*/, &gs_sensor_data)))
		//    {
  //              APS_ERR("call %s hwmsen_get_interrupt_data gesture fail = %d\n",__func__,err);
		//    }
		//    if(test_bit(CMC_BIT_PS,&data->enable)){
		//		if(gesture_ret == 0 || ( gesture_ret &(0x3<<8) ) != 0x00 ){
		//			data->ps_distance = 1;//away
		//			ps_sensor_data.values[0] = 1;
		//			ps_sensor_data.values[1] = 0;
		//			ps_sensor_data.values[2] = 0;	
		//			ps_sensor_data.value_divide = 1;
		//			ps_sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		//		}
		//		else{
		//			data->ps_distance = 0;//away
		//			ps_sensor_data.values[0] = 0;
		//			ps_sensor_data.values[1] = 0;
		//			ps_sensor_data.values[2] = 0;	
		//			ps_sensor_data.value_divide = 1;
		//			ps_sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		//		}
		//		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &ps_sensor_data)))
		//		{
		//			APS_ERR("call %s hwmsen_get_interrupt_data proximity fail = %d\n",__func__,err);
		//		}
		//	}
		//    
		//	
		//	printk(KERN_INFO "report gesture_ret=%d\n",gesture_ret);
		//	output_gs_debug(gesture_ret);
		//}


}


// It is necessary to change by the data acuisition method.
// See the kernel driver
#define Polling_Interval	10		//Polling(hrtiemr)  10msec
//#define Polling_Interval	9.61	//Interrupt 		9.61msec

static struct pulse_params st_pulse;

	//float 	mult_data = 0;	
	//float 	func_ave_result = 0;

	
// *******************************************************************
// * Name	: initPULSEparams(Input)
// * Input	: struct pulse_params pointer
// * Output	: 
// * Note	: Initialize variables for GS
// *******************************************************************
void initPULSEparams(struct pulse_params *st_pulse){
	int i;
	st_pulse->d12_ave_now			= 0;
	st_pulse->d12_now				= 0;
	st_pulse->pulse_ave_num			= 5;
 	st_pulse->pulse_ave				= 0;
 	st_pulse->pulse_ave_pre			= 0;
 	st_pulse->d12_lhpf				= 0;
 	st_pulse->moving_ave_num		= 32;
 	st_pulse->hpf_num				= 32;
 	st_pulse->mult_data				= 1;
 	st_pulse->count					= 85;
 	st_pulse->pulse_filer_range		= SLOW_PULSE_RANGE;

 	for( i=0; i<32; i++){
 		st_pulse->d12[i] = 0;
 		st_pulse->d12_ave[i] = 0;
	}
 	
 	st_pulse->pulse[0] = 85;
 	st_pulse->pulse[1] = 85;
 	st_pulse->pulse[2] = 85;
 	st_pulse->pulse[3] = 85;
 	st_pulse->pulse[4] = 85;
 	st_pulse->min = 0;
	st_pulse->min_pre = 0;
	st_pulse->ao_flag = 0;
	st_pulse->pulse_count_start_flag = 0;
//	for(i=0; i<4; i++){
//		st_pulse->raw_d[0][i]		= 0;
//	};
 	
}

// *******************************************************************
// * Name	: func_lpf
// * Input	: Lux_data calculated by getLux_function
// * Output	: average data of data
// * Note	: LP_filter for every 16times
// *******************************************************************
int func_lpf(struct pulse_params *st_pulse){
  
  int a;
  int ave = 0;

	for(a=0; a < st_pulse->moving_ave_num; a++){
		ave += st_pulse->d12[a];
	}
	ave = ave / st_pulse->moving_ave_num;
    return ave;
}


// *******************************************************************
// * Name	: func_hpf
// * Input	: Lux_data calculated by getLux_function
// * Output	: HP data of data
// * Note	: HP_filter for every 16times
// *******************************************************************
int func_hpf(struct pulse_params *st_pulse, int q){
  
  signed long hpf;
  int r;
  
	r = q + 1;
	if(r >= st_pulse->hpf_num){
		r = r - st_pulse->hpf_num;
	}
	
	hpf = st_pulse->d12_ave[q] - st_pulse->d12_ave[r];

    return hpf;
}


// *******************************************************************
// * Name	: func_median5
// * Input	: Lux_data calculated by getLux_function
// * Output	: median data of Lux_data
// * Note	: median_filter for every 3times
// *******************************************************************
int func_median5(int data[])
  {
  
  int a, b;
  int temp, median, sub_data[5];
  
	for(a=0; a<5; a++){
		sub_data[a] = data[a];
	}
  
  	for(a=0; a<5-1; a++){
			for(b=5-1; b>a; b--){
				if(sub_data[b-1] > sub_data[b]){
					temp = sub_data[b];
					sub_data[b] = sub_data[b-1];
					sub_data[b-1] = temp;
				}
			}
	}
	median = sub_data[5/2];
    return median;
}
// *******************************************************************
// * Name	: filer_calibration
// * Input	: 
// * Output	: 
// * Note	: 
// *******************************************************************
void func_filering_constant_calibration(struct pulse_params *st_pulse){
	int c;
	
	if( (st_pulse->pulse_filer_range == SLOW_PULSE_RANGE) && (st_pulse->pulse_ave >=100) ){
		st_pulse->pulse_filer_range = FAST_PULSE_RANGE;
		st_pulse->moving_ave_num = 16;
 		st_pulse->hpf_num = 16;
		st_pulse->count = 0;
		st_pulse->pulse_count_start_flag = 0;
		st_pulse->min = 0;
		st_pulse->min_pre = 0;
		for(c=0; c>48; c++){
			st_pulse->d12_ave[c] = 0;
			st_pulse->d12[c] = 0;
		}
	}
	else if( (st_pulse->pulse_filer_range == FAST_PULSE_RANGE) && (st_pulse->pulse_ave <=80) ){
		st_pulse->pulse_filer_range = SLOW_PULSE_RANGE;
		st_pulse->moving_ave_num = 32;
 		st_pulse->hpf_num = 32;
		st_pulse->count = 0;
		st_pulse->pulse_count_start_flag = 0;
		st_pulse->min = 0;
		st_pulse->min_pre = 0;
		for(c=0; c>48; c++){
			st_pulse->d12_ave[c] = 0;
			st_pulse->d12[c] = 0;
		}
	}
}


// ____________________________________________________________
// public

void initPULSELib(void){
	// init
	initPULSEparams(&st_pulse);
}

//void setGP2AP050Params(struct gs_params *p_gs){
//	memcpy(&st_gs, p_gs, sizeof(struct gs_params));
//}

//struct gs_params * getCurrentGP2AP050Params(struct gs_params *dst){
//	memcpy(dst, &st_gs, sizeof(struct gs_params));
//	return dst;
//}

struct pulse_params * getCurrentPULSEParamsDirect(){
	return &st_pulse;
}

// *******************************************************************
// * Name	: getPulse
// * Input	: raw_data
// * Output	: bpm
// * Note	: 
// *******************************************************************
int getPulse(u16* raw_data){

	static int d12pre = 0;
	static int p = 0;

		if(st_pulse.q >= MOVING_AVE_NUM){
		st_pulse.q = 0;
	}
	printk(KERN_ERR "%s-%d: raw_data=%d %d %d %d\n",__func__,__LINE__, raw_data[0], raw_data[1], raw_data[2], raw_data[3]) ;
	st_pulse.d12[st_pulse.q] = raw_data[1] + raw_data[2];
	st_pulse.d12_now = st_pulse.d12[st_pulse.q];
	st_pulse.d12_ave[st_pulse.q] = func_lpf(&st_pulse);
	st_pulse.d12_ave_now = st_pulse.d12_ave[st_pulse.q];
	st_pulse.d12_lhpf = func_hpf(&st_pulse, st_pulse.q);
	
	if( (st_pulse.d12_lhpf < 100) && (st_pulse.d12_lhpf > -100) ){
		st_pulse.mult_data = st_pulse.d12_lhpf * d12pre;
	}else{
		st_pulse.mult_data = 10000;
	}
	
	if(st_pulse.d12[st_pulse.q] <= 1000){
		st_pulse.pulse_count_start_flag = 0;
		st_pulse.pulse_ave = 0;
	}
	
	if(st_pulse.pulse_count_start_flag == 1){
		st_pulse.count++;
	}

	if( (st_pulse.d12_lhpf < d12pre) && (st_pulse.min > st_pulse.d12_lhpf) ){
		st_pulse.min = st_pulse.d12_lhpf;
	}	
	
	if(st_pulse.min <= -300){
		st_pulse.min = -300;
	}
	
	if( (st_pulse.mult_data <= 0) && (d12pre > st_pulse.d12_lhpf) && (st_pulse.min_pre/10 >= st_pulse.min) && (st_pulse.d12[st_pulse.moving_ave_num-1] >= 1000) && (d12pre != 0) ){
		if(st_pulse.ao_flag == 0) {
			if(st_pulse.pulse_count_start_flag == 0){
				st_pulse.pulse_count_start_flag = 1;
			}else if(st_pulse.count >= 20){		//count:20 = 300bpm
				st_pulse.min_pre = st_pulse.min;
				st_pulse.pulse[p] = st_pulse.count;
			st_pulse.pulse_ave = 6000/func_median5(st_pulse.pulse);//bpm = 60sec / ((count * polling_ms)/1000)

				st_pulse.count = 0;
				p++;
			}else{
				st_pulse.pulse_count_start_flag = 0;
			}
		}
		st_pulse.min = 0;
		st_pulse.ao_flag = 0;
		if(p >= st_pulse.pulse_ave_num){
			p = 0;
		}
	}
	
	d12pre = st_pulse.d12_lhpf;
	func_filering_constant_calibration(&st_pulse);
	
	if(st_pulse.count >300){			//count:300 = 20bpm
		st_pulse.count = 0;
		st_pulse.min = 0;
		st_pulse.min_pre = 0;
	}
	st_pulse.q = st_pulse.q + 1; 
	return st_pulse.pulse_ave;

}


static void gp2ap_gs_data_polling( struct work_struct *work )
{
	struct gp2ap054a_psgs_als_priv  *data = container_of( work, struct gp2ap054a_psgs_als_priv, gs_polling_work ) ;
	//short int			adc_data[5] ={0,};
	//u16			adc_data[5] ={0,};
	//u8					rdata[10] ={0,};
	u16 				adc_data12;
	u16			adc_data_gs[12] ={0,};
	u16			adc_data[5] ={0,};
	u8					rdata[24] ={0,};
	int					i ,num,err;
	int 				gesture_ret, pulse_counter;
	hwm_sensor_data ps_sensor_data;
	hwm_sensor_data gs_sensor_data;
	hwm_sensor_data pulse_sensor_data;
	u8		value ;
	int j;
	int res = 0;

	printk(KERN_ERR "gp2ap_gs_data_polling mode=%d\n" ,data->mode) ;
	if( data != NULL )
	{
		mutex_lock( &data->mutex ) ;

		for(j=0;j<3;j++)//Read 3 set
			hwmsen_read_block(data->client,REG_D0_LSB_PRE2+8*j,&rdata[8*j],8);
			//hwmsen_read_block(data->client,REG_D0_LSB,&rdata,8);

		mutex_unlock( &data->mutex ) ;

		for( i = 0 ; i < 12 ; i++ )// 3 set * 4 data = 12
		{
			adc_data_gs[i] = ( rdata[i*2+1] << 8 ) | rdata[i*2]  ;
			//adc_data[i] = ( ( rdata[i*2+1] << 8 ) | rdata[i*2] ) ;
		}

		if( data->mode == SENSOR_MODE_PULSE_COUNTER )
			adc_data12 = adc_data_gs[9] + adc_data_gs[10];
	printk(KERN_ERR "gp2ap_gs_data_polling mode=%d, adc_data12=%d, analog_offset_reg=%d\n" ,data->mode, adc_data12, data->analog_offset_reg) ;
			
		if( ( data->mode == SENSOR_MODE_PULSE_COUNTER ) && ( adc_data12 > 5000 ) && ( data->analog_offset_reg < 0x3F ) ){
			data->analog_offset_reg = data->analog_offset_reg + 0x09;
			value = data->analog_offset_reg;
			gp2ap_i2c_write( REG_PANEL, &value, data->client ) ;
	printk(KERN_ERR "gp2ap_gs_data_polling mode=%d getPulse 1\n" ,data->mode) ;
		}else if( ( data->mode == SENSOR_MODE_PULSE_COUNTER ) && ( adc_data12 < 2000 ) && ( data->analog_offset_reg > 0x00 ) ){
			data->analog_offset_reg = data->analog_offset_reg - 0x09;
			value = data->analog_offset_reg;
			gp2ap_i2c_write( REG_PANEL, &value, data->client ) ;
	printk(KERN_ERR "gp2ap_gs_data_polling mode=%d getPulse 2\n" ,data->mode) ;
		}else{
			if( data->mode == SENSOR_MODE_PULSE_COUNTER )
			{
				for(j=0;j<4;j++)
					adc_data[j] = adc_data_gs[j+8];
				pulse_counter = getPulse(adc_data);
				printk(KERN_ERR "gp2ap_gs_data_polling getPulse 3 adc_data12=%d, pulse_counter=%d\n", adc_data12, pulse_counter) ;
				pulse_sensor_data.values[0] = pulse_counter;
				pulse_sensor_data.values[1] = 0;
				pulse_sensor_data.values[2] = 0;
				pulse_sensor_data.value_divide = 1;
				pulse_sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
				  if((res = hwmsen_get_interrupt_data(ID_PROXIMITY, &pulse_sensor_data)))
				  {
					APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", res);
				  }
			}
			else//GS
			{
				for(i=0; i<3; i++)
				{
					for(j=0;j<4;j++)
						adc_data[j] = adc_data_gs[j+4*i];

				gesture_ret = getGP2AP050Gesture(adc_data, data);
				if(data->last_gesture!=gesture_ret){
					data->last_gesture=gesture_ret;
					gs_sensor_data.values[0] = gesture_ret;
					gs_sensor_data.values[1] = 0;
					gs_sensor_data.values[2] = 0;
					gs_sensor_data.value_divide = 1;
					gs_sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
				  if((res = hwmsen_get_interrupt_data(ID_PROXIMITY, &gs_sensor_data)))
				  {
					APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", res);
				  }
				}
				}
			}
		}
	}
	if(data->hw->polling_mode_gs == 0)
			mt_eint_unmask(CUST_EINT_ALS_NUM);
}

static enum hrtimer_restart gp2ap_gs_polling_timer_func( struct hrtimer *timer )
{
	struct gp2ap054a_psgs_als_priv  *data = container_of( timer, struct gp2ap054a_psgs_als_priv, gs_polling_timer ) ;

	printk(KERN_ERR "gp2ap_gs_polling_timer_func \n" ) ;
	if( data != NULL )
	{
		//schedule_work( &data->gs_polling_work ) ;
		queue_work(data->gs_polling_workqueue,&data->gs_polling_work);
		hrtimer_forward_now( &data->gs_polling_timer, data->gs_polling_delay ) ;
	}
	return HRTIMER_RESTART ;
//	return HRTIMER_NORESTART ;
}

static void gp2ap_ps_work_avg_func( struct work_struct *work )
{
	struct gp2ap054a_psgs_als_priv  *data = container_of( work, struct gp2ap054a_psgs_als_priv, ps_avg_work ) ;
	int					D2_data ;
	u8					rdata[2] = { 0, } ;

	printk(KERN_ERR "gp2ap_ps_work_avg_func\n" ) ;

	if( data != NULL )
	{
		mutex_lock( &data->mutex ) ;
		//gp2ap_i2c_read( REG_ADR_18, rdata, sizeof( rdata ), data->client ) ;
		hwmsen_read_block(data->client,REG_ADR_18,rdata,2);
		D2_data = ( rdata[1] << 8 ) | rdata[0] ;
		data->ps_avg_tmp[2] += D2_data ;
		if( D2_data < data->ps_avg_tmp[0] )
		{
			data->ps_avg_tmp[0] = D2_data ;
		}
		if( D2_data > data->ps_avg_tmp[1] )
		{
			data->ps_avg_tmp[1] = D2_data ;
		}

		data->ps_avg_read_cnt++ ;
		if( data->ps_avg_read_cnt < PS_AVG_READ_COUNT )
		{
			hrtimer_start( &data->ps_avg_read_timer, data->ps_avg_read_delay, HRTIMER_MODE_REL ) ;
		}
		else
		{
			data->ps_avg[0] = data->ps_avg_tmp[0] ;
			data->ps_avg[1] = data->ps_avg_tmp[1] ;
			data->ps_avg[2] = data->ps_avg_tmp[2] / PS_AVG_READ_COUNT ;
			printk(KERN_ERR "avg end\n" ) ;
		}
		mutex_unlock( &data->mutex ) ;
	}
}

static enum hrtimer_restart gp2ap_ps_avg_read_timer_func( struct hrtimer *timer )
{
	struct gp2ap054a_psgs_als_priv  *data = container_of( timer, struct gp2ap054a_psgs_als_priv, ps_avg_read_timer ) ;

	printk(KERN_ERR "gp2ap_ps_avg_read_timer_func\n" ) ;

	if( data != NULL )
	{
		schedule_work( &data->ps_avg_work ) ;
	}

	return HRTIMER_NORESTART ;
}

static void gp2ap_ps_avg_timer_func( struct work_struct *work )
{
	struct gp2ap054a_psgs_als_priv  *data = container_of( ( struct delayed_work * )work,
												struct gp2ap054a_psgs_als_priv, ps_avg_timer_work ) ;

	printk(KERN_ERR "ps_avg_timer_func\n" ) ;

	if( data != NULL )
	{
		data->ps_avg_read_cnt = 0 ;
		data->ps_avg_tmp[0] = 0x7fffffff ;
		data->ps_avg_tmp[1] = 0 ;
		data->ps_avg_tmp[2] = 0 ;
		hrtimer_start( &data->ps_avg_read_timer, data->ps_avg_read_delay, HRTIMER_MODE_REL ) ;

		schedule_delayed_work( &data->ps_avg_timer_work, msecs_to_jiffies( PS_AVG_POLL_DELAY ) ) ;
	}
}
static void gp2ap_ps_work_int_func( struct work_struct *work )
{
	struct gp2ap054a_psgs_als_priv  *data = container_of( work, struct gp2ap054a_psgs_als_priv, ps_int_work ) ;
	unsigned char		value ;
	char				distance ;
	u8					rdata ;
	int 				err;
	hwm_sensor_data ps_sensor_data;

	if( data == NULL )
	{
		return ;
	}

	mutex_lock( &data->mutex ) ;

	hwmsen_read_byte(data->client,REG_ADR_02,&rdata);
	if( (rdata & 0x08) == 0x08)
		distance = 0;	//near
	else 
		distance = 1;	//far

	if( data->ps_distance != distance )
	{
		data->ps_distance = distance ;

		pr_debug( "proximity = %d\n", distance ) ;
	}


#if 0
	// 0 : proximity, 1 : away
	//distance = gpio_get_value_cansleep( GPIO_ALS_EINT_PIN ) ;
	distance = mt_get_gpio_in( GPIO_ALS_EINT_PIN ) ;

	if( data->ps_distance != distance )
	{
		data->ps_distance = distance ;
		
//		disable_irq( data->ps_irq ) ;
		mt_eint_mask(CUST_EINT_ALS_NUM);

		value = ( OP_SHUTDOWN | INT_NOCLEAR ) ;
		gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;

		//gp2ap_i2c_read( REG_ADR_02, &rdata, sizeof( rdata ), data->client ) ;
		hwmsen_read_byte(data->client,REG_ADR_02,&rdata);
		if( distance == 0 )
		{ // detection = Falling Edge
			value = PRST_1 | ( rdata & 0x1F ) ;
			gp2ap_i2c_write( REG_ADR_02, &value, data->client ) ;
		}
		else
		{ // none Detection
			value = PRST_4 | ( rdata & 0x1F ) ;
			gp2ap_i2c_write( REG_ADR_02, &value, data->client ) ;
		}

//		enable_irq( data->ps_irq ) ;
		mt_eint_unmask(CUST_EINT_ALS_NUM);

		value = ( OP_RUN | INT_NOCLEAR ) ;
		gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;

		printk(KERN_ERR "proximity = %d\n", distance ) ;
	}
#endif

	mutex_unlock( &data->mutex ) ;
	ps_sensor_data.values[0] = distance;
	ps_sensor_data.values[1] = 0;
	ps_sensor_data.values[2] = 0;
	ps_sensor_data.value_divide = 1;
	ps_sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	if(err = hwmsen_get_interrupt_data(ID_PROXIMITY,&ps_sensor_data) ){
		printk(KERN_ERR"%s line=%d report ID_PROXIMITY data=%d FAIL!",__func__,__LINE__,distance);
	}
	else{
		printk(KERN_ERR"%s line=%d report ID_PROXIMITY data=%d SUCCESS!",__func__,__LINE__,distance);
	}
	//input_report_abs( data->proximity_input_dev, ABS_X, distance ) ;
	//input_sync( data->proximity_input_dev ) ;
}

static void gp2ap054a_psgs_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO) 
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "gp2ap054a")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "gp2ap054a")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}

/********************************************************************/
int gp2ap054a_als_enable_als(struct i2c_client *client, int enable)
{
	struct gp2ap054a_psgs_als_priv *obj = i2c_get_clientdata(client);
	u8		value ;
	u8		rdata ;

	if(enable == 1)
	{
		if(0 == test_bit(CMC_BIT_PS,  &obj->enable) && 0 == test_bit(CMC_BIT_GS,  &obj->enable)
			&& 0 == test_bit(CMC_BIT_PULSE,  &obj->enable))
		{
			value = RST ;	// RST
			gp2ap_i2c_write( REG_ADR_02, &value, client ) ;
			
			gp2ap_init_device(0, obj );
			//value = ( INTVAL_0 | IS_110 | PIN_DETECT | FREQ_327_5 );
			//gp2ap_i2c_write( REG_ADR_03, &value, client ) ;
			//value = ( OP_RUN | OP_CONTINUOUS | OP_ALS ) ;		// ALS mode
			value = ( OP_RUN | OP_ALS ) ;		// ALS mode
			gp2ap_i2c_write( REG_ADR_00, &value, client ) ;
		}
		else
		{
			value = OP_SHUTDOWN ; // shutdown
			gp2ap_i2c_write( REG_ADR_00, &value, client ) ;
			//value = ( u8 )( PRST_4 | RES_A_14 | RANGE_A_8 );
			//value = ( u8 )( RES_A_14 | RANGE_A_8 ) | ( rdata & 0xC0 ) ;
			//gp2ap_i2c_write( REG_ADR_01, &value, client ) ;
			//value = ( INTVAL_0 | IS_110 | PIN_DETECT | FREQ_327_5 );
			//gp2ap_i2c_write( REG_ADR_03, &value, client ) ;
			//value = ( OP_RUN | OP_CONTINUOUS | OP_PS_ALS ) ;	// ALS & PS mode
			value = ( OP_RUN | OP_PS_ALS ) ;	// ALS & PS mode
			gp2ap_i2c_write( REG_ADR_00, &value, client ) ;
		}
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		set_bit(CMC_BIT_ALS,  &obj->pending_intr);
	}
	else
	{
		//init_by_pass_filter=0;
		//init_ch01=0;
		//g_count=0;
		if(0 == test_bit(CMC_BIT_PS,  &obj->enable) && 0 == test_bit(CMC_BIT_GS,  &obj->enable)
			&& 0 == test_bit(CMC_BIT_PULSE,  &obj->enable))
		{
			value = OP_SHUTDOWN ; // shutdown
			gp2ap_i2c_write( REG_ADR_00, &value, client ) ;
		}
		else
		{
			value = OP_SHUTDOWN ; // shutdown
			gp2ap_i2c_write( REG_ADR_00, &value, client ) ;
			//value = ( INTVAL_16 | IS_110 | PIN_DETECT | FREQ_327_5 );
			//gp2ap_i2c_write( REG_ADR_03, &value, client ) ;
			//value = ( OP_RUN | OP_CONTINUOUS | OP_PS ) ;		// PS mode
			value = ( OP_RUN | OP_PS ) ;		// PS mode
			gp2ap_i2c_write( REG_ADR_00, &value, client ) ;
		}
		atomic_set(&obj->als_deb_on, 0);
	}
	return 0;
}

/********************************************************************/
long gp2ap054a_psgs_read_ps(struct i2c_client *client, u16 *data)
{
	long res =-1;
	u8	  rdata[2]= {0} ;

	msleep( 20 ) ;
	res = hwmsen_read_block(client,REG_D4_LSB,rdata,0x02);
	if(res!= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_PS_EXIT_ERR;
	}
	printk("--%s,raw data:0x%x,0x%x\n",__func__,rdata[0],rdata[1]);
	*data = ( rdata[1] << 8 ) | rdata[0] ;
	printk("--%s,data:0x%x\n",__func__,*data);
	return 0;
	READ_PS_EXIT_ERR:
	return res;

}

/* on/off function2 */
static int als_onoff_simplified( u8 onoff, struct gp2ap054a_psgs_als_priv *data )
{
	u8		value ;

	pr_debug( "light_sensor onoff = %d\n", onoff ) ;

	if( onoff )
	{
		//if( !data->gs_enabled )
		if(0 == test_bit(CMC_BIT_PS,  &data->enable) && 0 == test_bit(CMC_BIT_GS,  &data->enable)) 
		{
			value = ( COMMAND1_WAKEUP | COMMAND1_ALS ) ;			// ALS mode
			gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;
		}
		else 
		{
			value = ( COMMAND1_SD	 ) ; 							// Shutdown
			gp2ap_i2c_write( REG_COMMND1, &value, data->client ) ;
			value = ( COMMAND1_WAKEUP | COMMAND1_ALS_GS ) ;			// GS & ALS mode
			gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;
		}

	}
	else
	{
		if(0 == test_bit(CMC_BIT_PS,  &data->enable) && 0 == test_bit(CMC_BIT_GS,  &data->enable))//if( !data->gs_enabled )
		{
			value = ( COMMAND1_SD ) ; 								// Shutdown
			gp2ap_i2c_write( REG_COMMND1, &value, data->client ) ;
		}
		else
		{
			value = ( COMMAND1_SD	 ) ; 							// Shutdown
			gp2ap_i2c_write( REG_COMMND1, &value, data->client ) ;
			value = ( COMMAND1_WAKEUP | COMMAND1_GS ) ; 			// GS mode
			gp2ap_i2c_write( REG_COMMND1, &value, data->client ) ;
		}
	}
	return 0 ;
}

/* *****************************************************************************
		ALS : Calculation formulas for calculating the illuminace value
***************************************************************************** */
//  C: Coefficient for correcting the variation in panel transmittance.
//	lux = c * (a * Clear - b * IR)/100
static u32
als_get_lux( struct gp2ap054a_psgs_als_priv *data )
{
//	struct gp2ap054a_als_priv *obj = i2c_get_clientdata(client);	 
	u32		data_als0 ;
	u32		data_als1 ;
	u32 	lux = 0 ;
	u32 	ratio ;
	u8		value ;
	u8		rdata[4] ;
	int 	gamma=1 ;
  	u8 		a, b ;	
  	u32		temp ;
  	u32		sub_data[3] ;
	int		adjust_offset;
		long res;
	static u32	lux_median[3];
	static int count=0;

	/* get data_als0(clear), data_als1(ir) */
	//gp2ap_i2c_read( REG_D5_LSB, rdata, sizeof( rdata ), data->client ) ;
	res = hwmsen_read_block(data->client,REG_D5_LSB,rdata,0x04);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		//goto READ_ALS_EXIT_ERR;
	}
	//printk("--%s,raw data:0x%x,0x%x\n",__func__,rdata[0],rdata[1]);

	data_als0 = ( rdata[1] << 8 ) | rdata[0] ;
	data_als1 = ( rdata[3] << 8 ) | rdata[2] ;
	
	
	//data_als0 = rdata[0]+rdata[1]*256;
	//data_als1 = rdata[2]+rdata[3]*256;
	
	
	/* calculate the ratio */	
	if( data_als0 == 0 )
	{
		ratio = 100 ;
	}
	else
	{
		ratio = ( data_als1 * 100 ) / data_als0 ;
	}

	/* lux calculation */
	
		/* calculate lux without adjusting parameters */
	if(ratio<=RATIO_FIRST_BOUND)
	{
		lux = (alfa1 * data_als0 - beta1 * data_als1) ;
		adjust_offset = CALC_OFFSET1 ;
	}
	else if (ratio<=RATIO_SECOND_BOUND)
	{
		lux = (alfa2 * data_als0 - beta2 * data_als1) ;
		adjust_offset = CALC_OFFSET2 ;
	}
	else if (ratio<=RATIO_THIRD_BOUND)
	{
		lux = (alfa3 * data_als0 - beta3 * data_als1) ;
		adjust_offset = CALC_OFFSET3 ;
	}

	/* coefficient for adjusting the difference in RANGE */
	if( data->als_mode == HIGH_LUX_MODE )
		gamma = GAMMA_HIGH_LUX_MODE;		// HIGH_LUX_RANGE   :ALS1_RANGE X512
	else if( data->als_mode == MIDDLE_LUX_MODE )
		gamma = GAMMA_MIDDLE_LUX_MODE;		// MIDDLE_LUX_RANGE :ALS1_RANGE X16
	else 
		gamma = GAMMA_LOW_LUX_MODE;			// LOW_LUX_RANGE    :ALS1_RANGE X1

	/* Exceptional process and precaution for calculation overflow */
	if ( ( data_als0 < ZERO_LUX_TH ) &&
				 ( data->als_mode == LOW_LUX_MODE ) )
	{/* ZERO output condition */
		lux = 0 ;
	}
	else if ( ( data_als0 > OVER_FLOW_COUNT ) &&
				 ( data->als_mode == HIGH_LUX_MODE ) )
	{/* raw data overflow */
		lux = MAX_LUX_VALUE ;
	}
	else if ( ratio>RATIO_THIRD_BOUND )
	{/* Illegal data */
		lux = data->als_lux_prev;
	}
	else
	{/* Gamma calculation with precaution for overflow */
		if ( lux >= 1000000 )
		{
			adjust_offset /= 1000 ;
			lux = ( lux /1000 ) * gamma / adjust_offset ;
		}
		else
		{
			lux = gamma * lux / adjust_offset ;
		}
	}
	
	/*  Lux mode (Range) change */
	/*  LOW ---> MIDDLE */
	if( ( data_als0 >= ALS_L_to_M_counts ) && ( data->als_mode == LOW_LUX_MODE ) )
	{
		data->als_mode = MIDDLE_LUX_MODE ;
		
		printk( KERN_INFO "change lux mode. MIDDLE_LUX_MODE!! \n" ) ;
		
		/* ALS Shutdown */
		als_onoff_simplified( 0, data ) ;
		
		/* change Range of ALS */
		data->regData[REG_ALS1] = ( data->regData[REG_ALS1] & 0x78 ) | MIDDLE_LUX_RANGE;
		value = data->regData[REG_ALS1];
		gp2ap_i2c_write( REG_ALS1, &value, data->client ) ;

		/* Start */
		als_onoff_simplified( 1, data ) ;

		lux = data->als_lux_prev ; 
	}
	/*  MIDDLE ---> HIGH */
	else if( ( data_als0 > ALS_M_to_H_counts ) && ( data->als_mode == MIDDLE_LUX_MODE ) )
	{
		data->als_mode = HIGH_LUX_MODE ;
		
		printk( KERN_INFO "change lux mode. HIGH_LUX_MODE!! \n" ) ;
		
		/* ALS Shutdown */
		als_onoff_simplified( 0, data ) ;
		
		/* change Range of ALS */
		data->regData[REG_ALS1] = ( data->regData[REG_ALS1] & 0x78 ) | HIGH_LUX_RANGE;
		value = data->regData[REG_ALS1];
		gp2ap_i2c_write( REG_ALS1, &value, data->client ) ;

		/* Start */
		als_onoff_simplified( 1, data ) ;
		
		lux = data->als_lux_prev ; 
	}
	/*  HIGH ---> MIDDLE */
	else if( ( data_als0 < ALS_H_to_M_counts ) && ( data->als_mode == HIGH_LUX_MODE ) )
	{
		data->als_mode = MIDDLE_LUX_MODE ;
		
		printk( KERN_INFO "change lux mode. MIDDLE_LUX_MODE!! \n" ) ;
		
		/* ALS Shutdown */
		als_onoff_simplified( 0, data ) ;
		
		/* change Range of ALS */
		data->regData[REG_ALS1] = ( data->regData[REG_ALS1] & 0x78 ) | MIDDLE_LUX_RANGE;
		value = data->regData[REG_ALS1];
		gp2ap_i2c_write( REG_ALS1, &value, data->client ) ;

		/* Start */
		als_onoff_simplified( 1, data ) ;
		
		lux = data->als_lux_prev ; 
	}
	/*  MIDDLE ---> LOW */
	else if( ( data_als0 < ALS_M_to_L_counts ) && ( data->als_mode == MIDDLE_LUX_MODE ) )    
	{
		data->als_mode = LOW_LUX_MODE ;
		
		printk( KERN_INFO "change lux mode. LOW_LUX_MODE!! \n" ) ;
		
		/* ALS Shutdown */
		als_onoff_simplified( 0, data ) ;
		
		/* change Range of ALS */
		data->regData[REG_ALS1] = ( data->regData[REG_ALS1] & 0x78 ) | LOW_LUX_RANGE;
		value = data->regData[REG_ALS1];
		gp2ap_i2c_write( REG_ALS1, &value, data->client ) ;

		/* Start */
		als_onoff_simplified( 1, data ) ;
		
		lux = data->als_lux_prev ; 
	}
	//printk( KERN_INFO "LUX Median. lux = %d\n", lux ) ;	

	/*  the median of 3 times of measured value */
	if(count >= 3)
	{
		count = 0;
	}
	lux_median[count] = lux;
		
 	for(a=0; a<3; a++){
		sub_data[a] = lux_median[a];
	}
 	//printk( KERN_INFO "LUX Median. lux = %d, sub_data0,1,2 = %d, %d, %d \n", lux, sub_data[0], sub_data[1], sub_data[2] ) ;
  	for(a=0; a<3-1; a++){
		for(b=3-1; b>a; b--){
			if(sub_data[b-1] > sub_data[b]){
				temp = sub_data[b];
				sub_data[b] = sub_data[b-1];
				sub_data[b-1] = temp;
			}
		}
	}
	lux = sub_data[1];
	count++;
	//printk( KERN_INFO "LUX Median. lux = %d, sub_data0,1,2 = %d, %d, %d \n", lux, sub_data[0], sub_data[1], sub_data[2] ) ;

	
	data->als_lux_prev = lux ; 

	pr_debug( "calc lux=%d\n", lux ) ;

	return lux ;

}
//================als_get_lux END

long gp2ap054a_psgs_read_als(struct i2c_client *client, u16 *data)
{
	struct gp2ap054a_psgs_als_priv *obj = i2c_get_clientdata(client);
	u32 	lux ;	
	
	lux = als_get_lux( obj );
	*data = lux;
	printk("--%s,lux=%d\n",__func__,*data);
}

/********************************************************************/
/********************************************************************/
static int gp2ap054a_psgs_get_ps_value(struct gp2ap054a_psgs_als_priv *obj, u16 ps)
{
	int val = 0;
	//int mask = atomic_read(&obj->ps_mask);
	//int invalid = 0;
	u8 rdata ;

	printk("--%s,enter\n",__func__);

	//mutex_lock( &obj->mutex ) ;

	hwmsen_read_byte(obj->client,REG_ADR_01,&rdata);
	if( (rdata & 0x08) == 0x08)
		val = 0;	//near
	else 
		val = 1;	//far

	if( obj->ps_distance != val )
	{
		obj->ps_distance = val ;

		pr_debug( "proximity = %d\n", val ) ;
	}

	//mutex_unlock( &obj->mutex ) ;


#if 0
	if(ps > atomic_read(&obj->ps_in_threshold))
	{
		val = 0;  /*close*/
	}
	else if (ps < atomic_read(&obj->ps_out_threshold))
	{
		val = 1;  /*far away*/
	}
	else {
		val = obj->ps_distance;
	}
	printk("--%s,val=%d\n",__func__,val);
	obj->ps_distance =val;
#endif
	/*
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	*/
	/*
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			
			if(mask)
			{
				APS_ERR("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_ERR("PS:  %05d => %05d\n", ps, val);
			}
		}
		if(0 == test_bit(CMC_BIT_PS,  &obj->enable))
		{
		  //if ps is disable do not report value
		  APS_ERR("PS: not enable and do not report this value\n");
		  return -1;
		}
		else
		{
		   return val;
		}
		
	}	
	else
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}
	*/	
	return val;
}
/********************************************************************/


/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t gp2ap054a_psgs_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "( BEEN COMMENT NOW)\n"); 
		/*atomic_read(&gp2ap054a_psgs_als_obj->i2c_retry), atomic_read(&gp2ap054a_psgs_als_obj->als_debounce),*/ 
		//atomic_read(&gp2ap054a_psgs_als_obj->ps_mask), atomic_read(&gp2ap054a_psgs_als_obj->ps_thd_val), atomic_read(&gp2ap054a_psgs_als_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	//int retry, als_deb, ps_deb, mask, thres;
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	/*
	//if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	if(4 == sscanf(buf, "%d %d %d", , &mask, &thres, &ps_deb))
	{ 
		//atomic_set(&gp2ap054a_psgs_als_obj->i2c_retry, retry);
		atomic_set(&gp2ap054a_psgs_als_obj->ps_mask, mask);
		atomic_set(&gp2ap054a_psgs_als_obj->ps_thd_val, thres);        
		atomic_set(&gp2ap054a_psgs_als_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	*/
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&gp2ap054a_psgs_als_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&gp2ap054a_psgs_als_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	if((res = gp2ap054a_psgs_read_als(gp2ap054a_psgs_als_obj->client, &gp2ap054a_psgs_als_obj->als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", gp2ap054a_psgs_als_obj->als);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	
	if((res = gp2ap054a_psgs_read_ps(gp2ap054a_psgs_als_obj->client, &gp2ap054a_psgs_als_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", gp2ap054a_psgs_als_obj->ps);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_show_reg(struct device_driver *ddri, char *buf)
{
	int i,err;
	u8 val;
	int len=0;
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	for(i = 0; i <=0x39; i++)//0xf
	{
		err=hwmsen_read_byte(gp2ap054a_psgs_als_obj->client,i,&val);
		printk(KERN_INFO"read err=%d reg0x%02x=%02x\n",err,i,val);
		if(err>=0){
			len += snprintf(buf+len, PAGE_SIZE-len, "Reg:0x%02x=0x%02x\n",i,val);
		}
		else{
			len += snprintf(buf+len, PAGE_SIZE-len, "Reg:0x%02x=err(%d)\n",i,err);
		}
	}
	return len;    
}
static ssize_t show_write_register(struct device_driver *ddri, char *buf)
{

	if(NULL == gp2ap054a_psgs_als_obj || NULL == gp2ap054a_psgs_als_obj->client)
	{
		APS_ERR("gp2ap054a_psgs_als_obj or i2c client data is null!!\n");
		return 0;
	}
	return sprintf(buf, "REG:%s\n", gp2ap054a_psgs_als_obj->write_register);
}
static ssize_t store_write_register(struct device_driver *ddri, char *buf, size_t count)
{

	int res;
	char* const delim = " ";
	char str_command[20];
	char *token=0;
	char *cur;
	char str_reg_addr[10];
	char str_reg_val[10];
	int  reg_addr,reg_value;
	if(gp2ap054a_psgs_als_obj==NULL){
		APS_ERR("gp2ap054a_als_obj is null!!\n");
		return 0;
	}
	sprintf(str_command,"%s",buf);
	cur=str_command;
	if(token=strsep(&cur,delim))
		sprintf(str_reg_addr,"%s",token);
	else
		goto error_fs;
	if(token=strsep(&cur,delim))
		sprintf(str_reg_val,"%s",token);
	else
		goto error_fs;
	
	reg_addr=simple_strtoul(str_reg_addr, NULL, 16)&0xff;
	reg_value=simple_strtoul(str_reg_val, NULL, 16)&0xff;
	
	if( gp2ap054a_psgs_als_obj->client != NULL ){
		res=gp2ap_i2c_write(reg_addr,&reg_value,gp2ap054a_psgs_als_obj->client);
		if(res < 0)
		{
			APS_DBG("%s--%d set register fail \n",__func__,__LINE__);
			sprintf(gp2ap054a_psgs_als_obj->write_register,"0x%02x:FAIL",reg_addr);
		}
		else
		{
			sprintf(gp2ap054a_psgs_als_obj->write_register,"0x%02x=0x%02x",reg_addr,reg_value);
		}
	}
	else {
		sprintf(gp2ap054a_psgs_als_obj->write_register,"null i2c client");
		APS_LOG("write register fail as null client\n");
	}
	return count;
error_fs:
	sprintf(gp2ap054a_psgs_als_obj->write_register,"WRONG ARGS");
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	//u8 dat;
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	
	if(gp2ap054a_psgs_als_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			gp2ap054a_psgs_als_obj->hw->i2c_num, gp2ap054a_psgs_als_obj->hw->power_id, gp2ap054a_psgs_als_obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "REGS:%02lX %02lX\n",
			gp2ap054a_psgs_als_obj->enable, gp2ap054a_psgs_als_obj->pending_intr);
	
	//len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&gp2ap054a_psgs_als_obj->als_suspend), atomic_read(&gp2ap054a_psgs_als_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t gp2ap054a_psgs_store_mode(struct device_driver *ddri, const char *buf, size_t count){
	int val,mode;
	if(!gp2ap054a_psgs_als_obj){
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	if( 1 != sscanf(buf, "%d", &mode) )
	{
		printk(KERN_INFO "can't parse mode info, return\n");
		return count;
	}
	if( mode >= SENSOR_MODE_OFF && mode <= SENSOR_MODE_PULSE_COUNTER ){
		change_work_mode(mode,gp2ap054a_psgs_als_obj);
		return count;
	}
	else{
		printk(KERN_ERR"%s wrong mode val=%d\n",__func__ , mode);
		return count;
	}
}

static ssize_t gp2ap054a_psgs_show_mode(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}

	
	len = snprintf(buf, PAGE_SIZE, "mode=%d\n",gp2ap054a_psgs_als_obj->mode);
	
	//len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&gp2ap054a_psgs_als_obj->als_suspend), atomic_read(&gp2ap054a_psgs_als_obj->ps_suspend));

	return len;
}


static ssize_t gp2ap054a_psgs_store_debug(struct device_driver *ddri, const char *buf, size_t count){
	int val;
	if(!gp2ap054a_psgs_als_obj){
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}
	if( 1 != sscanf(buf, "%d", &val) )
	{
		printk(KERN_INFO "can't parse mode info, return\n");
		return count;
	}
	if(val == 1 || val ==0 ){
		printk("PSGS: change debug from %d to %d\n",debug,val);
		debug=val;
	}
	else{
		printk(KERN_ERR"PSGS: wrong debug mode\n");
	}
	return count;
}

static ssize_t gp2ap054a_psgs_show_debug(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!gp2ap054a_psgs_als_obj)
	{
		APS_ERR("gp2ap054a_psgs_als_obj is null!!\n");
		return 0;
	}

	
	len = snprintf(buf, PAGE_SIZE, "debug=%d\n",debug);
	
	//len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&gp2ap054a_psgs_als_obj->als_suspend), atomic_read(&gp2ap054a_psgs_als_obj->ps_suspend));

	return len;
}



/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct gp2ap054a_psgs_als_priv *obj, const char* buf, size_t count, u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, gp2ap054a_psgs_show_als, NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, gp2ap054a_psgs_show_ps, NULL);
//static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, gp2ap054a_psgs_show_config,	gp2ap054a_psgs_store_config);
//static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, NULL, NULL);
//static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, NULL, NULL);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, gp2ap054a_psgs_show_trace,		gp2ap054a_psgs_store_trace);
//static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, NULL, NULL);
//static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, NULL, NULL);
//static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, NULL, NULL);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, gp2ap054a_psgs_show_reg, NULL);
static DRIVER_ATTR(mode,     S_IWUSR | S_IRUGO, gp2ap054a_psgs_show_mode, gp2ap054a_psgs_store_mode);
static DRIVER_ATTR(debug,     S_IWUSR | S_IRUGO, gp2ap054a_psgs_show_debug, gp2ap054a_psgs_store_debug);
static DRIVER_ATTR(write_register,   S_IWUSR | S_IRUGO, show_write_register,store_write_register);
//static DRIVER_ATTR(ps_base_value,   S_IWUSR | S_IRUGO, show_ps_base_value, store_ps_base_value);
//static DRIVER_ATTR(ps_para_index,   S_IWUSR | S_IRUGO, show_ps_para_index,store_ps_para_index);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *gp2ap054a_psgs_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    //&driver_attr_config,
    //&driver_attr_alslv,
   // &driver_attr_alsval,
   // &driver_attr_status,
   // &driver_attr_send,
    //&driver_attr_recv,
    &driver_attr_reg,
    &driver_attr_mode,
    &driver_attr_debug,
    &driver_attr_write_register,
    //&driver_attr_ps_base_value,
    //&driver_attr_ps_para_index,
};

/*----------------------------------------------------------------------------*/
static int gp2ap054a_psgs_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(gp2ap054a_psgs_attr_list)/sizeof(gp2ap054a_psgs_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, gp2ap054a_psgs_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", gp2ap054a_psgs_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int gp2ap054a_psgs_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(gp2ap054a_psgs_attr_list)/sizeof(gp2ap054a_psgs_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, gp2ap054a_psgs_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/


/*----------------------------------interrupt functions--------------------------------*/
static int intr_flag = 0;
/*----------------------------------------------------------------------------*/
static int gp2ap054a_psgs_check_intr(struct i2c_client *client) 
{
	struct gp2ap054a_psgs_als_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];
	u8			rdata[18] ;

printk(KERN_INFO"ddd %s start\n",__func__);    
	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
		return 0;
printk(KERN_INFO"ddd %s start -1\n",__func__);    
#if 0
	res = hwmsen_read_byte_sr(client,REG_ADR_00,buffer);
	if(res < 0)
	{
		goto EXIT_ERR;
	}

	APS_ERR("gp2ap054a_psgs_check_intr status=0x%x\n", buffer[0]);
#endif
//just support interrupt for ps

	if((1 == test_bit(CMC_BIT_PS,  &obj->enable))
		&&(gp2ap054a_psgs_als_obj->hw->polling_mode_ps == 0))
	{
		set_bit(CMC_BIT_PS, &obj->pending_intr);
	}
	else
	{
		clear_bit(CMC_BIT_PS, &obj->pending_intr);
	}
	clear_bit(CMC_BIT_ALS, &obj->pending_intr);
	#if 0
	if(buffer[0]&0x02)
	{
		set_bit(CMC_BIT_ALS, &obj->pending_intr);
	}
	else
	{
		clear_bit(CMC_BIT_ALS, &obj->pending_intr);
	}
	#endif
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG)
	{
		APS_LOG("check intr: 0x%08lX\n", obj->pending_intr);
	}
printk(KERN_INFO"ddd %s start -end\n",__func__);    
	return res;

EXIT_ERR:
	APS_ERR("gp2ap054a_psgs_check_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/
static void gp2ap054a_psgs_eint_work(struct work_struct *work)
{
	struct gp2ap054a_psgs_als_priv *obj = (struct gp2ap054a_psgs_als_priv *)container_of(work, struct gp2ap054a_psgs_als_priv, eint_work);
	
	hwm_sensor_data sensor_data;
	int res = 0;
	unsigned char databuf[2]= {0};
	msleep( 20 ) ;
printk(KERN_INFO"ddd %s 1- enable=%d\n",__func__, test_bit(CMC_BIT_PS, &obj->enable));
       if(!test_bit(CMC_BIT_PS, &obj->enable))
        return;
printk(KERN_INFO"ddd %s 2- enable=%d\n",__func__, test_bit(CMC_BIT_PS, &obj->enable));    
	res = gp2ap054a_psgs_check_intr(obj->client);
printk(KERN_INFO"ddd %s 3- res=%d\n",__func__, res);    
	if(res != 0)
	{
		APS_ERR("check intrs: %d\n", res);
		goto EXIT_INTR_ERR;
	}
	/*
	if((1<<CMC_BIT_ALS) & obj->pending_intr)
	{
	  //get raw data
	  APS_LOG(" als change\n");
	  if((res = gp2ap054a_psgs_read_p(obj->client, &obj->als)))
	  {
		 APS_ERR("cm3623 read als data: %d\n", res);
	  }
	  //map and store data to hwm_sensor_data
	 
 	  sensor_data.values[0] = gp2ap054a_psgs_get_als_value(obj, obj->als);
	  sensor_data.value_divide = 1;
	  sensor_data.values[0] = 
	  sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	  //APS_LOG("als raw %x -> value %d \n", obj->als,sensor_data.values[0]);
	  //let up layer to know
	  if((res = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data)))
	  {
		APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", res);
	  }
	  
	}
	*/
	//if((1<<CMC_BIT_PS) &  obj->pending_intr)
	
	{
	  //get raw data
	  printk(" ps change\n");
	  if((res = gp2ap054a_psgs_read_ps(obj->client, &obj->ps)))
	  {
		 printk("cm3623 read ps data: %d\n", res);
		 goto EXIT_INTR_ERR;
	  }
	  //map and store data to hwm_sensor_data
	  sensor_data.values[0] = gp2ap054a_psgs_get_ps_value(obj, obj->ps);
	  sensor_data.values[1] = obj->ps;
	  sensor_data.value_divide = 1;
	  sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	  //let up layer to know
	
	
      printk("************pin value = %d data =%d\r\n",mt_get_gpio_in(GPIO_ALS_EINT_PIN),obj->ps);
	printk("ps_out_threshold = %d ps_in_threshold = %d", atomic_read(&obj->ps_out_threshold), atomic_read(&obj->ps_in_threshold));
       #if 0
	if(sensor_data.values[0] == 0)
	{
		  printk("%s wait far away\n",__func__);
                databuf[0] = 0x04;	
                databuf[1] = (u8)((atomic_read(&obj->ps_out_threshold)) & 0x00FF);
                res = i2c_master_send(obj->client, databuf, 0x2);

			    databuf[0] = 0x05;	
			    databuf[1] = (u8)(((atomic_read(&obj->ps_out_threshold)) & 0xFF00) >> 8);
			    res = i2c_master_send(obj->client, databuf, 0x2);

			    databuf[0] = 0x06;	
			    databuf[1] = (u8)(0x00FF);
			    res = i2c_master_send(obj->client, databuf, 0x2);

			    databuf[0] = 0x07; 
			    databuf[1] = (u8)((0xFF00) >> 8);;
			    res = i2c_master_send(obj->client, databuf, 0x2);
	}
	else
	{
	        printk("%s wait colse\n",__func__);
		 databuf[0] = 0x04;	
                databuf[1] = (u8)(0 & 0x00FF);
                res = i2c_master_send(obj->client, databuf, 0x2); 

			    databuf[0] = 0x05;	
			    databuf[1] = (u8)((0 & 0xFF00) >> 8);
			    res = i2c_master_send(obj->client, databuf, 0x2);

			    databuf[0] = 0x06;	
			    databuf[1] = (u8)((atomic_read(&obj->ps_in_threshold)) & 0x00FF);
			    res = i2c_master_send(obj->client, databuf, 0x2);

			    databuf[0] = 0x07; 
			    databuf[1] = (u8)(((atomic_read(&obj->ps_in_threshold)) & 0xFF00) >> 8);;
			    res = i2c_master_send(obj->client, databuf, 0x2);
	}
	#endif
	  if((res = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
	  {
		APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", res);
		goto EXIT_INTR_ERR;
	  }
	}
	mt_eint_unmask(CUST_EINT_ALS_NUM);
	return;
	EXIT_INTR_ERR:
	mt_eint_unmask(CUST_EINT_ALS_NUM);

	APS_ERR("gp2ap054a_psgs_eint_work err: %d\n", res);

}
/*----------------------------------------------------------------------------*/
static void gp2ap054a_psgs_eint_func(void)
{
	struct gp2ap054a_psgs_als_priv *obj = gp2ap054a_psgs_als_obj;
	if(!obj)
	{
		return;
	}
	printk(KERN_INFO"ddd %s mode=%d\n",__func__, obj->mode);	
	//schedule_work(&obj->eint_work);
	if( obj->mode == SENSOR_MODE_PROXIMITY )
		schedule_work( &obj->eint_work ) ;
	if(( obj->mode == SENSOR_MODE_GESTURE  )||( obj->mode == SENSOR_MODE_PULSE_COUNTER  ))
		schedule_work( &obj->gs_polling_work ) ;
}

int gp2ap054a_psgs_setup_eint(struct i2c_client *client)
{
	struct gp2ap054a_psgs_als_priv *obj = i2c_get_clientdata(client);        

	//g_gp2ap054a_psgs_ptr = obj;
	
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
  mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

//	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
//	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINTF_TRIGGER_FALLING,  gp2ap054a_psgs_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);  
    return 0;
}
/*-------------------------------MISC device related------------------------------------------*/



/************************************************************/
static int gp2ap054a_psgs_open(struct inode *inode, struct file *file)
{
	file->private_data = gp2ap054a_psgs_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/
static int gp2ap054a_psgs_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static long gp2ap054a_psgs_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
		struct i2c_client *client = (struct i2c_client*)file->private_data;
		struct gp2ap054a_psgs_als_priv *obj = i2c_get_clientdata(client);  
		long err = 0;
		void __user *ptr = (void __user*) arg;
		int dat;
		uint32_t enable;
		//int ps_result;
		switch (cmd)
		{
			case ALSPS_SET_PS_MODE:
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if( (err = change_work_mode(SENSOR_MODE_PROXIMITY,obj)) )
					{
						APS_ERR("enable ps fail: %ld\n", err); 
						goto err_out;
					}
					
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((err = change_work_mode(SENSOR_MODE_OFF, obj)))
					{
						APS_ERR("disable ps fail: %ld\n", err); 
						goto err_out;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
				break;
	
			case ALSPS_GET_PS_MODE:
				enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
	
			case ALSPS_GET_PS_DATA:    
				
				if((err = gp2ap054a_psgs_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				
				dat = gp2ap054a_psgs_get_ps_value(obj, obj->ps);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}  
				break;
	
			case ALSPS_GET_PS_RAW_DATA:   
				
				if((err = gp2ap054a_psgs_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				
				dat = obj->ps;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}  
				break;			  
			/*
			case ALSPS_SET_ALS_MODE:
	
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = gp2ap054a_psgs_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %ld\n", err); 
						goto err_out;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = gp2ap054a_psgs_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %ld\n", err); 
						goto err_out;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				break;
	
			case ALSPS_GET_ALS_MODE:
				enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
	
			
			case ALSPS_GET_ALS_DATA: 				
				if((err = gp2ap054a_psgs_read_als(obj->client, &obj->als)))
				{
					goto err_out;
				}
	
				dat = gp2ap054a_psgs_get_als_value(obj, obj->als);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
			
	
			case ALSPS_GET_ALS_RAW_DATA:					
				if((err = gp2ap054a_psgs_read_als(obj->client, &obj->als)))
				{
					goto err_out;
				}
	
				dat = obj->als;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
			*/
			/*----------------------------------for factory mode test---------------------------------------*/
			#if 0
			case ALSPS_GET_PS_TEST_RESULT:
				if((err = gp2ap054a_psgs_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				if(obj->ps > atomic_read(&obj->ps_in_threshold))
					{
						ps_result = 0;
					}
				else	ps_result = 1;
				
				if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
			/*------------------------------------------------------------------------------------------*/
			#endif
			default:
				APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
				err = -ENOIOCTLCMD;
				break;
		}
	
		err_out:
		return err;    
	}
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations gp2ap054a_psgs_fops = {
	.owner = THIS_MODULE,
	.open = gp2ap054a_psgs_open,
	.release = gp2ap054a_psgs_release,
	.unlocked_ioctl = gp2ap054a_psgs_unlocked_ioctl,
};

static struct miscdevice gp2ap054a_psgs_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "psgs",
	.fops = &gp2ap054a_psgs_fops,
};

/*--------------------------------------------------------------------------------------*/
static void gp2ap054a_psgs_early_suspend(struct early_suspend *h)
{
	struct gp2ap054a_psgs_als_priv *obj = container_of(h, struct gp2ap054a_psgs_als_priv, early_drv);	
	//int err;
	APS_FUN();	  
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	//atomic_set(&obj->als_suspend, 1);
	/*
	if((err = gp2ap054a_psgs_enable_als(obj->client, 0)))
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
	*/
	if( test_bit(CMC_BIT_GS,&obj->enable) && test_bit(CMC_BIT_PS,&obj->enable) )	{
		change_work_mode(SENSOR_MODE_PROXIMITY,obj);
	}
	
}

static void gp2ap054a_psgs_late_resume(struct early_suspend *h) 
{
	struct gp2ap054a_psgs_als_priv *obj = container_of(h, struct gp2ap054a_psgs_als_priv, early_drv);		  
	//int err;
	//hwm_sensor_data sensor_data;
	//memset(&sensor_data, 0, sizeof(sensor_data));
	APS_FUN();
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	//atomic_set(&obj->als_suspend, 0);
	if( test_bit(CMC_BIT_GS,&obj->enable) && test_bit(CMC_BIT_PS,&obj->enable) ){
		change_work_mode(SENSOR_MODE_GESTURE,obj);
	}

}
/*--------------------------------------------------------------------------------*/
static void gp2ap_init_device( u8 mode, struct gp2ap054a_psgs_als_priv *data )
{
	int		i ;
	u8		value ;

	for( i = 1 ; i < sizeof( data->regData ) ; i++ )
	{
		gp2ap_i2c_write( i, &data->regData[i], data->client ) ;
	}

	/* VDDCOER setting & Reflective cancellation */
	if( mode == SENSOR_MODE_GESTURE || mode == SENSOR_MODE_PROXIMITY )
	{
		value = REFLECTIVE_CANCEL_GS;
		gp2ap_i2c_write( REG_PANEL, &value, data->client ) ;
	}
	else if ( mode == SENSOR_MODE_PULSE_COUNTER )
	{
		data->analog_offset_reg = 0x36;
		value = data->analog_offset_reg;
		gp2ap_i2c_write( REG_PANEL, &value, data->client ) ;
	}

}

static int gp2ap054a_psgs_init_client(struct i2c_client *client)
{
	int 	i ;
	int res = 0;

	for( i = 1 ; i < sizeof( gp2ap_init_data) ; i++ )
	{
		res = gp2ap_i2c_write( i, &gp2ap_init_data[i], client ) ;
		if(res!=0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto EXIT_ERR;
		}
	}
	if(gp2ap054a_psgs_als_obj->hw->polling_mode_gs == 0){
		printk(KERN_ERR "%s gs_polling=0\n",__func__ ) ;
		res = gp2ap054a_psgs_setup_eint(client);
		if(res!=0)
		{
			APS_ERR("setup eint: %d\n", res);
			return res;
		}
	}
	return 0;
	
	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}

/*--------------------------------------------------------------------------------*/
static void gp2ap_start_sensor( u8 mode, struct gp2ap054a_psgs_als_priv *data )
{
	u8		value ;
	int		i ;

	printk(KERN_ERR "sensor mode = %d\n", mode ) ;

	if( mode == SENSOR_MODE_GESTURE || mode == SENSOR_MODE_PULSE_COUNTER )
	{
		data->regData[REG_PS_LT_LSB] = 0x00 ;
		data->regData[REG_PS_LT_MSB] = 0x00 ;
		data->regData[REG_PS_HT_LSB] = 0xFF ;
		data->regData[REG_PS_HT_MSB] = 0xFF ;
		//data->regData[REG_ADR_01] = ( data->regData[REG_ADR_01] & 0x07 ) | INTVAL_1_56 | INTSEL_D4 ;
		data->regData[REG_ADR_07] = 0xEA ;
		data->regData[REG_ADR_02] = 0x4E ;
		//data->regData[REG_PS7] = ( 0xC8  | PS3_GS_INTVAL6P25 ) ;
		//data->regData[REG_PS6] = (PS2_IS128 | PS2_SUM16 | 0x01) ;

		if( mode == SENSOR_MODE_GESTURE )
		{
			data->regData[REG_ADR_06] = (PS2_IS128 | PS2_SUM16 | 0x01) ;
		}
		else if( mode == SENSOR_MODE_PULSE_COUNTER )
		{
			data->regData[REG_ADR_06] = (PS2_IS32  | PS2_SUM16 | 0x01) ;
		}

		gp2ap_init_device(mode, data ); 


		if(0 == test_bit(CMC_BIT_ALS,  &data->enable))
		{
			value = ( COMMAND1_WAKEUP | COMMAND1_GS ) ; 			/* PS mode */
		}
		else
		{
			value = ( COMMAND1_SD	 ) ; 							/* Shutdown */
			gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;
			value = ( OP_RUN | OP_PS_ALS ) ;	// ALS & PS mode
		}
		gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;

	}
	else if( mode == SENSOR_MODE_PROXIMITY )
	{
		for( i = 0 ; i < 4 ; i++ )
		{
			data->regData[REG_ADR_08+i] = gp2ap_ps_th[i] ;
		}
		data->regData[REG_ADR_07] = 0xEB ;
		data->regData[REG_ADR_06] = (PS2_IS128 | PS2_SUM16 | 0x01) ;
		data->regData[REG_ADR_02] = 0x1E ;
		gp2ap_init_device(mode, data ); 
		if(0 == test_bit(CMC_BIT_ALS,  &data->enable)){
			value = ( COMMAND1_WAKEUP | COMMAND1_GS ) ; 			/* PS mode */
		}
		else{
			value =  COMMAND1_SD ; 			/* Shutdown */
			gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;
			value = ( OP_RUN | OP_PS_ALS ) ;	// ALS & PS mode
		}
		gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;
	}
}

static void gp2ap_stop_sensor( u8 mode, struct gp2ap054a_psgs_als_priv *data )
{
	u8		value ;
	int		i ;
	if(0 == test_bit(CMC_BIT_ALS,  &data->enable)){
		mutex_lock( &data->mutex ) ;
		value = OP_SHUTDOWN ; // shutdown
		gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;
		mutex_unlock( &data->mutex ) ;
	}else{
			value = RST ;	// RST
			gp2ap_i2c_write( REG_ADR_02, &value, data->client ) ;
			
			gp2ap_init_device(0, data );
			//value = ( INTVAL_0 | IS_110 | PIN_DETECT | FREQ_327_5 );
			//gp2ap_i2c_write( REG_ADR_03, &value, client ) ;
			//value = ( OP_RUN | OP_CONTINUOUS | OP_ALS ) ;		// ALS mode
			value = ( OP_RUN | OP_ALS ) ;		// ALS mode
			gp2ap_i2c_write( REG_ADR_00, &value, data->client ) ;
	}

	if( (mode == SENSOR_MODE_GESTURE || mode == SENSOR_MODE_PULSE_COUNTER ) 
		&& data->enable_gesture && !data->hw->polling_mode_gs){
		data->enable_gesture = 0;
		mt_eint_mask(CUST_EINT_ALS_NUM);
	}
	else if( (mode == SENSOR_MODE_GESTURE || mode == SENSOR_MODE_PULSE_COUNTER ) && data->enable_gesture)
	{
		data->enable_gesture = 0;
		hrtimer_cancel( &data->gs_polling_timer ) ;
		flush_work( &data->gs_polling_work ) ;
		cancel_work_sync( &data->gs_polling_work ) ;
	}
	else if( mode == SENSOR_MODE_PROXIMITY )
	{
		if( data->ps_avg_enabled )
		{
			cancel_delayed_work_sync( &data->ps_avg_timer_work ) ;
			data->ps_avg_read_cnt = PS_AVG_READ_COUNT ;
			cancel_work_sync( &data->ps_avg_work ) ;
			hrtimer_cancel( &data->ps_avg_read_timer ) ;
		}
//		disable_irq( data->ps_irq ) ;
//		disable_irq_wake( data->ps_irq ) ;
		data->ps_avg_enabled = false ;
		data->ps_distance = 1 ;
		data->ps_avg[0] = 0 ;
		data->ps_avg[1] = 0 ;
		data->ps_avg[2] = 0 ;
		for( i = 0 ; i < 4 ; i++ )
		{
			gp2ap_ps_th[i] = data->regData[REG_ADR_08+i] ;
		}
	}
}
static int change_work_mode(int mode,struct gp2ap054a_psgs_als_priv * data){
	int value;
	int err;
	hwm_sensor_data ps_sensor_data;

	if( data->mode == mode )
	{
		return 0 ;
	}
	printk(KERN_INFO "%s %d change mode from %d to %d\n",__func__,__LINE__,data->mode,mode);
	if( mode == SENSOR_MODE_OFF )
	{
		gp2ap_stop_sensor( data->mode, data ) ;
		
		hrtimer_cancel( &data->gs_polling_timer ) ;
		flush_work( &data->gs_polling_work ) ;
		cancel_work_sync( &data->gs_polling_work ) ;
		
		mutex_lock( &data->mutex ) ;
		data->mode = mode ;
		mutex_unlock( &data->mutex ) ;
		printk( KERN_INFO "sensor disabe!! \n" ) ;
		//input_report_abs( data->input_dev, ABS_CONTROL_REPORT, ( mode << 24 ) | data->gs_delay ) ;
	}
	else if( mode == SENSOR_MODE_GESTURE || mode == SENSOR_MODE_PULSE_COUNTER )
	{
		if( data->mode != SENSOR_MODE_OFF )
		{
			gp2ap_stop_sensor( data->mode, data ) ;
		}
		mutex_lock( &data->mutex ) ;
		gp2ap_start_sensor( mode, data ) ;
		data->mode = mode ;
		data->enable_gesture = 1;
		mutex_unlock( &data->mutex ) ;
		//input_report_abs( data->input_dev, ABS_CONTROL_REPORT, ( mode << 24 ) | data->gs_delay ) ;
		msleep( 150 ) ;
		if(data->hw->polling_mode_gs == 0)
			mt_eint_unmask(CUST_EINT_ALS_NUM);
		else
			hrtimer_start( &data->gs_polling_timer, data->gs_polling_delay, HRTIMER_MODE_REL ) ;
		printk( KERN_INFO "gesture sensor enable!! \n" ) ;
	}
	else if( mode == SENSOR_MODE_PROXIMITY )
	{
		if( data->mode != SENSOR_MODE_OFF )
		{
			gp2ap_stop_sensor( data->mode, data );
		}
		//hrtimer_cancel( &data->gs_polling_timer );
		//flush_work( &data->gs_polling_work ) ;
		//cancel_work_sync( &data->gs_polling_work );
		
		mutex_lock( &data->mutex );
		gp2ap_start_sensor( mode, data );
		data->mode = mode ;
//		enable_irq_wake( data->ps_irq );
		//value = gpio_get_value_cansleep( data->ps_gpio );
		//value = mt_get_gpio_in( GPIO_ALS_EINT_PIN );
		mutex_unlock( &data->mutex );
		msleep( 200 ) ;
		//input_report_abs( data->input_dev, ABS_CONTROL_REPORT, ( mode << 24 ) | data->gs_delay ) ;
		//input_report_abs( data->input_dev, ABS_DISTANCE_REPORT, value ) ;
		/*
		ps_sensor_data.values[0] = value;
		ps_sensor_data.values[1] = 0;
		ps_sensor_data.values[2] = 0;
		ps_sensor_data.value_divide = 1;
		ps_sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		if(err= hwmsen_get_interrupt_data(ID_PROXIMITY,&ps_sensor_data)){
			printk(KERN_ERR"%s line=%d report ID_PROXIMITY data=%d FAIL!",__func__,__LINE__,value);
		}
		else{
			printk(KERN_ERR"%s line=%d report ID_PROXIMITY data=%d SUCCESS!",__func__,__LINE__,value);
		}
		printk( KERN_INFO "proximity_sensor enable report distance=%d!! \n",value ) ;
		*/
//		enable_irq( data->ps_irq ) ;
		if(data->hw->polling_mode_ps == 0)
			mt_eint_unmask(CUST_EINT_ALS_NUM);
	}
	//input_sync( data->input_dev ) ;
	printk(KERN_INFO "DDD %s enable_prox=%d enable_gesture=%d mode=%d",__func__,
			data->enable_proximity,
			data->enable_gesture,
			data->mode);
	return 0;
}

long gp2ap054a_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
		long err = 0;
		int value;
		hwm_sensor_data* sensor_data;
		struct gp2ap054a_psgs_als_priv *obj = (struct gp2ap054a_psgs_als_priv *)self;
		//APS_FUN(f);
		//printk(KERN_INFO"DDD %s command=%d",__func__,command);
		
		switch (command)
		{
			case SENSOR_DELAY:
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Set delay parameter error!\n");
					err = -EINVAL;
				}
				break;
	
			case SENSOR_ENABLE:
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Enable sensor parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					value = *(int *)buff_in;	
					printk(KERN_INFO"ddd %s sensor_enable command=%d value=%d enable=0x%lx\n",
						__func__,command,value,obj->enable);
					if(value)
					{
						if((err = gp2ap054a_als_enable_als(obj->client, 1)))
						{
							APS_ERR("enable als fail: %ld\n", err); 
							return -1;
						}
						set_bit(CMC_BIT_ALS, &obj->enable);
					}
					else
					{
						if((err = gp2ap054a_als_enable_als(obj->client, 0)))
						{
							APS_ERR("disable als fail: %ld\n", err); 
							return -1;
						}
						clear_bit(CMC_BIT_ALS, &obj->enable);
					}
					
				}
				break;
	
			case SENSOR_GET_DATA:
				if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
				{
					APS_ERR("get sensor data parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					sensor_data = (hwm_sensor_data *)buff_out;
									
					//if((err = gp2ap054a_als_read_als(obj->client, &obj->als)))
					if((err = gp2ap054a_psgs_read_als(obj->client, &obj->als)))
					{
						err = -1;;
					}
					else
					{
						sensor_data->values[0] = obj->als;//gp2ap054a_als_get_als_value(obj, obj->als);
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					}				
				}
				break;
			default:
				APS_ERR("light sensor operate function no this parameter %d!\n", command);
				err = -1;
				break;
		}
		
		return err;

}
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
extern long gp2ap054a_psgs_gs_operate(void* self, uint32_t command, void* buff_in, int size_in,void* buff_out, int size_out, int* actualout);
extern long gp2ap054a_psgs_pulse_operate(void* self, uint32_t command, void* buff_in, int size_in,void* buff_out, int size_out, int* actualout);

long gp2ap054a_psgs_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
		long err = 0;
		int value;
		int val;
		hwm_sensor_data* sensor_data;
		struct gp2ap054a_psgs_als_priv *obj = (struct gp2ap054a_psgs_als_priv *)self;		
		//APS_FUN(f);
if( obj->mode == SENSOR_MODE_GESTURE  )
{
	err = gp2ap054a_psgs_gs_operate(self, command, buff_in, size_in, buff_out, size_out, actualout);
	return err;
}
else if( obj->mode == SENSOR_MODE_PULSE_COUNTER )
{
	err = gp2ap054a_psgs_pulse_operate(self, command, buff_in, size_in, buff_out, size_out, actualout);
	return err;
}

		printk(KERN_INFO"ddd %s command=%d\n",__func__,command);
		switch (command)
		{
			case SENSOR_DELAY:
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Set delay parameter error!\n");
					err = -EINVAL;
				}
				break;
	
			case SENSOR_ENABLE:
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Enable sensor parameter error!\n");
					err = -EINVAL;
				}
				else
				{				
					value = *(int *)buff_in;
					printk(KERN_INFO"ddd %s sensor_enable command=%d value=%d enable=0x%lx mode=%d\n",
						__func__,command,value,obj->enable,obj->mode);
					if(value)
					{
						//if( !test_bit(CMC_BIT_GS,&obj->enable) )
						{
							change_work_mode(SENSOR_MODE_PROXIMITY,obj);
						}
						printk(KERN_INFO"%s PS enable-S=0x%lx value=%d\n",__func__,obj->enable,value);
						set_bit(CMC_BIT_PS, &obj->enable);
						//change_work_mode(SENSOR_MODE_GESTURE,self);
						//set_bit(CMC_BIT_GS, &obj->enable);
						printk(KERN_INFO"%s PS enable-E=0x%lx value=%d\n",__func__,obj->enable,value);
					}
					else
					{
						if( !test_bit(CMC_BIT_GS,&obj->enable) )
						{
							change_work_mode(SENSOR_MODE_OFF,obj);
							//return -1;
						}
						clear_bit(CMC_BIT_PS, &obj->enable);
					}
				}
				break;
	
			case SENSOR_GET_DATA:
				if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
				{
					APS_ERR("get sensor data parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					sensor_data = (hwm_sensor_data *)buff_out;				
					
					if((err = gp2ap054a_psgs_read_ps(obj->client, &obj->ps)))
					{
						err = -1;;
					}
					else
					{
						val = gp2ap054a_psgs_get_ps_value(obj, obj->ps);
						sensor_data->values[0] = val;
						sensor_data->values[1] = obj->ps;
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
						if(debug){
							printk(KERN_INFO"%s SENSOR_GET_DATA report ID_PROXIMITY data=%d raw_data=%d\n",__func__,val,obj->ps);
						}
						/*
						if(err=hwmsen_get_interrupt_data(ID_PROXIMITY,sensor_data)){
							printk(KERN_ERR"%s line=%d report ID_PROXIMITY data=%d FAIL!",__func__,__LINE__,value);
						}
						else{
							printk(KERN_ERR"%s line=%d report ID_PROXIMITY data=%d FAIL!",__func__,__LINE__,value);
						}
						*/
					}				
				}
				break;
			default:
				APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
				err = -1;
				break;
		}
		//if(debug){
		//	printk(KERN_INFO"%s command=%d err=%d\n",__func__,command,err);
		//}
		return err;

}

long gp2ap054a_psgs_gs_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
		long err = 0;
		int value;
		hwm_sensor_data* sensor_data;
		struct gp2ap054a_psgs_als_priv *obj = (struct gp2ap054a_psgs_als_priv *)self;
		//APS_FUN(f);
		printk(KERN_INFO"ddd %s command=%d\n",__func__,command);
		switch (command)
		{
			case SENSOR_DELAY:
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Set delay parameter error!\n");
					err = -EINVAL;
				}
				break;
	
			case SENSOR_ENABLE:
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Enable sensor parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					value = *(int *)buff_in;	
					printk(KERN_INFO"ddd %s sensor_enable command=%d value=%d enable=0x%lx mode=%d\n",
						__func__,command,value,obj->enable,obj->mode);
					if(value)
					{
						change_work_mode(SENSOR_MODE_GESTURE,self);
						set_bit(CMC_BIT_GS, &obj->enable);
						printk(KERN_INFO"%s GS enable=0x%lx value=%d\n",__func__,obj->enable,value);
					}
					else
					{
						if(test_bit(CMC_BIT_PS,&obj->enable)){
							change_work_mode(SENSOR_MODE_PROXIMITY,self);
						}
						else{
							change_work_mode(SENSOR_MODE_OFF,self);
						}
							clear_bit(CMC_BIT_GS, &obj->enable);
					}
					
				}
				break;
	
			case SENSOR_GET_DATA:
				if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
				{
					APS_ERR("get sensor data parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					/*
					sensor_data = (hwm_sensor_data *)buff_out;
									
					if((err = gp2ap054a_psgs_read_als(obj->client, &obj->als)))
					{
						err = -1;;
					}
					else
					{
						sensor_data->values[0] = obj->last_gesture ;//gp2ap054a_psgs_get_als_value(obj, obj->als);
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
						hwmsen_get_interrupt_data(ID_GESTURE,sensor_data);
					}
					*/
					err = -1;				
				}
				break;
			default:
				APS_ERR("light sensor operate function no this parameter %d!\n", command);
				err = -1;
				break;
		}
		
		return err;

}

long gp2ap054a_psgs_pulse_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
		long err = 0;
		int value;
		hwm_sensor_data* sensor_data;
		struct gp2ap054a_psgs_als_priv *obj = (struct gp2ap054a_psgs_als_priv *)self;
		//APS_FUN(f);
		printk(KERN_INFO"ddd %s command=%d\n",__func__,command);
		switch (command)
		{
			case SENSOR_DELAY:
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Set delay parameter error!\n");
					err = -EINVAL;
				}
				break;
	
			case SENSOR_ENABLE:
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Enable sensor parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					value = *(int *)buff_in;	
					printk(KERN_INFO"ddd %s sensor_enable command=%d value=%d enable=0x%lx mode=%d\n",
						__func__,command,value,obj->enable,obj->mode);
					if(value)
					{
						change_work_mode(SENSOR_MODE_PULSE_COUNTER,self);
						set_bit(CMC_BIT_PULSE, &obj->enable);
						printk(KERN_INFO"%s GS enable=0x%lx value=%d\n",__func__,obj->enable,value);
					}
					else
					{
						if(test_bit(CMC_BIT_PS,&obj->enable)){
							change_work_mode(SENSOR_MODE_PROXIMITY,self);
						}
						else{
							change_work_mode(SENSOR_MODE_OFF,self);
						}
							clear_bit(CMC_BIT_PULSE, &obj->enable);
					}
					
				}
				break;
	
			case SENSOR_GET_DATA:
				if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
				{
					APS_ERR("get sensor data parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					/*
					sensor_data = (hwm_sensor_data *)buff_out;
									
					if((err = gp2ap054a_psgs_read_als(obj->client, &obj->als)))
					{
						err = -1;;
					}
					else
					{
						sensor_data->values[0] = obj->last_gesture ;//gp2ap054a_psgs_get_als_value(obj, obj->als);
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
						hwmsen_get_interrupt_data(ID_GESTURE,sensor_data);
					}
					*/
					err = -1;				
				}
				break;
			default:
				APS_ERR("light sensor operate function no this parameter %d!\n", command);
				err = -1;
				break;
		}
		
		return err;

}

/*--------------------------------------------------------------------------------*/


/*-----------------------------------i2c operations----------------------------------*/
static int gp2ap054a_psgs_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gp2ap054a_psgs_als_priv *obj;
	struct hwmsen_object obj_ps, obj_gs, obj_als;
	int err = 0;
	int i =0;
	
	/*if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(*obj));*/
	
	obj = gp2ap054a_psgs_als_obj;
	//gp2ap054a_psgs_als_obj = obj;
	//gp2ap054a_als_obj = obj;
	
	obj->hw = get_cust_alsps_hw();//get custom file data struct
	
	INIT_WORK(&obj->eint_work, gp2ap054a_psgs_eint_work);

	obj->client = client;
	i2c_set_clientdata(client, obj);

	/*-----------------------------value need to be confirmed-----------------------------------------*/

	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	
	//atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	//atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	atomic_set(&obj->ps_in_threshold,  0x2e00);//as default  300
	atomic_set(&obj->ps_out_threshold,  0x1e00);//as default 240
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_gpio = GPIO_ALS_EINT_PIN;
	
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	obj->als_mode = MIDDLE_LUX_MODE;
	
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	
	atomic_set(&obj->i2c_retry, 3);
	obj->enable=0;
	clear_bit(CMC_BIT_GS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PULSE, &obj->enable);
	
	for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )
	{
		obj->regData[i] = gp2ap_init_data[i] ;
	}

	gp2ap054a_psgs_i2c_client = client;

	if((err = gp2ap054a_psgs_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("gp2ap054a_psgs_init_client() OK!\n");

	if((err = misc_register(&gp2ap054a_psgs_device)))
	{
		APS_ERR("gp2ap054a_psgs_i2c_probe register failed\n");
		goto exit_misc_device_register_failed;
	}
	APS_LOG("gp2ap054a_psgs_i2c_probe misc_register OK!\n");

	/*------------------------gp2ap054a_psgs attribute file for debug--------------------------------------*/
	if((err = gp2ap054a_psgs_create_attr(&gp2ap054a_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	APS_LOG("gp2ap054a_psgs_i2c_probe gp2ap054a_psgs_create_attr OK!\n");
	/*------------------------gp2ap054a_psgs attribute file for debug--------------------------------------*/

	obj_ps.self = gp2ap054a_psgs_als_obj;
	obj_ps.polling = obj->hw->polling_mode_ps;	
	obj_ps.sensor_operate = gp2ap054a_psgs_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach ID_PROXIMITY fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	APS_LOG("gp2ap054a_psgs_i2c_probe ID_PROXIMITY hwmsen_attach OK!\n");

	obj_gs.self = gp2ap054a_psgs_als_obj;
	obj_gs.polling = obj->hw->polling_mode_gs;
	obj_gs.sensor_operate = gp2ap054a_psgs_gs_operate;
	if((err = hwmsen_attach(ID_TEMPRERATURE/*ID_GESTURE*/, &obj_gs)))
	{
		APS_ERR("attach ID_GESTURE fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	APS_LOG("gp2ap054a_psgs_i2c_probe ID_GESTURE(ID_TEMPRERATURE) hwmsen_attach OK!\n");

	obj_als.self = gp2ap054a_psgs_als_obj;
	obj_als.polling = obj->hw->polling_mode_als;
	obj_als.sensor_operate = gp2ap054a_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	APS_LOG("ddd %s hwmsen_attach  OK!\n",__func__);
	
	#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend  = gp2ap054a_psgs_early_suspend,
	obj->early_drv.resume   = gp2ap054a_psgs_late_resume,    
	register_early_suspend(&obj->early_drv);
	#endif
	i = OP_SHUTDOWN ;
	err = gp2ap_i2c_write( REG_ADR_00, &i, obj->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "threre is no such device. !! \n" ) ;
		//goto err_no_device ;
	}
    alsps_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	exit_sensor_obj_attach_fail:
	exit_misc_device_register_failed:
		misc_deregister(&gp2ap054a_psgs_device);
	exit_init_failed:
		kfree(obj);
	exit:
	gp2ap054a_psgs_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
    alsps_init_flag = -1;
	return err;
}

static int gp2ap054a_psgs_i2c_remove(struct i2c_client *client)
{
	int err;	
	/*------------------------gp2ap054a_psgs attribute file for debug--------------------------------------*/	
	if((err = gp2ap054a_psgs_delete_attr(&gp2ap054a_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("gp2ap054a_psgs_delete_attr fail: %d\n", err);
	} 
	/*----------------------------------------------------------------------------------------*/
	
	if((err = misc_deregister(&gp2ap054a_psgs_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
		
	gp2ap054a_psgs_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;

}
#if 0
static int gp2ap054a_psgs_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, gp2ap054a_PSGS_DEV_NAME);
	return 0;

}
#endif
static int gp2ap054a_psgs_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}

static int gp2ap054a_psgs_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/

static int alsps_local_init(void) 
{
	struct gp2ap054a_psgs_als_priv *gp2ap;
	char value;
	int err = -1;
	APS_FUN();  
	struct alsps_hw *hw = get_cust_alsps_hw();
	
	gp2ap054a_psgs_power(hw, 1); //*****************   

	gp2ap = kzalloc(sizeof(struct gp2ap054a_psgs_als_priv),GFP_KERNEL);
	if(!gp2ap){
		printk( KERN_ERR "kzalloc error !! \n" ) ;
		return -ENOMEM ;
	}
	memset(gp2ap,0,sizeof(*gp2ap));
	gp2ap054a_psgs_als_obj = gp2ap;

	//for workqueque
	gp2ap->gs_polling_workqueue= create_workqueue("gesture_workqueue");
	if(gp2ap->gs_polling_workqueue==NULL){
		printk(KERN_ERR"%s line %d FETAL ERR,create_workqueue FAIL\n",__func__,__LINE__);
		goto err_fail_queque;
	}
	
	mutex_init(&gp2ap->mutex); 
	gp2ap->mode = SENSOR_MODE_OFF;
	gp2ap->gs_delay = SENSOR_DEFAULT_DELAY;

	
	hrtimer_init( &gp2ap->gs_polling_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL ) ;
	gp2ap->gs_polling_delay = ns_to_ktime( gp2ap->gs_delay * 100 * NSEC_PER_USEC ) ;
	gp2ap->gs_polling_timer.function = gp2ap_gs_polling_timer_func ;

	INIT_WORK( &gp2ap->gs_polling_work, gp2ap_gs_data_polling ) ;
	gp2ap->ps_distance = 1 ;
	
	gp2ap->ps_avg_delay = PS_AVG_READ_INTERVAL ;
	hrtimer_init( &gp2ap->ps_avg_read_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL ) ;
	gp2ap->ps_avg_read_delay = ns_to_ktime( gp2ap->ps_avg_delay * NSEC_PER_MSEC ) ;
	gp2ap->ps_avg_read_timer.function = gp2ap_ps_avg_read_timer_func ;

	INIT_WORK( &gp2ap->ps_int_work, gp2ap_ps_work_int_func ) ;
	INIT_DELAYED_WORK( &gp2ap->ps_avg_timer_work, gp2ap_ps_avg_timer_func ) ;
	INIT_WORK( &gp2ap->ps_avg_work, gp2ap_ps_work_avg_func ) ;
	initGSparams(&st_gs);
	initPULSELib();

//	platform_set_drvdata( pdev, gp2ap ) ;
//	dev_set_drvdata(&pdev->dev, gp2ap); //???
	
	if(i2c_add_driver(&gp2ap054a_psgs_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	APS_ERR("add driver gp2ap054a_psgs_i2c_driver ok!\n");
	
	if(-1 == alsps_init_flag)
	{
	   goto err_no_device ;
	}

	/*
	value = OP_SHUTDOWN ;
	err = gp2ap_i2c_write( REG_ADR_00, &value, gp2ap->client ) ;
	if( err < 0 )
	{
		printk( KERN_ERR "threre is no such device. !! \n" ) ;
		goto err_no_device ;
	}
	*/

	return 0;
err_no_device:
	mutex_destroy(&gp2ap->mutex);
err_fail_queque:
	kfree( gp2ap ) ;
return err ;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove(void)
{
	
	APS_FUN(); 
	struct alsps_hw *hw = get_cust_alsps_hw();
	
	gp2ap054a_psgs_power(hw, 0);//*****************  
	
	i2c_del_driver(&gp2ap054a_psgs_i2c_driver);
	return 0;
}



/*----------------------------------------------------------------------------*/
#if 0
static struct platform_driver gp2ap054a_psgs_driver = {
	.probe      = gp2ap054a_psgs_probe,
	.remove     = gp2ap054a_psgs_remove,    
	.driver     = {
		.name  = "als_ps",
	}
};
#endif
/*----------------------------------------------------------------------------*/
static int __init gp2ap054a_psgs_init(void)
{
	APS_FUN();
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(hw->i2c_num, &i2c_gp2ap054a_psgs, 1);
	alsps_driver_add(&gp2ap054a_init_info);
#if 0
	if(platform_driver_register(&gp2ap054a_psgs_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit gp2ap054a_psgs_exit(void)
{
	APS_FUN();
#if 0
	platform_driver_unregister(&gp2ap054a_psgs_driver);
#endif
	if(gp2ap054a_psgs_als_obj!=NULL){
		flush_workqueue(gp2ap054a_psgs_als_obj->gs_polling_workqueue);
		destroy_workqueue(gp2ap054a_psgs_als_obj->gs_polling_workqueue);
	}
}
/*----------------------------------------------------------------------------*/
module_init(gp2ap054a_psgs_init);
module_exit(gp2ap054a_psgs_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("sharp");
MODULE_DESCRIPTION("gp2ap054a_psgs driver");
MODULE_LICENSE("GPL");

