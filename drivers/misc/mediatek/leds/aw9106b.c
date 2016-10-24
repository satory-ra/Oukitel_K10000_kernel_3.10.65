
#if defined(CONFIG_AW9106B_BREATH_SUPPORT)

#define HCT_AW_WITOUT_CUST_H

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/leds-mt65xx.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>


#include <linux/sched.h>
#include <linux/kthread.h>


#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
//#include <linux/ISSI_leds.h>

#include <cust_leds.h>


#include <mach/mt_pwm.h>
#include <mach/mt_gpio.h>

#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
//#include <mach/mt_pmic_feature_api.h>
#include <mach/mt_boot.h>
#include <linux/i2c.h>

#include <asm/atomic.h>

#define AW9106B_TAG "aw9106b "

#define AW_LEDS_DEBUG_DUMMY(format, args...) 

static int debug_enable = 1;
#define AW_LEDS_DEBUG(format, args...) do{ \
	if(debug_enable) \
	{\
		printk(KERN_EMERG AW9106B_TAG format,##args);\
	}\
}while(0)


#if defined(K36_JXUN_P5E_PROJ)||defined(K36_TX_TX038_PROJ)||defined(K36_PY_P85_PROJ)
#define AW9106_RESET_PIN GPIO108
#else
#define AW9106_RESET_PIN GPIO140
#endif 


#if defined(CONFIG_CONFIG_BREATHLIGHT_I2C_USE_GPIO)

#define SCCB_SERIAL_AW9106_DATA_PIN	GPIO_BH_SDA_PIN
#define SCCB_SERIAL_AW9106_CLK_PIN		GPIO_BH_SCL_PIN
#define SCCB_SERIAL_AW9106_SDB_PIN		GPIO_BH_PN_PIN


#define SET_SCCB_AW9106_CLK_OUTPUT		mt_set_gpio_dir(SCCB_SERIAL_AW9106_CLK_PIN,GPIO_DIR_OUT) 
#define SET_SCCB_AW9106_DATA_OUTPUT	mt_set_gpio_dir(SCCB_SERIAL_AW9106_DATA_PIN,GPIO_DIR_OUT)
#define SET_SCCB_AW9106_DATA_INPUT		mt_set_gpio_dir(SCCB_SERIAL_AW9106_DATA_PIN,GPIO_DIR_IN)
#define SET_SCCB_AW9106_CLK_HIGH		mt_set_gpio_out(SCCB_SERIAL_AW9106_CLK_PIN,GPIO_OUT_ONE)
#define SET_SCCB_AW9106_CLK_LOW		mt_set_gpio_out(SCCB_SERIAL_AW9106_CLK_PIN,GPIO_OUT_ZERO)
#define SET_SCCB_AW9106_DATA_HIGH		mt_set_gpio_out(SCCB_SERIAL_AW9106_DATA_PIN,GPIO_OUT_ONE)
#define SET_SCCB_AW9106_DATA_LOW		mt_set_gpio_out(SCCB_SERIAL_AW9106_DATA_PIN,GPIO_OUT_ZERO)
#define SET_SCCB_AW9106_DATA_MODE		mt_set_gpio_mode(SCCB_SERIAL_AW9106_DATA_PIN,GPIO_MODE_GPIO) 
#define SET_SCCB_AW9106_CLK_MODE		mt_set_gpio_mode(SCCB_SERIAL_AW9106_CLK_PIN,GPIO_MODE_GPIO)
#define SET_SCCB_AW9106_SDB_MODE		mt_set_gpio_mode(SCCB_SERIAL_AW9106_SDB_PIN,GPIO_MODE_GPIO)
#define SET_SCCB_AW9106_SDB_OUTPUT		mt_set_gpio_dir(SCCB_SERIAL_AW9106_SDB_PIN,GPIO_DIR_OUT)
#define SET_SCCB_AW9106_SDB_HIGH		mt_set_gpio_out(SCCB_SERIAL_AW9106_SDB_PIN,GPIO_OUT_ONE)
#define SET_SCCB_AW9106_SDB_LOW		mt_set_gpio_out(SCCB_SERIAL_AW9106_SDB_PIN,GPIO_OUT_ZERO)

#endif

#define SET_AW9106B_RESET_MODE		mt_set_gpio_mode(AW9106_RESET_PIN,GPIO_MODE_GPIO) 
#define SET_AW9106B_DATA_HIGH		mt_set_gpio_out(AW9106_RESET_PIN,GPIO_OUT_ONE)
#define SET_AW9106B_DATA_LOW		mt_set_gpio_out(AW9106_RESET_PIN,GPIO_OUT_ZERO)


#define AW9106_REG_EN  	0x00
#define AW9106_REG_PWM  	0x07 	
#define AW9106_REG_CTRL  	0x14    
#define AW9106_REG_UPDATE 	0x16
#define AW9106_REG_RESET  	0x17
#define AW9106_REG_VAL_RESET    0x01


#ifndef GPIO_BH_SDA_PIN
	#define GPIO_BH_SDA_PIN 0xff
#endif
#ifndef GPIO_BH_SCL_PIN
	#define GPIO_BH_SCL_PIN 0xff
#endif
#ifndef GPIO_BH_PN_PIN
	#define GPIO_BH_PN_PIN 0xff
#endif


#define SCCB_SERIAL_AW9106_ADDR_WRITE	0xa8     
#define SCCB_SERIAL_AW9106_ADDR_READ	0xa9	
#define AW9106_I2C_DELAY		0xff     

#define AW9106_LED_MAX 	18		
static unsigned int count = 0;


struct i2c_client *i2c_client_aw9106 = NULL;
static const struct i2c_device_id aw9106_i2c_id[] = {{"aw9106", 0}, {}};
static unsigned short force[] = {0, 0xB6, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};
static struct i2c_board_info __initdata i2c_aw9106 = { I2C_BOARD_INFO("aw9106", (0xB6 >> 1))};


static struct timer_list AW9106timer;
static struct workqueue_struct *aw9106_queue;
static struct work_struct aw9106_work;
static struct work_struct aw9106_horse_work;

char aw9106_state=0;

static int issi_led_function_mode=-1;

#if defined(CONFIG_CONFIG_BREATHLIGHT_I2C_USE_GPIO)

static u8 get_SDA_bit(void)
{
	return mt_get_gpio_in(SCCB_SERIAL_AW9106_DATA_PIN);
}

#define AW9106_I2C_START_TRANSMISSION \
         SET_SCCB_AW9106_DATA_OUTPUT;SET_SCCB_AW9106_CLK_OUTPUT;SET_SCCB_AW9106_DATA_HIGH;\
	udelay(2);SET_SCCB_AW9106_CLK_HIGH;\
	udelay(2);SET_SCCB_AW9106_DATA_LOW;\
	udelay(2);SET_SCCB_AW9106_CLK_LOW;\
	udelay(2);

#define AW9106_I2C_STOP_TRANSMISSION \
        SET_SCCB_AW9106_DATA_OUTPUT;SET_SCCB_AW9106_CLK_OUTPUT;SET_SCCB_AW9106_DATA_LOW;\
	udelay(2);SET_SCCB_AW9106_DATA_LOW;\
	udelay(2);SET_SCCB_AW9106_CLK_HIGH;\
	udelay(2);SET_SCCB_AW9106_DATA_HIGH;\
	udelay(2);

static u8 i2c_send_byte(u8 send_byte)
{
	u8 rc = 0;
	u8 out_mask = 0x80;
	u8 value;
	u8 count = 8;

	SET_SCCB_AW9106_DATA_OUTPUT;
	SET_SCCB_AW9106_CLK_OUTPUT;

	while(count > 0)
        {
		SET_SCCB_AW9106_CLK_LOW;
		udelay(10);
		value = ((send_byte & out_mask) ? 1 : 0);
		if (value == 1) {
			SET_SCCB_AW9106_DATA_HIGH;
		}
		else {
			SET_SCCB_AW9106_DATA_LOW;
		}
		send_byte <<= 1;
		udelay(10);

		SET_SCCB_AW9106_CLK_HIGH;
		udelay(10);

		count--;
	}
	SET_SCCB_AW9106_CLK_LOW;
	udelay(40);
	SET_SCCB_AW9106_DATA_INPUT;
	udelay(40);
	SET_SCCB_AW9106_CLK_HIGH;
	udelay(10);
	rc = get_SDA_bit();
	udelay(10);
	SET_SCCB_AW9106_CLK_LOW;

	return rc;  
}
#endif

//const U8 AW9106_NUM_INDEX[AW9106_LED_MAX]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};/*18*/

static int AW9106_Write(kal_uint8 reg,kal_uint8 value)
{
    // this is just for test
    return 0;
    
#ifdef CONFIG_BREATHLIGHT_I2C_USE_GPIO

    volatile kal_uint32 i;
    kal_int8 try_counts=5;
    u8 acknowledge;
    AW9106_I2C_START_TRANSMISSION;

    acknowledge=i2c_send_byte(AW9106_I2C_WRITE_ADDR);
    if(acknowledge)
        goto chip_nack;

    acknowledge=i2c_send_byte(reg);
    if(acknowledge)
        goto reg_nack;

    acknowledge=i2c_send_byte(value);

    AW9106_I2C_STOP_TRANSMISSION;

    return acknowledge;
    
chip_nack:
        printk("%s: aw9106 Err: chipa=0x%x, chip_nack \n",__func__,SCCB_SERIAL_AW9106_ADDR_WRITE);
        goto error_exit ;

reg_nack:
        printk("%s: aw9106 chipa=0x%x,reg=0x%x,  reg_nack\n",__func__,SCCB_SERIAL_AW9106_ADDR_WRITE,reg);
        goto error_exit ;
            

#else

    u8 databuf[2];	
    int res=0;

    databuf[0] = reg;	
    //databuf[1] = reg_value[0] &0xFE;
    databuf[1] = value ;
    i2c_client_aw9106->addr = (0xb6 >> 1);

    res = i2c_master_send(i2c_client_aw9106, databuf, 0x2);
    if(res <= 0)
    {
    	printk("AW9106 write i2c error !! res=%d\n",res);
        goto error_exit;
        
    }
   return 0;
   
#endif

error_exit:
        return -1;

   
}


#define HCT_PROJ_MAX_SUPPORT_LIGHT   8
#define HCT_LED_BREATH_MAX_LEVEL      255

atomic_t   led_soft_breath_off_flag ; 


int light_trans_index[HCT_PROJ_MAX_SUPPORT_LIGHT]={0,1,2,3,4,5,6,7};

int led_soft_breath_level =0;
unsigned int current_breath_leds =0;

unsigned int soft_breath_work_flag=0;
unsigned int led_horse_work_flag=0;

///---this wq is for led breath sync
static unsigned int soft_breath_flag=0;
wait_queue_head_t soft_breath_wq;

// ------ this wq is for horse to syn with breath call
static unsigned int soft_breath_finished_flag=0;
wait_queue_head_t soft_breath_finished_wq;
wait_queue_head_t hct_horse_work_finished_wq;

static unsigned int soft_horse_finished_flag=0;
static unsigned int led_horse_wq_flag=0;
wait_queue_head_t led_horse_wq;

static struct work_struct ISSI_led_function_work_breath;      // --- Work queue ---
static struct work_struct ISSI_led_function_work_horse;      // --- Work queue ---
static struct work_struct ISSI_led_function_work_stop;      // --- Work queue ---

/*----------------------------------------------------------------------------*/
struct led_breath_range_paramter {
    int          level_range;
    int          level_step;
};

struct led_breath_paramter{
   struct led_breath_range_paramter range_pramter[6];
    int   time_val;
};

void hct_all_effect_stop();


const struct led_breath_paramter prj_breath_paramter[4]=
{
        { //[0]  --------------slow
                 100, 2 ,    // range_pramter[0]
                 200, 5 ,
                 255, 8 ,
                 200, -8 ,    
                 100, -5 ,   // range_pramter[4]
                 0, -2,
                15,      // time_val  unit: ms
        },

                
        {//[1]  ---------middle
                 50, 5 ,    // range_pramter[0]
                 100, 5 ,
                 255, 5 ,
                 100, -5 ,    
                 50, -5 ,   // range_pramter[4]
                  0, -5,
                10,      // time_val  unit: ms
        },

        {//[2]  ------------fast
                 50, 5 ,    // range_pramter[0]
                 100, 5 ,
                 255, 5 ,
                 100, -5 ,    
                 50, -5 ,   // range_pramter[4]
                 0, -5,
                5,      // time_val  unit: ms
        },


        {//[2]  ------------user defined
                 50, 20 ,    // range_pramter[0]
                 100,  20,
                 255, 15 ,
                 100, -5 ,    
                 50, -5 ,   // range_pramter[4]
                 0, -10,
                5,      // time_val  unit: ms
        },                
};


//----this macro is for soft breath leds level, add the only one set leds on
#define SET_LEDS_LEVEL(leds,level)  \
do{\
	int i=0;\
	for(i=0;i<HCT_PROJ_MAX_SUPPORT_LIGHT;i++)\
	{\
		if((0x01<<i)&leds){\
                AW9106_Write(0x20+light_trans_index[i],level);\
		AW_LEDS_DEBUG_DUMMY("jay set leds logic_chn=%d,p_chn=%d,level=%d\n",i,light_trans_index[i],level);}\
	}\
}while(0)


#define SET_ALL_LEDS_LEVEL(leds,level)  \
do{\
    int i=0;\
    for(i=0;i<HCT_PROJ_MAX_SUPPORT_LIGHT;i++)\
    {\
        if((0x01<<i)&leds){\
                AW9106_Write(0x20+light_trans_index[i],level);\
                AW_LEDS_DEBUG_DUMMY("jay_set leds logic_chn=%d,p_chn=%d,level=%d\n",i,light_trans_index[i],level);}\
         else{AW9106_Write(0x20+light_trans_index[i],0);\
            }\
    }\
}while(0)
        


struct led_breath_paramter * current_paramter;

void led_soft_one_finished()
{
    AW_LEDS_DEBUG("-------Jay_ led_soft_one_finished -------\n");

    soft_breath_flag = 1;
    wake_up_interruptible(&soft_breath_wq);

}

void led_soft_breath_timout(unsigned long n)
{
   AW_LEDS_DEBUG_DUMMY("-------Jay_ led_breath_timeout -------\n");
   if(atomic_read(&led_soft_breath_off_flag) != 1)
   {
       queue_work(aw9106_queue,&aw9106_work);
   }
   else
   {
       led_soft_one_finished();  // this is to wakeup the 
   }
   return;

}

static unsigned int current_range_index=0;

static void hct_led_soft_breath_work(struct work_struct *work)
{

    
   if(atomic_read(&led_soft_breath_off_flag))
   {
     led_soft_one_finished();
     return;
    }  
   
    AW_LEDS_DEBUG_DUMMY("%s:current_range_index=%d,led_level=%d\n",__func__,current_range_index,led_soft_breath_level);
    
    if(current_paramter->range_pramter[current_range_index].level_step>0)
    {
        if (led_soft_breath_level+current_paramter->range_pramter[current_range_index].level_step>current_paramter->range_pramter[current_range_index].level_range)
        {
            current_range_index++;
        
        }
    }
    else
    {
        if (led_soft_breath_level+current_paramter->range_pramter[current_range_index].level_step<current_paramter->range_pramter[current_range_index].level_range)
        {
            current_range_index++;
        
        }
    
    }
    led_soft_breath_level += current_paramter->range_pramter[current_range_index].level_step;
                
    if(led_soft_breath_level<=0)
    {
        current_range_index=0;
	led_soft_breath_level = 0;
        SET_LEDS_LEVEL(current_breath_leds, led_soft_breath_level);
        led_soft_one_finished();
        return;
    }
    SET_LEDS_LEVEL(current_breath_leds, led_soft_breath_level);


    AW9106timer.function = led_soft_breath_timout;
    AW9106timer.expires = jiffies+ msecs_to_jiffies(current_paramter->time_val);
    init_timer(&AW9106timer);
    add_timer(&AW9106timer);
    return;
    
}


static void hct_leds_soft_breath_common(unsigned int leds,int breath_mode)
{

    if(atomic_read(&led_soft_breath_off_flag))
         return;
    
    led_soft_breath_level=0;
    current_breath_leds = leds;

    AW9106timer.function = led_soft_breath_timout;
    AW9106timer.data = leds;
    SET_LEDS_LEVEL(leds,led_soft_breath_level);
    #if 1
    AW9106timer.expires = jiffies+1 ; //+ msecs_to_jiffies(current_paramter->time_val);
    init_timer(&AW9106timer);
    add_timer(&AW9106timer);
    #else
    led_soft_breath_timout(leds);
    #endif
	 	
}


static S32 hct_leds_soft_breath(unsigned int * breath_seq, int seq_lenth, int breath_mode, int breath_cont)
{
     int i=0;
     unsigned int breath_led=0;
     int breath_cnt = breath_cont;
     int infeient_flag = (breath_cont==0)?1:0;
     if(breath_mode >=4)
     {
        AW_LEDS_DEBUG("ERROR----- bad breath_mode\n");
        goto finish_exit;
     }
     
 restart:
     soft_breath_work_flag=1;
     
     current_paramter = prj_breath_paramter+breath_mode;
     
     for(i=0;i<seq_lenth;i++)
     {
         if(atomic_read(&led_soft_breath_off_flag))
		 goto finish_exit;
		 	
         breath_led = *(breath_seq+i);
         hct_leds_soft_breath_common(breath_led,breath_mode);	

         if(atomic_read(&led_soft_breath_off_flag))
		 goto finish_exit;
         
         wait_event_interruptible(soft_breath_wq, soft_breath_flag);
         soft_breath_flag = 0;
	 	 
     }
     if(1 == infeient_flag)goto restart;

      breath_cnt--;

     if(breath_cnt!=0)
	 	goto restart;


 finish_exit:
     AW_LEDS_DEBUG("hct-soft_breath finished----\n");

     soft_breath_work_flag=0;

     soft_breath_finished_flag=1;
     wake_up_interruptible(&soft_breath_finished_wq);
     
    return 0;

}

// on/off time max is 16.384s
// rasing/fainling time max is 4096ms
#define bright_div_val   128
unsigned int timems_trans_arry[8]= {0, 256, 512,1024, 2048, 4096, 8192, 16384};

static void hct_leds_breath_by_chip(unsigned int leds,int rasing_time, int on_time,int failing_time,int off_time)
{
    unsigned int rasing_val=0, on_val=0,failig_val=0,off_val=0;
    unsigned char blink_en_val=0;
    int i=0;

    rasing_val = (rasing_time+bright_div_val)/256;
    on_val = (on_time+bright_div_val)/256;
    failig_val = (failing_time+bright_div_val)/256;
    off_val = (off_time+bright_div_val)/256;

    if(rasing_val>5)
        rasing_val = 5;
    if(failig_val>=5)
        failig_val = 5;

    if(on_val>=7)
        on_val = 7;
    if(off_val>=7)
        off_val = 7;

    AW9106_Write(0x12,0x00);   //OUT4~5
    AW9106_Write(0x13,0x00);   //OUT0~3

    AW9106_Write(0x04,0x03);
    AW9106_Write(0x05,0x0F);
    AW9106_Write(0x15,(failig_val<<3)|rasing_val);
    AW9106_Write(0x16,(off_val<<3)|on_val);

    
    for(i=0;i<HCT_PROJ_MAX_SUPPORT_LIGHT;i++)
    {
    	if((0x01<<i)&leds){
                blink_en_val = blink_en_val|(0x01<<light_trans_index[i]);}
    }
    if(blink_en_val>0x1F)
    {
        AW_LEDS_DEBUG("----ERRRR_ hct_leds_breath_by_chip, some channel is no support this mode -------\n");
        return;
    }
    
    AW_LEDS_DEBUG("----hct_leds_breath_by_chip, ras=%d,on=%d,fail=%d,off=%d ,blik=%x-------\n",rasing_val,on_val,failig_val,off_val,blink_en_val);

    AW9106_Write(0x14,blink_en_val);
    AW9106_Write(0x11,0x80);
    

}

//////////////////////////////////////////////////////////////////  this horse -------------

atomic_t   led_horse_off_flag ;
unsigned int current_horse_leds =0;


void led_soft_horse_timout(unsigned long n)
{
    AW_LEDS_DEBUG("-------Jay_ led_soft_horse_timout -------\n");

    led_horse_wq_flag=1;
    wake_up_interruptible(&led_horse_wq);

finish_exit:
    return;
    
}


static void hct_leds_horse_common(unsigned int leds,int time_val)
{
     if(atomic_read(&led_horse_off_flag))
        goto finish_exit;	 
     
    current_horse_leds = leds;
    SET_ALL_LEDS_LEVEL(current_horse_leds, HCT_LED_BREATH_MAX_LEVEL);
    AW_LEDS_DEBUG("%s:set leds=%x, timelast=%d\n",__func__,leds,time_val);

    AW9106timer.function = led_soft_horse_timout;
    AW9106timer.data = leds;
    AW9106timer.expires = jiffies+ msecs_to_jiffies(time_val);
    init_timer(&AW9106timer);
    add_timer(&AW9106timer);

finish_exit:
    return;
}


static S32 hct_leds_horse(unsigned int * horse_seq, int seq_lenth, unsigned int * time_seq, int time_lenth, int horse_cont)
{
    int i=0;
    unsigned int horse_led=0;
    int horse_cnt = horse_cont;
    int horse_last_time;
    int infeient_flag = (horse_cont==0)?1:0;
    int acture_seq_lenth;

    if((seq_lenth!=time_lenth)||(*(horse_seq+(seq_lenth-1))!=0))
    {
        AW_LEDS_DEBUG("%s: wrong paramter\n",__func__);
        return;
    }

    AW_LEDS_DEBUG("%s: se_L=%d,led_of_flg=%d\n",__func__,seq_lenth,atomic_read(&led_horse_off_flag));
    acture_seq_lenth=infeient_flag?seq_lenth-1:seq_lenth;


restart:

     led_horse_work_flag = 1;
      
     for(i=0;i<seq_lenth;i++)
     {
         if(atomic_read(&led_horse_off_flag))
            goto finish_exit;
            
         horse_led = *(horse_seq+i);
         horse_last_time = *(time_seq+i);
         hct_leds_horse_common(horse_led,horse_last_time);   
         if(atomic_read(&led_horse_off_flag))
            goto finish_exit;
         {
            wait_event_interruptible(led_horse_wq, led_horse_wq_flag);
            led_horse_wq_flag = 0;
          }
         
     }
     if(1 == infeient_flag)goto restart;

      horse_cnt--;

     if(horse_cnt!=0)
        goto restart;

     finish_exit:
        
     AW_LEDS_DEBUG("----------hct_leds_horse function finished \n");
     soft_horse_finished_flag=1;
     wake_up_interruptible(&hct_horse_work_finished_wq);
     return 0;

}



/////////////////////////////////////////////////////------------this is common interface
 void hct_aw_power_onoff( kal_bool onoff )
{
    AW_LEDS_DEBUG("%s, onof=%d, aw_stat=%d\n", __func__, onoff,aw9106_state);
    if((aw9106_state==1)&&(onoff==1))
    {
        return ;
    }

    SET_AW9106B_RESET_MODE;
    SET_AW9106B_DATA_HIGH;

    AW9106_Write(0x7f,0x00);  
    AW9106_Write(0x7f,0x00); 
    SET_AW9106B_DATA_LOW;

    udelay(200); 
    if (onoff ==1)
    {  
        SET_AW9106B_DATA_HIGH;
        udelay(30); 


        AW9106_Write(0x7f,0x00); //
        AW9106_Write(0x7f,0x00); 

        AW9106_Write(0x12,0x00);   //OUT4~9
        AW9106_Write(0x12,0x00);   //OUT4~9
        AW9106_Write(0x13,0x00);   //OUT0~3
        AW9106_Write(0x13,0x00);   //OUT0~3

        aw9106_state=1;
    }
    else
    {
        aw9106_state=0;
    }
}


void aw_hct_all_effect_stop()
{

        AW_LEDS_DEBUG("%s----start:breath_work=%d,horse_=%d\n",__func__,soft_breath_work_flag,led_horse_work_flag);

        if(soft_breath_work_flag)
        {
            atomic_set(&led_soft_breath_off_flag, 1);
            wait_event_interruptible(soft_breath_finished_wq, soft_breath_finished_flag);
            soft_breath_finished_flag = 0;
            del_timer_sync(&AW9106timer);

            atomic_set(&led_soft_breath_off_flag, 0);
        }


        if(led_horse_work_flag)
        {
            atomic_set(&led_horse_off_flag, 1);
            wait_event_interruptible(hct_horse_work_finished_wq, soft_horse_finished_flag);
            del_timer_sync(&AW9106timer);
            atomic_set(&led_horse_off_flag, 0);
        }
        AW_LEDS_DEBUG("%s----finished---\n",__func__);
        
        SET_ALL_LEDS_LEVEL(0xFFF,0);
        hct_aw_power_onoff(0);


}



#if 1 // Jay_new_function
#include <linux/time.h>

static struct hrtimer aw_led_kthread_timer;
 ktime_t aw_led_ktime;
 int aw_thread_wakeup_flag=0;

static DEFINE_MUTEX(aw_led_mutex);
static DECLARE_WAIT_QUEUE_HEAD(aw_led_thread_wq);

 static void aw_thread_breath(void);
 static void aw_thread_horse_work(void);


 typedef enum
 {
     SMART_KEY_IDLE =0,
     SMART_KEY_DETECTING =1
 }SMRT_KEY_STATUE;

 
 typedef enum
 {
     LED_IDLE_STOP =0,
     LED_HORSE_RAISING =1,
     LED_SOFT_BREATHING =2,
     LED_CHIP_BREATHING =3,
     LED_MAX_STATUS
 }LED_WORK_STATUS;


 
static LED_WORK_STATUS aw_led_work_status=LED_IDLE_STOP;


struct wake_lock led_thread_lock;



void wake_up_aw_thread(void)
 {
    aw_thread_wakeup_flag = 1;
     wake_up(&aw_led_thread_wq);
 }

#if 0
enum hrtimer_restart aw_kthread_hrtimer_func(struct hrtimer *timer)
 {
     printk("Jay_ smart_key_hrtimer_out\n");
     if(current_key_status ==SMART_KEY_UP)
     {
          wake_up_smart_key();
     }
     else  // this means now is key down
     {
         smart_key_thread_timeout = 1;
     }
     
    return HRTIMER_NORESTART;
}
void aw_led_timer_init(void)
 {
    aw_led_ktime = ktime_set(2,0);       // 3s, 10* 1000 ms
    hrtimer_init(&aw_led_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

    aw_led_kthread_timer.function = aw_kthread_hrtimer_func;   
  //  battery_xlog_printk(BAT_LOG_CRTI, "battery_kthread_hrtimer_init : done\n" );
}

 void aw_led_timer_start(void)
  {
     hrtimer_start(&aw_led_kthread_timer, aw_led_ktime, HRTIMER_MODE_REL);
 }
 
 void aw_led_timer_stop(void)
 {
     hrtimer_cancel(&aw_led_kthread_timer);
 }
#endif
 
 static int aw_thread_kthread(void *x)
{

    /* Run on a process content */
    while (1) {
            wait_event(aw_led_thread_wq, aw_thread_wakeup_flag);
            aw_thread_wakeup_flag=0;
            printk("aw9106b: aw waked up --,--statu=%d --\n",aw_led_work_status);

            mutex_lock(&aw_led_mutex);

            switch(aw_led_work_status)
            {
                 case LED_IDLE_STOP:
                    aw_hct_all_effect_stop();
                    break;
                 case LED_SOFT_BREATHING:
                    aw_thread_breath();
                    break;
                 case LED_CHIP_BREATHING:                   
                    hct_leds_breath_by_chip(0x8,300,100,300,10000);
                    break;
                 case LED_HORSE_RAISING:
                    aw_thread_horse_work();
                    break;
                 default:
                    aw_hct_all_effect_stop();
                    break;
            };
            mutex_unlock(&aw_led_mutex);
            printk("Jay_debug: thread finished\n");
            
            issi_led_function_mode = -1;
        }
    return 0;
}


#endif


static int issi_led_function_breath_mode=-1;
static int issi_led_function_horse_mode=-1;
static int issi_led_function_chip_hose_mode=-1;


//////////////////////////////////////////////////////////////////
// abort this two wrok , breath light use work1, and horse use work2..
//          work3 is defined for stop all horse and breath light work.
/////////////////////////////////////////////////////////////////



unsigned int breath_seq_mod1[]= {0x8,0x1C,0x3E,0x7F, 0x40,0x20,0x10,0x8,0x4,0x2,0x1};

unsigned int breath_seq_mod2[]= {0x7F};
unsigned int breath_seq_mod5[]= {0x70,0x7c,0x7f};


unsigned int horse_seq_mod2[]=        {0x8,  0,    0x1c, 0,    0x3E,    0,    0x7F,  0,  0x40,0x20,0x10,0x8, 0x4,  0x2,0x1,0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0 , 0};
unsigned int horse_seq_timemod2[]= {400, 400, 300,  300,  200,   200,   100,  100, 50,     50,   50,   50,   50,   50,  50, 50,   50,   50,   50,     50,    50,   400, 0};

unsigned int horse_seq_mod3[]=        {0x7F, 0x0, 0x7F,0x0,0x7F,0x0,0x7F,0x0,0x7F,0x0,0x7F,0x0,0x7F,0};
unsigned int horse_seq_timemod3[]= {100,  200,  100, 200, 100,  200, 100, 200,100,  200,100,  200,100,  0};


unsigned int horse_seq_mod4[]=        {0,    0x70, 0x0, 0x7C,    0x0, 0x7F, 0x0};
unsigned int horse_seq_timemod4[]= {400, 500,   100,   400,   100,   300,   20};


unsigned int horse_seq_mod6[]=        {0x8, 0x0};
unsigned int horse_seq_timemod6[]= {500,  10000};

unsigned int horse_seq_mod7[]=         {0x8, 0, 0x14, 0 ,  0x22, 0, 0x41, 0, 0x22, 0, 0x14, 0 , 0x8, 0 , 0x1c, 0 , 0x3e, 0 , 0x7f,0,  0x3e,0,  0x1c,0,  0x08, 0};
unsigned int horse_seq_timemod7[]= { 50, 10,  50, 10,    50,   10,  50, 10,   50, 10,   50, 10,  50, 10,    50,   10,  50, 10,   50, 10,    50, 10, 50 ,10, 50, 10};

unsigned int horse_seq_mod11[]=        {0x8, 0, 0x14, 0 ,  0x22, 0, 0x41, 0, 0x22, 0, 0x14, 0 , 0x8, 0 , 0x1c, 0 , 0x3e, 0 , 0x7f,0,  0x3e,0,  0x1c,0,  0x08, 0};
unsigned int horse_seq_timemod11[]= { 50, 10,  50, 10,    50,   10,  50, 10,   50, 10,   50, 10,  50, 10,    50,   10,  50, 10,   50, 10,    50, 10, 50 ,10, 50, 10};

unsigned int horse_seq_mod12[]=        {0x70,  0, 0x38, 0,  0x1C,  0, 0x0F, 0,  0x70, 0,  0x38,  0, 0x1C, 0,  0x0F, 0,  0x70, 0,  0x38, 0, 0x1C, 0,  0x0F,   0};  // 3  OK
unsigned int horse_seq_timemod12[]= {500, 100, 500,  100,   500,   100,   500, 100,  500,   100,   500, 100,   500, 100,  500,   100,    500, 100,   500, 100,  500,   100,    500,   100,};

unsigned int horse_seq_mod13[]=        {0x33, 0,  0x66, 0,  0x33,  0, 0x66, 0,  0x33, 0,  0x66,  0};
unsigned int horse_seq_timemod13[]= { 200, 10, 200,   10,   200,   10,   200, 10,  200,   10,   200, 10};  // mode 4 for music

unsigned int horse_seq_mod14[]=        {0x40, 0x20, 0x10, 0x08, 0x04, 0x02,  0x01,0x40, 0x20, 0x10, 0x08, 0x04, 0x02,  0x01,0x40, 0x20, 0x10, 0x08, 0x04, 0x02,  0x01, 0};
unsigned int horse_seq_timemod14[]= {  200, 200,  200,   200,    200,   200,   200, 200,  200,    200,   200,  200  ,  200,  200,   200,  200,   200,   200 ,  200,   200,   200 ,  200};  // mode 4 for music

unsigned int horse_seq_mod15[]=        {0x08,  0x1c, 0x3e, 0x7f, 0x3e, 0x1c,  0x08, 0x1c, 0x3e, 0x7f, 0x3e, 0x1c,  0x08, 0x1c, 0x3e, 0x7f, 0x3e, 0x1c,  0x08, 0};
unsigned int horse_seq_timemod15[]= {  200, 200,  200,    200,    200,   200,   200,  200,   200,   200,   200,  200  ,  200,  200,    200,   200,   200,   200 ,  200,   200};  // mode 4 for music

unsigned int horse_seq_mod16[]=        {0x08,  0, 0x08,   0x0,    0x41, 0x22,  0x14, 0x8, 0x14, 0x22, 0x41, 0x0,  0x08,  0, 0x08,   0x0,    0x41, 0x22,  0x14, 0x8, 0x14, 0x22, 0x41, 0x0,};
unsigned int horse_seq_timemod16[]= {  200, 100,  200,  100,   200,   200,   200,   400,   200,   200,   200,  200 , 200, 100,  200,  100,   200,   200,   200,   400,   200,   200,   200,  200  };  // mode 4 for music


unsigned int horse_seq_mod17[]=        {0x8, 0, 0x14, 0 ,  0x22, 0, 0x41, 0, 0x22, 0, 0x14, 0 , 0x8, 0 , 0x1c, 0 , 0x3e, 0 , 0x7f,0,  0x3e,0,  0x1c,0,  0x08,  0, \
                                                                    0x70,  0, 0x38, 0,  0x1C,  0, 0x0F,   0,     0x70, 0,      0x38,  0,    0x1C,   0,  0x0F,   0,     0x70,   0,   0x38,   0,   0x1C,   0,    0x0F,  0,\
                                                                    0x33, 0,  0x66, 0,  0x33,  0, 0x66, 0,  0x33, 0,  0x66,  0,\
                                                                    0x40, 0x20, 0x10, 0x08, 0x04, 0x02,  0x01,0x40, 0x20, 0x10, 0x08, 0x04, 0x02,  0x01,0x40, 0x20, 0x10, 0x08, 0x04, 0x02,  0x01, 0,\
                                                                    0x08,  0x1c, 0x3e, 0x7f, 0x3e, 0x1c,  0x08, 0x1c, 0x3e, 0x7f, 0x3e, 0x1c,  0x08, 0x1c, 0x3e, 0x7f, 0x3e, 0x1c,  0x08, 0,\
                                                                    0x08,  0, 0x08,   0x0,    0x41, 0x22,  0x14, 0x8, 0x14, 0x22, 0x41, 0x0,  0x08,  0, 0x08,   0x0,    0x41, 0x22,  0x14, 0x8, 0x14, 0x22, 0x41, 0x0,\
                                                                    };
unsigned int horse_seq_timemod17[]= {50, 10,  50, 10,    50,   10,  50, 10,   50, 10,   50, 10,  50, 10,    50,   10,  50, 10,   50, 10,    50, 10, 50 ,10, 50, 10,  \
                                                                   500, 100, 500,  100,   500, 100,  500, 100,  500,   100,   500, 100,   500, 100,  500,   100,    500, 100,   500, 100,  500,   100,    500,  500,\
                                                                   200, 10, 200,   10,   200,   10,   200, 10,  200,   10,   200, 200, \
                                                                   200, 200,  200,   200,    200,   200,   200, 200,  200,    200,   200,  200  ,  200,  200,   200,  200,   200,   200 ,  200,   200,   200 ,  200,\
                                                                   200, 200,  200,    200,    200,   200,   200,  200,   200,   200,   200,  200  ,  200,  200,    200,   200,   200,   200 ,  200,   200,\
                                                                   200, 100,  200,  100,   200,   200,   200,   400,   200,   200,   200,  200 , 200, 100,  200,  100,   200,   200,   200,   400,   200,   200,   200,  200, \
                                                                    };  // mode 4 for music



static void aw_thread_breath(void)
{
    int temp_issi_led_function_mode =issi_led_function_breath_mode ;
    AW_LEDS_DEBUG("%s:setmode =%d \n",__func__, issi_led_function_breath_mode);
    if(issi_led_function_breath_mode<0)
    {
        AW_LEDS_DEBUG("%s: %d,this is wrong mode, pls check \n",__func__, issi_led_function_mode);
        goto exit;
        
    }
    
    if(soft_breath_work_flag)
    {
        AW_LEDS_DEBUG("%s:mod:%d,we canot swith breath to breath pls check \n",__func__, issi_led_function_mode);
        goto exit;
    }

    AW_LEDS_DEBUG("%s:setmode =%d \n",__func__, issi_led_function_mode);

///we rereset /the cha/nel//            
     AW9106_Write(0x11,0x00);

    switch(temp_issi_led_function_mode)
    {
        case 1: // 129
            hct_leds_soft_breath(breath_seq_mod1,sizeof(breath_seq_mod1)/sizeof(unsigned int),1,0);
            break;
            
        case 5: // 129
            hct_leds_soft_breath(breath_seq_mod5,sizeof(breath_seq_mod5)/sizeof(unsigned int),1,2);
            break;

        defult:
        case 10: //138    
            hct_leds_soft_breath(breath_seq_mod2,sizeof(breath_seq_mod2)/sizeof(unsigned int),3,0);
        break;            
    }

exit:

    AW_LEDS_DEBUG("---------------%s:finished \n",__func__);

    
}


static void aw_thread_horse_work(void)
{
    int temp_issi_led_function_mode =issi_led_function_horse_mode ;
    if(issi_led_function_horse_mode<0)
    {
        AW_LEDS_DEBUG("%s:this is wrong mode %d, pls check \n",__func__, issi_led_function_horse_mode);
        return;
        
    }
    
    AW_LEDS_DEBUG("%s:setmode =%d \n",__func__, issi_led_function_horse_mode);
    ///////reset the led to normal mode////////
    AW9106_Write(0x11,0x00);

    switch(temp_issi_led_function_mode)
    {
         case 2: // 130
             hct_leds_horse(horse_seq_mod2, sizeof(horse_seq_mod2)/sizeof(unsigned int), horse_seq_timemod2, sizeof(horse_seq_timemod2)/sizeof(unsigned int), 0);
         break;
         
         case 3: // 131
             hct_leds_horse(horse_seq_mod3, sizeof(horse_seq_mod3)/sizeof(unsigned int), horse_seq_timemod3, sizeof(horse_seq_timemod3)/sizeof(unsigned int), 0);
         break;
         
         case 4: // 132
             hct_leds_horse(horse_seq_mod4, sizeof(horse_seq_mod4)/sizeof(unsigned int), horse_seq_timemod4, sizeof(horse_seq_timemod4)/sizeof(unsigned int), 0);
         break;
         
         case 6: // 134 --ok
             //hct_leds_horse(horse_seq_mod6, sizeof(horse_seq_mod6)/sizeof(unsigned int), horse_seq_timemod6, sizeof(horse_seq_timemod6)/sizeof(unsigned int), 0);
             hct_leds_breath_by_chip(0x8,300,100,300,10000);
         break;
         
         case 7: // 135
             hct_leds_horse(horse_seq_mod7, sizeof(horse_seq_mod7)/sizeof(unsigned int), horse_seq_timemod7, sizeof(horse_seq_timemod7)/sizeof(unsigned int), 0);
         break;

         case 11: // 139--ok
             hct_leds_horse(horse_seq_mod11, sizeof(horse_seq_mod11)/sizeof(unsigned int), horse_seq_timemod11, sizeof(horse_seq_timemod11)/sizeof(unsigned int), 0);
         break;

          case 12: // 140--ok
             hct_leds_horse(horse_seq_mod12, sizeof(horse_seq_mod12)/sizeof(unsigned int), horse_seq_timemod12, sizeof(horse_seq_timemod12)/sizeof(unsigned int), 0);
         break;

         case 13: // 141--ok
             hct_leds_horse(horse_seq_mod13, sizeof(horse_seq_mod13)/sizeof(unsigned int), horse_seq_timemod13, sizeof(horse_seq_timemod13)/sizeof(unsigned int), 0);
         break;

         case 14: // 142 --ok
             hct_leds_horse(horse_seq_mod14, sizeof(horse_seq_mod14)/sizeof(unsigned int), horse_seq_timemod14, sizeof(horse_seq_timemod14)/sizeof(unsigned int), 0);
         break;
         
          case 15: // 143--ok
             hct_leds_horse(horse_seq_mod15, sizeof(horse_seq_mod15)/sizeof(unsigned int), horse_seq_timemod15, sizeof(horse_seq_timemod15)/sizeof(unsigned int), 0);
         break;
         
          case 16: // 144--ok
             hct_leds_horse(horse_seq_mod16, sizeof(horse_seq_mod16)/sizeof(unsigned int), horse_seq_timemod16, sizeof(horse_seq_timemod16)/sizeof(unsigned int), 0);
         break;
         
          case 17: // 145--ok
             hct_leds_horse(horse_seq_mod17, sizeof(horse_seq_mod17)/sizeof(unsigned int), horse_seq_timemod17, sizeof(horse_seq_timemod17)/sizeof(unsigned int), 0);
         break;      
         
         defult:
            break;
            
    }
    AW_LEDS_DEBUG("---------------%s:finished \n",__func__);

    
}




static void hct_led_breath_setting(void)
{
    AW_LEDS_DEBUG("%s:setmode =%d \n",__func__, issi_led_function_mode);
    if(issi_led_function_mode<0)
    {
        AW_LEDS_DEBUG("%s: %d,this is wrong mode, pls check \n",__func__, issi_led_function_mode);
        goto exit;
        
    }
    issi_led_function_breath_mode = issi_led_function_mode;
    issi_led_function_horse_mode=-1;
    aw_led_work_status =LED_SOFT_BREATHING;
    wake_up_aw_thread();

exit:
    AW_LEDS_DEBUG("---------------%s:finished \n",__func__);

    
}

static void hct_led_horse_setting(void)
{
    AW_LEDS_DEBUG("%s:setmode =%d \n",__func__, issi_led_function_mode);
    if(issi_led_function_mode<0)
    {
        AW_LEDS_DEBUG("%s: %d,this is wrong mode, pls check \n",__func__, issi_led_function_mode);
        goto exit;
        
    }
    issi_led_function_horse_mode= issi_led_function_mode;
    issi_led_function_breath_mode=-1;
    aw_led_work_status =LED_HORSE_RAISING;
    wake_up_aw_thread();

exit:
    AW_LEDS_DEBUG("---------------%s:finished \n",__func__);

    
}


static void hct_led_working_stop(void)
{
    AW_LEDS_DEBUG("%s:setmode =%d \n",__func__, issi_led_function_mode);

    aw_hct_all_effect_stop();
    
    issi_led_function_horse_mode= -1;
    issi_led_function_breath_mode=-1;
    aw_led_work_status =LED_IDLE_STOP;
    wake_up_aw_thread();

exit:
    AW_LEDS_DEBUG("---------------%s:finished \n",__func__);

    
}



void ISSI_led_function(int onoff,int mode,int led_brightness)
{
    AW_LEDS_DEBUG("%s:onff=%d, mode=%d====aw_s=%d==================\n",__func__, onoff,mode,aw9106_state);

    if((aw9106_state==1))
    {
        hct_led_working_stop();
    }

   aw9106_state = onoff;


/*  now setting is project related*/

    issi_led_function_mode=mode;
    switch(mode)
    {
         case 1:  // 129
         case 5:
         case 10:
            hct_led_breath_setting();
            break;

         case 2:
         case 3:
         case  6:
         case 7:   
         case 4: // 132
         case 11:
         case 12:
         case 13:
         case 14:
         case 15:         
         case 16:
         case 17:
             hct_led_horse_setting();
            break;
            
         default:  // 156
             hct_led_working_stop();
            break;
    }
    AW_LEDS_DEBUG("%s:finished ----==========\n",__func__);


/*  now setting is project related*/

}


////////////////////////////////////////////////////////////////


static s32 aw9106_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	i2c_client_aw9106 = client;
	printk("aw9106_i2c_probe \n");
	return 0;
}


static int aw9106_i2c_remove(struct i2c_client *client)
{
	return 0;
}


static int aw9106_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	return 0;
}



static struct i2c_driver aw9106_i2c_driver =
{
    .probe = aw9106_i2c_probe,
    .remove = aw9106_i2c_remove,
    .detect = aw9106_i2c_detect,
    .driver.name = "aw9106",
    .id_table = aw9106_i2c_id,
    .address_list = (const unsigned short *) forces,
};

static int AW9106_probe(struct platform_device *pdev) 
{
	//struct alsps_hw *hw = ap3216c_get_cust_alsps_hw();

	//AP3216C_power(hw, 1);
	printk("aw9106_probe \n");
#if 0//(LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))	    
	AP3216C_force[0] = hw->i2c_num;
	AP3216C_force[1] = hw->i2c_addr[0];
	APS_DBG("I2C = %d, addr =0x%x\n",AP3216C_force[0],AP3216C_force[1]);
#endif

	if(i2c_add_driver(&aw9106_i2c_driver))
	{
		i2c_del_driver(&aw9106_i2c_driver);
		printk("add driver error\n");;
		return -1;
	} 
    
	aw9106_queue = create_singlethread_workqueue("aw9106_timer_queue");
	INIT_WORK(&aw9106_work, hct_led_soft_breath_work);

	init_waitqueue_head(&soft_breath_wq);
        init_waitqueue_head(&soft_breath_finished_wq);
        init_waitqueue_head(&led_horse_wq);
        init_waitqueue_head(&hct_horse_work_finished_wq);
        
        kthread_run(aw_thread_kthread, NULL, "aw_thread_kthread");
	init_timer(&AW9106timer);
	AW9106timer.data = 0;

	printk(KERN_ERR"hct-alsps in func %s line %d\n",__func__,__LINE__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int AW9106_remove(struct platform_device *pdev)
{
	//struct alsps_hw *hw = ap3216c_get_cust_alsps_hw();
	//APS_FUN();    

	//AP3216C_power(hw, 0);    
	i2c_del_driver(&aw9106_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver AW9106_driver = {
	.probe      = AW9106_probe,
	.remove     = AW9106_remove,    
	.driver     = {
		.name  = "aw9106_dev",
//		.owner = THIS_MODULE,
	}
};

static struct platform_device AW9106_device = {
	.name	= "aw9106_dev",
	.id	= -1,
};


static int __init  AW9106_driver_init(void)
{
	 i2c_register_board_info(1, &i2c_aw9106, 1);

#if 0
	if (i2c_add_driver(&aw9106_i2c_driver) != 0)
	{
		printk("unable to add i2c AW9106_driver_init.\n");
		i2c_del_driver(&aw9106_i2c_driver);
		return -1;
	}
#endif

       if(platform_device_register(&AW9106_device))
       {
           printk("unable to register AW9106_device \n");
           return -ENODEV;
       }
	if(platform_driver_register(&AW9106_driver))
	{
		printk("unable to register AW9106_driver \n");
		return -ENODEV;
	}
    
	printk("AW9106_driver_init ok \n");
}


static void __exit  AW9106_driver_exit(void)
{
	i2c_del_driver(&aw9106_i2c_driver);
}


EXPORT_SYMBOL(ISSI_led_function);

module_init(AW9106_driver_init);
module_exit(AW9106_driver_exit);
#endif
