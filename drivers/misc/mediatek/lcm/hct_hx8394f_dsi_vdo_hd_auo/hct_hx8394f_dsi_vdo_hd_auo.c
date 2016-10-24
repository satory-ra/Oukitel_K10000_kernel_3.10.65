/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifdef BUILD_LK
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define LCM_ID                      								(0x94)

#define REGFLAG_DELAY             								(0XFE)
#define REGFLAG_END_OF_TABLE      								(0x100)	// END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    									(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 										(lcm_util.udelay(n))
#define MDELAY(n) 										(lcm_util.mdelay(n))

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
    {REGFLAG_DELAY, 150, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	//{0xf0, 5, {0x55, 0xaa, 0x52, 0x08, 0x01}},
	{0xB9, 3, {0xFF,0x83,0x94}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

	params->dsi.mode   = SYNC_PULSE_VDO_MODE;	//SYNC_PULSE_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine. 
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST; 
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      	= LCM_DSI_FORMAT_RGB888;
	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;
	// Video mode setting		
	params->dsi.intermediat_buffer_num 	= 2;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
        params->dsi.word_count=720 * 3;
	params->dsi.vertical_sync_active				= 2;//5
	params->dsi.vertical_backporch					= 5;
	params->dsi.vertical_frontporch					= 9;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
params->dsi.horizontal_sync_active				= 30;
	params->dsi.horizontal_backporch				= 64;//66
	params->dsi.horizontal_frontporch				= 64;//70
	//params->dsi.horizontal_blanking_pixel		       		= 60;
	params->dsi.horizontal_active_pixel		       		= FRAME_WIDTH;
	// Bit rate calculation

	params->dsi.PLL_CLOCK=225;//227;//254;//254//247 
	params->dsi.ssc_disable=0;

	params -> dsi.cont_clock=0;
	//params -> dsi.clk_lp_per_line_enable=1;//1=per frame 3=per line
	//params->dsi.noncont_clock = 1;
	//params->dsi.noncont_clock_period = 1; //1=per frame 3=per line

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable      = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 0x09;//09 ,45,d9
    params->dsi.lcm_esd_check_table[0].count = 3;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[0].para_list[2] = 0x06;
	params->dsi.lcm_esd_check_table[1].cmd = 0xd9;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[2].cmd = 		0x45;
	params->dsi.lcm_esd_check_table[2].count =		 2;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0x05;
	params->dsi.lcm_esd_check_table[2].para_list[1] = 0x0e;

}

static unsigned int lcm_init_resgister(void)
{
unsigned int data_array[16];

data_array[0] = 0x00043902;
data_array[1] = 0x9483FFB9;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00073902;
data_array[1] = 0x680363BA;
data_array[2] = 0x00C0B26B;
dsi_set_cmdq(&data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x000B3902;
data_array[1] = 0x721250B1;
data_array[2] = 0x71443209;
data_array[3] = 0x00354f31;
dsi_set_cmdq(&data_array, 4, 1);
MDELAY(1);

data_array[0] = 0x00073902;
data_array[1] = 0x648065B2;
data_array[2] = 0x002F0705;
dsi_set_cmdq(&data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x00163902;
data_array[1] = 0x016501B4;
data_array[2] = 0x01650165;
data_array[3] = 0x00357E05;
data_array[4] = 0x0165013F;
data_array[5] = 0x01650165;
data_array[6] = 0x00007E05;
dsi_set_cmdq(&data_array, 7, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x005353B6;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00223902;
data_array[1] = 0x0F0000D3;
data_array[2] = 0x1001010F;
data_array[3] = 0x00103210;
data_array[4] = 0x15320000;
data_array[5] = 0x32043504;
data_array[6] = 0x14051415;
data_array[7] = 0x00003337;
data_array[8] = 0x37000037;
data_array[9] = 0x00004005;
dsi_set_cmdq(&data_array, 10, 1);
MDELAY(1);

data_array[0] = 0x002D3902;
data_array[1] = 0x181818D5;
data_array[2] = 0x18181818;
data_array[3] = 0x18181818;
data_array[4] = 0x25181818;
data_array[5] = 0x18262724;
data_array[6] = 0x01040518;
data_array[7] = 0x03060700;
data_array[8] = 0x53565702;
data_array[9] = 0x55505152;
data_array[10] = 0x23202154;
data_array[11] = 0x18181822;
data_array[12] = 0x00000018;
dsi_set_cmdq(&data_array, 13, 1);
MDELAY(1);

data_array[0] = 0x002D3902;
data_array[1] = 0x181818D6;
data_array[2] = 0x18181818;
data_array[3] = 0x18181818;
data_array[4] = 0x22181818;
data_array[5] = 0x18212023;
data_array[6] = 0x06030218;
data_array[7] = 0x04010007;
data_array[8] = 0x14111005;
data_array[9] = 0x12171615;
data_array[10] = 0x24272613;
data_array[11] = 0x18181825;
data_array[12] = 0x00000018;
dsi_set_cmdq(&data_array, 13, 1);
MDELAY(1);

data_array[0] = 0x003B3902;
data_array[1] = 0x0B0300E0;
data_array[2] = 0x1B171411;
data_array[3] = 0x5A473419;
data_array[4] = 0x8F856E62;
data_array[5] = 0xA6AAA698;
data_array[6] = 0x6159AFB8;
data_array[7] = 0x67626261;
data_array[8] = 0x03007F67;
data_array[9] = 0x1714110B;
data_array[10] = 0x4734191B;
data_array[11] = 0x856E625A;
data_array[12] = 0xAAA6988F;
data_array[13] = 0x59AFB8A6;
data_array[14] = 0x62626161;
data_array[15] = 0x007F6767;
dsi_set_cmdq(&data_array, 16, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x00731FC0;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x00000bCC; //03
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000002D4;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00083902;
data_array[1] = 0x508140BF;
data_array[2] = 0x01FC1A00;
dsi_set_cmdq(&data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000001bd;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);
data_array[0] = 0x00023902;
data_array[1] = 0x000060b1;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);
data_array[0] = 0x00023902;
data_array[1] = 0x000000bd;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);
data_array[0] = 0x00023902;
data_array[1] = 0x00000035;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);
data_array[0] = 0x00110500;
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(150);

data_array[0] = 0x00290500;
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(20);
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

    lcm_init_resgister();
   // dsi_set_cmdq_V3(lcm_initialization_setting_V3, sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
  //	dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
    //push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
#ifndef BUILD_LK
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(30);
	SET_RESET_PIN(1);
	MDELAY(120);
//	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);	//wqtao. enable
#endif
}

static unsigned int lcm_compare_id(void);

static void lcm_resume(void)
{
#ifndef BUILD_LK
//        lcm_compare_id();
	lcm_init();
#endif
}



static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	int array[4];
	unsigned char buffer[2];

	if(lcm_esd_test)
	{
	    lcm_esd_test = FALSE;
	    return TRUE;
	}

	/// please notice: the max return packet size is 1
	/// if you want to change it, you can refer to the following marked code
	/// but read_reg currently only support read no more than 4 bytes....
	/// if you need to read more, please let BinHan knows.
	/*
	        unsigned int data_array[16];
	        unsigned int max_return_size = 1;
	        
	        data_array[0]= 0x00003700 | (max_return_size << 16);    
	        
	        dsi_set_cmdq(&data_array, 1, 1);
	*/

	array[0]=0x00043902;
	array[1]=0x9483FFB9;// page enable
	dsi_set_cmdq(array, 2, 1);
//		MDELAY(20);

	array[0]=0x00083902; 
	array[1]=0x009341BA;// page enable 
	array[2]=0x1800A416; 
	dsi_set_cmdq(array, 3, 1); 
//		MDELAY(10); 

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(array, 1, 1);
//		MDELAY(10);
	read_reg_v2(0x0a, buffer, 1);

	#ifndef BUILD_LK
	printk("[%s] hct_hx8394f_dsi_vdo_hd_* lcm esd check. arthur %x\n", __FUNCTION__, buffer[0]);
	#endif

	if(buffer[0] == 0x1c)
	{
	    return FALSE;
	}
	else
	{            
	    return TRUE;
	}
#endif
}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	return TRUE;
}
	
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#define AUX_IN0_LCD_ID	12
#define ADC_MIN_VALUE	0x800
//0				0
//47 / 147 * 4096 = 0x51D		0.575V
//100/ 147 * 4096 = 0xAE2		1.22V

static unsigned int lcm_compare_id(void)
{
	unsigned int id1 = 0;
	unsigned int id2 = 0;
	unsigned char buffer[2];
	unsigned int array[16];  
	int adcdata[4] = {0};
	int rawdata = 0;
	int ret = 0;
	ret = IMM_GetOneChannelValue(AUX_IN0_LCD_ID, adcdata, &rawdata);

#if defined(BUILD_LK)
	printf("hct_hx8394f_dsi_vdo_hd_auo adc = %x adcdata= %x %x, ret=%d, ADC_MIN_VALUE=%x\r\n",rawdata, adcdata[0], adcdata[1],ret, ADC_MIN_VALUE);
#else
	printk("hct_hx8394f_dsi_vdo_hd_auo adc = %x adcdata= %x %x, ret=%d, ADC_MIN_VALUE=%x\r\n",rawdata, adcdata[0], adcdata[1],ret, ADC_MIN_VALUE);
#endif

	if(rawdata > ADC_MIN_VALUE && (ret==0))
	{	 
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(120);//Must over 6 ms

	array[0] = 0x00043902;
    	array[1] = 0x9483ffb9;
    	dsi_set_cmdq(array, 2, 1);
	MDELAY(10); 

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);

	read_reg_v2(0xF4, buffer, 2);
	id1 = buffer[0]; //we only need ID
	id2 = buffer[1]; 

#ifdef BUILD_LK
	printf("[HX8394D]%s,  id = 0x%x\n", __func__, id1);
#else
	printk("[HX8394D]%s,  id = 0x%x\n", __func__, id1);
#endif

    return (LCM_ID == id1)?1:0;
	}
	else
		return 0;
	
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_hx8394f_dsi_vdo_hd_auo = 
{
	.name			  = "hct_hx8394f_dsi_vdo_hd_auo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,	
	.compare_id     = lcm_compare_id,	
//	.esd_check   = lcm_esd_check,	
//	.esd_recover   = lcm_esd_recover,	
//	.update         = lcm_update,
};

