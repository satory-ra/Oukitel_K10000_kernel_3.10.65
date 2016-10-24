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
		
{0xB9,3,{0xFF,0x83,0x94}},
				
//  Set MIPI
{0xBA, 6, {0x63,0x03,0x68,0x6B,0xB2,0xC0}},
		        
//  B1-Set Power
{0xB1, 10, {0x50,0x12,0x72,0x09,0x33,0x54,0xB1,0x51,0x70,0x39}},
	
//  B2-Set DISP 
{0xB2, 7, {0x00,0x80,0x64,0X0E,0x09,0x2F,0x11}},
        // 0X02Îªµã·­×ª
//B4-Set GIP Timing
{0xB4, 21, {0x01,0x65,0x01,0x65,0x01,0x65,0x01,0x05,0x7E,0x35,0x00,0x3F,0x01,0x65,0x01,0x65,0x01,0x65,0x01,0x05,0x7E}},

// Set VCOM 
{0xB6, 2, {0x81,0x81}}, 

{0xD3, 34, {0x00,0x00,0x07,0x07,0x40,0x01,0x10,0x00,0x32,0x10,0x07,0x00,0x07,0x32,0x15,0x07,0x05,0x07,0x32,0x10,0x00,0x00,0x00,0x17,0x11,0x09,0x09,0x17,0x10,0x07,0x07,0x0E,0x40,0x18}},

  {0xbd, 1, {0x01}},

  {0xD3, 4, {0x00,0x04,0x01,0x02}},

  {0xbd, 1, {0x00}},

// D5, Forward scan
{0xD5, 44, {0x06 ,0x07 ,0x04 ,0x05 ,0x02 ,0x03 ,0x00 ,0x01 ,0x20 ,0x21 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x1A ,0x1A ,0x1B ,0x1B ,0x18 ,0x18 ,0x18 ,0x18 }},
	
// D6, GIP Backword			
{0xD6,44, {0x05 ,0x04 ,0x07 ,0x06 ,0x01 ,0x00 ,0x03 ,0x02 ,0x18 ,0x18 ,0x21 ,0x20 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x1A ,0x1A ,0x1,0x1B ,0x18 ,0x18 ,0x18 ,0x18 }},
		
{0xD8,12, {0xAA ,0xAA, 0xAA ,0xAA ,0xAA ,0xA0 ,0xAA ,0xAA ,0xAA ,0xAA ,0xAA ,0xA0 }},
  
{0xbd, 1, {0x01}},    
	
{0xD8,23, {0xD8,0xAA ,0xAA ,0xAA ,0xAA ,0xAA ,0xA0 ,0xAA ,0xAA ,0xAA ,0xAA ,0xAA ,0xA0 ,0xAA ,0xAA ,0xAA ,0xAA ,0xAA ,0xA0 ,0xAA ,0xAA ,0xAA ,0xAA}}, 

  
{0xE0, 58, {0x00 ,0x02 ,0x09 ,0x10 ,0x13 ,0x17 ,0x1B, 0x18 ,0x35 ,0x48 ,0x5B ,0x5A, 0x65 ,0x7A ,0x80 ,0x88 ,0x96 ,0x9D ,0x9A ,0xA7 ,0xB8 ,0x5F ,0x59 ,0x5C ,0x5F ,0x65 ,0x6C ,0x79 ,0x7F ,0x00 ,0x02 ,0x09 ,0x10 ,0x13 ,0x17 ,0x1B ,0x18 ,0x35 ,0x48 ,0x5B, 0x5A ,0x65 ,0x7A ,0x80,0x88 ,0x96 ,0x9D ,0x9A ,0xA7 ,0xB8 ,0x5F ,0x59 ,0x5C ,0x5F ,0x65 ,0x6C ,0x79 ,0x7F }},
       
{0xC0, 2, {0x1F ,0x73 }},

{0xCC, 1, {0x03}}, //0b		      

{0xD4, 1, {0x02}},

{0xbd, 1, {0x02}},
    	
{0xD8, 12, {0xD8 ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xF0 ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xF0 }},
       	
{0xbd, 1, {0x00}},

{0xbd, 1, {0x01}},

{0xbd, 1, {0x60}},

{0xbd, 1, {0x00}},

{0x11,  1,  {0x00}},
{REGFLAG_DELAY, 200, {0}},
  
{0xbF, 7, {0x00,0x40 ,0x81 ,0x50 ,0x02 ,0x1A ,0xFC ,0x02}}, 

{0x29,  0,  {0x00}},
{REGFLAG_DELAY, 200, {0}},
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},
	// Display ON
	//{0x2C, 1, {0x00}},
	//{0x13, 1, {0x00}},
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
	params->dsi.vertical_sync_active				= 3;//5
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 10;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 38;
	params->dsi.horizontal_backporch				= 38;//66
	params->dsi.horizontal_frontporch				= 38;//70
	//params->dsi.horizontal_blanking_pixel		       		= 60;
	params->dsi.horizontal_active_pixel		       		= FRAME_WIDTH;
	// Bit rate calculation

	params->dsi.PLL_CLOCK=190;//227;//254;//254//247  240
}

static unsigned int lcm_init_resgister(void)
{
    unsigned int data_array[16];
  
    data_array[0] = 0x00043902;
    data_array[1] = 0x9483ffb9;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00073902;
    data_array[1] = 0x680363ba;
    data_array[2] = 0x00c0b26b;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x000b3902;
    data_array[1] = 0x761650b1;
    data_array[2] = 0xb1543109;
    data_array[3] = 0x00396b31;
    dsi_set_cmdq(data_array, 4, 1);

    data_array[0] = 0x00083902;
    data_array[1] = 0x648000b2;
    data_array[2] = 0x112f080d;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00163902;
    data_array[1] = 0x016501b4;
    data_array[2] = 0x04650165;
    data_array[3] = 0x00238e08;
    data_array[4] = 0x0165013f;
    data_array[5] = 0x04650165;
    data_array[6] = 0x00008e08;
    dsi_set_cmdq(data_array, 7, 1);

    data_array[0] = 0x00033902;
    data_array[1] = 0x007777b6;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x003b3902;
    data_array[1] = 0x18100be0;
    data_array[2] = 0x27231f1d;
    data_array[3] = 0x705d4d25;
    data_array[4] = 0x95938171;
    data_array[5] = 0x96979890;
    data_array[6] = 0x6161c1aa;
    data_array[7] = 0x63616160;
    data_array[8] = 0x100b686a;
    data_array[9] = 0x231f1d18;
    data_array[10] = 0x5d4d2527;
    data_array[11] = 0x86726f6f;
    data_array[12] = 0xb0ae978d;
    data_array[13] = 0x5bbdb2a9;
    data_array[14] = 0x67615d57;
    data_array[15] = 0x007a796f;
    dsi_set_cmdq(data_array, 16, 1);

    data_array[0] = 0x00033902;
    data_array[1] = 0x00731fc0;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x0000aad2;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x00000bcc;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000003d4;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00233902;
    data_array[1] = 0x0f0000d3;
    data_array[2] = 0x1001400f;
    data_array[3] = 0x06103200;
    data_array[4] = 0x15320600;
    data_array[5] = 0x32060506;
    data_array[6] = 0x00000010;
    data_array[7] = 0x08081117;
    data_array[8] = 0x07071017;
    data_array[9] = 0x0018400d;
    dsi_set_cmdq(data_array, 10, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000002bd;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00053902;
    data_array[1] = 0x010400d3;
    data_array[2] = 0x00000002;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000000bd;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x002d3902;
    data_array[1] = 0x040706d5;
    data_array[2] = 0x00030205;
    data_array[3] = 0x18212001;
    data_array[4] = 0x18181818;
    data_array[5] = 0x18181818;
    data_array[6] = 0x18181818;
    data_array[7] = 0x18181818;
    data_array[8] = 0x18181818;
    data_array[9] = 0x18181818;;
    data_array[10] = 0x1b1a1a18;
    data_array[11] = 0x1818181b;
    data_array[12] = 0x00000018;
    dsi_set_cmdq(data_array, 13, 1);

    data_array[0] = 0x002d3902;
    data_array[1] = 0x070405d6;
    data_array[2] = 0x03000106;
    data_array[3] = 0x21181802;
    data_array[4] = 0x18181820;
    data_array[5] = 0x18181818;
    data_array[6] = 0x18181818;
    data_array[7] = 0x18181818;
    data_array[8] = 0x18181818;
    data_array[9] = 0x18181818;;
    data_array[10] = 0x1b1a1a18;
    data_array[11] = 0x1818181b;
    data_array[12] = 0x00000018;
    dsi_set_cmdq(data_array, 13, 1);

    data_array[0] = 0x000d3902;
    data_array[1] = 0xaaaaaad8;
    data_array[2] = 0xaaa0aaaa;
    data_array[3] = 0xaaaaaaaa;
    data_array[4] = 0x000000a0;
    dsi_set_cmdq(data_array, 5, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000001bd;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00193902;
    data_array[1] = 0xaaaaaad8;
    data_array[2] = 0xaaa0aaaa;
    data_array[3] = 0xaaaaaaaa;
    data_array[4] = 0xaaaaaaa0;
    data_array[5] = 0xaaa0aaaa;
    data_array[6] = 0xaaaaaaaa;
    data_array[7] = 0x000000a0;
    dsi_set_cmdq(data_array, 8, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000002bd;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x000d3902;
    data_array[1] = 0xffffffd8;
    data_array[2] = 0xfff0ffff;
    data_array[3] = 0xffffffff;
    data_array[4] = 0x000000f0;
    dsi_set_cmdq(data_array, 5, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000000bd;
    dsi_set_cmdq(data_array, 2, 1);


    data_array[0] = 0x00023902;
    data_array[1] = 0x000001bd;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000060b1;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000000bd;
    dsi_set_cmdq(data_array, 2, 1);


    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(200);
	
    data_array[0] = 0x00083902;
    data_array[1] = 0x508140bf;
    data_array[2] = 0x01fc1a00;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);
    ///////////////////////////
};


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
   // push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
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
	//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);	//wqtao. enable
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
	printf("hct_hx8394f_dsi_vdo_hd_xp adc = %x adcdata= %x %x, ret=%d, ADC_MIN_VALUE=%x\r\n",rawdata, adcdata[0], adcdata[1],ret, ADC_MIN_VALUE);
#else
	printk("hct_hx8394f_dsi_vdo_hd_xp adc = %x adcdata= %x %x, ret=%d, ADC_MIN_VALUE=%x\r\n",rawdata, adcdata[0], adcdata[1],ret, ADC_MIN_VALUE);
#endif

	if(rawdata < ADC_MIN_VALUE && (ret==0))
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
LCM_DRIVER hct_hx8394f_dsi_vdo_hd_xp = 
{
	.name			  = "hct_hx8394f_dsi_vdo_hd_xp",
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

