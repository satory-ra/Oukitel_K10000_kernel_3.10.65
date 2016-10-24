
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

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
    #include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                          (720)
#define FRAME_HEIGHT                                         (1280)
#define LCM_ID_R63317                                     (0x3317)
#define REGFLAG_DELAY                                         0xAB
#define REGFLAG_END_OF_TABLE                                  0xAA   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
//#define   LCM_DSI_CMD_MODE                            0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                             (lcm_util.udelay(n))
#define MDELAY(n)                                             (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)            lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                        lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                    lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                            lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

#define changcmd(a,b,c,d) ((d<<24)|(c<<16)|(b<<8)|a)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
/*                
{0xB0, 1,{0xFA}},
{0xB9, 2,{0x00,0x00}},
{0xB7, 2,{0x60,0x02}},
{0xDE, 2,{0x03,0x00}},
{0xD6, 2,{0x05,0x00}},
{0xB8, 2,{0x00,0x00}},
{0xB1, 2,{0x08,0x02}},
{0xB2, 2,{0x28,0x08}},
{0xB3, 2,{0x64,0x08}},
{0xB4, 2,{0xD0,0x02}},
{0xB5, 2,{0x00,0x05}},
{0xB6, 2,{0x07,0x00}},
{0xBA, 2,{0x06,0xC0}},
{0xBB, 2,{0x09,0x00}},
{0xB9, 2,{0x01,0x00}},
                   
{0xBD, 2,{0x00,0x00}},

{0xB0, 1,{0x00}},

{0xE3, 1,{0x01}},

{0xB3, 1,{0x00}},

{0xB6, 2,{0x41,0x01}},

{0xC0, 6,{0x09,0x48,0xAB,0x3A,0x02,0x7F}},
{0xC1, 21,{0x15,0x00,0x15,0x00,0x00,0x98,0x07,0x21,0x0A,0x07,0x21,0x0A,0x00,0x00,0x98,0x00,0x00,0x86,0x17,0x86,0x17}},
         
{0xC2, 23,{0x01,0x04,0x84,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0xC4, 3,{0xCA,0x36,0x00}},
{0xC6, 3,{0x41,0x55,0x70}},
{0xC7, 34,{0x54,0x97,0x22,0x03,0xC0,0xAA,0x00,0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x04,0x0D,0x14,0x10,0x28,0x08,0x2D,0x95,0x5C,0x63,0xAC,0xB9,0xA4,0x16,0x42,0x05,0x01,0x14,0x90,0x40}},
     
{0xC9, 30,{0x0C,0x19,0x1C,0x22,0x32,0x3E,0x48,0x57,0x3A,0x43,0x4F,0x61,0x69,0x77,0x7F,0x0C,0x19,0x1C,0x22,0x32,0x3E,0x48,0x57,0x3A,0x43,0x4F,0x61,0x69,0x77,0x7F}},
         
{0xD0, 10,{0x08,0x4C,0xC4,0x80,0x00,0x44,0x0C,0xB5,0x12,0x91}},
         
{0xD3, 25,{0x1B,0x33,0xBB,0xBB,0xB3,0x33,0x33,0x33,0x00,0x01,0x00,0xD0,0xB8,0xA0,0x08,0x4E,0x4E,0x33,0x33,0x1F,0xF2,0x07,0x3D,0xBF,0xDD}},
         
{0xD4, 7,{0x00,0x00,0x00,0x01,0x01,0x01,0x01}},
     
{0xC1, 2,{0x0a,0x00}},

 
{0xB7, 1,{0x80,0x02}},
{0xBC, 1,{0x01,0x00}},
{0xBF, 1,{0xbf}},*/

 
 
{0xB0,1,{0x00}}, 
 
 
{0xE3,1,{0x01}}, 
 
 
{0xB3,1,{0x00}}, 
 
 
{0xB6,2,{0x21,0x01}}, 
 
 
{0xC0,6,{0x09,0x48,0xAB,0x3A,0x02,0x7F}}, 
 
 
{0xC1,21,{0x15,0x00,0x15,0x00,0x00,0x98,0x07,0x21,0x0A,0x07,0x21,0x0A,0x00,0x00,0x98,0x00,0x00,0x86,0x17,0x86,0x17}}, 
 
 
 
{0xC2,23,{0x01,0x04,0x84,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
 
{0xC4,3,{0xCA,0x36,0x00}}, 
 
 
{0xC6,3,{0x41,0x55,0x70}}, 
 
 
{0xC7,34,{0x54,0x97,0x22,0x03,0xC0,0xAA,0x00,0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x04,0x0D,0x14,0x10,0x28,0x08,0x2D,0x95,0x5C,0x63,0xAC,0xB9,0xA4,0x16,0x42,0x05,0x01,0x14,0x90,0x40}}, 
 
 
{0xC9,30,{0x02,0x12,0x1A,0x24,0x32,0x40,0x4A,0x59,0x3D,0x46,0x53,0x61,0x6A,0x72,0x7F,0x02,0x12,0x1A,0x24,0x32,0x40,0x4A,0x59,0x3D,0x46,0x53,0x61,0x6A,0x72,0x7F}}, 
 
 
 
 
{0xD0,10,{0x08,0x4C,0xC4,0x80,0x00,0x44,0x0C,0xB5,0x12,0x91}}, 
 
 
{0xD3,25,{0x1B,0x33,0xBB,0xBB,0xB3,0x33,0x33,0x33,0x00,0x01,0x00,0xD0,0xB8,0xA0,0x08,0x4E,0x4E,0x33,0x33,0x1F,0xF2,0x07,0x3D,0xBF,0xDD}}, 
 
 
{0xD4,7,{0x06,0x00,0x00,0x00,0xfc,0x00,0xfb}}, //01 01 01 00 
 
 
{0xC1,2,{0x10,0x00}}, 
 
 
{0xB7,2,{0x80,0x02}}, 
 
{0xBC,2,{0x01,0x00}}, 
 
{0xBF,1,{0xbf}}, 

{0xff, 1, {0x00}},
{0xfa, 1, {0x00}},
{0xfa, 1, {0x00}},
{0xfa, 1, {0x00}},
{0xfa, 1, {0x00}},
{0xfa, 1, {0x00}},

//{0xff}; //单送一个FF
//{0xfa};//单送一个FA
//{0xfa};//单送一个FA
//{0xfa};//单送一个FA
//{0xfa};//单送一个FA
//{0xfa};//单送一个FA

    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 150, {}},
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},

// Setting ending by predefined flag
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Display ON
    //{0x2C, 1, {0x00}},
    //{0x13, 1, {0x00}},
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

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



static void init_lcm_registers(void)
{
    unsigned int data_array[16];
        data_array[0] = 0x00022902;                           
    data_array[1] = changcmd(0xB0,0x00,0x00,0x0);
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00022902;                           
    data_array[1] = changcmd(0xE3,0x01,0x00,0x0);
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00022902;                           
    data_array[1] = changcmd(0xB3,0x00,0x00,0x0);
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00032902;                           
    data_array[1] = changcmd(0xB6,0x21,0x01,0x0);
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00072902;                          
    data_array[1] = changcmd(0xC0,0x09,0x48,0xAB); 
    data_array[2] = changcmd(0x3A,0x02,0x7F,0x00);
    dsi_set_cmdq(data_array, 3, 1); 

    

    data_array[0] = 0x00162902;                          
    data_array[1] = changcmd(0xC1,0x15,0x00,0x15); 
    data_array[2] = changcmd(0x00,0x00,0x98,0x07);
    data_array[3] = changcmd(0x21,0x0A,0x07,0x21); 
    data_array[4] = changcmd(0x0A,0x00,0x00,0x98);
    data_array[5] = changcmd(0x00,0x00,0x86,0x17); 
    data_array[6] = changcmd(0x86,0x17,0x00,0x00);
    dsi_set_cmdq(data_array, 7, 1);

    data_array[0] = 0x00182902;                          
    data_array[1] = changcmd(0xC2,0x01,0x04,0x84); 
    data_array[2] = changcmd(0x00,0x00,0x00,0x00);
    data_array[3] = changcmd(0x00,0x00,0x00,0x00); 
    data_array[4] = changcmd(0x00,0x00,0x00,0x00);
    data_array[5] = changcmd(0x00,0x00,0x00,0x00); 
    data_array[6] = changcmd(0x00,0x00,0x00,0x00);
    dsi_set_cmdq(data_array, 7, 1);
    
    
    data_array[0] = 0x00042902;                          
    data_array[1] = changcmd(0xC4,0xCA,0x36,0x00); 
    dsi_set_cmdq(data_array, 2, 1);
    
    
    data_array[0] = 0x00042902;                 
    data_array[1] = changcmd(0xC6,0x41,0x55,0x70); 
    dsi_set_cmdq(data_array, 2, 1);
    
    
    data_array[0] = 0x00232902;                          
    data_array[1] = changcmd(0xC7,0x54,0x97,0x22);
    data_array[2] = changcmd(0x03,0xC0,0xAA,0x00); 
    data_array[3] = changcmd(0x00,0x00,0x00,0xC0);  
    data_array[4] = changcmd(0x00,0x00,0x00,0x04); 
    data_array[5] = changcmd(0x0D,0x14,0x10,0x28); 
    data_array[6] = changcmd(0x08,0x2D,0x95,0x5C); 
    data_array[7] = changcmd(0x63,0xAC,0xB9,0xA4); 
    data_array[8] = changcmd(0x16,0x42,0x05,0x01);
    data_array[9] = changcmd(0x14,0x90,0x40,0x00);  
    dsi_set_cmdq(data_array, 10, 1);

    data_array[0] = 0x001f2902;                          
    data_array[1] = changcmd(0xC9,0x02,0x12,0x1A);
    data_array[2] = changcmd(0x24,0x32,0x40,0x4A); 
    data_array[3] = changcmd(0x59,0x3D,0x46,0x53);  
    data_array[4] = changcmd(0x61,0x6A,0x72,0x7F); 
    data_array[5] = changcmd(0x02,0x12,0x1A,0x24); 
    data_array[6] = changcmd(0x32,0x40,0x4A,0x59); 
    data_array[7] = changcmd(0x3D,0x46,0x53,0x61); 
    data_array[8] = changcmd(0x6A,0x72,0x7F,0x00); 
    dsi_set_cmdq(data_array, 9, 1);

    data_array[0] = 0x000b2902;                          
    data_array[1] = changcmd(0xD0,0x08,0x4C,0xC4);
    data_array[2] = changcmd(0x80,0x00,0x44,0x0C); 
    data_array[3] = changcmd(0xB5,0x12,0x91,0x00);  
    dsi_set_cmdq(data_array, 4, 1);
    
    data_array[0] = 0x001a2902;                          
    data_array[1] = changcmd(0xD3,0x1B,0x33,0xBB); //HX5186C
    data_array[2] = changcmd(0xBB,0xB3,0x33,0x33); 
    data_array[3] = changcmd(0x33,0x00,0x01,0x00);  
    data_array[4] = changcmd(0xD0,0xB8,0xA0,0x08); 
    data_array[5] = changcmd(0x4E,0x4E,0x33,0x33); 
    data_array[6] = changcmd(0x1F,0xF2,0x07,0x3D);
    data_array[7] = changcmd(0xBF,0xDD,0x00,0x00); 
    dsi_set_cmdq(data_array, 8, 1);
    
    
    
    data_array[0] = 0x00082902;                     
    data_array[1] = changcmd(0xD4,0x06,0x00,0x00);
    data_array[2] = changcmd(0x01,0x01,0x01,0x00);
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00032902;                     
    data_array[1] = changcmd(0xc1,0x10,0x00,0x00);
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00032902;                     
    data_array[1] = changcmd(0xb7,0x80,0x02,0x00);
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00032902;                     
    data_array[1] = changcmd(0xbc,0x01,0x00,0x00);
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00022902;                     
    data_array[1] = changcmd(0xbf,0xbf,0x00,0x00);
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00110500; // Sleep Out
    dsi_set_cmdq(data_array, 1, 1);
    
    MDELAY(120);
    
    data_array[0] = 0x00290500; //set_display_on
    dsi_set_cmdq(data_array, 1, 1);
    
    MDELAY(20);
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
    params->dbi.te_mode             = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

    params->dsi.mode   = SYNC_EVENT_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting       
    params->dsi.intermediat_buffer_num = 2;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
        
    params->dsi.vertical_sync_active                = 2;
    params->dsi.vertical_backporch                    = 6;//4;//16;///8 13 17
    params->dsi.vertical_frontporch                    = 8;//16;///8
    params->dsi.vertical_active_line                = FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active                = 12;//8;
    params->dsi.horizontal_backporch                = 40;//40//60;
    params->dsi.horizontal_frontporch                = 100;//60;//60;//140;
    params->dsi.horizontal_blanking_pixel              = 60;
    params->dsi.horizontal_active_pixel                = FRAME_WIDTH;
   // params->dsi.TA_GO =5;
    //params->dsi.compatibility_for_nvk = 1;

    // Bit rate calculation

    params->dsi.PLL_CLOCK = 221;
      params->dsi.ssc_disable=1;

        params->dsi.cont_clock=0; //1;

    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20); 
    SET_RESET_PIN(1);
    MDELAY(30);

    //push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    init_lcm_registers();

}



static void lcm_suspend(void)
{
    //push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    unsigned int data_array[16];
    /*SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20); 
    SET_RESET_PIN(1);
    MDELAY(30);*/

    data_array[0] = 0x00280500;                           
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(30);    

    data_array[0] = 0x00100500;                           
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
/*
    data_array[0] = 0x00022902;
    data_array[1] = 0x000000b0;                            
    dsi_set_cmdq(data_array, 2, 1);
        

    data_array[0] = 0x00000500;                           
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0] = 0x00000500;                           
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0] = 0x00022902;
    data_array[1] = 0x000001b1;                           
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);
    */


    SET_RESET_PIN(0);
    MDELAY(20);
    
}


static void lcm_resume(void)
{
    lcm_init();
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);

    unsigned int data_array[16];

    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
    unsigned int id=0;
    unsigned char buffer[4];
    unsigned int array[16];  

    array[0] = 0x00013700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    
    read_reg_v2(0x3A, buffer, 2);
#ifdef BUILD_LK
        printf("%s, myf LK r63317 0A debug:  id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2],buffer[3]);
#else
        printk("%s,  myf kernel 0A r63317 horse debug: id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__,buffer[0],buffer[1],buffer[2],buffer[3]);
#endif

    array[0] = 0x00022902;                          
    array[1] = 0x000000b0; 
    dsi_set_cmdq(array, 2, 1);

    array[0] = 0x00043700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    
    read_reg_v2(0xbf, buffer, 4);
    id = buffer[2]<<8 | buffer[3]; //we only need ID
    #ifdef BUILD_LK
        printf("%s, myf LK r63317 debug: id=0x%08x,  id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2],buffer[3]);
    #else
        printk("%s,  myf kernel r63317 horse debug: id=0x%08x,id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__,id, buffer[0],buffer[1],buffer[2],buffer[3]);
    #endif
    #ifdef BUILD_LK
        printf("%s, LK nt35590 debug: nt35590 id = 0x%08x\n", __func__, id);
    #else
        printk("%s, kernel nt35590 horse debug: nt35590 id = 0x%08x\n", __func__, id);
    #endif

    if(id == LCM_ID_R63317)
        return 1;
    else
        return 0;
}


static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
    char  buffer[3];
    int   array[4];

    if(lcm_esd_test)
    {
        lcm_esd_test = FALSE;
        return TRUE;
    }

    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0x36, buffer, 1);
    if(buffer[0]==0x90)
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



LCM_DRIVER hct_r63317_dsi_vdo_hd_jdi = 
{
    .name        = "hct_r63317_dsi_vdo_hd_jdi",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    //.esd_check = lcm_esd_check,
    //.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};


   
