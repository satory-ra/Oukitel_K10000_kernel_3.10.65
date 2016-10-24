
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

#define FRAME_WIDTH  										(1080)
#define FRAME_HEIGHT 										(1920)
#define LCM_ID_NT35596 										(0x96)
#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#define   LCM_DSI_CMD_MODE							0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

#define changcmd(a,b,c,d) ((d<<24)|(c<<16)|(b<<8)|a)

#define   LCM_DSI_CMD_MODE							0

void NT35596_DCS_write_1A_1P(unsigned char cmd, unsigned char para)
{
	unsigned int data_array[16];

	data_array[0] = (0x00023902);
	data_array[1] = (0x00000000 | (para << 8) | (cmd));
	dsi_set_cmdq(data_array, 2, 1);

}

void NT35596_DCS_write_1A_0P(unsigned char cmd)
{
	unsigned int data_array[16];

	data_array[0]=(0x00000500 | (cmd<<16));
	dsi_set_cmdq(data_array, 1, 1);

}

static void init_lcm_registers(void)
{
//LCD driver initialization

NT35596_DCS_write_1A_1P(0xFF,0x05);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0xc5,0x01);
MDELAY(100);
NT35596_DCS_write_1A_1P(0xFF,0xEE);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0x1F,0x45);
NT35596_DCS_write_1A_1P(0x24,0x4F);
NT35596_DCS_write_1A_1P(0x38,0xC8);
NT35596_DCS_write_1A_1P(0x39,0x2C);
NT35596_DCS_write_1A_1P(0x1E,0xBB);
NT35596_DCS_write_1A_1P(0x1D,0x0F);
NT35596_DCS_write_1A_1P(0x7E,0xB1);
NT35596_DCS_write_1A_1P(0xFF,0x00);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0x35,0x01);
NT35596_DCS_write_1A_1P(0xFF,0x01);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0x00,0x01);
NT35596_DCS_write_1A_1P(0x01,0x44);//55
NT35596_DCS_write_1A_1P(0x02,0x40);
NT35596_DCS_write_1A_1P(0x05,0x50);//40
NT35596_DCS_write_1A_1P(0x06,0x4A);//1B
NT35596_DCS_write_1A_1P(0x07,0x24);
NT35596_DCS_write_1A_1P(0x08,0x0C);
NT35596_DCS_write_1A_1P(0x0B,0x87);
NT35596_DCS_write_1A_1P(0x0C,0x87);
NT35596_DCS_write_1A_1P(0x0E,0xB0);
NT35596_DCS_write_1A_1P(0x0F,0xB3);
NT35596_DCS_write_1A_1P(0x11,0x10);
NT35596_DCS_write_1A_1P(0x12,0x10);
NT35596_DCS_write_1A_1P(0x13,0x05);
NT35596_DCS_write_1A_1P(0x14,0x4A);
NT35596_DCS_write_1A_1P(0x15,0x18);
NT35596_DCS_write_1A_1P(0x16,0x18);
NT35596_DCS_write_1A_1P(0x18,0x00);
NT35596_DCS_write_1A_1P(0x19,0x77);
NT35596_DCS_write_1A_1P(0x1A,0x55);
NT35596_DCS_write_1A_1P(0x1B,0x13);
NT35596_DCS_write_1A_1P(0x1C,0x00);
NT35596_DCS_write_1A_1P(0x1D,0x00);
NT35596_DCS_write_1A_1P(0x1E,0x13);
NT35596_DCS_write_1A_1P(0x1F,0x00);
NT35596_DCS_write_1A_1P(0x23,0x00);
NT35596_DCS_write_1A_1P(0x24,0x00);
NT35596_DCS_write_1A_1P(0x25,0x00);
NT35596_DCS_write_1A_1P(0x26,0x00);
NT35596_DCS_write_1A_1P(0x27,0x00);
NT35596_DCS_write_1A_1P(0x28,0x00);
NT35596_DCS_write_1A_1P(0x35,0x00);
NT35596_DCS_write_1A_1P(0x66,0x00);
NT35596_DCS_write_1A_1P(0x58,0x82);
NT35596_DCS_write_1A_1P(0x59,0x02);
NT35596_DCS_write_1A_1P(0x5a,0x02);
NT35596_DCS_write_1A_1P(0x5B,0x02);
NT35596_DCS_write_1A_1P(0x5C,0x82);
NT35596_DCS_write_1A_1P(0x5D,0x82);
NT35596_DCS_write_1A_1P(0x5E,0x02);
NT35596_DCS_write_1A_1P(0x5F,0x02);
NT35596_DCS_write_1A_1P(0x72,0x31);
NT35596_DCS_write_1A_1P(0xFF,0x05);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0x00,0x01);
NT35596_DCS_write_1A_1P(0x01,0x0B);
NT35596_DCS_write_1A_1P(0x02,0x0C);
NT35596_DCS_write_1A_1P(0x03,0x09);
NT35596_DCS_write_1A_1P(0x04,0x0a);
NT35596_DCS_write_1A_1P(0x05,0x00);
NT35596_DCS_write_1A_1P(0x06,0x0F);
NT35596_DCS_write_1A_1P(0x07,0x10);
NT35596_DCS_write_1A_1P(0x08,0x00);
NT35596_DCS_write_1A_1P(0x09,0x00);
NT35596_DCS_write_1A_1P(0x0A,0x00);
NT35596_DCS_write_1A_1P(0x0B,0x00);
NT35596_DCS_write_1A_1P(0x0C,0x00);
NT35596_DCS_write_1A_1P(0x0d,0x13);
NT35596_DCS_write_1A_1P(0x0e,0x15);
NT35596_DCS_write_1A_1P(0x0f,0x17);
NT35596_DCS_write_1A_1P(0x10,0x01);
NT35596_DCS_write_1A_1P(0x11,0x0b);
NT35596_DCS_write_1A_1P(0x12,0x0c);
NT35596_DCS_write_1A_1P(0x13,0x09);
NT35596_DCS_write_1A_1P(0x14,0x0a);
NT35596_DCS_write_1A_1P(0x15,0x00);
NT35596_DCS_write_1A_1P(0x16,0x0f);
NT35596_DCS_write_1A_1P(0x17,0x10);
NT35596_DCS_write_1A_1P(0x18,0x00);
NT35596_DCS_write_1A_1P(0x19,0x00);
NT35596_DCS_write_1A_1P(0x1a,0x00);
NT35596_DCS_write_1A_1P(0x1b,0x00);
NT35596_DCS_write_1A_1P(0x1c,0x00);
NT35596_DCS_write_1A_1P(0x1d,0x13);
NT35596_DCS_write_1A_1P(0x1e,0x15);
NT35596_DCS_write_1A_1P(0x1f,0x17);
NT35596_DCS_write_1A_1P(0x20,0x00);
NT35596_DCS_write_1A_1P(0x21,0x03);
NT35596_DCS_write_1A_1P(0x22,0x01);
NT35596_DCS_write_1A_1P(0x23,0x40);
NT35596_DCS_write_1A_1P(0x24,0x40);
NT35596_DCS_write_1A_1P(0x25,0xed);
NT35596_DCS_write_1A_1P(0x29,0x58);
NT35596_DCS_write_1A_1P(0x2a,0x12);
NT35596_DCS_write_1A_1P(0x2b,0x01);
NT35596_DCS_write_1A_1P(0x4b,0x06);
NT35596_DCS_write_1A_1P(0x4c,0x11);
NT35596_DCS_write_1A_1P(0x4d,0x20);
NT35596_DCS_write_1A_1P(0x4e,0x02);
NT35596_DCS_write_1A_1P(0x4f,0x02);
NT35596_DCS_write_1A_1P(0x50,0x20);
NT35596_DCS_write_1A_1P(0x51,0x61);
NT35596_DCS_write_1A_1P(0x52,0x01);
NT35596_DCS_write_1A_1P(0x53,0x63);
NT35596_DCS_write_1A_1P(0x54,0x77);
NT35596_DCS_write_1A_1P(0x55,0xed);
NT35596_DCS_write_1A_1P(0x5b,0x00);
NT35596_DCS_write_1A_1P(0x5c,0x00);
NT35596_DCS_write_1A_1P(0x5d,0x00);
NT35596_DCS_write_1A_1P(0x5e,0x00);
NT35596_DCS_write_1A_1P(0x5f,0x15);
NT35596_DCS_write_1A_1P(0x60,0x75);
NT35596_DCS_write_1A_1P(0x61,0x00);
NT35596_DCS_write_1A_1P(0x62,0x00);
NT35596_DCS_write_1A_1P(0x63,0x00);
NT35596_DCS_write_1A_1P(0x64,0x00);
NT35596_DCS_write_1A_1P(0x65,0x00);
NT35596_DCS_write_1A_1P(0x66,0x00);
NT35596_DCS_write_1A_1P(0x67,0x00);
NT35596_DCS_write_1A_1P(0x68,0x04);
NT35596_DCS_write_1A_1P(0x69,0x00);
NT35596_DCS_write_1A_1P(0x6a,0x00);
NT35596_DCS_write_1A_1P(0x6c,0x40);
NT35596_DCS_write_1A_1P(0x75,0x01);
NT35596_DCS_write_1A_1P(0x76,0x01);
NT35596_DCS_write_1A_1P(0x7a,0x80);
NT35596_DCS_write_1A_1P(0x7b,0xc5);
NT35596_DCS_write_1A_1P(0x7c,0xd8);
NT35596_DCS_write_1A_1P(0x7d,0x60);
NT35596_DCS_write_1A_1P(0x7f,0x15);//10
NT35596_DCS_write_1A_1P(0x80,0x81);
NT35596_DCS_write_1A_1P(0x83,0x05);
NT35596_DCS_write_1A_1P(0x93,0x08);
NT35596_DCS_write_1A_1P(0x94,0x10);
NT35596_DCS_write_1A_1P(0x8a,0x00);
NT35596_DCS_write_1A_1P(0x9b,0x0f);
NT35596_DCS_write_1A_1P(0xea,0xff);
NT35596_DCS_write_1A_1P(0xec,0x00);
//////GAMMA///////////////////////////
NT35596_DCS_write_1A_1P(0xff,0x01);
NT35596_DCS_write_1A_1P(0xfb,0x01);
NT35596_DCS_write_1A_1P(0x75,0x00);
NT35596_DCS_write_1A_1P(0x76,0x18);
NT35596_DCS_write_1A_1P(0x77,0x00);
NT35596_DCS_write_1A_1P(0x78,0x38);
NT35596_DCS_write_1A_1P(0x79,0x00);
NT35596_DCS_write_1A_1P(0x7a,0x65);
NT35596_DCS_write_1A_1P(0x7b,0x00);
NT35596_DCS_write_1A_1P(0x7c,0x84);
NT35596_DCS_write_1A_1P(0x7d,0x00);
NT35596_DCS_write_1A_1P(0x7e,0x9b);
NT35596_DCS_write_1A_1P(0x7f,0x00);
NT35596_DCS_write_1A_1P(0x80,0xaf);
NT35596_DCS_write_1A_1P(0x81,0x00);
NT35596_DCS_write_1A_1P(0x82,0xc1);
NT35596_DCS_write_1A_1P(0x83,0x00);
NT35596_DCS_write_1A_1P(0x84,0xd2);
NT35596_DCS_write_1A_1P(0x85,0x00);
NT35596_DCS_write_1A_1P(0x86,0xdf);
NT35596_DCS_write_1A_1P(0x87,0x01);
NT35596_DCS_write_1A_1P(0x88,0x11);
NT35596_DCS_write_1A_1P(0x89,0x01);
NT35596_DCS_write_1A_1P(0x8a,0x38);
NT35596_DCS_write_1A_1P(0x8b,0x01);
NT35596_DCS_write_1A_1P(0x8c,0x76);
NT35596_DCS_write_1A_1P(0x8d,0x01);
NT35596_DCS_write_1A_1P(0x8e,0xa7);
NT35596_DCS_write_1A_1P(0x8f,0x01);
NT35596_DCS_write_1A_1P(0x90,0xf3);
NT35596_DCS_write_1A_1P(0x91,0x02);
NT35596_DCS_write_1A_1P(0x92,0x2f);
NT35596_DCS_write_1A_1P(0x93,0x02);
NT35596_DCS_write_1A_1P(0x94,0x30);
NT35596_DCS_write_1A_1P(0x95,0x02);
NT35596_DCS_write_1A_1P(0x96,0x66);
NT35596_DCS_write_1A_1P(0x97,0x02);
NT35596_DCS_write_1A_1P(0x98,0xa0);
NT35596_DCS_write_1A_1P(0x99,0x02);
NT35596_DCS_write_1A_1P(0x9a,0xc5);
NT35596_DCS_write_1A_1P(0x9b,0x02);
NT35596_DCS_write_1A_1P(0x9c,0xf8);
NT35596_DCS_write_1A_1P(0x9d,0x03);
NT35596_DCS_write_1A_1P(0x9e,0x1b);
NT35596_DCS_write_1A_1P(0x9f,0x03);
NT35596_DCS_write_1A_1P(0xa0,0x46);
NT35596_DCS_write_1A_1P(0xa2,0x03);
NT35596_DCS_write_1A_1P(0xa3,0x52);
NT35596_DCS_write_1A_1P(0xa4,0x03);
NT35596_DCS_write_1A_1P(0xa4,0x03);
NT35596_DCS_write_1A_1P(0xa5,0x62);
NT35596_DCS_write_1A_1P(0xa6,0x03);
NT35596_DCS_write_1A_1P(0xa7,0x71);
NT35596_DCS_write_1A_1P(0xa9,0x03);
NT35596_DCS_write_1A_1P(0xaa,0x83);
NT35596_DCS_write_1A_1P(0xab,0x03);
NT35596_DCS_write_1A_1P(0xac,0x94);
NT35596_DCS_write_1A_1P(0xad,0x03);
NT35596_DCS_write_1A_1P(0xae,0xa3);
NT35596_DCS_write_1A_1P(0xaf,0x03);
NT35596_DCS_write_1A_1P(0xb0,0xad);
NT35596_DCS_write_1A_1P(0xb1,0x03);
NT35596_DCS_write_1A_1P(0xb2,0xcc);
NT35596_DCS_write_1A_1P(0xb3,0x00);
NT35596_DCS_write_1A_1P(0xb4,0x18);
NT35596_DCS_write_1A_1P(0xb5,0x00);
NT35596_DCS_write_1A_1P(0xb6,0x38);
NT35596_DCS_write_1A_1P(0xb7,0x00);
NT35596_DCS_write_1A_1P(0xb8,0x65);
NT35596_DCS_write_1A_1P(0xb9,0x00);
NT35596_DCS_write_1A_1P(0xba,0x84);
NT35596_DCS_write_1A_1P(0xbb,0x00);
NT35596_DCS_write_1A_1P(0xbc,0x9b);
NT35596_DCS_write_1A_1P(0xbd,0x00);
NT35596_DCS_write_1A_1P(0xbe,0xaf);
NT35596_DCS_write_1A_1P(0xbf,0x00);
NT35596_DCS_write_1A_1P(0xc0,0xc1);
NT35596_DCS_write_1A_1P(0xc1,0x00);
NT35596_DCS_write_1A_1P(0xc2,0xd2);
NT35596_DCS_write_1A_1P(0xc3,0x00);
NT35596_DCS_write_1A_1P(0xc4,0xdf);
NT35596_DCS_write_1A_1P(0xc5,0x01);
NT35596_DCS_write_1A_1P(0xc6,0x11);
NT35596_DCS_write_1A_1P(0xc7,0x01);
NT35596_DCS_write_1A_1P(0xc8,0x38);
NT35596_DCS_write_1A_1P(0xc9,0x01);
NT35596_DCS_write_1A_1P(0xca,0x76);
NT35596_DCS_write_1A_1P(0xcb,0x01);
NT35596_DCS_write_1A_1P(0xcc,0xa7);
NT35596_DCS_write_1A_1P(0xcd,0x01);
NT35596_DCS_write_1A_1P(0xce,0xf3);
NT35596_DCS_write_1A_1P(0xcf,0x02);
NT35596_DCS_write_1A_1P(0xd0,0x2f);
NT35596_DCS_write_1A_1P(0xd1,0x02);
NT35596_DCS_write_1A_1P(0xd2,0x30);
NT35596_DCS_write_1A_1P(0xd3,0x02);
NT35596_DCS_write_1A_1P(0xd4,0x66);
NT35596_DCS_write_1A_1P(0xd5,0x02);
NT35596_DCS_write_1A_1P(0xd6,0xa0);
NT35596_DCS_write_1A_1P(0xd7,0x02);
NT35596_DCS_write_1A_1P(0xd8,0xc5);
NT35596_DCS_write_1A_1P(0xd9,0x02);
NT35596_DCS_write_1A_1P(0xda,0xf8);
NT35596_DCS_write_1A_1P(0xdb,0x03);
NT35596_DCS_write_1A_1P(0x02,0x00);
NT35596_DCS_write_1A_1P(0xdc,0x1b);
NT35596_DCS_write_1A_1P(0xdd,0x03);
NT35596_DCS_write_1A_1P(0xde,0x46);
NT35596_DCS_write_1A_1P(0xdf,0x03);
NT35596_DCS_write_1A_1P(0xe0,0x52);
NT35596_DCS_write_1A_1P(0xe1,0x03);
NT35596_DCS_write_1A_1P(0xe2,0x62);
NT35596_DCS_write_1A_1P(0xe3,0x03);
NT35596_DCS_write_1A_1P(0xe4,0x71);
NT35596_DCS_write_1A_1P(0xe5,0x71);
NT35596_DCS_write_1A_1P(0xe6,0x83);
NT35596_DCS_write_1A_1P(0xe7,0x03);
NT35596_DCS_write_1A_1P(0xe8,0x94);
NT35596_DCS_write_1A_1P(0xe9,0x03);
NT35596_DCS_write_1A_1P(0xea,0xa3);
NT35596_DCS_write_1A_1P(0xeb,0x03);
NT35596_DCS_write_1A_1P(0xec,0xad);
NT35596_DCS_write_1A_1P(0xed,0x03);
NT35596_DCS_write_1A_1P(0xee,0xcc);
NT35596_DCS_write_1A_1P(0xef,0x00);
NT35596_DCS_write_1A_1P(0xf0,0x18);
NT35596_DCS_write_1A_1P(0xf1,0x00);
NT35596_DCS_write_1A_1P(0xf2,0x38);
NT35596_DCS_write_1A_1P(0xf3,0x00);
NT35596_DCS_write_1A_1P(0xf4,0x65);
NT35596_DCS_write_1A_1P(0xf5,0x00);
NT35596_DCS_write_1A_1P(0xf6,0x84);
NT35596_DCS_write_1A_1P(0xf7,0x00);
NT35596_DCS_write_1A_1P(0xf8,0x9b);
NT35596_DCS_write_1A_1P(0xf9,0x00);
NT35596_DCS_write_1A_1P(0xfa,0xaf);
////cmd 2
NT35596_DCS_write_1A_1P(0xff,0x02);
NT35596_DCS_write_1A_1P(0xfb,0x01);
NT35596_DCS_write_1A_1P(0x00,0x00);
NT35596_DCS_write_1A_1P(0x01,0xc1);
NT35596_DCS_write_1A_1P(0x02,0x00);
NT35596_DCS_write_1A_1P(0x03,0xd2);
NT35596_DCS_write_1A_1P(0x04,0x00);
NT35596_DCS_write_1A_1P(0x05,0xdf);
NT35596_DCS_write_1A_1P(0x06,0x01);
NT35596_DCS_write_1A_1P(0x07,0x11);
NT35596_DCS_write_1A_1P(0x08,0x01);
NT35596_DCS_write_1A_1P(0x09,0x38);
NT35596_DCS_write_1A_1P(0x0a,0x01);
NT35596_DCS_write_1A_1P(0x0b,0x76);
NT35596_DCS_write_1A_1P(0x0c,0x01);
NT35596_DCS_write_1A_1P(0x0d,0xa7);
NT35596_DCS_write_1A_1P(0x0e,0x01);
NT35596_DCS_write_1A_1P(0x0f,0xf3);
NT35596_DCS_write_1A_1P(0x10,0x02);
NT35596_DCS_write_1A_1P(0x11,0x2f);
NT35596_DCS_write_1A_1P(0x12,0x02);
NT35596_DCS_write_1A_1P(0x13,0x30);
NT35596_DCS_write_1A_1P(0x14,0x02);
NT35596_DCS_write_1A_1P(0x15,0x66);
NT35596_DCS_write_1A_1P(0x16,0x02);
NT35596_DCS_write_1A_1P(0x17,0xa0);
NT35596_DCS_write_1A_1P(0x18,0x02);
NT35596_DCS_write_1A_1P(0x02,0x00);
NT35596_DCS_write_1A_1P(0x19,0xc5);
NT35596_DCS_write_1A_1P(0x02,0x00);
NT35596_DCS_write_1A_1P(0x1a,0x02);
NT35596_DCS_write_1A_1P(0x1b,0xf8);
NT35596_DCS_write_1A_1P(0x1c,0x03);
NT35596_DCS_write_1A_1P(0x1d,0x1b);
NT35596_DCS_write_1A_1P(0x1e,0x03);
NT35596_DCS_write_1A_1P(0x1f,0x46);
NT35596_DCS_write_1A_1P(0x20,0x03);
NT35596_DCS_write_1A_1P(0x21,0x52);
NT35596_DCS_write_1A_1P(0x22,0x03);
NT35596_DCS_write_1A_1P(0x23,0x62);
NT35596_DCS_write_1A_1P(0x24,0x03);
NT35596_DCS_write_1A_1P(0x25,0x71);
NT35596_DCS_write_1A_1P(0x26,0x03);
NT35596_DCS_write_1A_1P(0x27,0x83);
NT35596_DCS_write_1A_1P(0x28,0x03);
NT35596_DCS_write_1A_1P(0x29,0x94);
NT35596_DCS_write_1A_1P(0x2a,0x03);
NT35596_DCS_write_1A_1P(0x2b,0xa3);
NT35596_DCS_write_1A_1P(0x2d,0x03);
NT35596_DCS_write_1A_1P(0x2f,0xad);
NT35596_DCS_write_1A_1P(0x30,0x03);
NT35596_DCS_write_1A_1P(0x31,0xcc);
NT35596_DCS_write_1A_1P(0x32,0x00);
NT35596_DCS_write_1A_1P(0x33,0x18);
NT35596_DCS_write_1A_1P(0x34,0x00);
NT35596_DCS_write_1A_1P(0x35,0x38);
NT35596_DCS_write_1A_1P(0x36,0x00);
NT35596_DCS_write_1A_1P(0x37,0x65);
NT35596_DCS_write_1A_1P(0x38,0x00);
NT35596_DCS_write_1A_1P(0x39,0x84);
NT35596_DCS_write_1A_1P(0x3a,0x00);
NT35596_DCS_write_1A_1P(0x3b,0x9b);
NT35596_DCS_write_1A_1P(0x3d,0x00);
NT35596_DCS_write_1A_1P(0x3f,0xaf);
NT35596_DCS_write_1A_1P(0x40,0x00);
NT35596_DCS_write_1A_1P(0x41,0xc1);
NT35596_DCS_write_1A_1P(0x02,0x00);
NT35596_DCS_write_1A_1P(0x42,0x00);
NT35596_DCS_write_1A_1P(0x43,0xd2);
NT35596_DCS_write_1A_1P(0x44,0x00);
NT35596_DCS_write_1A_1P(0x45,0xdf);
NT35596_DCS_write_1A_1P(0x46,0x01);
NT35596_DCS_write_1A_1P(0x47,0x11);
NT35596_DCS_write_1A_1P(0x48,0x01);
NT35596_DCS_write_1A_1P(0x49,0x38);
NT35596_DCS_write_1A_1P(0x4a,0x01);
NT35596_DCS_write_1A_1P(0x4b,0x76);
NT35596_DCS_write_1A_1P(0x4c,0x01);
NT35596_DCS_write_1A_1P(0x4d,0xa7);
NT35596_DCS_write_1A_1P(0x4e,0x01);
NT35596_DCS_write_1A_1P(0x4f,0xf3);
NT35596_DCS_write_1A_1P(0x50,0x02);
NT35596_DCS_write_1A_1P(0x51,0x2f);
NT35596_DCS_write_1A_1P(0x52,0x02);
NT35596_DCS_write_1A_1P(0x53,0x30);
NT35596_DCS_write_1A_1P(0x54,0x02);
NT35596_DCS_write_1A_1P(0x55,0x66);
NT35596_DCS_write_1A_1P(0x56,0x02);
NT35596_DCS_write_1A_1P(0x58,0xa0);
NT35596_DCS_write_1A_1P(0x59,0x02);
NT35596_DCS_write_1A_1P(0x5a,0xc5);
NT35596_DCS_write_1A_1P(0x5b,0x02);
NT35596_DCS_write_1A_1P(0x5c,0xf8);
NT35596_DCS_write_1A_1P(0x5d,0x03);
NT35596_DCS_write_1A_1P(0x5e,0x1b);
NT35596_DCS_write_1A_1P(0x5f,0x03);
NT35596_DCS_write_1A_1P(0x60,0x46);
NT35596_DCS_write_1A_1P(0x61,0x03);
NT35596_DCS_write_1A_1P(0x62,0x52);
NT35596_DCS_write_1A_1P(0x63,0x03);
NT35596_DCS_write_1A_1P(0x64,0x62);
NT35596_DCS_write_1A_1P(0x65,0x03);
NT35596_DCS_write_1A_1P(0x66,0x71);
NT35596_DCS_write_1A_1P(0x67,0x03);
NT35596_DCS_write_1A_1P(0x68,0x83);
NT35596_DCS_write_1A_1P(0x69,0x03);
NT35596_DCS_write_1A_1P(0x6a,0x94);
NT35596_DCS_write_1A_1P(0x6b,0x03);
NT35596_DCS_write_1A_1P(0x6c,0xa3);
NT35596_DCS_write_1A_1P(0x6d,0x03);
NT35596_DCS_write_1A_1P(0x6e,0xad);
NT35596_DCS_write_1A_1P(0x6f,0x03);
NT35596_DCS_write_1A_1P(0x70,0xcc);
NT35596_DCS_write_1A_1P(0x71,0x00);
NT35596_DCS_write_1A_1P(0x72,0x18);
NT35596_DCS_write_1A_1P(0x73,0x00);
NT35596_DCS_write_1A_1P(0x74,0x38);
NT35596_DCS_write_1A_1P(0x75,0x00);
NT35596_DCS_write_1A_1P(0x76,0x65);
NT35596_DCS_write_1A_1P(0x77,0x00);
NT35596_DCS_write_1A_1P(0x78,0x84);
NT35596_DCS_write_1A_1P(0x79,0x00);
NT35596_DCS_write_1A_1P(0x7a,0x9b);
NT35596_DCS_write_1A_1P(0x7b,0x00);
NT35596_DCS_write_1A_1P(0x7c,0xaf);
NT35596_DCS_write_1A_1P(0x7d,0x00);
NT35596_DCS_write_1A_1P(0x7e,0xc1);
NT35596_DCS_write_1A_1P(0x7f,0x00);
NT35596_DCS_write_1A_1P(0x80,0xd2);
NT35596_DCS_write_1A_1P(0x81,0x00);
NT35596_DCS_write_1A_1P(0x82,0xdf);
NT35596_DCS_write_1A_1P(0x83,0x01);
NT35596_DCS_write_1A_1P(0x84,0x11);
NT35596_DCS_write_1A_1P(0x85,0x01);
NT35596_DCS_write_1A_1P(0x86,0x38);
NT35596_DCS_write_1A_1P(0x87,0x01);
NT35596_DCS_write_1A_1P(0x88,0x76);
NT35596_DCS_write_1A_1P(0x89,0x01);
NT35596_DCS_write_1A_1P(0x8a,0xa7);
NT35596_DCS_write_1A_1P(0x8b,0x01);
NT35596_DCS_write_1A_1P(0x8c,0xf3);
NT35596_DCS_write_1A_1P(0x8d,0x02);
NT35596_DCS_write_1A_1P(0x8e,0x2f);
NT35596_DCS_write_1A_1P(0x8f,0x02);
NT35596_DCS_write_1A_1P(0x90,0x30);
NT35596_DCS_write_1A_1P(0x91,0x02);
NT35596_DCS_write_1A_1P(0x92,0x66);
NT35596_DCS_write_1A_1P(0x93,0x02);
NT35596_DCS_write_1A_1P(0x94,0xa0);
NT35596_DCS_write_1A_1P(0x95,0x02);
NT35596_DCS_write_1A_1P(0x96,0xc5);
NT35596_DCS_write_1A_1P(0x97,0x02);
NT35596_DCS_write_1A_1P(0x98,0xf8);
NT35596_DCS_write_1A_1P(0x99,0x03);
NT35596_DCS_write_1A_1P(0x02,0x00);
NT35596_DCS_write_1A_1P(0x9a,0x1b);
NT35596_DCS_write_1A_1P(0x9b,0x03);
NT35596_DCS_write_1A_1P(0x9c,0x46);
NT35596_DCS_write_1A_1P(0x9d,0x03);
NT35596_DCS_write_1A_1P(0x9e,0x52);
NT35596_DCS_write_1A_1P(0xa0,0x62);
NT35596_DCS_write_1A_1P(0xa2,0x03);
NT35596_DCS_write_1A_1P(0xa3,0x71);
NT35596_DCS_write_1A_1P(0xa4,0x03);
NT35596_DCS_write_1A_1P(0xa5,0x83);
NT35596_DCS_write_1A_1P(0xa6,0x03);
NT35596_DCS_write_1A_1P(0xa7,0x94);
NT35596_DCS_write_1A_1P(0xa8,0x03);
NT35596_DCS_write_1A_1P(0xa9,0x03);
NT35596_DCS_write_1A_1P(0xaa,0xa3);
NT35596_DCS_write_1A_1P(0xab,0x03);
NT35596_DCS_write_1A_1P(0xac,0xad);
NT35596_DCS_write_1A_1P(0xad,0x03);
NT35596_DCS_write_1A_1P(0xae,0xcc);
NT35596_DCS_write_1A_1P(0xaf,0x00);
NT35596_DCS_write_1A_1P(0xb0,0x18);
NT35596_DCS_write_1A_1P(0xb1,0x00);
NT35596_DCS_write_1A_1P(0xb2,0x38);
NT35596_DCS_write_1A_1P(0xb3,0x00);
NT35596_DCS_write_1A_1P(0xb4,0x65);
NT35596_DCS_write_1A_1P(0xb5,0x00);
NT35596_DCS_write_1A_1P(0xb6,0x84);
NT35596_DCS_write_1A_1P(0xb7,0x00);
NT35596_DCS_write_1A_1P(0xb8,0x9b);
NT35596_DCS_write_1A_1P(0xb9,0x00);
NT35596_DCS_write_1A_1P(0xba,0xaf);
NT35596_DCS_write_1A_1P(0xbb,0x00);
NT35596_DCS_write_1A_1P(0xbc,0xc1);
NT35596_DCS_write_1A_1P(0xbd,0x00);
NT35596_DCS_write_1A_1P(0xbe,0xd2);
NT35596_DCS_write_1A_1P(0xbf,0x00);
NT35596_DCS_write_1A_1P(0xc0,0xdf);
NT35596_DCS_write_1A_1P(0xc1,0x01);
NT35596_DCS_write_1A_1P(0xc2,0x11);
NT35596_DCS_write_1A_1P(0xc3,0x01);
NT35596_DCS_write_1A_1P(0xc4,0x38);
NT35596_DCS_write_1A_1P(0xc5,0x01);
NT35596_DCS_write_1A_1P(0xc6,0x76);
NT35596_DCS_write_1A_1P(0xc7,0x01);
NT35596_DCS_write_1A_1P(0xc8,0xa7);
NT35596_DCS_write_1A_1P(0xc9,0x01);
NT35596_DCS_write_1A_1P(0xca,0xf3);
NT35596_DCS_write_1A_1P(0xcb,0x02);
NT35596_DCS_write_1A_1P(0xcc,0x2f);
NT35596_DCS_write_1A_1P(0xcd,0x02);
NT35596_DCS_write_1A_1P(0xce,0x30);
NT35596_DCS_write_1A_1P(0xcf,0x02);
NT35596_DCS_write_1A_1P(0xd0,0x66);
NT35596_DCS_write_1A_1P(0xd1,0x02);
NT35596_DCS_write_1A_1P(0xd2,0xa0);
NT35596_DCS_write_1A_1P(0xd3,0x02);
NT35596_DCS_write_1A_1P(0xd4,0xc5);
NT35596_DCS_write_1A_1P(0xd6,0xf8);
NT35596_DCS_write_1A_1P(0xd7,0x03);
NT35596_DCS_write_1A_1P(0xd8,0x1b);
NT35596_DCS_write_1A_1P(0xd9,0x03);
NT35596_DCS_write_1A_1P(0xda,0x46);
NT35596_DCS_write_1A_1P(0xdb,0x03);
NT35596_DCS_write_1A_1P(0xdc,0x52);
NT35596_DCS_write_1A_1P(0xde,0x62);
NT35596_DCS_write_1A_1P(0xdf,0x03);
NT35596_DCS_write_1A_1P(0xe0,0x71);
NT35596_DCS_write_1A_1P(0xe1,0x03);
NT35596_DCS_write_1A_1P(0xe2,0x83);
NT35596_DCS_write_1A_1P(0xe3,0x03);
NT35596_DCS_write_1A_1P(0xe4,0x94);
NT35596_DCS_write_1A_1P(0xe5,0x03);
NT35596_DCS_write_1A_1P(0xe6,0xa3);
NT35596_DCS_write_1A_1P(0xe7,0x03);
NT35596_DCS_write_1A_1P(0xe8,0xad);
NT35596_DCS_write_1A_1P(0xe9,0x03);
NT35596_DCS_write_1A_1P(0xea,0xcc);
////////GAMMA END////////

NT35596_DCS_write_1A_1P(0xff,0x01);
NT35596_DCS_write_1A_1P(0xfb,0x01);
NT35596_DCS_write_1A_1P(0xff,0x02);
NT35596_DCS_write_1A_1P(0xfb,0x01);
NT35596_DCS_write_1A_1P(0xff,0x04);
NT35596_DCS_write_1A_1P(0xfb,0x01);
NT35596_DCS_write_1A_1P(0xff,0x00);
NT35596_DCS_write_1A_1P(0xD3,0x05);//5
NT35596_DCS_write_1A_1P(0xD4,0x04);//4

NT35596_DCS_write_1A_0P(0x11); 
MDELAY(50);
NT35596_DCS_write_1A_1P(0xff,0x00);
NT35596_DCS_write_1A_1P(0x35,0x00);
NT35596_DCS_write_1A_0P(0x29);
MDELAY(10);

NT35596_DCS_write_1A_1P(0x51,0x00);
NT35596_DCS_write_1A_1P(0x5e,0x00);
NT35596_DCS_write_1A_1P(0x53,0x2c);
NT35596_DCS_write_1A_1P(0x55,0x00);

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
#if (LCM_DSI_CMD_MODE)
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
#endif

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode = SYNC_EVENT_VDO_MODE;
#endif
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 0;// 0;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=FRAME_WIDTH*3;	
		
	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 3;
	params->dsi.vertical_frontporch					= 20;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 118;
	params->dsi.horizontal_frontporch				= 118;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
   // params->dsi.TA_GO =5;
	//params->dsi.compatibility_for_nvk = 1;

	// Bit rate calculation
	params->dsi.PLL_CLOCK = 500;
    	params->dsi.cont_clock=0; //1;
	
}

static void lcm_init(void)
{
#if defined(GPIO_LCM_LDO3V3_EN_PIN)
    lcm_util.set_gpio_mode(GPIO_LCM_LDO3V3_EN_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO_LCM_LDO3V3_EN_PIN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCM_LDO3V3_EN_PIN, GPIO_OUT_ONE);
    MDELAY(20);
#endif

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

//	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	init_lcm_registers();

}



static void lcm_suspend(void)
{	
	unsigned int data_array[16];
    
	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(150);

    NT35596_DCS_write_1A_1P(0xFF,0x05);
    NT35596_DCS_write_1A_1P(0xFB,0x01);
    NT35596_DCS_write_1A_1P(0xC5,0x00);
	
#if defined(GPIO_LCM_LDO3V3_EN_PIN)
    lcm_util.set_gpio_mode(GPIO_LCM_LDO3V3_EN_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO_LCM_LDO3V3_EN_PIN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCM_LDO3V3_EN_PIN, GPIO_OUT_ZERO);
    MDELAY(20);
#endif
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
	unsigned char buffer[2];
	unsigned int array[16];
	
#if defined(GPIO_LCM_LDO3V3_EN_PIN)
    lcm_util.set_gpio_mode(GPIO_LCM_LDO3V3_EN_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO_LCM_LDO3V3_EN_PIN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCM_LDO3V3_EN_PIN, GPIO_OUT_ONE);
    MDELAY(20);
#endif

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID
    #ifdef BUILD_LK
		printf("%s, LK nt35596 debug: id = 0x%08x\n", __func__, id);
    #else
		printk("%s, kernel nt35596 debug: id = 0x%08x\n", __func__, id);
    #endif

    if(id == LCM_ID_NT35596)
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



LCM_DRIVER hct_nt35596_dsi_vdo_fhd_lg = 
{
	.name		= "hct_nt35596_dsi_vdo_fhd_lg",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};
