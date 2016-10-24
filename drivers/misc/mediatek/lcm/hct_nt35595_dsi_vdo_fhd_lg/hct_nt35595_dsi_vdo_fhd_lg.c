/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

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
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (1080)
#define FRAME_HEIGHT                                        (1920)
#define LCM_ID                       (0x95)

#define REGFLAG_DELAY               (0XFE)
#define REGFLAG_END_OF_TABLE        (0x100) // END OF REGISTERS MARKER


#define LCM_DSI_CMD_MODE                                    0

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

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
    
 {0xFF,1,{0x10}},

{0xBB,1,{0x10}},
{0x3B,5,{0x03,0x0A,0x0A,0x0A,0x0A}},
{0x53,1,{0x24}}, //BCTRL,1,{DD,BL (when CABC On,1,{0x2C)
{0x55,1,{0x00}}, //Image_Enhancement,1,{CABC (1-UI,2-Still,3-Moving)
{0x5E,1,{0x00}}, //CABC minimum brightness
{0x11,1,{0x00}},
{REGFLAG_DELAY, 120, {}},                                          
{0xFF,1,{0x24}}, //CMD2 Page 4 Entrance
{REGFLAG_DELAY, 10, {}},  
{0xFB,1,{0x01}}, //Don't Reload MTP
//SRGB/SMX_REG/CTB/CRL
//{0x9D,1,{0xB0}}, //FWD 
{0x9D,1,{0xB0}}, //BWD//b6    
//Resolution Setting (1920x1080)
{0x72,1,{0x00}},
//BP/FP Setting for Command Mode (BP=4,1,{FP=4)
{0x93,1,{0x04}}, //FP
{0x94,1,{0x04}}, //BP
//Set Inversion Type (Pixel Column)
{0x9B,1,{0x0F}}, //Inversion (Column)
{0x8A,1,{0x33}},
//Less Transition Function Setting 
{0x86,1,{0x1B}}, //Alternation
{0x87,1,{0x39}}, //Alternation 
{0x88,1,{0x1B}}, //Alternation 
{0x89,1,{0x39}}, //Alternation
{0x8B,1,{0xF4}}, //MUX_LTF (Less Transition)  
{0x8C,1,{0x01}}, //Alternation
//RTN Setting
{0x90,1,{0x79}}, //RTNA_V
{0x91,1,{0x4C}}, //DIVB/RTNA_V/DIVA/RTNB/RTNA2/RTNA
{0x92,1,{0x79}}, //RTNA
{0x95,1,{0xE4}}, //RTNB
//IDLE Inversion ŒöÁ€
{0xDE,1,{0xFF}}, //Idle Mode Column Inversion
//Qualcomm decoder on
{0xDF,1,{0x82}}, //RTN setting for Qualcomm Decoder On 

//CGOUT Mapping 
{0x00,1,{0x0F}},
{0x01,1,{0x00}}, 
{0x02,1,{0x00}}, 
{0x03,1,{0x00}}, 
{0x04,1,{0x0B}}, 
{0x05,1,{0x0C}}, 
{0x06,1,{0x00}}, 
{0x07,1,{0x00 }},
{0x08,1,{0x00 }},
{0x09,1,{0x00}}, 
{0x0A,1,{0X03}}, 
{0x0B,1,{0X04}}, 
{0x0C,1,{0x01}}, 
{0x0D,1,{0x13}}, 
{0x0E,1,{0x15 }},
{0x0F,1,{0x17}}, 
{0x10,1,{0x0F}}, 
{0x11,1,{0x00}}, 
{0x12,1,{0x00 }},
{0x13,1,{0x00 }},
{0x14,1,{0x0B }},
{0x15,1,{0x0C }},
{0x16,1,{0x00}}, 
{0x17,1,{0x00}}, 
{0x18,1,{0x00}}, 
{0x19,1,{0x00}}, 
{0x1A,1,{0x03}}, 
{0x1B,1,{0X04}}, 
{0x1C,1,{0x01}}, 
{0x1D,1,{0x13}}, 
{0x1E,1,{0x15}}, 
{0x1F,1,{0x17}},
//Start Pulse related(GVST),1,{GVST Setting 
{0x20,1,{0x09}}, //Start Pulse related (GVST)
{0x21,1,{0x01}}, //STV_RISE_SEL 
{0x22,1,{0x00}}, //STV_FALL_SEL
{0x23,1,{0x00}}, //FS_REG
{0x24,1,{0x00}}, //FW_REG
{0x25,1,{0x6D}}, //What ? ==> GVST setting for power-on sequence (pls. set to 0x6D)
{0x26,1,{0x00}}, //FS_REG_V
{0x27,1,{0x00}}, //FW_REG_V
//GCK(GCLK1/GCLK2),1,{GCLK Setting
{0x2F,1,{0x02}}, //GCK(GCLK1/GCLK2)
{0x30,1,{0x04}}, //What ?  ==> Set H/L toogle line numbers of each GCK and GCK toogle sequence number
{0x31,1,{0x49}}, //What ?  ==> Set GCK Sequence in Forward/Backward scan
{0x32,1,{0x23}}, //What ?  ==> Set GCK Sequence in Forward/Backward scan
{0x33,1,{0x01}}, //STI_REG/SWI_REG/STI_REG_V/SWI_REG_V/GCK_VPM
{0x34,1,{0x00}}, //STI_REG
{0x35,1,{0x69}}, //SWI_REG
{0x36,1,{0x00}}, //What ?  ==> GCK power-on/off sequence setting
{0x37,1,{0x2D}}, //What ?  ==> GCK power-on/off sequence setting
{0x38,1,{0x08}}, //What ?  ==> GCK power-on/off sequence setting
{0x39,1,{0x00 }},//STI_REG_V
{0x3A,1,{0x69}}, //SWI_REG_V
//UD (U2D/D2U),1,{U2D,D2U 
{0x29,1,{0x58 }},//UD(U2D,1,{D2U)
{0x2A,1,{0x16}},
//CTRL (APO,1,{use CTRL1),1,{APO                               
{0x5B,1,{0x00 }},//CTRL (SPO,1,{use CTRL1)
{0x5F,1,{0x75 }},
{0x63,1,{0x00 }},
{0x67,1,{0x04}}, 
//MUX (SERx/xSELx),1,{MUX (¿¬±žŽë»ó) 
//{0x7A,1,{0x01}}, //What? MUX(SERx/xSELx) ==> Not used,1,{pls. remove it
{0x7B,1,{0x80}}, //What?                 ==> MUX sequence setting in Display Line
{0x7C,1,{0xD8}}, //What?                 ==> MUX power-on/off sequencesetting 
{0x7D,1,{0x60}}, //What?                 ==> MUX power-on/off sequencesetting
{0x7E,1,{0x10}}, //MUXS
{0x7F,1,{0x19}}, //MUXW
{0x80,1,{0x00}}, //SD_SHIFT_POS (MUX Rising EQ)
{0x81,1,{0x06}}, //MUXG1
{0x82,1,{0x03}}, //MUXG2
{0x83,1,{0x00}}, //EQT_POS (Source Rising EQ)
{0x84,1,{0x03}}, //SOEHT (Source OP enable Time) MUXG1+MUXG2-SOEHT>=4
{0x85,1,{0x07}}, //SDT_REG (Source Output Hold Time)
{0x74,1,{0x10}}, //MUXS_V (MUX Rising Portion)
{0x75,1,{0x19}}, //MUXW_V (MUX On Time)
{0x76,1,{0x06}}, //MUXG1_V (MUX non-overlap Time)
{0x77,1,{0x03}}, //MUXG2_V (MUX non-overlap Time / b wo RGB switch)
//Source Control
{0x78,1,{0x00}}, //SD_SHIFT_NEG (MUX Falling EQ)
{0x79,1,{0x00}}, //EQ_NEG (Source Falling EQ)
{0x99,1,{0x33}}, //PTA/SPDUMA
//{0xA0,1,{0x33}}, //PTB/SPDUMB
{0x98,1,{0x00}},
//LTPS Abnormal PWROFF Ctrl. 
{0xB3,1,{0x28}},
{0xB4,1,{0x05}}, 
{0xB5,1,{0x10}}, 
//CTRL PIN
//{0xC4,1,{0x24}}, //FTE_SEL/FTE1_SEL
//{0xC5,1,{0x30}}, //LEDPWM_SEL/VSOUT_SEL
//{0xC6,1,{0x00}}, //HSOUT_SEL

{0xFF,1,{0x20}}, //Page 0,1,{power-related setting    
{REGFLAG_DELAY, 10, {}},  
{0x00,1,{0x01}}, //Panel Type (Normally Black) 
{0x01,1,{0x55}}, //DCA_2 / DCA_3 
{0x02,1,{0x45}}, //DCA_4 / DCB_2 
{0x03,1,{0x55}}, //DCB_3 / DCB_4    
{0x05,1,{0x50}}, //VGH(2xAVDD) / VGL(AVEE-VCI1) Voltage  
{0x06,1,{0x9E}}, //VGH w/Clamping (+9V) 
{0x07,1,{0xA8}}, //VGL w/Clamping (-9V) 
{0x08,1,{0x0C}}, //VCI1_regulator/VCL_regulator/VCL_chargepump/BTA3(VCI/-VCI)     
{0x0B,1,{0x96}}, //VRHP 4.35V
{0x0C,1,{0x96}}, //VRHN 4.35V 
{0x0E,1,{0x00}}, //VGHO_REG  
{0x0F,1,{0x00}}, //VGLO_REG     
{0x11,1,{0x29}}, //Vcom  
{0x12,1,{0x29}}, //What ?            ==> VCOM level setting for Backward Scan
{0x13,1,{0x03}}, //Vcom Last Bit     
{0x14,1,{0x0A}}, //pre-regulating disable 
{0x15,1,{0x99}}, //AVDDR +5.4V(Off) 
{0x16,1,{0x99}}, //AVEER -5.4V(Off)
{0x6D,1,{0x44}}, //Source Bias Control     
{0x58,1,{0x05}}, //GCKR_EQT1 (EQ On HighBit 8) - GCLK
{0x59,1,{0x05}},//GCKR_EQT2           
{0x5A,1,{0x05}}, //GCKF_EQT1 
{0x5B,1,{0x05}}, //GCKF_EQT2 
{0x5C,1,{0x00}}, //MUXR_EQT1 (1EQ On HighBit 8) - MUX 
{0x5D,1,{0x00}}, //MUXR_EQT2 (2EQ On HighBit 8)  
{0x5E,1,{0x00}}, //MUXF_EQT1 
{0x5F,1,{0x00}}, //MUXF_EQT2

//PWM Control
{0x1B,1,{0x39}},
{0x1C,1,{0x39}},
{0x1D,1,{0x47}},

{0xFF,1,{0x20}}, //Page 0,1,{power-related setting    		
{REGFLAG_DELAY, 10, {}},  		
//R+		
{0x75,1,{0x00}},		
{0x76,1,{0x00}},		
{0x77,1,{0x00}},		
{0x78,1,{0x22}},		
{0x79,1,{0x00}},		
{0x7A,1,{0x46}},		
{0x7B,1,{0x00}},		
{0x7C,1,{0x5C}},		
{0x7D,1,{0x00}},		
{0x7E,1,{0x76}},		
{0x7F,1,{0x00}},		
{0x80,1,{0x8D}},		
{0x81,1,{0x00}},		
{0x82,1,{0xA6}},		
{0x83,1,{0x00}},		
{0x84,1,{0xB8}},		
{0x85,1,{0x00}},		
{0x86,1,{0xC7}},		
{0x87,1,{0x00}},		
{0x88,1,{0xF6}},		
{0x89,1,{0x01}},		
{0x8A,1,{0x1D}},		
{0x8B,1,{0x01}},		
{0x8C,1,{0x54}},		
{0x8D,1,{0x01}},		
{0x8E,1,{0x81}},		
{0x8F,1,{0x01}},		
{0x90,1,{0xCB}},		
{0x91,1,{0x02}},		
{0x92,1,{0x05}},		
{0x93,1,{0x02}},	
{0x94,1,{0x07}},		
{0x95,1,{0x02}},	
{0x96,1,{0x47}},		
{0x97,1,{0x02}},		
{0x98,1,{0x82}},		
{0x99,1,{0x02}},		
{0x9A,1,{0xAB}},		
{0x9B,1,{0x02}},		
{0x9C,1,{0xDC}},		
{0x9D,1,{0x03}},		
{0x9E,1,{0x01}},		
{0x9F,1,{0x03}},		
{0xA0,1,{0x3A}},		
{0xA2,1,{0x03}},		
{0xA3,1,{0x56}},		
{0xA4,1,{0x03}},		
{0xA5,1,{0x6D}},		
{0xA6,1,{0x03}},		
{0xA7,1,{0x89}},		
{0xA9,1,{0x03}},		
{0xAA,1,{0xA3}},		
{0xAB,1,{0x03}},		
{0xAC,1,{0xC9}},		
{0xAD,1,{0x03}},		
{0xAE,1,{0xDD}},		
{0xAF,1,{0x03}},		
{0xB0,1,{0xF5}},		
{0xB1,1,{0x03}},		
{0xB2,1,{0xFF}},			
//R-		
{0xB3,1,{0x00}},		
{0xB4,1,{0x00}},		
{0xB5,1,{0x00}},		
{0xB6,1,{0x22}},		
{0xB7,1,{0x00}},		
{0xB8,1,{0x46}},		
{0xB9,1,{0x00}},		
{0xBA,1,{0x5C}},		
{0xBB,1,{0x00}},		
{0xBC,1,{0x76}},		
{0xBD,1,{0x00}},		
{0xBE,1,{0x8D}},		
{0xBF,1,{0x00}},		
{0xC0,1,{0xA6}},		
{0xC1,1,{0x00}},		
{0xC2,1,{0xB8}},		
{0xC3,1,{0x00}},		
{0xC4,1,{0xC7}},		
{0xC5,1,{0x00}},		
{0xC6,1,{0xF6}},		
{0xC7,1,{0x01}},		
{0xC8,1,{0x1D}},		
{0xC9,1,{0x01}},		
{0xCA,1,{0x54}},		
{0xCB,1,{0x01}},		
{0xCC,1,{0x81}},		
{0xCD,1,{0x01}},		
{0xCE,1,{0xCB}},		
{0xCF,1,{0x02}},		
{0xD0,1,{0x05}},		
{0xD1,1,{0x02}},		
{0xD2,1,{0x07}},		
{0xD3,1,{0x02}},		
{0xD4,1,{0x47}},		
{0xD5,1,{0x02}},		
{0xD6,1,{0x82}},		
{0xD7,1,{0x02}},		
{0xD8,1,{0xAB}},		
{0xD9,1,{0x02}},		
{0xDA,1,{0xDC}},		
{0xDB,1,{0x03}},		
{0xDC,1,{0x01}},		
{0xDD,1,{0x03}},		
{0xDE,1,{0x3A}},		
{0xDF,1,{0x03}},		
{0xE0,1,{0x56}},		
{0xE1,1,{0x03}},		
{0xE2,1,{0x6D}},		
{0xE3,1,{0x03}},		
{0xE4,1,{0x89}},		
{0xE5,1,{0x03}},		
{0xE6,1,{0xA3}},		
{0xE7,1,{0x03}},		
{0xE8,1,{0xC9}},		
{0xE9,1,{0x03}},		
{0xEA,1,{0xDD}},		
{0xEB,1,{0x03}},		
{0xEC,1,{0xF5}},		
{0xED,1,{0x03}},		
{0xEE,1,{0xFF}},		
		
//G+		
{0xEF,1,{0x00}},		
{0xF0,1,{0x00}},		
{0xF1,1,{0x00}},		
{0xF2,1,{0x22}},		
{0xF3,1,{0x00}},		
{0xF4,1,{0x46}},		
{0xF5,1,{0x00}},		
{0xF6,1,{0x5C}},		
{0xF7,1,{0x00}},		
{0xF8,1,{0x76}},		
{0xF9,1,{0x00}},		
{0xFA,1,{0x8D}},		
		
{0xFF,1,{0x21}}, //Page 0,1,{power-related setting    		
{REGFLAG_DELAY, 10, {}},  		
{0x00,1,{0x00}},		
{0x01,1,{0xA6}},		
{0x02,1,{0x00}},		
{0x03,1,{0xB8}},		
{0x04,1,{0x00}},		
{0x05,1,{0xC7}},		
{0x06,1,{0x00}},		
{0x07,1,{0xF6}},		
{0x08,1,{0x01}},		
{0x09,1,{0x1D}},		
{0x0A,1,{0x01}},		
{0x0B,1,{0x54}},		
{0x0C,1,{0x01}},		
{0x0D,1,{0x81}},		
{0x0E,1,{0x01}},		
{0x0F,1,{0xCB}},		
{0x10,1,{0x02}},		
{0x11,1,{0x05}},		
{0x12,1,{0x02}},		
{0x13,1,{0x07}},		
{0x14,1,{0x02}},		
{0x15,1,{0x47}},		
{0x16,1,{0x02}},		
{0x17,1,{0x82}},		
{0x18,1,{0x02}},		
{0x19,1,{0xAB}},		
{0x1A,1,{0x02}},		
{0x1B,1,{0xDC}},		
{0x1C,1,{0x03}},		
{0x1D,1,{0x01}},		
{0x1E,1,{0x03}},		
{0x1F,1,{0x3A}},		
{0x20,1,{0x03}},		
{0x21,1,{0x56}},		
{0x22,1,{0x03}},		
{0x23,1,{0x6D}},		
{0x24,1,{0x03}},		
{0x25,1,{0x89}},		
{0x26,1,{0x03}},		
{0x27,1,{0xA3}},		
{0x28,1,{0x03}},		
{0x29,1,{0xC9}},		
{0x2A,1,{0x03}},		
{0x2B,1,{0xDD}},		
{0x2D,1,{0x03}},		
{0x2F,1,{0xF5}},		
{0x30,1,{0x03}},		
{0x31,1,{0xFF}},		
//G-		
{0x32,1,{0x00}},		
{0x33,1,{0x00}},		
{0x34,1,{0x00}},		
{0x35,1,{0x22}},		
{0x36,1,{0x00}},		
{0x37,1,{0x46}},		
{0x38,1,{0x00}},		
{0x39,1,{0x5C}},		
{0x3A,1,{0x00}},		
{0x3B,1,{0x76}},		
{0x3D,1,{0x00}},		
{0x3F,1,{0x8D}},		
{0x40,1,{0x00}},		
{0x41,1,{0xA6}},		
{0x42,1,{0x00}},		
{0x43,1,{0xB8}},		
{0x44,1,{0x00}},		
{0x45,1,{0xC7}},		
{0x46,1,{0x00}},		
{0x47,1,{0xF6}},		
{0x48,1,{0x01}},		
{0x49,1,{0x1D}},		
{0x4A,1,{0x01}},		
{0x4B,1,{0x54}},		
{0x4C,1,{0x01}},		
{0x4D,1,{0x81}},		
{0x4E,1,{0x01}},		
{0x4F,1,{0xCB}},		
{0x50,1,{0x02}},		
{0x51,1,{0x05}},		
{0x52,1,{0x02}},		
{0x53,1,{0x07}},		
{0x54,1,{0x02}},		
{0x55,1,{0x47}},		
{0x56,1,{0x02}},		
{0x58,1,{0x82}},		
{0x59,1,{0x02}},		
{0x5A,1,{0xAB}},		
{0x5B,1,{0x02}},		
{0x5C,1,{0xDC}},		
{0x5D,1,{0x03}},		
{0x5E,1,{0x01}},		
{0x5F,1,{0x03}},		
{0x60,1,{0x3A}},		
{0x61,1,{0x03}},		
{0x62,1,{0x56}},		
{0x63,1,{0x03}},		
{0x64,1,{0x6D}},		
{0x65,1,{0x03}},		
{0x66,1,{0x89}},		
{0x67,1,{0x03}},		
{0x68,1,{0xA3}},		
{0x69,1,{0x03}},		
{0x6A,1,{0xC9}},		
{0x6B,1,{0x03}},		
{0x6C,1,{0xDD}},		
{0x6D,1,{0x03}},		
{0x6E,1,{0xF5}},		
{0x6F,1,{0x03}},		
{0x70,1,{0xFF}},		
//B+		
{0x71,1,{0x00}},		
{0x72,1,{0x00}},		
{0x73,1,{0x00}},		
{0x74,1,{0x22}},		
{0x75,1,{0x00}},		
{0x76,1,{0x46}},		
{0x77,1,{0x00}},		
{0x78,1,{0x5C}},		
{0x79,1,{0x00}},		
{0x7A,1,{0x76}},		
{0x7B,1,{0x00}},		
{0x7C,1,{0x8D}},		
{0x7D,1,{0x00}},		
{0x7E,1,{0xA6}},		
{0x7F,1,{0x00}},		
{0x80,1,{0xB8}},		
{0x81,1,{0x00}},		
{0x82,1,{0xC7}},		
{0x83,1,{0x00}},		
{0x84,1,{0xF6}},		
{0x85,1,{0x01}},		
{0x86,1,{0x1D}},		
{0x87,1,{0x01}},		
{0x88,1,{0x54}},		
{0x89,1,{0x01}},		
{0x8A,1,{0x81}},		
{0x8B,1,{0x01}},		
{0x8C,1,{0xCB}},		
{0x8D,1,{0x02}},		
{0x8E,1,{0x05}},		
{0x8F,1,{0x02}},		
{0x90,1,{0x07}},		
{0x91,1,{0x02}},		
{0x92,1,{0x47}},		
{0x93,1,{0x02}},		
{0x94,1,{0x82}},		
{0x95,1,{0x02}},		
{0x96,1,{0xAB}},		
{0x97,1,{0x02}},		
{0x98,1,{0xDC}},		
{0x99,1,{0x03}},		
{0x9A,1,{0x01}},		
{0x9B,1,{0x03}},		
{0x9C,1,{0x3A}},		
{0x9D,1,{0x03}},		
{0x9E,1,{0x56}},		
{0x9F,1,{0x03}},		
{0xA0,1,{0x6D}},		
{0xA2,1,{0x03}},		
{0xA3,1,{0x89}},		
{0xA4,1,{0x03}},		
{0xA5,1,{0xA3}},		
{0xA6,1,{0x03}},		
{0xA7,1,{0xC9}},		
{0xA9,1,{0x03}},		
{0xAA,1,{0xDD}},		
{0xAB,1,{0x03}},		
{0xAC,1,{0xF5}},		
{0xAD,1,{0x03}},		
{0xAE,1,{0xFF}},		
//B-		
{0xAF,1,{0x00}},		
{0xB0,1,{0x00}},		
{0xB1,1,{0x00}},		
{0xB2,1,{0x22}},		
{0xB3,1,{0x00}},		
{0xB4,1,{0x46}},		
{0xB5,1,{0x00}},		
{0xB6,1,{0x5C}},		
{0xB7,1,{0x00}},		
{0xB8,1,{0x76}},		
{0xB9,1,{0x00}},		
{0xBA,1,{0x8D}},		
{0xBB,1,{0x00}},		
{0xBC,1,{0xA6}},		
{0xBD,1,{0x00}},		
{0xBE,1,{0xB8}},		
{0xBF,1,{0x00}},		
{0xC0,1,{0xC7}},		
{0xC1,1,{0x00}},		
{0xC2,1,{0xF6}},		
{0xC3,1,{0x01}},		
{0xC4,1,{0x1D}},		
{0xC5,1,{0x01}},		
{0xC6,1,{0x54}},		
{0xC7,1,{0x01}},		
{0xC8,1,{0x81}},		
{0xC9,1,{0x01}},		
{0xCA,1,{0xCB}},		
{0xCB,1,{0x02}},		
{0xCC,1,{0x05}},		
{0xCD,1,{0x02}},		
{0xCE,1,{0x07}},		
{0xCF,1,{0x02}},		
{0xD0,1,{0x47}},		
{0xD1,1,{0x02}},		
{0xD2,1,{0x82}},		
{0xD3,1,{0x02}},		
{0xD4,1,{0xAB}},		
{0xD5,1,{0x02}},		
{0xD6,1,{0xDC}},		
{0xD7,1,{0x03}},		
{0xD8,1,{0x01}},		
{0xD9,1,{0x03}},		
{0xDA,1,{0x3A}},		
{0xDB,1,{0x03}},		
{0xDC,1,{0x56}},		
{0xDD,1,{0x03}},		
{0xDE,1,{0x6D}},		
{0xDF,1,{0x03}},		
{0xE0,1,{0x89}},		
{0xE1,1,{0x03}},		
{0xE2,1,{0xA3}},		
{0xE3,1,{0x03}},		
{0xE4,1,{0xC9}},		
{0xE5,1,{0x03}},		
{0xE6,1,{0xDD}},		
{0xE7,1,{0x03}},		
{0xE8,1,{0xF5}},		
{0xE9,1,{0x03}},		
{0xEA,1,{0xFF}},	

{0xFF,1,{0x21}}, //Page 1,1,{Gamma Default Update
{REGFLAG_DELAY, 10, {}},  
{0xEB,1,{0x30}},
{0xEC,1,{0x17}},
{0xED,1,{0x20}},
{0xEE,1,{0x0F}},
{0xEF,1,{0x1F}},
{0xF0,1,{0x0F}},
{0xF1,1,{0x0F}},
{0xF2,1,{0x07}},

{0xFF,1,{0x23}}, //CMD2 Page 3 Entrance
{REGFLAG_DELAY, 10, {}},  
{0x08,1,{0x04}}, //ABC CTRL9 PWM 41kHz  

{0xFF,1,{0x10}}, //Return To CMD1
{REGFLAG_DELAY, 10, {}},  
{0x35,1,{0x00}},
{0x29,1,{0x00}},  
{0x51,1,{0xFF}}, //write display brightness

{0x11,1,{0x00}},  //Sleep Out
{REGFLAG_DELAY, 120, {}},  

{0x29,1,{0x00}}, //Display On         
{REGFLAG_DELAY, 40, {}}, 

                          
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_esd_setting[] = {
	{0xFF,1,{0xEE}},
	{0x26,1,{0x08}},
	{0x26,1,{0x00}},
	{0xFF,1,{0x00}},
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

static struct LCM_setting_table lcm_compare_id_setting[] = {
    // Display off sequence
    {0xf0, 5, {0x55, 0xaa, 0x52, 0x08, 0x01}},
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

    for(i = 0; i < count; i++)
    {
        unsigned int cmd;
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
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
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
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;	

		
		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 16;
		params->dsi.vertical_frontporch					= 25;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 50;
		params->dsi.horizontal_backporch				= 80;
		params->dsi.horizontal_frontporch				= 60;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
#if 0
		//1 Every lane speed
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
		params->dsi.fbk_div =0x12;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#else
		params->dsi.PLL_CLOCK=220;//440;//227;//254;//254//247
#endif

}

static void lcm_init(void)
{
    lcm_util.set_gpio_mode(GPIO_LCD_ENN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO_LCD_ENN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCD_ENN, GPIO_OUT_ONE);
	MDELAY(10);
    lcm_util.set_gpio_mode(GPIO_LCD_DRV_EN_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO_LCD_DRV_EN_PIN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ONE);
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(50);
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#ifdef BUILD_LK
    printf("[erick-lk]%s\n", __func__);
#else
    printk("[erick-k]%s\n", __func__);
#endif
}


static void lcm_suspend(void)
{
#ifndef BUILD_LK
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);   //wqtao. enable
    #ifdef BUILD_LK
        printf("[erick-lk]%s\n", __func__);
    #else
        printk("[erick-k]%s\n", __func__);
    #endif
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

    data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}
#endif


static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	unsigned char buffer[6];
	unsigned int data_array[16];

	if(lcm_esd_test)
	{
	    lcm_esd_test = FALSE;
	    return TRUE;
	}

	data_array[0] = 0x00023700;
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0x0a, buffer, 2);
	printk("[%s] hct_sh1282_dsi_vdo_fwvga_boe esd check: id = %x\n", __FUNCTION__, buffer[0]);

	if(buffer[0] == 0x9c)
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


static unsigned int lcm_compare_id(void)
{
 	unsigned int id =0;
	unsigned char buffer[2];
	unsigned int arry[16];

  lcm_util.set_gpio_mode(GPIO_LCD_ENN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO_LCD_ENN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCD_ENN, GPIO_OUT_ONE);
MDELAY(10); 
    lcm_util.set_gpio_mode(GPIO_LCD_DRV_EN_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO_LCD_DRV_EN_PIN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ONE);
	
	SET_RESET_PIN(1);
	MDELAY(10); 
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);

	arry[0]= 0x00023700; ; 
	
	dsi_set_cmdq(arry, 1, 1);

	read_reg_v2(0xF4,buffer,2);
	
	id = buffer[0];

	#ifdef BUILD_LK
		printf("%s,LK nt35595 debug: nt35595 id =%x\n",__func__,id);
	#else
		printk("%s,kernel nt35595 horse debug: nt35595 id =%x\n",__func__,id);
	#endif

	if(id == LCM_ID)
		return 1;
	else
		return 0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_nt35595_dsi_vdo_fhd_lg = 
{
    .name           = "hct_nt35595_dsi_vdo_fhd_lg",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,   
    .compare_id    = lcm_compare_id,    
#if (LCM_DSI_CMD_MODE)
    //.set_backlight    = lcm_setbacklight,
    //.esd_check   = lcm_esd_check, 
    //.esd_recover   = lcm_esd_recover, 
    .update         = lcm_update,
#endif  //wqtao
    .esd_check   = lcm_esd_check, 
    .esd_recover   = lcm_esd_recover, 
};

