#ifndef __GP2AP054A_PSGS_ALS_H__
#define __GP2AP054A_PSGS_ALS_H__ 

#include <linux/ioctl.h>

// Reg.
#define REG_ADR_00	0x00	// Read & Write
#define REG_ADR_01	0x01	// Read & Write
#define REG_ADR_02	0x02	// Read & Write
#define REG_ADR_03	0x03	// Read & Write
#define REG_ADR_04	0x04	// Read & Write
#define REG_ADR_05	0x05	// Read & Write
#define REG_ADR_06	0x06	// Read & Write
#define REG_ADR_07	0x07	// Read & Write
#define REG_ADR_08	0x08	// Read & Write
#define REG_ADR_09	0x09	// Read & Write
#define REG_ADR_0A	0x0A	// Read & Write
#define REG_ADR_0B	0x0B	// Read & Write
#define REG_ADR_0C	0x0C	// Read & Write
#define REG_ADR_0D	0x0D	// Read & Write
#define REG_ADR_0E	0x0E	// Read & Write
#define REG_ADR_0F	0x0F	// Read & Write
#define REG_ADR_10	0x10	// Read  Only
#define REG_ADR_11	0x11	// Read  Only
#define REG_ADR_12	0x12	// Read  Only
#define REG_ADR_13	0x13	// Read  Only
#define REG_ADR_14	0x14	// Read  Only
#define REG_ADR_15	0x15	// Read  Only
#define REG_ADR_16	0x16	// Read  Only
#define REG_ADR_17	0x17	// Read  Only
#define REG_ADR_18	0x18	// Read  Only
#define REG_ADR_19	0x19	// Read  Only
#define REG_ADR_1B	0x1B	// TS1 sample

	#define REG_D0_LSB_PRE3		0x14
	#define REG_D0_LSB_PRE2		0x1E
	#define REG_D0_LSB			0x2C
	#define REG_D0_MSB			0x2D
	#define REG_D1_LSB			0x2E
	#define REG_D1_MSB			0x2F
	#define REG_D2_LSB			0x30
	#define REG_D2_MSB			0x31
	#define REG_D3_LSB			0x32
	#define REG_D3_MSB			0x33
	#define REG_D4_LSB			0x34
	#define REG_D4_MSB			0x35
	#define REG_D5_LSB			0x36
	#define REG_D5_MSB			0x37
	#define REG_D6_LSB			0x38
	#define REG_D6_MSB			0x39
	#define REG_D7_LSB			0x3A
	#define REG_D7_MSB			0x3B
	#define REG_D8_LSB			0x3C
	#define REG_D8_MSB			0x3D
	#define REG_DEVICE_ID		0x3E
	#define REG_PANEL   		0x41
	
#define REG_D5_LSB			0x36
#define REG_D5_MSB			0x37
#define REG_D6_LSB			0x38
#define REG_D6_MSB			0x39
	
#define LOW_LUX_RANGE	 			( ALS1_RANGEX1 )
#define MIDDLE_LUX_RANGE 			( ALS1_RANGEX16 )
#define HIGH_LUX_RANGE	 			( ALS1_RANGEX512 )
	// COMMAND1
	#define COMMAND1_WAKEUP			0x80
	#define COMMAND1_SD				0x00
	#define COMMAND1_ALS_GS			0x00
	#define COMMAND1_ALS			0x10
	#define COMMAND1_GS				0x20


	// COMMAND2
	#define COMMAND2_NO_INT_CLEAR	0x0F
	#define COMMAND2_INT_CLEAR		0x00
	#define COMMAND2_GS_INT_CLEAR	0x0E
	#define COMMAND2_PS_INT_CLEAR	0x03
	#define COMMAND2_ALS_INT_CLEAR	0x0D

	
	// COMMAND3
	#define COMMAND3_INT_PROX		0x00
	#define COMMAND3_INT_PS			0x10
	#define COMMAND3_INT_ALS		0x20
	#define COMMAND3_INT_GS			0x40
	#define COMMAND3_INT_PS_LEVEL	0x00
	#define COMMAND3_INT_PS_PULSE	0x02
	#define COMMAND3_INT_ALS_LEVEL	0x00
	#define COMMAND3_INT_ALS_PULSE	0x04
	#define COMMAND3_INT_GS_LEVEL	0x00
	#define COMMAND3_INT_GS_PULSE	0x08
	#define COMMAND3_REG_RST		0x01

	
	// ALS1
	#define ALS1_RES18			0x00
	#define ALS1_RES16			0x08
	#define ALS1_RES14			0x10
	#define ALS1_RES12			0x18
	#define ALS1_RANGEX1		0x00
	#define ALS1_RANGEX2		0x01
	#define ALS1_RANGEX4		0x02
	#define ALS1_RANGEX8		0x03
	#define ALS1_RANGEX16		0x04
	#define ALS1_RANGEX32		0x05
	#define ALS1_RANGEX64		0x06
	#define ALS1_RANGEX138		0x07
	#define ALS1_RANGEX277		0x86
	#define ALS1_RANGEX555		0x87	
	
	// ALS2
	#define ALS2_ALS_INTVAL0	0x00
	#define ALS2_ALS_INTVAL1P56	0x01
	#define ALS2_ALS_INTVAL6P25	0x02
	#define ALS2_ALS_INTVAL25	0x03
	#define ALS2_ALS_INTVAL50	0x04
	#define ALS2_ALS_INTVAL100	0x05
	#define ALS2_ALS_INTVAL200	0x06
	#define ALS2_ALS_INTVAL400	0x07


	// PS1
	#define PS1_PRST1			0x00
	#define PS1_PRST2			0x20
	#define PS1_PRST3			0x40
	#define PS1_PRST4			0x60
	#define PS1_PRST5			0x80
	#define PS1_PRST6			0xA0
	#define PS1_PRST7			0xC0
	#define PS1_PRST8			0xE0
	#define PS1_RES14			0x00
	#define PS1_RES12			0x08
	#define PS1_RES10			0x10
	#define PS1_RES8			0x18
	#define PS1_RANGEX1			0x00
	#define PS1_RANGEX2			0x01
	#define PS1_RANGEX4	 		0x02
	#define PS1_RANGEX8			0x03
	#define PS1_RANGEX16		0x04
	#define PS1_RANGEX32		0x05
	#define PS1_RANGEX64		0x06
	#define PS1_RANGEX128		0x07
	
	
	// PS2	
	#define PS2_IS2				0x00
	#define PS2_IS4				0x20
	#define PS2_IS8				0x40
	#define PS2_IS16			0x60
	#define PS2_IS32			0x80
	#define PS2_IS64			0xA0
	#define PS2_IS128			0xC0
	#define PS2_IS256			0xE0
	#define PS2_SUM4			0x00
	#define PS2_SUM8			0x04
	#define PS2_SUM12			0x08
	#define PS2_SUM16			0x0C
	#define PS2_SUM20			0x10
	#define PS2_SUM24			0x14
	#define PS2_SUM28			0x18
	#define PS2_SUM32			0x1C

	
	//PS3
	#define PS3_GS_INT1			0x00
	#define PS3_GS_INT2			0x10
	#define PS3_GS_INT3			0x20
	#define PS3_GS_INT4			0x30
	#define PS3_GS_INTVAL0		0x00
	#define PS3_GS_INTVAL1P56	0x01
	#define PS3_GS_INTVAL6P25	0x02
	#define PS3_GS_INTVAL25		0x03
	#define PS3_GS_INTVAL50		0x04
	#define PS3_GS_INTVAL100	0x05
	#define PS3_GS_INTVAL200	0x06
	#define PS3_GS_INTVAL400	0x07
	
	
	
	#define REG_PS_LT_LSB	 	0x08
	#define REG_PS_LT_MSB		0x09
	#define REG_PS_HT_LSB		0x0A
	#define REG_PS_HT_MSB		0x0B
	
	#define REG_COMMND1    		0x00
	#define REG_COMMND2    		0x01
	#define REG_COMMND3 		0x02
	#define REG_ALS1   			0x03
	#define REG_ALS2			0x04
	#define REG_PS5				0x05
	#define REG_PS6				0x06
	#define REG_PS7				0x07
	
	// PS2	
	#define PS2_IS2				0x00
	#define PS2_IS4				0x20
	#define PS2_IS9				0x40
	#define PS2_IS19			0x60
	#define PS2_IS38			0x80
	#define PS2_IS75			0xA0
	#define PS2_IS150			0xC0
	#define PS2_IS280			0xE0
	#define PS2_SUM4			0x00
	#define PS2_SUM8			0x04
	#define PS2_SUM12			0x08
	#define PS2_SUM16			0x0C
	#define PS2_SUM20			0x10
	#define PS2_SUM24			0x14
	#define PS2_SUM28			0x18
	#define PS2_SUM32			0x1C
	
	#define REFLECTIVE_CANCEL_GS		0x00
#define REFLECTIVE_CANCEL_PULSE		0x2D

#define REG_PANEL   		0x41
/* event code */
#define ABS_CONTROL_REPORT		( ABS_THROTTLE )
#define ABS_ADC_REPORT			( ABS_MISC )
#define ABS_DISTANCE_REPORT		( ABS_DISTANCE )

/* platform data */
struct gp2ap050_platform_data
{
	int		gpio ;
} ;

/*
#define DEVICE_NAME 			"GP2AP050"
#define GESTURE_SENSOR_NAME		"light"//"gesture_sensor"
#define PROXIMITY_SENSOR_NAME		"proximity"
#define GP2AP_INPUT_DEV_NAME		"gesture_sensor"
*/
#define SENSOR_MODE_OFF			( 0 )		/* Sensor OFF */
#define SENSOR_MODE_GESTURE		( 1 )		/* Gesture mode */
#define SENSOR_MODE_PROXIMITY	( 2 )		/* Proximity mode */
#define SENSOR_MODE_PULSE_COUNTER   ( 3 )		/* Pulse Counter */

#define SENSOR_DEFAULT_DELAY	( 100 )		/* 10 ms unit=1/10ms(100us) */
#define SENSOR_MIN_DELAY		(  30 )		/*  3 ms unit=1/10ms(100us) */
#define SENSOR_MAX_DELAY		( 200 )		/* 20 ms unit=1/10ms(100us) */

#define PS_AVG_READ_COUNT		( 40 )		/* sensor read count */
#define PS_AVG_READ_INTERVAL	( 40 )		/* 40 ms */
#define PS_AVG_POLL_DELAY		( 2000 )	/* 2000 ms */

#define LOW_LUX_MODE		( 0 )
#define HIGH_LUX_MODE		( 1 )

// Reg. 00H
#define	OP_SHUTDOWN		0x00	// OP3:0
#define	OP_RUN			0xC0	// OP3:1 OP2:1
#define	OP_CONTINUOUS		0x40	// OP2:1
#define	OP_PS_ALS		0x00	// OP01:00
#define	OP_ALS			0x10	// OP01:01
#define	OP_PS			0x20	// OP01:10
#define	INT_NOCLEAR		0x0C	// PROX:1 FLAG:1

// Reg. 01H
#define	INTVAL_0		0x00	// INTVAL:00
#define	INTVAL_1_56		0x40	// INTVAL:01
#define	INTVAL_6_25		0x80	// INTVAL:10
#define	INTVAL_25		0x03	// INTVAL:11
#define	INTSEL_D0		0x00	// INTSEL:000
#define	INTSEL_D1		0x08	// INTSEL:001
#define	INTSEL_D2		0x10	// INTSEL:010
#define	INTSEL_D3		0x18	// INTSEL:011
#define	INTSEL_D4		0x20	// INTSEL:100
#define	PIN_FRAG		0x00	// PIN:0
#define	PIN_PROX		0x04	// PIN:1
#define	INTTYPE_L		0x00	// INTTYPE:0
#define	INTTYPE_P		0x02	// INTTYPE:1
#define	RST				0x01	// RST:1

// Reg. 02H
#define	PRST_1			0x00	// PRST:000
#define	PRST_2			0x20	// PRST:001
#define	PRST_3			0x40	// PRST:010
#define	PRST_4			0x60	// PRST:011
#define	PRST_5			0x80	// PRST:100
#define	PRST_6			0xa0	// PRST:101
#define	PRST_7			0xc0	// PRST:110
#define	PRST_8			0xe0	// PRST:111
#define	RES_14			0x00	// RES:00
#define	RES_12			0x08	// RES:01
#define	RES_10			0x10	// RES:10
#define	RES_8			0x18	// RES:11
#define	RANGE_1			0x00	// RANGE:000
#define	RANGE_2			0x01	// RANGE:001
#define	RANGE_4			0x02	// RANGE:010
#define	RANGE_8			0x03	// RANGE:011
#define	RANGE_16		0x04	// RANGE:100
#define	RANGE_32		0x05	// RANGE:101
#define	RANGE_64		0x06	// RANGE:110
#define	RANGE_128		0x07	// RANGE:111

// Reg. 03H
#define	IS_16_25		0x00	// IS:000
#define	IS_32_5			0x20	// IS:001
#define	IS_65			0x40	// IS:010
#define	IS_130			0x60	// IS:011
#define	IS_180			0xe0	// IS:111
#define	SUM_1			0x00	// SUM:000
#define	SUM_2			0x04	// SUM:001
#define	SUM_4			0x08	// SUM:010
#define	SUM_8			0x0c	// SUM:011
#define	SUM_16			0x10	// SUM:100
#define	SUM_32			0x14	// SUM:101
#define	SUM_64			0x18	// SUM:110
#define	SUM_128			0x1c	// SUM:111
#define	PULSE_24		0x00	// PULSE:00
#define	PULSE_8			0x01	// PULSE:01
#define	PULSE_4			0x10	// PULSE:10

const int STATE_ONE 			= 1;
const int STATE_TWO 			= 2;
const int STATE_THREE 			= 3;

const int DIR_RIGHT 			= 0x8;
const int DIR_LEFT 			= 0x4;
const int DIR_TOP 				= 0x2;
const int DIR_BOTTOM 			= 0x1;
const int SPEED_HIGH 			= 0x2;
const int SPEED_MID 			= 0x1;
const int STANDARD_SAMPLING_RATE= 100;

const int ZOOM_LEVEL1			= 1;
const int ZOOM_LEVEL2			= 2;
const int ZOOM_LEVEL3			= 3;
const int ZOOM_LEVEL4			= 4;
const int ZOOM_LEVEL5			= 5;
const int ZOOM_LEVEL_DEF		= 3;//ZOOM_LEVEL3
const short int OVER_FLOW_DATA 	= 4095;
static int sampling_rate = 100;
//static short act_os_d[4]={0x03FF, 0x03FF, 0x03FF, 0x03FF};
//38.3471	17.744	39.5761	70.2575
//static short act_os_d[4]={38, 18, 40, 70};
static short act_os_d[4]={0x500, 0x500, 0x500, 0x500};
static unsigned short sub_os_data[4];
const short int max_aoc_counts = 8000;//1600;
const short int  zoom_z_th[5] = {
		 600,
		 900,
		1350,
		2025,
		3037
};

#define alfa1					5408//0.00585
#define alfa2					9611//0.00830
#define alfa3					3616//0.003725
#define beta1					0
#define beta2					14010//-0.01228
#define beta3					402//-0.00465
#define CALC_OFFSET1			100000
#define CALC_OFFSET2			100000
#define CALC_OFFSET3			10000
#define RATIO_FIRST_BOUND		30//0.20
#define RATIO_SECOND_BOUND		60//0.60
#define RATIO_THIRD_BOUND		90//0.80

#define MAX_LUX_VALUE				50000
#define OVER_FLOW_COUNT		 		30000
#define ZERO_LUX_TH					5   //50 

/* ALS mode change */
#define LOW_LUX_MODE				( 0 )
#define	MIDDLE_LUX_MODE				( 1 )
#define	HIGH_LUX_MODE				( 2 )

#define LOW_LUX_RANGE	 			( ALS1_RANGEX4 )
#define MIDDLE_LUX_RANGE 			( ALS1_RANGEX64 )
#define HIGH_LUX_RANGE	 			( ALS1_RANGEX555 )

#define ALS_L_to_M_counts		 	35000
#define ALS_M_to_H_counts		 	35000

#define ALS_H_to_M_counts		 	3600
#define ALS_M_to_L_counts		 	1800

/* coefficient for adjusting the difference in RANGE */
#define GAMMA_LOW_LUX_MODE			1			/*   4/ 4  = 1    basis  */
#define GAMMA_MIDDLE_LUX_MODE		16			/*   64/4  = 16	         */
#define GAMMA_HIGH_LUX_MODE			139			/*   555/4 = 139         */

#define LUX_MEDIAN					1


#endif
