#ifndef SPI_SENSOR_H
#define SPI_SENSOR__H

#define MTK_HC

#include <linux/types.h>

#ifdef MTK_HC
	#include <mach/mt_gpio.h>
	#include <cust_gpio_usage.h>
#endif

#define SPI_IOC_MAGIC			'k'
//#define SPISENSOR_MAJOR			150	
#define N_SPI_MINORS			32	
#define SF104_SENSOR 			1

#define CMD_LENGTH  			(2 + 1 + 1 + 2 + 2)
#define APDU_OFFSET 			(2 + 1 + 1 + 2)

#define SPI_CPHA				0x01
#define SPI_CPOL				0x02

#define SPI_MODE_0				(0|0)
#define SPI_MODE_1				(0|SPI_CPHA)
#define SPI_MODE_2				(SPI_CPOL|0)
#define SPI_MODE_3				(SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH				0x04
#define SPI_LSB_FIRST			0x08
#define SPI_3WIRE				0x10
#define SPI_LOOP				0x20
#define SPI_NO_CS				0x40
#define SPI_READY				0x80

enum RT_APDU{
	RT_OK = 0x00,			//0x00
	RT_FAI,					//0x01
	RT_COMMAND_ERROR,		//0x02
	RT_PARAM_ERROR,			//0x03
	RT_TIME_OUT,			//0x04
	RT_ECC_ERROR,			//0x05
	RT_PACKAGE_ERROR = 0x08,	//0x08
	RT_OK_WITH_PACKAGE	= 0xfe,
};

#define CMD_FAIL		0x69e0
#define CMD_ERR			0x69e1
#define CMD_PARM_ERR	0x69e2
#define CMD_TIME_OUT	0x69e3
#define CMD_ECC_ERR		0x69e4
#define CMD_PKG_ERR		0x69e5

typedef unsigned short uint16;

struct requst_cmd{
	unsigned char * tx_buf;
	unsigned int txlen;
	unsigned char * rx_buf;
	unsigned int  *rxlen;
	int *status;
	int timeout;
	//int type;
};

struct APDU_ACK{
	uint16	pid;
	char    cmd;
	char 	sw;
	uint16	lc; 
	uint16     dumy;
}__attribute__((aligned(4)));
#define SF115_POLL_DELAY		300*1000  //50us

#define SPI_CS_PIN        	GPIO_SPI_CS_PIN 
#define SPI_SCK_PIN       	GPIO_SPI_SCK_PIN 
#define SPI_MISO_PIN      	GPIO_SPI_MISO_PIN 
#define SPI_MOSI_PIN      	GPIO_SPI_MOSI_PIN 

#ifdef MTK_HC

#define SF115_GPIO_READR  	GPIO_FIGERPRINT_INT		//GPIO120 //Do Not Modify Linc 20150522
#define CUST_EINT_FINGER_NUM 	CUST_EINT_FIGERPRINT_INT_NUM
#define SF115_GPIO_RST		GPIO_FIGERPRINT_RST
#ifdef GPIO_FIGERPRINT_PWR_EN_PIN
	#define SF115_GPIO_3V3		GPIO_FIGERPRINT_PWR_EN_PIN
#endif
#ifdef GPIO_FIGERPRINT_PWR_EN2_PIN
	#define SF115_GPIO_1V8		GPIO_FIGERPRINT_PWR_EN2_PIN
#endif
#define SF115_GPIO_MODE			GPIO_MODE_00
#define SPI_CS_PIN_MODE        GPIO_MODE_01 
#define SPI_SCK_PIN_MODE        GPIO_MODE_01 
#define SPI_MISO_PIN_MODE       GPIO_MODE_01 
#define SPI_MOSI_PIN_MODE       GPIO_MODE_01 

#else

#define SF115_GPIO_READR  	EXYNOS4_GPX3(5)

#endif





#define SPI_FIFO_LENGTH 	32
#define SPI_DMAC_LENGTH 	1024

#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

struct spi_ioc_transfer {
	__u64		tx_buf;
	__u64		rx_buf;

	__u32		len;
	__u32		speed_hz;

	__u16		delay_usecs;
	__u8		bits_per_word;
	__u8		cs_change;
	__u32		pad;
};

/* leave chip selected when we're done, for quicker re-select? */
#if	0
#define	CS_CHANGE(xfer)	((xfer).cs_change = 1)
#else
#define	CS_CHANGE(xfer)	((xfer).cs_change = 0)
#endif

/* not all platforms use <asm-generic/ioctl.h> or _IOC_TYPECHECK() ... */
#define SPI_MSGSIZE(N) \
	((((N)*(sizeof (struct spi_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct spi_ioc_transfer))) : 0)
		
#define SPI_IOC_MESSAGE(N) 			_IOW(SPI_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])

/* Read / Write of SPI mode (SPI_MODE_0..SPI_MODE_3) */
#define SPI_IOC_RD_MODE				_IOR(SPI_IOC_MAGIC, 1, __u8)
#define SPI_IOC_WR_MODE				_IOW(SPI_IOC_MAGIC, 1, __u8)

/* Read / Write SPI bit justification */
#define SPI_IOC_RD_LSB_FIRST		_IOR(SPI_IOC_MAGIC, 2, __u8)
#define SPI_IOC_WR_LSB_FIRST		_IOW(SPI_IOC_MAGIC, 2, __u8)

/* Read / Write SPI device word length (1..N) */
#define SPI_IOC_RD_BITS_PER_WORD	_IOR(SPI_IOC_MAGIC, 3, __u8)
#define SPI_IOC_WR_BITS_PER_WORD	_IOW(SPI_IOC_MAGIC, 3, __u8)

/* Read / Write SPI device default max speed hz */
#define SPI_IOC_RD_MAX_SPEED_HZ		_IOR(SPI_IOC_MAGIC, 4, __u32)
#define SPI_IOC_WR_MAX_SPEED_HZ		_IOW(SPI_IOC_MAGIC, 4, __u32)

/* Write the value to be used by the GPIO */
#define SPI_IOC_COS_ISP				_IOW(SPI_IOC_MAGIC, 5,__u8)
#define SPI_IOC_SET_IMAGE			_IOWR(SPI_IOC_MAGIC, 6, __u8)
#define SPI_IOC_SENSOR_RST			_IOW(SPI_IOC_MAGIC, 7,__u8)
#define SPI_IOC_REQUST_CMD			_IOWR(SPI_IOC_MAGIC, 8, struct requst_cmd)

#define SPI_IOC_WAKEBYTELEN_1B		_IOW(SPI_IOC_MAGIC, 12, __u32)
#define SPI_IOC_WAKEBYTELEN_32B		_IOW(SPI_IOC_MAGIC, 13, __u32)
#define SPI_IOC_INPUT_OFF			_IOW(SPI_IOC_MAGIC, 14, __u32)
#define SPI_IOC_INPUT_ON			_IOW(SPI_IOC_MAGIC, 15, __u32)
#endif 
