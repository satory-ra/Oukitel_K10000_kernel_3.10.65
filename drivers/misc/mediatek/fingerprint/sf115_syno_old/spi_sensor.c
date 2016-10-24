/****************************************************************************************
*File name		:Synochip sf115 sensor driver version 0.0.1
*Author			:Cndy
*Date			:2014-12-30
*Version      	:v0.1
*Description  	:This is a simple driver demo. 	
*ModifyRecord 	:
*//***************************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/input.h>
#include <linux/sched.h>
#include <linux/kthread.h>

#include <linux/spi/spi.h>

#include <asm/uaccess.h>

#include <asm/atomic.h>
#include <linux/wait.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/completion.h> 
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/jiffies.h>
#include <linux/dma-mapping.h>

#include "spi_sensor.h"


#ifdef MTK_HC
	#include <mach/eint.h>
	#include <mach/mt_gpio.h>
	#include <mach/mt_spi.h>
	#include <cust_eint.h>
#else
	#include <mach/gpio.h>
	#include <plat/gpio-cfg.h>
#endif

#define OFF_INPUT

static DECLARE_BITMAP(minors, N_SPI_MINORS);

struct hw_info{
	unsigned int speed;
	unsigned int img_line;
	unsigned int img_row;
};

struct spi_sensor_data {
	dev_t				devt;
	spinlock_t			spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct mutex		buf_lock;
	int					status;
	unsigned int		users;
	unsigned int        size;
	unsigned int 		speed;
	unsigned int		master;
	unsigned long 		timeout;
	unsigned int 		high_mising;
	unsigned int  		rxlen;
	struct input_dev    *input_dev;

	u8					*txbuffer;
	u8 				    *rxbuffer;
	struct timer_list 	timer;
	struct requst_cmd	*requst;
	struct completion   completed;
	struct work_struct  fasync_wq;
	struct fasync_struct *fasync_q;
	int irq_read;
};

#define SPI_SPEED	4000000
static unsigned bufsiz = 4096;
static unsigned char g_rxtxbuf[1024] = {0};
static struct spi_sensor_data	*g_spi_sensor =NULL;
static atomic_t master = ATOMIC_INIT(0); //默认等待中断
static atomic_t ato_wake_byte_len = ATOMIC_INIT(1);  	
static atomic_t ato_input_fun = ATOMIC_INIT(0);  																	

unsigned int SPISENSOR_MAJOR = 0;
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static int spi_sensor_update(struct file *filp);
static int spi_sensor_fasync(int fd, struct file * filp, int on) ;
static int sf115_send_packeg(struct spi_sensor_data	*spi_sensor,ssize_t len,unsigned char *buff);
static int sf115_recive_packeg(struct spi_sensor_data	*spi_sensor,ssize_t len,unsigned char *buff);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");							
/*---------------------------spi transfer api----------------------------------------------*/
#ifdef MTK_HC
struct mt_chip_conf chip_config = {
		.setuptime =10,//6, //10,//15,//10 ,//3,
		.holdtime =10, //6,//10,//15,//10,//3,
		.high_time =60,//4,//6,//8,//12, //25,//8,      //10--6m   15--4m   20--3m  30--2m  [ 60--1m 120--0.5m  300--0.2m]
		.low_time = 60,//4,//6,//8,//12,//25,//8,
		.cs_idletime = 60,//30,// 60,//100,//12,
		.ulthgh_thrsh = 0,

		.rx_mlsb = SPI_MSB, 
		.tx_mlsb = SPI_MSB,		 
		.tx_endian = 0,
		.rx_endian = 0,

		.cpol = SPI_CPOL_0,
		.cpha = SPI_CPHA_1,
		
		.com_mod = FIFO_TRANSFER,
		.pause = 0,
		.finish_intr = 1,
};
static void mt_spi_set_mode(struct spi_sensor_data *spi_sensor, int mode)
{
	struct mt_chip_conf* spi_par;
	spi_par =&chip_config;
	if(!spi_par)
	{
		printk( "spi config fail\n");
		return;
	}

	if(1 == mode)
	{
		if(spi_par->com_mod == DMA_TRANSFER) {
			return;
		}
		
			spi_par->com_mod = DMA_TRANSFER;
	}
	else
	{
		if(spi_par->com_mod == FIFO_TRANSFER) {
			return;
		}
		
		spi_par->com_mod = FIFO_TRANSFER;
	}

	if(spi_setup(spi_sensor->spi))
	{
		printk( "spi_setup  fail\n");
	}
		
	return;
}
#endif
#ifndef MTK_HC
static ssize_t  init_gpio(struct spi_sensor_data *spi_sensor)
{
	ssize_t	status = 0;
	
	// GPIO reset
/*	status = gpio_request(SF115_GPIO_RST, "fpxx_reset_gpio");
	if (status) {
		status = -EFAULT;
		PDEBUG("func=%s line = %d Reset gpio_request failed\n", __func__, __LINE__);
		goto error0;
	}
	status = s3c_gpio_cfgpin(SF115_GPIO_RST, S3C_GPIO_OUTPUT);
	if (status) {		
		status = -EFAULT;
		PDEBUG("func=%s line = %d Reset gpio_direction_output failed\n", __func__, __LINE__);
		goto error0;
	} */
	// GPIO INT
	status = gpio_request(SF115_GPIO_READR, "sf105_gpio_ready");
	if (status) {	
		status = -EFAULT;
		dev_err(&spi_sensor->spi->dev,"func=%s line = %d Reset gpio_request failed\n", __func__, __LINE__);
		goto error0;
	}		
	status = s3c_gpio_setpull(SF115_GPIO_READR, S3C_GPIO_PULL_UP);
	if (status) {	
		status = -EFAULT;
		dev_err(&spi_sensor->spi->dev,"func=%s line = %d Reset gpio_config_output failed\n", __func__, __LINE__);
		goto error0;
	}
	
	status = s3c_gpio_cfgpin(SF115_GPIO_READR, S3C_GPIO_INPUT);
	if (status) {	
		status = -EFAULT;
		dev_err(&spi_sensor->spi->dev,"func=%s line = %d Reset gpio_direction_output failed\n", __func__, __LINE__);
		goto error0;
	}	
	
error0:
	return status;	
}
#endif
static void spi_sensor_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spi_sensor_sync(struct spi_sensor_data *spi_sensor, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spi_sensor_complete;
	message->context = &done;

	spin_lock_irq(&spi_sensor->spi_lock);
	if (spi_sensor->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spi_sensor->spi, message);
	spin_unlock_irq(&spi_sensor->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	
	return status;
}

static inline ssize_t
spi_sensor_reset(u32 value)
{
#ifdef MTK_HC
	 mt_set_gpio_out(SF115_GPIO_RST,value?1:0);
#endif
	return 0;
}

/*
*if  internal cs be used,send a dummy data let cs chang and wait for ready.
*/
static inline ssize_t
spi_sensor_dummy_write(struct spi_sensor_data*spi_sensor, size_t len)
{
	unsigned char dummy[32];
	struct spi_transfer	t = {
			.tx_buf		= &dummy,
			.rx_buf    = &dummy,
			.len		= len,
			.cs_change  = 0,
			.tx_dma	= 0,
			.rx_dma	= 0,
			.bits_per_word = 0,
			.speed_hz   = spi_sensor->speed?:SPI_SPEED,
		};

	struct spi_message	m;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
#ifdef MTK_HC
	mt_spi_set_mode(spi_sensor, 0);
#endif
	return spi_sensor_sync(spi_sensor, &m);
}
static inline ssize_t
spi_sensor_sync_write(struct spi_sensor_data *spi_sensor, size_t len)
{	
	int tmp ,ret = 0;
	unsigned int packge,offset = 0;
	unsigned char * pbuf = NULL;
	unsigned int PACKEG_LENGTH;
	pbuf = spi_sensor->txbuffer;
	if(len > SPI_DMAC_LENGTH)
		PACKEG_LENGTH = SPI_DMAC_LENGTH;
	else 
		PACKEG_LENGTH = SPI_FIFO_LENGTH;
	
	packge = len / PACKEG_LENGTH;
	offset = len % PACKEG_LENGTH;
	for(tmp = 0; tmp < packge; tmp++){
		ret = sf115_send_packeg(spi_sensor,PACKEG_LENGTH,pbuf);
		if(ret != PACKEG_LENGTH){
			dev_err(&spi_sensor->spi->dev,"send a packeg error!\n");
			goto Tdone;
		}
		pbuf += PACKEG_LENGTH;
	}
	
	if(offset){
		ret = sf115_send_packeg(spi_sensor,offset,pbuf);
		if(ret != offset){
			dev_err(&spi_sensor->spi->dev,"send a packeg error!\n");
			goto Tdone;
		}
	}
		
	ret = len;
Tdone:
	return ret;		
}

static inline ssize_t
spi_sensor_sync_read(struct spi_sensor_data *spi_sensor, size_t len)
{	
	int tmp ,ret = 0;
	unsigned int packge,offset = 0;
	unsigned char * pbuf = NULL;
	unsigned int PACKEG_LENGTH;
	
	pbuf = spi_sensor->txbuffer;
	if(len > SPI_DMAC_LENGTH)
		PACKEG_LENGTH = SPI_DMAC_LENGTH;
	else 
		PACKEG_LENGTH = SPI_FIFO_LENGTH;
	
	packge = len / PACKEG_LENGTH;
	offset = len % PACKEG_LENGTH;
	for(tmp = 0; tmp < packge; tmp++){
		ret = sf115_recive_packeg(spi_sensor,PACKEG_LENGTH,pbuf);
		if(ret != PACKEG_LENGTH){
			dev_err(&spi_sensor->spi->dev,"send a packeg error!\n");
			goto Rdone;
		}
		pbuf += PACKEG_LENGTH;
	}
	
	if(offset){
		ret = sf115_recive_packeg(spi_sensor,offset,pbuf);
		if(ret != offset){
			dev_err(&spi_sensor->spi->dev,"send a packeg error!\n");
			goto Rdone;
		}
	}
		
	ret = len;
Rdone:
	return ret;
		
}
/*---------------------------the end----------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spi_sensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spi_sensor_data	*spi_sensor;
	ssize_t			status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;
	spi_sensor = filp->private_data;

	mutex_lock(&spi_sensor->buf_lock);
	status = spi_sensor_sync_read(spi_sensor, count);
	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, spi_sensor->txbuffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spi_sensor->buf_lock);

	return status;
}
/* Write-only message with current device setup */
static ssize_t
spi_sensor_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct spi_sensor_data	*spi_sensor;
	ssize_t			status = 0;
	unsigned long		missing;
	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;
	spi_sensor = filp->private_data;

	mutex_lock(&spi_sensor->buf_lock);
	missing = copy_from_user(spi_sensor->txbuffer, buf, count);
	if (missing == 0) {
		status = spi_sensor_sync_write(spi_sensor, count);
	} else
		status = -EFAULT;
	mutex_unlock(&spi_sensor->buf_lock);
	return status;
}
//获取apdude 数据包长度。
static int sf115_get_apdu_length(struct spi_sensor_data *spi_sensor,char *data)
{
	unsigned int seconds = 0;
	struct spi_message	m;
	struct spi_transfer	t = {
			.tx_buf = &g_rxtxbuf,
			.rx_buf		= data,
			.cs_change  = 0,
			.len		= 6,
			.tx_dma	= 0,
			.rx_dma	= 0,
			.bits_per_word = 0,
			.speed_hz   = spi_sensor->speed?:SPI_SPEED,
		};
#ifdef MTK_HC		
	//wait ready single going high
	while(!mt_get_gpio_in(SF115_GPIO_READR)){
		//wait ready
		if(mt_get_gpio_in(SF115_GPIO_READR))
			break;
		//high single hapaned before.
		if(spi_sensor->high_mising)
			goto mising;
		//timeout
		if(seconds > SF115_POLL_DELAY){
			dev_err(&spi_sensor->spi->dev,"wait device high error!\n");
			return -EIO;
		}
		seconds += 50;
		udelay(50);
	}
	
	if(mt_get_gpio_in(SF115_GPIO_READR)){
		//wait for device ready
		atomic_inc(&master);
		seconds = spi_sensor->requst->timeout?:1000;
		if(!wait_for_completion_timeout(&spi_sensor->completed,msecs_to_jiffies(seconds))){
			dev_err(&spi_sensor->spi->dev,"wait for ready single timeout!\n");
			atomic_dec(&master);
			return -ETIMEDOUT;
		}
	}
mising:	
	spi_message_init(&m);
	memset(&g_rxtxbuf,0,1024);
	spi_message_add_tail(&t, &m);
	mt_spi_set_mode(spi_sensor, 0);
	return spi_sensor_sync(spi_sensor, &m);
#else
	//wait ready single going high
	while(!gpio_get_value(SF115_GPIO_READR)){
		//wait ready
		if(gpio_get_value(SF115_GPIO_READR))
			break;
		//high single hapaned before.
		if(spi_sensor->high_mising)
			goto mising;
		//timeout
		if(seconds > SF115_POLL_DELAY){
			dev_err(&spi_sensor->spi->dev,"wait device high error!\n");
			return -EIO;
		}
		seconds += 50;
		udelay(50);
	}
	
	if(gpio_get_value(SF115_GPIO_READR)){
		//wait for device ready
		atomic_inc(&master);
		seconds = spi_sensor->requst->timeout?:1000;
		if(!wait_for_completion_timeout(&spi_sensor->completed,msecs_to_jiffies(seconds))){
			dev_err(&spi_sensor->spi->dev,"wait for ready single timeout!\n");
			atomic_dec(&master);
			return -ETIMEDOUT;
		}
	}
mising:	
	spi_message_init(&m);
	memset(&g_rxtxbuf,0,1024);
	spi_message_add_tail(&t, &m);
	return spi_sensor_sync(spi_sensor, &m);
#endif
}

static int sf115_recive_packeg(struct spi_sensor_data	*spi_sensor,ssize_t len,unsigned char *buff)
{
	struct spi_message	m;
	struct spi_transfer t = {
		.len = len, 
		.tx_buf =&g_rxtxbuf,
		.rx_buf = buff ,
		.cs_change  = 0,
		.bits_per_word = 0,
		.speed_hz	= spi_sensor->speed?:SPI_SPEED,
	};
#ifdef MTK_HC	
	int type = (len > SPI_FIFO_LENGTH) ? 1:0;
#endif
	memset(&g_rxtxbuf,0,1024);
	
	spi_message_init(&m); 
	spi_message_add_tail(&t, &m);
#ifdef MTK_HC
	mt_spi_set_mode(spi_sensor, type);
#endif
	return spi_sensor_sync(spi_sensor, &m);
}


static int sf115_send_packeg(struct spi_sensor_data	*spi_sensor,ssize_t len,unsigned char *buff)
{
	struct spi_message	m;
	struct spi_transfer t = {
		.len = len, 
		.tx_buf =buff,
		.rx_buf =&g_rxtxbuf ,
		.cs_change  = 0,
		.bits_per_word = 0,
		.speed_hz	= spi_sensor->speed?:SPI_SPEED,
	};
#ifdef MTK_HC	
	int type = (len > SPI_FIFO_LENGTH) ? 1:0;
#endif
	memset(&g_rxtxbuf,0,1024);
	
	spi_message_init(&m); 
	spi_message_add_tail(&t, &m);
#ifdef MTK_HC
	mt_spi_set_mode(spi_sensor, type);
#endif
	return spi_sensor_sync(spi_sensor, &m);
}

static int sf115_read_apdu(struct spi_sensor_data	*spi_sensor,ssize_t len,unsigned char *buff)
{
	long tmp ,ret = 0;
	unsigned int packge,offset = 0;
	unsigned char * pbuf = buff;
	unsigned int PACKEG_LENGTH;

	if(len > SPI_DMAC_LENGTH)
		PACKEG_LENGTH = SPI_DMAC_LENGTH;
	else 
		PACKEG_LENGTH = SPI_FIFO_LENGTH;
	
	packge = len / PACKEG_LENGTH;
	offset = len % PACKEG_LENGTH;

	for(tmp = 0; tmp < packge; tmp++){
		ret = sf115_recive_packeg(spi_sensor,PACKEG_LENGTH,pbuf);
		if(ret != PACKEG_LENGTH){
			spi_sensor->timeout = jiffies +  HZ/2;//忽略接下来1S钟的中端。
			dev_err(&spi_sensor->spi->dev,"send a packeg error!\n");
			return ret;
		}
		pbuf += PACKEG_LENGTH;
	}
	
	if(offset){
		ret = sf115_recive_packeg(spi_sensor,offset,pbuf);
		if(ret != offset){
			spi_sensor->timeout = jiffies +  HZ/2;//忽略接下来1S钟的中端。
			dev_err(&spi_sensor->spi->dev,"send a packeg error!\n");
			return ret;
		}
	}
	
	return len;
}
static  ssize_t sf115_send_cmd(struct spi_sensor_data *spi_sensor,unsigned int length)
{
	long tmp ,ret = 0;
	unsigned char * pbuf = NULL;
	unsigned int packge,offset = 0;
	unsigned int PACKEG_LENGTH;
	

	length += CMD_LENGTH;
	pbuf= spi_sensor->txbuffer;
	spi_sensor->high_mising = 0;
	
	if(length > SPI_DMAC_LENGTH)
		PACKEG_LENGTH = SPI_DMAC_LENGTH;
	else 
		PACKEG_LENGTH = SPI_FIFO_LENGTH;
	
	packge = length / PACKEG_LENGTH;
	offset = length % PACKEG_LENGTH;

	for(tmp = 0; tmp < packge; tmp++){
		ret = sf115_send_packeg(spi_sensor,PACKEG_LENGTH,pbuf);
		if(ret != PACKEG_LENGTH){
			dev_err(&spi_sensor->spi->dev,"send a packeg error!\n");
			return ret;
		}
		pbuf += PACKEG_LENGTH;
	}
	
	if(offset){
		ret = sf115_send_packeg(spi_sensor,offset,pbuf);
		if(ret != offset){
			dev_err(&spi_sensor->spi->dev,"send a packeg error!\n");
			return ret;
		}
	}
	return ret;
}

static int sf115_requst_fun(void *data)
{
	int	status = 0;
	struct APDU_ACK packegs;
	unsigned int size = 0;
	unsigned short tmp, ecc = 0;
	unsigned char *pbuf = NULL,*pack = NULL;
	struct spi_sensor_data	*spi_sensor = data;
	pbuf = spi_sensor->rxbuffer;
	memset((char *)pbuf,0,spi_sensor->size);
	do{
		//读包，获取LC 
		ecc = 0;
		//先获取应答包取出LC ,再读取APDU;
		memset((char *)&packegs,0,sizeof(packegs));
		status = sf115_get_apdu_length(spi_sensor,(char *)&packegs);
		if(status > 0) {
			/* dev_info(&spi_sensor->spi->dev,"pid:%x lc:%x cmd:%x sw:%x\n",\
				packegs.pid,packegs.lc,packegs.cmd,packegs.sw); */
			
			//读取APdu ，判断空间是否足够apdu + ecc
			if (spi_sensor->size < (size + packegs.lc + 2)){
				dev_info(&spi_sensor->spi->dev,"requst size is to big than image buff,"
					"And you should malloc the image buff use ioctl" 
					"SPI_IOC_SET_IMAGE and try again!\n");
					status = -ENOSPC;
				goto done; 
			}
			status = sf115_read_apdu(spi_sensor,packegs.lc + 2,pbuf);
	 		if(status == (packegs.lc + 2)){
				for(tmp = 0;tmp < packegs.lc; tmp++){
					ecc += *(pbuf + tmp);
				}
				
				pack = (unsigned char *)&packegs;
				for(tmp = 0;tmp < APDU_OFFSET; tmp++){
					ecc += *(pack + tmp);
				}
					
				if(ecc != (*( pbuf + packegs.lc) + (*( pbuf + packegs.lc + 1) << 8))){
					dev_info(&spi_sensor->spi->dev,"ecc error! %x \n",ecc);
					spi_sensor->timeout = jiffies +  HZ/2;//忽略接下来1S钟的中端。
					status = RT_ECC_ERROR;
					goto done; 
				}
					
				pbuf += packegs.lc;//丢去ecc
				size += packegs.lc;
	 			switch(packegs.sw){
					case RT_OK_WITH_PACKAGE:
						break;
	 				case RT_FAI:
	 				case RT_PACKAGE_ERROR:
	 				case RT_PARAM_ERROR:
	 				case RT_TIME_OUT:
	 				case RT_ECC_ERROR:
	 				case RT_COMMAND_ERROR:
						status = packegs.sw;
						break;
					case RT_OK:
						status = 0;
						break;
	 				default:
						status = -EFAULT;
	 					break;
	 			}
	 		}else {
					spi_sensor->timeout = jiffies +  HZ/2;//忽略接下来1S钟的中端。
					dev_info(&spi_sensor->spi->dev,"read apdu packeg error! "
						"status = %d \n",status);
				   goto done;
			}
		}else{
			spi_sensor->timeout = jiffies +  HZ/2;//忽略接下来1S钟的中端。
			dev_info(&spi_sensor->spi->dev,"get apdu length error!\n");
			goto done;
		}
	
	}while(packegs.sw == RT_OK_WITH_PACKAGE);
done:
	spi_sensor->rxlen = size;
	spi_sensor->status = status;
	return status;
}

ssize_t spi_sensor_requst(struct spi_sensor_data *spi_sensor, struct requst_cmd *requst)
{
	int status = -1;
	int timeout= 0;
	unsigned short ecc = 0;
	unsigned int len,tmp,seconds;
	char *tx = NULL;
	char *ms = NULL;
	char *rx = NULL;
	
	if (!spi_sensor) {
		dev_err(&spi_sensor->spi->dev,"spi_sensor_requst spi_sensor is null!\n");
		return -ENOMEM;
	}
		
	spi_sensor->master = 1;
	memset(spi_sensor->txbuffer,0,bufsiz);
	if(!spi_sensor->rxbuffer || !requst->tx_buf){
		spi_sensor->master = 0;
		dev_err(&spi_sensor->spi->dev,"tx/rx buffer is null ,please malloc it first use ioctl!\n");
		return -ENOMEM;
	}
	len = requst->txlen;
	tx = spi_sensor->txbuffer;
	ms = spi_sensor->txbuffer + APDU_OFFSET;
	if(copy_from_user(ms, (const u8 __user *)(uintptr_t) requst->tx_buf,requst->txlen)){
		spi_sensor->master = 0;
		dev_err(&spi_sensor->spi->dev,"get user data error!\n");
		return -EFAULT;
	}
	
	*(tx + 0) = 0xef;
	*(tx + 1) = 0x01;
	*(tx + 2) = 0xe0;
	*(tx + 3) = 0x00;
	*(tx + 4) = (len >> 0) & 0xff;
	*(tx + 5) = (len >> 8) & 0xff;
	
	for(tmp = 0; tmp < len + 6; tmp++)
		ecc +=  *(tx + tmp);
		
	*(tx + len + 6) = (ecc >> 0) & 0xff;
	*(tx + len + 7) = (ecc >> 8) & 0xff;

#ifdef MTK_HC
	//wait ready high,wait 1s
	while(!mt_get_gpio_in(SF115_GPIO_READR)){
		
		timeout+=50;
		if(mt_get_gpio_in(SF115_GPIO_READR))
			break;
		
		if(timeout > SF115_POLL_DELAY ){
			dev_err(&spi_sensor->spi->dev,"wait device high ready single error!"); 
			//return -ERESTART;
			break;
		}

		udelay(50);
	};
	
	//在关闭中断响应的时间内不做收发包处理
	while(time_before(jiffies,spi_sensor->timeout));
	/*
	*if ready is high, sand a dummy data to let cs chang,
	*and wait device wake up ready.
	*/
	if(mt_get_gpio_in(SF115_GPIO_READR)){
		if (atomic_read(&ato_wake_byte_len))  // 1  32b
			spi_sensor_dummy_write(g_spi_sensor,32);
		else		// 0  1b
			spi_sensor_dummy_write(g_spi_sensor,1);
		if(mt_get_gpio_in(SF115_GPIO_READR)){
			//wait for device ready
			atomic_inc(&master);
			seconds = spi_sensor->requst->timeout?:1000;
			if(!wait_for_completion_timeout(&spi_sensor->completed,msecs_to_jiffies(seconds))){
				spi_sensor->master = 0;
				spi_sensor->timeout = jiffies +  HZ/2;//忽略接下来1S钟的中端。
				dev_err(&spi_sensor->spi->dev,"wait for ready single timeout seconds = %d!\n",seconds);
				atomic_dec(&master);
				return -ETIMEDOUT;
			}
		}
	}else{
		spi_sensor->master = 0;
		dev_err(&spi_sensor->spi->dev,"wake up device error,device busy!,"
			"a cs change shoule be shend again!\n");
		return -EIO;
	}
#else
	//wait ready high,wait 1s
	while(!gpio_get_value(SF115_GPIO_READR)){
		
		timeout+=50;
		if(gpio_get_value(SF115_GPIO_READR))
			break;
		
		if(timeout > SF115_POLL_DELAY ){
			dev_err(&spi_sensor->spi->dev,"wait device high ready single error!"); 
			//return -ERESTART;
			break;
		}

		udelay(50);
	};
	
	//在关闭中断响应的时间内不做收发包处理
	while(time_before(jiffies,spi_sensor->timeout));
	/*
	*if ready is high, sand a dummy data to let cs chang,
	*and wait device wake up ready.
	*/
	if(gpio_get_value(SF115_GPIO_READR)){
		if (atomic_read(&ato_wake_byte_len))  // 1  32b
			spi_sensor_dummy_write(g_spi_sensor,32);
		else		// 0  1b
			spi_sensor_dummy_write(g_spi_sensor,1);
		if(gpio_get_value(SF115_GPIO_READR)){
			//wait for device ready
			atomic_inc(&master);
			seconds = spi_sensor->requst->timeout?:1000;
			if(!wait_for_completion_timeout(&spi_sensor->completed,msecs_to_jiffies(seconds))){
				spi_sensor->master = 0;
				spi_sensor->timeout = jiffies +  HZ/2;//忽略接下来1S钟的中端。
				dev_err(&spi_sensor->spi->dev,"wait for ready single timeout seconds = %d!\n",seconds);
				atomic_dec(&master);
				return -ETIMEDOUT;
			}
		}
	}else{
		spi_sensor->master = 0;
		dev_err(&spi_sensor->spi->dev,"wake up device error,device busy!,"
			"a cs change shoule be shend again!\n");
		return -EIO;
	}
#endif	
	//now send apdu cmd to device,and let a mask remmber 
	status = sf115_send_cmd(spi_sensor,len);
	if(status) {
		//spi_sensor->master = 1;
		
		//LE = 0情况；只返回SW1SW2
		/*if(spi_sensor->requst->type == 0){
			//udelay(SF115_POLL_DELAY);
			while(!mt_get_gpio_in(SF115_GPIO_READR));
			if(mt_get_gpio_in(SF115_GPIO_READR)){
				//wait for device ready
				atomic_inc(&master);
				seconds = spi_sensor->requst->timeout?:1000;
				memset(spi_sensor->rxbuffer,0,spi_sensor->size);
				//printk("##########HZ=%d######seconds = %d#############\n",HZ,seconds);
				if(!wait_for_completion_timeout(&spi_sensor->completed,msecs_to_jiffies(seconds))){
					dev_err(&spi_sensor->spi->dev,"wait for ready single timeout!\n");
					spi_sensor->timeout = jiffies +  HZ;//忽略接下来1S钟的中端。
					spi_sensor->master = 0;
					atomic_dec(&master);
					return -ETIMEDOUT;
				}
			}
			status = sf115_recive_packeg(spi_sensor,2,spi_sensor->rxbuffer);
			if(status == 2){
				status = 0;	
				spi_sensor->status = 0;
				spi_sensor->rxlen = status;
			}
		}
		//LE != 0 时 处理APDU 
		else*/ if(!sf115_requst_fun(spi_sensor)){
			
			status = spi_sensor->status;
			rx = (unsigned char *) (spi_sensor->rxbuffer + spi_sensor->rxlen);
			switch (status)
			{
	 				case RT_FAI:
						*(rx + 0) = (CMD_FAIL >> 8) & 0xff;
						*(rx + 1) = (CMD_FAIL >> 0) & 0xff;
						break;
	 				case RT_PACKAGE_ERROR:
						*(rx + 0) = (CMD_PKG_ERR >> 8) & 0xff;
						*(rx + 1) = (CMD_PKG_ERR >> 0) & 0xff;
						break;
	 				case RT_PARAM_ERROR:
						*(rx + 0) = (CMD_PARM_ERR >> 8) & 0xff;
						*(rx + 1) = (CMD_PARM_ERR >> 0) & 0xff;						
						break;
	 				case RT_TIME_OUT:
						*(rx + 0) = (CMD_TIME_OUT >> 8) & 0xff;
						*(rx + 1) = (CMD_TIME_OUT >> 0) & 0xff;							
						break;
	 				case RT_ECC_ERROR:
						*(rx + 0) = (CMD_ECC_ERR >> 8) & 0xff;
						*(rx + 1) = (CMD_ECC_ERR >> 0) & 0xff;						
						break;
	 				case RT_COMMAND_ERROR:
						*(rx + 0) = (CMD_ERR >> 8) & 0xff;
						*(rx + 1) = (CMD_ERR >> 0) & 0xff;
						break;
	 				case RT_OK:
					    break;
					default:
						*(rx + 0) = (CMD_ERR >> 8) & 0xff;
						*(rx + 1) = (CMD_ERR >> 0) & 0xff;
						break;
			}
			
		}

		len = spi_sensor->rxlen + (!status ? 0 : 2);
		if(requst->rx_buf){
			if(requst->rxlen){
				//dev_info(&spi_sensor->spi->dev,"rxbuff  rxlen = %x len = %x\n",*requst->rxlen,len);
				if(len > (*requst->rxlen)){
					dev_err(&spi_sensor->spi->dev,"rx buff is big than user buff! rxlen = %d\n",*requst->rxlen);
					return -EFAULT;
				}
				__put_user(len,requst->rxlen);
			}
			if(requst->status)
				__put_user(status,requst->status);
			status = __copy_to_user((u8 __user *)(uintptr_t) requst->rx_buf,spi_sensor->rxbuffer,len);
		}
	} else{
		
		dev_err(&spi_sensor->spi->dev,"send requst cmd error!\n");
	}
	spi_sensor->master = 0;
	return status;
}
static int spi_sensor_message(struct spi_sensor_data *spi_sensor,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total;
	u8			*buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;
	
	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = spi_sensor->txbuffer;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		buf += k_tmp->len;
		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spi_sensor->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}
	status = spi_sensor_sync(spi_sensor, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = spi_sensor->txbuffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;

				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;
done:
	kfree(k_xfers);
	return status;
}


static int spi_sensor_set_imagesz(struct file *filp,ssize_t size)
{
	struct spi_sensor_data	*spi_sensor;
	spi_sensor = filp->private_data;
	if(size > 128*1024 ) 
		return -ENOMEM;
	
	if (spi_sensor->rxbuffer) 
		kfree(spi_sensor->rxbuffer);
	spi_sensor->rxbuffer = (void *)kzalloc(size,GFP_DMA);
	if (!spi_sensor->rxbuffer){
		dev_err(&spi_sensor->spi->dev,"malloc thesensor image buff error!\n");
		return -ENOMEM;
	}else {
		spi_sensor->size = size;
		return 0;
	}
}

static long
spi_sensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	u32	tmp;
	int	err = 0;
	int	retval = 0;
	unsigned n_ioc;
	struct spi_device	*spi;
	struct requst_cmd	*requst;
	struct spi_ioc_transfer	*ioc;
	struct spi_sensor_data	*spi_sensor;
#ifdef MTK_HC
	struct mt_chip_conf* spi_par;
	spi_par =&chip_config;
#endif
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spi_sensor = filp->private_data;
	spin_lock_irq(&spi_sensor->spi_lock);
	spi = spi_dev_get(spi_sensor->spi);
	spin_unlock_irq(&spi_sensor->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for double duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 */
	mutex_lock(&spi_sensor->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_WAKEBYTELEN_1B:
		atomic_set(&ato_wake_byte_len, 0);	
		break;
	case SPI_IOC_WAKEBYTELEN_32B:
		atomic_set(&ato_wake_byte_len, 1);	
		break;	
	case SPI_IOC_INPUT_OFF:
		atomic_set(&ato_input_fun, 0);	
		break;
	case SPI_IOC_INPUT_ON:
		atomic_set(&ato_input_fun, 1);	
		break;
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		spi_sensor->speed = spi->max_speed_hz;
		break;

	/* write requests */
	case SPI_IOC_WR_MODE: 
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
 
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			} 
			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;
			spi->max_speed_hz = tmp;
#ifdef MTK_HC		
		if(!spi_par || (spi->max_speed_hz > 100*1000000)){
			dev_err(&spi->dev, "set spi speed error!\n");
			retval = -EFAULT;
			break;
		}
		printk("set sf115 spi speed %d\n",spi->max_speed_hz);
		//SPI_CLOCK_PERIED is 100000 kHz
		u8 time = 1000000 * 100 / spi->max_speed_hz;
		if(spi->max_speed_hz == 4000000)
			spi_par->high_time  = spi_par->low_time = 15;//4M
		else if (spi->max_speed_hz == 1000000)
			spi_par->high_time  = spi_par->low_time = 60;//1M
		else 
			spi_par->high_time  = spi_par->low_time = time / 2;
		
		printk("set sf115 spi speed high = %d low = %d\n",spi_par->high_time,spi_par->low_time);
#endif
		retval = spi_setup(spi);
		if (retval < 0)
			spi->max_speed_hz = save;
		else
			dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		
		}
		
		break;
	case SPI_IOC_COS_ISP:
		retval = spi_sensor_update(filp);
		break;

	case SPI_IOC_SET_IMAGE:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			 spi_sensor_set_imagesz(filp,tmp);
		}
		break;
	/*
	* send a dummy byte ,let cs change and wake up device !
	*/
	case SPI_IOC_SENSOR_RST:

		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
#ifdef MTK_HC		
			spi_sensor_reset(tmp);
#endif
			printk("reset sensor suceffually!\n");
			retval = 0;
		}
		break;
	case SPI_IOC_REQUST_CMD:

		if (!spi_sensor->requst) {
			printk("requst memory error!\n");
			retval = -ENOMEM;
			break;
		}

		
		requst = spi_sensor->requst;
		memset(requst,0,sizeof(struct requst_cmd));
		if (__copy_from_user(requst, (void __user *)arg, sizeof(struct requst_cmd))) {
			printk("copy from user error!\n");
			retval = -EFAULT;
			break;
		}

		
		if(requst->txlen > (bufsiz - CMD_LENGTH)){
			
			dev_err(&spi_sensor->spi->dev,"tx buff error,buff overfllow ,"
				"txbuff length is %d!,alviable size is %d\n",bufsiz,bufsiz - CMD_LENGTH);
				retval = -ENOMEM;
				break;
		}
		
		retval = spi_sensor_requst(spi_sensor, requst);
		break;
	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}
		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;
		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}

		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		retval = spi_sensor_message(spi_sensor, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spi_sensor->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spi_sensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return spi_sensor_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spi_sensor_compat_ioctl NULL
#endif /* CONFIG_COMPAT */
static int spi_sensor_open(struct inode *inode, struct file *filp)
{
	
	int	status = -ENXIO;
	struct spi_sensor_data	*spi_sensor;
	mutex_lock(&device_list_lock);
	list_for_each_entry(spi_sensor, &device_list, device_entry) {
		if (spi_sensor->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (spi_sensor->users > 0) {
		mutex_unlock(&device_list_lock);
		printk(KERN_ERR "Spi sensor: %s: Too many users\n", __func__);
		return -EPERM;
	}

	if (status == 0) {
		if (!spi_sensor->txbuffer) {
			spi_sensor->txbuffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spi_sensor->txbuffer) {
				dev_dbg(&spi_sensor->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			spi_sensor->users++;
			filp->private_data = spi_sensor;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("spi_sensor: nothing for minor %d\n", iminor(inode));
	spi_sensor->master = 0; 
	spi_sensor->timeout = jiffies;	
	mutex_unlock(&device_list_lock);
	return status;
}

static int spi_sensor_release(struct inode *inode, struct file *filp)
{
	struct spi_sensor_data	*spi_sensor;
	int			status = 0;

	spi_sensor = filp->private_data;	
	
	mutex_lock(&device_list_lock);

	/* last close? */
	spi_sensor->users--;
	if (!spi_sensor->users) {
		int		dofree;

		kfree(spi_sensor->txbuffer);
		spi_sensor->txbuffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spi_sensor->spi_lock);
		dofree = (spi_sensor->spi == NULL);
		spin_unlock_irq(&spi_sensor->spi_lock);

		if (dofree){
			
			//kfree(spi_sensor);
			dev_err(&spi_sensor->spi->dev,"spi_sensor_release spi_sensor!\n");
		}
		spi_sensor_fasync(-1, filp, 0);	
	}
	filp->private_data = NULL;
	mutex_unlock(&device_list_lock); 

	return status;
}
/**********************add 2014/12/17 for sf115 sensor**************************/

static int spi_sensor_fasync(int fd, struct file * filp, int on) 
{
    int retval;
	struct spi_sensor_data	*spi_sensor;
	spi_sensor = filp->private_data;
	
    retval=fasync_helper(fd,filp,on,&spi_sensor->fasync_q);
    if(retval<0)
      return retval;
	
    return 0;
}
#if 0
unsigned int spi_sensor_poll(struct file *filp, poll_table *wait)
{
    struct spi_sensor_data	*spi_sensor;
    unsigned int mask = 0;
	spi_sensor = filp->private_data;
    poll_wait(filp, &spi_sensor->inq,  wait);
  	//poll_wait(filp, &spi_sensor->outq,  wait);
    
   // if (spi_sensor->wready)  mask |=  POLLOUT | POLLWRNORM;  /* writeable */

	if (spi_sensor->rready > 0) { 
		mask |=  POLLIN  | POLLRDNORM;  /* readable */
	}
	
	if(spi_sensor->rready < 0) { 
		mask |= POLLERR;
	}
	
    return mask;
}
#endif
static void sf115_fasync_work(struct work_struct *work )
{
	struct spi_sensor_data	*spi_sensor;
	spi_sensor = container_of(work,struct spi_sensor_data,fasync_wq);
	if (atomic_read(&ato_wake_byte_len))  // 1  32b
	spi_sensor_dummy_write(g_spi_sensor,32);//power disable
	else		// 0  1b
		spi_sensor_dummy_write(g_spi_sensor,1);
    if (spi_sensor->fasync_q)
      kill_fasync(&spi_sensor->fasync_q, SIGIO, POLL_IN);
	
}
#ifdef MTK_HC
//在中断上半部检测ready信号，
//如果为为主机发送则表示从机准备完成。
//如果为从机唤醒则触发下半部中断读取图像。
//master == 1 主机控制发送
//master == 0 从机中断触发
static irqreturn_t sf115_hard_irq(void)
{
	//struct spi_sensor_data	*spi_sensor = handle; 
	if(!g_spi_sensor){
		printk("g_spi_sensor is null!\n");
		return IRQ_HANDLED;
	}	
	//Remove the GPIO jitter
	if(mt_get_gpio_in(SF115_GPIO_READR))
		return IRQ_HANDLED;

	if(mt_get_gpio_in(SF115_GPIO_READR))
		return IRQ_HANDLED;

	if(mt_get_gpio_in(SF115_GPIO_READR))
		return IRQ_HANDLED;

	if(mt_get_gpio_in(SF115_GPIO_READR))
		return IRQ_HANDLED;
	
	if(mt_get_gpio_in(SF115_GPIO_READR))
		return IRQ_HANDLED;

	g_spi_sensor->high_mising = 1;
	if(atomic_dec_and_test(&master)){
	
		complete(&g_spi_sensor->completed);
		return IRQ_HANDLED;
	}
	 
	if(g_spi_sensor->master){
		atomic_set(&master,0);
		return IRQ_HANDLED;
	}
	if(time_before(jiffies,g_spi_sensor->timeout)){
		atomic_set(&master,0);
		return IRQ_HANDLED;
	}
	if(!atomic_inc_return(&master)){
		//spi_sensor_dummy_write(g_spi_sensor,4);//power disable
		if (atomic_read(&ato_input_fun))
			mod_timer(&g_spi_sensor->timer, jiffies + msecs_to_jiffies(500));
		schedule_work(&g_spi_sensor->fasync_wq);
	}
	
	return IRQ_HANDLED;
	//return !atomic_inc_return(&master)? IRQ_WAKE_THREAD : IRQ_HANDLED;
}
#else
//在中断上半部检测ready信号，
//如果为为主机发送则表示从机准备完成。
//如果为从机唤醒则触发下半部中断读取图像。
//master == 1 主机控制发送
//master == 0 从机中断触发
static irqreturn_t sf115_hard_irq(int irq, void *handle)
{
	struct spi_sensor_data	*spi_sensor = handle; 
	if(!spi_sensor){
		printk("spi_sensor is null!\n");
		return IRQ_HANDLED;
	}
	
	//Remove the GPIO jitter
	if(gpio_get_value(SF115_GPIO_READR))
		return IRQ_HANDLED;

	if(gpio_get_value(SF115_GPIO_READR))
		return IRQ_HANDLED;

	if(gpio_get_value(SF115_GPIO_READR))
		return IRQ_HANDLED;

	if(gpio_get_value(SF115_GPIO_READR))
		return IRQ_HANDLED;
	
	if(gpio_get_value(SF115_GPIO_READR))
		return IRQ_HANDLED;

	spi_sensor->high_mising = 1;
	if(atomic_dec_and_test(&master)){
	
		complete(&spi_sensor->completed);
		return IRQ_HANDLED;
	}
	 
	if(spi_sensor->master){
		atomic_set(&master,0);
		return IRQ_HANDLED;
	}
	if(time_before(jiffies,spi_sensor->timeout)){
		atomic_set(&master,0);
		return IRQ_HANDLED;
	}
	if(!atomic_inc_return(&master)){
		//spi_sensor_dummy_write(g_spi_sensor,4);//power disable
		if (atomic_read(&ato_input_fun))
			mod_timer(&spi_sensor->timer, jiffies + msecs_to_jiffies(500));
		schedule_work(&spi_sensor->fasync_wq);
	}
	
	return IRQ_HANDLED;
	//return !atomic_inc_return(&master)? IRQ_WAKE_THREAD : IRQ_HANDLED;
}

#endif
static irqreturn_t  sf115_irq(int irq, void *handle)
{
	struct spi_sensor_data	*spi_sensor = handle;
	//mutex_lock(&spi_sensor->buf_lock);
 	schedule_work(&spi_sensor->fasync_wq);
	//mutex_unlock(&spi_sensor->buf_lock);
	/*
	*Move to the application layer
	*cndy 2014-12-30.
	*/
#if 0
	atomic_inc(&master);
	
	//sleep and wait here for ready.
    wait_for_completion(&spi_sensor->completed); 
	//确认ready被拉低
	if(!mt_get_gpio_in(SF115_GPIO_READR)){
		
		/********获取sf115图片********/

	}
	atomic_dec(&master);
#endif 	
	return IRQ_HANDLED;
}


static void sf115_power_timer(unsigned long data)
{
	struct spi_sensor_data	*spi_sensor = (struct spi_sensor_data	*)data;

	input_report_key(spi_sensor->input_dev, KEY_POWER, 1);	
	input_sync(spi_sensor->input_dev);	
	input_report_key(spi_sensor->input_dev, KEY_POWER, 0);	
	input_sync(spi_sensor->input_dev);
}

/*
static void sf115_cos_isp(const struct firmware *fw, void *context)
{
	///update cos  here !
}
*/

static int spi_sensor_update(struct file *filp)
{
	int ret;
	const struct firmware *fw = NULL;
	struct spi_sensor_data	*spi_sensor;
	spi_sensor = filp->private_data;

	/*ret = request_firmware_nowait(THIS_MODULE,!NULL,"as503.hex",\
					spi_sensor->spi->dev,spi_sensor,sf115_cos_isp);*/
					
	ret = request_firmware(&fw,"as503.hex",&spi_sensor->spi->dev);
	if(ret){

			;/** update cos  here !**/
	}
	
	release_firmware(fw);

	return ret;
}

static const struct file_operations spi_sensor_fops = {
	.owner 			=	THIS_MODULE,
	.write 			=	spi_sensor_write,
	.read 			=	spi_sensor_read,
	.unlocked_ioctl = 	spi_sensor_ioctl,
	.compat_ioctl 	= 	spi_sensor_compat_ioctl,
	.open	 		=	spi_sensor_open,
	.release 		=	spi_sensor_release,
	//.poll 			=	spi_sensor_poll,
	.fasync			=	spi_sensor_fasync,
};

/*---------------------------the end----------------------------------------*/
 
static struct class *spi_sensor_class;
/*-------------------------------------------------------------------------*/

static int  spi_sensor_probe(struct spi_device *spi)
{
	
	int	status = 0;
	unsigned long minor;
	static struct spi_sensor_data	*spi_sensor;

	spi_sensor = kzalloc(sizeof(*spi_sensor), GFP_KERNEL);
	if (!spi_sensor)
		return -ENOMEM;

	spi_sensor->spi = spi;
	spin_lock_init(&spi_sensor->spi_lock);
	mutex_init(&spi_sensor->buf_lock);

	INIT_LIST_HEAD(&spi_sensor->device_entry);

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spi_sensor->devt = MKDEV(SPISENSOR_MAJOR, minor);
		dev = device_create(spi_sensor_class, &spi->dev, spi_sensor->devt,
				    spi_sensor, "SF115");
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
		
	} else {
		
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spi_sensor->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if ( !status )
		spi_set_drvdata(spi, spi_sensor);
	else
		goto spi_err;
	
	//malloc default rx buff
	spi_sensor->size = 4096;
	spi_sensor->rxbuffer = (void *)kzalloc(4096,GFP_KERNEL);
	if(!spi_sensor->rxbuffer){
		spi_sensor->size = 0;
		dev_info(&spi->dev,"rx buffer malloc error!\n");
		goto buff_err;
	}
	
	spi_sensor->requst = kmalloc(sizeof(struct requst_cmd),GFP_KERNEL);
	if (!spi_sensor->requst) {
		dev_info(&spi->dev,"malloc requst buffer error!\n");
		goto buff_err;
	}

	spi_sensor->input_dev = input_allocate_device();
	if (!spi_sensor->input_dev) {
		printk("Error: no memory\n");		
		return -EBUSY;	
	}
	
	spi_sensor->input_dev->evbit[0] =  BIT_MASK(EV_KEY);     /*设置按键信息*/  
	set_bit(KEY_POWER,spi_sensor->input_dev->keybit);
	spi_sensor->input_dev->name = "input_dev";
	spi_sensor->input_dev->phys = "input(mt)";
	spi_sensor->input_dev->id.bustype = BUS_HOST;
	spi_sensor->input_dev->id.vendor = 0xDEAD;
	spi_sensor->input_dev->id.product = 0xBEEF;
	spi_sensor->input_dev->id.version = 0x0102;
	 
	status = input_register_device(spi_sensor->input_dev);
	if (status) {
		printk(" Failed to register intput device\n");  
		input_free_device(spi_sensor->input_dev);
	}	
	//set reset gpio high
#ifdef MTK_HC
	spi_sensor_reset(1);
#endif
	g_spi_sensor = spi_sensor;
	init_completion(&spi_sensor->completed);
	INIT_WORK(&spi_sensor->fasync_wq,sf115_fasync_work);
	setup_timer(&spi_sensor->timer, sf115_power_timer,(unsigned long)spi_sensor);
#ifdef MTK_HC
	mt_eint_set_hw_debounce(CUST_EINT_FINGER_NUM,1);	
	mt_eint_registration(CUST_EINT_FINGER_NUM, EINTF_TRIGGER_FALLING, sf115_hard_irq, 1);
#else
	spi_sensor->irq_read = gpio_to_irq(SF115_GPIO_READR);	
	status = init_gpio(spi_sensor);
	if (status) {		
		dev_info(&spi->dev,"Init GPIO error!\n");		
		goto buff_err;
	}		
	status = request_irq(spi_sensor->irq_read, sf115_hard_irq, 
			IRQ_TYPE_EDGE_FALLING, spi->dev.driver->name, spi_sensor);
	if (status) {		
		dev_info(&spi->dev,"trying pin irq %d error!\n", spi_sensor->irq_read);			
		goto buff_err;		
	}		

#endif


	return status;
	
buff_err:
		if(spi_sensor->rxbuffer)
			kfree(spi_sensor->rxbuffer);
spi_err:	
	if(spi_sensor)
		kfree(spi_sensor);
	return status;
}

static int  spi_sensor_remove(struct spi_device *spi)
{
	struct spi_sensor_data	*spi_sensor = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spi_sensor->spi_lock);
	spi_sensor->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spi_sensor->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spi_sensor->device_entry);
	device_destroy(spi_sensor_class, spi_sensor->devt);
	clear_bit(MINOR(spi_sensor->devt), minors);
	input_free_device(spi_sensor->input_dev);
	if(spi_sensor->rxbuffer){
		kfree(spi_sensor->rxbuffer);
		spi_sensor->rxbuffer = NULL;
	}
	
	
	
	if (!spi_sensor->requst)
		kfree(spi_sensor->requst);
		
	if (spi_sensor->users == 0)
		kfree(spi_sensor);
	
	del_timer_sync(&spi_sensor->timer);
	mutex_unlock(&device_list_lock);

	return 0;
}

#ifdef MTK_HC
static int mt_spi_gpio_set(void)
	{
#ifdef SF115_GPIO_3V3
		mt_set_gpio_mode(SF115_GPIO_3V3, GPIO_MODE_00);
		mt_set_gpio_dir(SF115_GPIO_3V3, GPIO_DIR_OUT);
		mt_set_gpio_out(SF115_GPIO_3V3, GPIO_OUT_ONE);
#endif
#ifdef SF115_GPIO_1V8
		mt_set_gpio_mode(SF115_GPIO_1V8, GPIO_MODE_00);
		mt_set_gpio_dir(SF115_GPIO_1V8, GPIO_DIR_OUT);
		mt_set_gpio_out(SF115_GPIO_1V8, GPIO_OUT_ONE);
#endif
		mdelay(1);
		mt_set_gpio_mode(SF115_GPIO_RST, GPIO_MODE_00);
		mt_set_gpio_dir(SF115_GPIO_RST, GPIO_DIR_OUT);
		mt_set_gpio_out(SF115_GPIO_RST, GPIO_OUT_ONE);

		mt_set_gpio_mode(SPI_CS_PIN, GPIO_SPI_CS_PIN_M_SPI_CSA);
		mt_set_gpio_dir(SPI_CS_PIN, GPIO_DIR_OUT);
		mt_set_gpio_pull_enable(SPI_CS_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(SPI_CS_PIN,GPIO_PULL_UP);

		mt_set_gpio_mode(SPI_SCK_PIN, SPI_SCK_PIN_MODE);
		mt_set_gpio_dir(SPI_SCK_PIN, GPIO_DIR_OUT);
		mt_set_gpio_pull_enable(SPI_SCK_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(SPI_SCK_PIN,GPIO_PULL_UP);

		mt_set_gpio_mode(SPI_MISO_PIN, SPI_MISO_PIN_MODE);
		mt_set_gpio_dir(SPI_MISO_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(SPI_MISO_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(SPI_MISO_PIN,GPIO_PULL_UP);
		
		mt_set_gpio_mode(SPI_MOSI_PIN, SPI_MOSI_PIN_MODE);
		mt_set_gpio_dir(SPI_MOSI_PIN, GPIO_DIR_OUT);
		mt_set_gpio_pull_enable(SPI_MOSI_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(SPI_MOSI_PIN,GPIO_PULL_UP);
		//ready 
		mt_set_gpio_mode(SF115_GPIO_READR, GPIO_CTP_EINT_PIN_M_EINT);
		mt_set_gpio_dir(SF115_GPIO_READR, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(SF115_GPIO_READR, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(SF115_GPIO_READR,GPIO_PULL_UP);

		//reset
		mt_set_gpio_mode(SF115_GPIO_RST, SF115_GPIO_MODE);
		mt_set_gpio_dir(SF115_GPIO_RST, GPIO_DIR_OUT);
		mt_set_gpio_pull_enable(SF115_GPIO_RST, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(SF115_GPIO_RST,GPIO_PULL_UP);
		
		return 0;
	}	
#endif
static struct spi_driver spi_sensor_driver = {
	.driver = {
		.name =		"spi_sensor",
		.owner =	THIS_MODULE,
	},
	.probe =	spi_sensor_probe,
	.remove =spi_sensor_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.
	 */
};

#ifdef MTK_HC
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias = "spi_sensor",
		.max_speed_hz = 4*1000*1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data=&chip_config
	}
};
#endif
/*-------------------------------------------------------------------------*/
struct cdev spisensor_dev;
static int __init spi_sensor_init(void)
{
	int status;
	dev_t devno;
    printk("synochip spi_sensor driver_init \n");
    
#ifdef MTK_HC	
	mt_spi_gpio_set();
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif

	status = alloc_chrdev_region(&devno, 0, 255, "spi_sensor");//dynamic alloc 

	if(status < 0)
		return status;
	
	SPISENSOR_MAJOR = MAJOR(devno);
	cdev_init(&spisensor_dev, &spi_sensor_fops);	
	spisensor_dev.owner = THIS_MODULE;
	status = cdev_add(&spisensor_dev,MKDEV(SPISENSOR_MAJOR, 0), 1/*N_SPI_MINORS*/);
	if(status != 0)
		return status;
	
	spi_sensor_class = class_create(THIS_MODULE, "spi_sensor");
	if (IS_ERR(spi_sensor_class)) {
		unregister_chrdev(SPISENSOR_MAJOR, spi_sensor_driver.driver.name);
		return PTR_ERR(spi_sensor_class);
	}
	
	status = spi_register_driver(&spi_sensor_driver);
	if (status < 0) {
		class_destroy(spi_sensor_class);
		unregister_chrdev(SPISENSOR_MAJOR, spi_sensor_driver.driver.name);
	}
	return status;
}
module_init(spi_sensor_init);

static void __exit spi_sensor_exit(void)
{
	
	cdev_del(&spisensor_dev);
	spi_unregister_driver(&spi_sensor_driver);
	class_destroy(spi_sensor_class);
}
module_exit(spi_sensor_exit);


MODULE_AUTHOR("cndychen@foxmail.com");
MODULE_DESCRIPTION("User mode SPI sensor device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:SF115 Sensor Driver");
