/*
 * drivers/fps1196/fps1196.c 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This file is based on kernel/irq/devres.c
 *
 * Copyright (c) 2011 John Crispin <blogic@openwrt.org>
 */



#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <mach/irqs.h>
#include <linux/kthread.h>
#include <mach/mt_spi.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <mach/mt_gpio.h>
#include <mach/emi_mpu.h>
#include <mach/mt_clkmgr.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <cust_eint.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/input.h>
#include <mach/eint.h>
#include <linux/switch.h>


#define SPI_DRV_NAME	"fps1196"
#define fps1196_height  96
#define fps1196_width   112
#define fps1196_image_size (fps1196_height*fps1196_width)
#define FPS1196_SPI_CLOCK_SPEED 10*1000*1000

#define FPS1196_IOCTL_MAGIC_NO			0xFB

#define FPS1196_INIT				 _IO (FPS1196_IOCTL_MAGIC_NO, 0)
#define FPS1196_GETIMAGE			_IOW(FPS1196_IOCTL_MAGIC_NO, 1,uint32_t)
#define FPS1196_INITERRUPT_MODE		        _IOW(FPS1196_IOCTL_MAGIC_NO, 2, uint32_t)
#define FPS1196_AGC_CADJUST		        _IOW(FPS1196_IOCTL_MAGIC_NO, 3, uint8_t)
#define FPS1196_POWERDOWN_MODE			_IO (FPS1196_IOCTL_MAGIC_NO,4)

#define KEY_INTERRUPT KEY_POWER


extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);


static struct fps1196_data {
	struct spi_device *spi;
	struct class *class;
	struct device *device;
	struct cdev wg_cdev;
	dev_t devno;
        spinlock_t              spi_lock;
        struct mutex buf_lock;
        struct list_head        device_entry;
        unsigned  users;
};

static DEFINE_MUTEX(device_list_lock);
static LIST_HEAD(device_list);

static int   mt_spi_init(void);
static void  mt_spi_exit(void);
static int   fps1196_probe(struct spi_device *spi);
static int   fps1196_open(struct inode *inode, struct file *file);
static ssize_t fps1196_write(struct file *file, const char *buff,size_t count, loff_t *ppos);
static ssize_t fps1196_read(struct file *file, char *buff,size_t count, loff_t *ppos);
static int  spi_send_cmd(struct fps1196_data *fps1196,u8 *tx,u8 *rx,u16 spilen);
static int  fps1196_dev_init(struct fps1196_data *spidev);
static long fps1196_ioctl(struct file *filp, unsigned int cmd,unsigned long arg) ;
static int fps1196_read_image(struct fps1196_data *fps1196,uint32_t timeout);
static void destroy_fps1196_device (struct fps1196_data * fps1196);
static int fps1196_release(struct inode *inode, struct file *file);
static void fps1196_exit_gpio_init(void);
static  void fps1196_eint_handler(void);
static int fps1196_create_inputdev(void);
static int fps1196_thread_func(void *unused);
static int fps1196_dev_interrupt_init(struct fps1196_data *fps1196);
static int fps1196_agc_init(struct fps1196_data *fps1196,unsigned long arg);
static int spi_send_cmd_fifo(struct fps1196_data *fps1196,u8 *tx,u8 *rx,u16 spilen);
static int mtspi_set_mode(int mode);
static void int_timer_handle(unsigned long arg);
static void fps1196_powerdown_mode(struct fps1196_data* fps1196);
static u8 imagebuf[10752]={0x00};
static struct fps1196_data *g_fps1196= NULL;
static struct input_dev *fps1196_inputdev = NULL;
static struct task_struct *fps1196_thread = NULL;

//Declare and initialize the queue
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int fps1196_interrupt_flag = 0;
static int interrupt_mode_flag = 0;
u8 imagetxcmd [12288] = {0x00} ;
u8 imagerxpix [12288] = {0x00};

//Timer operation
static uint8_t int_count = 0;
static struct timer_list int_timer;

struct device *fps1196_device;

#define GPIO_FPS1196_RESET_PIN       GPIO_FIGERPRINT_RST
#define GPIO_FPS1196_POWER_PIN       GPIO_FIGERPRINT_PWR_EN_PIN 


#define		FPS1196_SPI_SCK_PIN		GPIO_SPI_SCK_PIN
#define		FPS1196_SPI_SCK_PIN_M_GPIO	GPIO_MODE_00
#define		FPS1196_SPI_SCK_PIN_M_SCK	GPIO_SPI_SCK_PIN_M_SPI_CKA

#define		FPS1196_SPI_CS_PIN		GPIO_SPI_CS_PIN
#define		FPS1196_SPI_CS_PIN_M_GPIO	GPIO_MODE_00
#define		FPS1196_SPI_CS_PIN_M_CS	        GPIO_SPI_CS_PIN_M_SPI_CSA
	
#define		FPS1196_SPI_MOSI_PIN		GPIO_SPI_MOSI_PIN 
#define		FPS1196_SPI_MOSI_PIN_M_GPIO	GPIO_MODE_00
#define		FPS1196_SPI_MOSI_PIN_M_MOSI	GPIO_SPI_MOSI_PIN_M_SPI_MOA

#define		FPS1196_SPI_MISO_PIN		GPIO_SPI_MISO_PIN
#define		FPS1196_SPI_MISO_PIN_M_GPIO	GPIO_MODE_00
#define		FPS1196_SPI_MISO_PIN_M_MISO	GPIO_SPI_MISO_PIN_M_SPI_MIA



//high_time and low_time Decision SCK frequency , FPS1196 Using 10M
static struct mt_chip_conf spi_conf=
{
 	.setuptime = 7,
	.holdtime = 7,
	.high_time = 7, 
	.low_time =  7,
	.cs_idletime = 6,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,  
	.tx_mlsb = 1,

	.tx_endian = 0, 
	.rx_endian = 0,

	.com_mod = DMA_TRANSFER,
	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
static struct spi_driver fps1196_driver = {
	.driver = {
		.name	= "fps1196",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= fps1196_probe,
};

static struct spi_board_info spi_board_fps1196[] __initdata = {
	[0] = {
		.modalias= "fps1196",
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
		.max_speed_hz = 16000000,
	},
};

static const struct file_operations fps1196_fops = {
	.owner = THIS_MODULE,
	.open  = fps1196_open,
	.write = fps1196_write,
	.read  = fps1196_read,
	.release = fps1196_release,
	.unlocked_ioctl = fps1196_ioctl,
};


// Configure IO to make it work in SPI mode	
static void set_spi_mode(int enable)
{
   if(enable)
	{
		mt_set_gpio_mode(FPS1196_SPI_CS_PIN,FPS1196_SPI_CS_PIN_M_CS);
                mt_set_gpio_pull_enable(FPS1196_SPI_CS_PIN,GPIO_PULL_ENABLE);
                mt_set_gpio_pull_select(FPS1196_SPI_CS_PIN,GPIO_PULL_UP);

                mt_set_gpio_mode(FPS1196_SPI_SCK_PIN,FPS1196_SPI_SCK_PIN_M_SCK);
                mt_set_gpio_pull_enable(FPS1196_SPI_SCK_PIN,GPIO_PULL_ENABLE);
                mt_set_gpio_pull_select(FPS1196_SPI_SCK_PIN,GPIO_PULL_DOWN);

                mt_set_gpio_mode(FPS1196_SPI_MISO_PIN,FPS1196_SPI_MISO_PIN_M_MISO);
                mt_set_gpio_pull_enable(FPS1196_SPI_MISO_PIN,GPIO_PULL_ENABLE);
                mt_set_gpio_pull_select(FPS1196_SPI_MISO_PIN,GPIO_PULL_DOWN);

                 mt_set_gpio_mode(FPS1196_SPI_MOSI_PIN,FPS1196_SPI_MOSI_PIN_M_MOSI);
                 mt_set_gpio_pull_enable(FPS1196_SPI_MOSI_PIN,GPIO_PULL_ENABLE);
                 mt_set_gpio_pull_select(FPS1196_SPI_MOSI_PIN,GPIO_PULL_DOWN);
                
                //RST pin from low to high
                mt_set_gpio_mode(GPIO_FPS1196_RESET_PIN , GPIO_MODE_00);
                mt_set_gpio_dir(GPIO_FPS1196_RESET_PIN , GPIO_DIR_OUT);
                mt_set_gpio_out(GPIO_FPS1196_RESET_PIN , GPIO_OUT_ZERO);
		mdelay(200);
                mt_set_gpio_out(GPIO_FPS1196_RESET_PIN , GPIO_OUT_ONE);

	}
	else{
		
                 mt_set_gpio_mode(FPS1196_SPI_CS_PIN,FPS1196_SPI_CS_PIN_M_GPIO);
                 mt_set_gpio_dir(FPS1196_SPI_CS_PIN,GPIO_DIR_IN);
                 mt_set_gpio_pull_enable(FPS1196_SPI_CS_PIN,GPIO_PULL_DISABLE);

                 mt_set_gpio_mode(FPS1196_SPI_SCK_PIN,FPS1196_SPI_SCK_PIN_M_GPIO);
                 mt_set_gpio_dir(FPS1196_SPI_SCK_PIN,GPIO_DIR_IN);
                 mt_set_gpio_pull_enable(FPS1196_SPI_SCK_PIN,GPIO_PULL_DISABLE);

                 mt_set_gpio_mode(FPS1196_SPI_MISO_PIN,FPS1196_SPI_MISO_PIN_M_GPIO);
                 mt_set_gpio_dir(FPS1196_SPI_MISO_PIN,GPIO_DIR_IN);
                 mt_set_gpio_pull_enable(FPS1196_SPI_MISO_PIN,GPIO_PULL_DISABLE);

                 mt_set_gpio_mode(FPS1196_SPI_MOSI_PIN,FPS1196_SPI_MOSI_PIN_M_GPIO);
                 mt_set_gpio_dir(FPS1196_SPI_MOSI_PIN,GPIO_DIR_IN);
                 mt_set_gpio_pull_enable(FPS1196_SPI_MOSI_PIN,GPIO_PULL_DISABLE);
	}
}

// Select SPI work mode
static int mtspi_set_mode(int mode)
{
  struct mt_chip_conf* spi_par;
  spi_par = &spi_conf;
  if (!spi_par)
  {
    return -1;
  }
  if (1 == mode)
  {
     if (spi_par-> com_mod == DMA_TRANSFER)
     {
         return 0;
     }
     spi_par-> com_mod = DMA_TRANSFER;
  }
  else
  {
    if (spi_par-> com_mod == FIFO_TRANSFER)
     {
         return 0;
     }
     spi_par-> com_mod = FIFO_TRANSFER;
  }

   spi_setup(g_fps1196->spi);
   return 0;
}


// SPI DMA mode
static int spi_send_cmd(struct fps1196_data *fps1196,u8 *tx,u8 *rx,u16 spilen)
{
    int ret=0;
    struct spi_message m;
    struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 5,
		.speed_hz = FPS1196_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = rx,
		.len = spilen,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
        mtspi_set_mode(1);  // dma 模式
   	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret= spi_sync(fps1196->spi,&m);
	return ret;   
}

// SPI FIFO mode
static int spi_send_cmd_fifo(struct fps1196_data *fps1196,u8 *tx,u8 *rx,u16 spilen)
{
    int ret=0;
    struct spi_message m;
    struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 5,
		.speed_hz = FPS1196_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = rx,
		.len = spilen,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
        mtspi_set_mode(0);  // fifo 模式
   	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret= spi_sync(fps1196->spi,&m);
	return ret;   
}

 
static long fps1196_ioctl(struct file *filp, unsigned int cmd,unsigned long arg) 
{
struct fps1196_data *fps1196 = filp->private_data;
struct spi_device *spi;
int error=0;
spi = spi_dev_get(fps1196->spi) ;

mutex_lock (&fps1196->buf_lock);
switch (cmd)
  {
 	case FPS1196_INIT:
		error= fps1196_dev_init(fps1196);
		break;
	case FPS1196_GETIMAGE:
		error = fps1196_read_image(fps1196,arg);
		break;
	case FPS1196_INITERRUPT_MODE:
		error = fps1196_dev_interrupt_init(fps1196);
		break; 
        case FPS1196_AGC_CADJUST :
		error= fps1196_agc_init(fps1196,arg);
		break;
        case FPS1196_POWERDOWN_MODE:
		fps1196_powerdown_mode(g_fps1196);
		break;
       default:
		error = -ENOTTY;
	        break;
	
  }
 mutex_unlock(&fps1196->buf_lock);
	return error;

}


static int fps1196_open(struct inode *inode, struct file *file)
{
struct fps1196_data *fps1196;
int status = -ENXIO;
mutex_lock(&device_list_lock); //Apply lock
list_for_each_entry(fps1196, &device_list, device_entry)
  {
     if (fps1196->devno == inode->i_rdev)
      {
        status = 0;
        break;
      }
  }
 if (status == 0)
  {
    fps1196->users++;
    file->private_data = fps1196;
    nonseekable_open(inode, file);
  }
  else
  {
    printk("spidev: nothing for minor %d\n", iminor(inode));
  }
 mutex_unlock(&device_list_lock); //free lock
 return status;
}


static ssize_t fps1196_write(struct file *file, const char *buff,size_t count, loff_t *ppos)
{
  return -ENOMEM;
}


static ssize_t fps1196_read(struct file *file, char  *buff,size_t count, loff_t *ppos)
{
  int ret=0;
  struct fps1196_data *fps1196 = file->private_data;
  ssize_t status = 0;
  struct spi_device *spi;
  mutex_lock (&fps1196->buf_lock);
  spi = spi_dev_get(fps1196->spi) ;
  ret = copy_to_user(buff,imagebuf,count);
  if (ret)
   {
    status = -EFAULT;
   }
  mutex_unlock(&fps1196->buf_lock);
return status;
}


static int fps1196_release(struct inode *inode, struct file *file)
{

  struct fps1196_data *fps1196;
  int status = 0;
  mutex_lock(&device_list_lock);
  fps1196 = file->private_data;
  file->private_data = NULL;
  fps1196->users--;
  if (!fps1196->users)
   {
    int dofree;
    spin_lock_irq(&fps1196->spi_lock);
    dofree = (fps1196->spi == NULL);
    spin_unlock_irq(&fps1196->spi_lock);
    if (dofree)
          kfree(fps1196);
   }
  mutex_unlock(&device_list_lock);
  return status;

}


static int  fps1196_probe(struct spi_device *spi)
{
 struct fps1196_data *fps1196 = NULL;
 int    status = -ENODEV;

 fps1196 = kzalloc(sizeof(struct fps1196_data),GFP_KERNEL);
 if (!fps1196)
   {
    return -ENOMEM;
   }

  //init head
  INIT_LIST_HEAD(&fps1196->device_entry);
  spin_lock_init(&fps1196->spi_lock);
 
  //init lock
  mutex_init(&fps1196->buf_lock);
  mutex_lock(&device_list_lock);

  mt_set_gpio_mode(GPIO_FPS1196_POWER_PIN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_FPS1196_POWER_PIN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_FPS1196_POWER_PIN, GPIO_OUT_ONE);

  cdev_init(&(fps1196->wg_cdev),&fps1196_fops);
  fps1196->wg_cdev.owner = THIS_MODULE;
  alloc_chrdev_region(&(fps1196->wg_cdev.dev),10,1,"fps1196");
  fps1196->devno = fps1196->wg_cdev.dev ;
  cdev_add(&(fps1196->wg_cdev),fps1196->devno,1);
  fps1196->class = class_create(THIS_MODULE,"fps1196");
  fps1196->device = device_create(fps1196->class, NULL, fps1196->devno,NULL, "fps1196");
  list_add(&fps1196->device_entry, &device_list);
  
  status = IS_ERR(fps1196->device)? PTR_ERR(fps1196->device):0;
  if (status !=0)
   {
     return status;
   }
  g_fps1196 = fps1196;
  spi_set_drvdata(spi,fps1196);   
  //spi init
  set_spi_mode(1);
  //init gpio work interrupt
  fps1196_exit_gpio_init();
  //create input device
  fps1196_create_inputdev();
  //Timer operation
  init_timer(&int_timer);
  int_timer.function = &int_timer_handle; 
  add_timer(&int_timer);

  fps1196->spi = spi;
  fps1196->spi->bits_per_word = 8;
  fps1196->spi->mode = SPI_MODE_0;
  fps1196->spi->controller_data = (void*)&spi_conf;
  spi_setup(fps1196->spi); 
 
  fps1196_device = fps1196->device ;

  mutex_unlock(&device_list_lock);
  return status;
}


// fps1196 init
static int  fps1196_dev_init(struct fps1196_data *fps1196)
{
 u8 reset [1] = {0x0c};
 u8 start[1]={0x18};
 
 u8 rx_data1[1]={0x00};
 u8 rx_data2[1]={0x00};
 
 u8 *test_tx;
 u8 *test_rx;

 test_rx = (u8*)kmalloc(7*sizeof(u8),GFP_KERNEL);
 test_tx=  (u8*)kmalloc(7*sizeof(u8),GFP_KERNEL);
 
 if ((NULL == test_rx)||(NULL == test_tx))
  {
    return -ENOMEM;
  }

 spi_send_cmd_fifo(fps1196,reset,rx_data1,1);
 spi_send_cmd_fifo(fps1196,reset,rx_data1,1);
 spi_send_cmd_fifo(fps1196,start,rx_data2,1);

 test_tx[0] = 0x21;
 test_tx[1] = 0x66;
 test_tx[2] = 0x66;
 test_tx[3] = 0xB5;
 test_tx[4] = 0x00;
 test_tx[5] = 0xf4;
 test_tx[6] = 0x70; //0x50
  
 spi_send_cmd_fifo(fps1196,test_tx,test_rx,7);
 spi_send_cmd_fifo(fps1196,test_tx,test_rx,7);

 interrupt_mode_flag = 0;

 if (test_rx[6] != 0x70)
   {
      kfree(test_rx);
      kfree(test_tx);
      return -1;
   }
 
 kfree(test_rx);
 kfree(test_tx);
 return 0;
}


static int fps1196_agc_init(struct fps1196_data *fps1196,unsigned long arg)
{
 u8 reset [1] = {0x0c};
 u8 start[1]={0x18};
 
 u8 rx_data1[1]={0x00};
 u8 rx_data2[1]={0x00};
 
 u8 *test_tx;
 u8 *test_rx;

 test_rx = (u8*)kmalloc(7*sizeof(u8),GFP_KERNEL);
 test_tx=  (u8*)kmalloc(7*sizeof(u8),GFP_KERNEL);
  if ((NULL == test_rx)||(NULL == test_tx))
  {
    return -ENOMEM;
  }

 spi_send_cmd_fifo(fps1196,reset,rx_data1,1);
 spi_send_cmd_fifo(fps1196,reset,rx_data1,1);
 spi_send_cmd_fifo(fps1196,start,rx_data2,1);

 test_tx[0] = 0x21;
 test_tx[1] = 0x66;
 test_tx[2] = 0x66;
 test_tx[3] = 0xB5;
 test_tx[4] = 0x00;
 test_tx[5] = arg;
 test_tx[6] = 0x70; //0x50
  
 spi_send_cmd_fifo(fps1196,test_tx,test_rx,7);
 spi_send_cmd_fifo(fps1196,test_tx,test_rx,7);

 interrupt_mode_flag = 0;

 if (test_rx[6] != 0x70)
 {
  mt_set_gpio_mode(GPIO_FPS1196_RESET_PIN , GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_FPS1196_RESET_PIN , GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_FPS1196_RESET_PIN , GPIO_OUT_ZERO);
  udelay(50);
  mt_set_gpio_out(GPIO_FPS1196_RESET_PIN , GPIO_OUT_ONE);

  spi_send_cmd_fifo(fps1196,test_tx,test_rx,7);
  spi_send_cmd_fifo(fps1196,test_tx,test_rx,7);
   if (test_rx[6] != 0x70)
   {
     kfree(test_rx);
     kfree(test_tx);
     return -1;
   }
 }
 kfree(test_rx);
 kfree(test_tx);
 return 0;

}



// fps1196 work finger interrupt mode
static int  fps1196_dev_interrupt_init(struct fps1196_data *fps1196)
{
 u8 reset [1] = {0x0c};
 u8 start[1]={0x18};
 u8 interrupt[1] = {0x14};
 
 u8 rx_data1[1]={0x00};
 u8 rx_data2[1]={0x00};

 u8 *test_tx;
 u8 *test_rx;
 test_rx = (u8*)kmalloc(7*sizeof(u8),GFP_KERNEL);
 test_tx=  (u8*)kmalloc(7*sizeof(u8),GFP_KERNEL);
 
 if ((NULL == test_rx)||(NULL == test_tx))
  {
    return -ENOMEM;
  }

 spi_send_cmd_fifo(fps1196,reset,rx_data1,1);
 spi_send_cmd_fifo(fps1196,reset,rx_data1,1);
 spi_send_cmd_fifo(fps1196,start,rx_data2,1);

 test_tx[0] = 0x21;
 test_tx[1] = 0x66;
 test_tx[2] = 0x66;
 test_tx[3] = 0xB5;
 test_tx[4] = 0x00;
 test_tx[5] = 0xfA;
 test_tx[6] = 0x70; //0x50

 spi_send_cmd_fifo(fps1196,test_tx,test_rx,7);
 spi_send_cmd_fifo(fps1196,test_tx,test_rx,7);

 spi_send_cmd_fifo(fps1196,interrupt,rx_data2,1);


 //Wake interrupt allows
 interrupt_mode_flag = 1;

 kfree(test_rx);
 kfree(test_tx);
 return 0;
}


static int fps1196_read_image(struct fps1196_data *fps1196,uint32_t timeout)
{
  uint32_t i=0,j=0,w=0;
  uint8_t Frameend = 0;
  uint8_t linehead=0,temp=0;
  int8_t  image_cap_flag = 0;

  memset(imagerxpix,0x00,12288);
  imagetxcmd[0] = 0x90;
  memset (&imagetxcmd[1],0x66,12287);
  memset(imagebuf,0x00,10752);
   
 while ((image_cap_flag == 0)&&(timeout != 0))
 {
   i = 0; 
   w = 0;
   spi_send_cmd(fps1196,imagetxcmd,imagerxpix,12288);
  while((Frameend == 0))
    {
     linehead=imagerxpix[i];
     if (linehead==0xAA)
      {
        i++;
        temp = imagerxpix[i];
       for (j=0;j<112;j++)
        {
          i++;
          imagebuf[w] = imagerxpix[i];
          w++;
        }
        if (temp==95)
         {
           Frameend = 1;
           image_cap_flag = 1;
           return 0;
         }
      }
     i++;
     if (i >= 12175)  
      { 
        Frameend = 1; 
        timeout --;
        if (timeout <= 1)
         {
            timeout =0 ;
            image_cap_flag = 1;
            return -1;               
         }
      }
    }
 }


  return 0;
}

// powerdown mode
static void fps1196_powerdown_mode(struct fps1196_data* fps1196)
{
static uint8_t powerdown_cmd[1] = {0x00};
uint8_t powerdown_rx [1] = {0x00};
spi_send_cmd_fifo(fps1196,powerdown_cmd,powerdown_rx,1); //发 0x18
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                    //
//                                             finger interrupt                                                       //
//                                                                                                                    // 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int fps1196_clear_interrupt(struct fps1196_data *fps1196)
{
static u8 start[1] = {0x18};
u8 start_rx[1] = {0x00};
static u8 fps1196_clc_cmd[1] = {0xA3};
u8 fps1196_clc_cmd_rx[1] = {0x00};
static u8 fps1196_int_cmd[1] = {0x14};
u8 fps1196_int_cmd_rx[1] = {0x00};


spi_send_cmd_fifo(fps1196,start,start_rx,1); //发 0x18

spi_send_cmd_fifo(fps1196,fps1196_clc_cmd,fps1196_clc_cmd_rx,1);
spi_send_cmd_fifo(fps1196,fps1196_int_cmd,fps1196_int_cmd_rx,1);

return 0;
}

// Interrupt handler function
static void fps1196_eint_handler(void)
{
 mt_eint_mask(CUST_EINT_FIGERPRINT_INT_NUM);
 if (interrupt_mode_flag == 0x01)
  {
     fps1196_interrupt_flag=1;
     wake_up_interruptible(&waiter);
     //Reset the timer's timeout, and activate
     mod_timer(&int_timer,jiffies + HZ/10);
   }
  else
   {
     fps1196_interrupt_flag = 0; 
   }
 mt_eint_unmask(CUST_EINT_FIGERPRINT_INT_NUM);
}

//Interrupt IO configuration
static void fps1196_exit_gpio_init(void)
{
 mt_set_gpio_mode(GPIO_FIGERPRINT_INT,GPIO_FIGERPRINT_INT_M_EINT);
 mt_set_gpio_dir(GPIO_FIGERPRINT_INT,GPIO_DIR_IN);
 mt_set_gpio_pull_enable(GPIO_FIGERPRINT_INT,GPIO_PULL_ENABLE);
 mt_set_gpio_pull_select(GPIO_FIGERPRINT_INT,GPIO_PULL_UP);

//Set interrupt starting mode

 mt_eint_registration(CUST_EINT_FIGERPRINT_INT_NUM, EINTF_TRIGGER_RISING, fps1196_eint_handler, 1);
 mt_eint_unmask(CUST_EINT_FIGERPRINT_INT_NUM);

}

//Thread processing function
static int fps1196_thread_func(void *unused)
{
  do
    {
     set_current_state(TASK_INTERRUPTIBLE);
     wait_event_interruptible(waiter, fps1196_interrupt_flag !=0);
     fps1196_interrupt_flag = 0;
     set_current_state(TASK_RUNNING); 
      
     int_count++;
     if (int_count >= 12)
     {
     int_count = 0;
     interrupt_mode_flag = 0;

     kobject_uevent(&fps1196_device->kobj,KOBJ_CHANGE);   
  
      //Stop timer
      del_timer(&int_timer);
     }
     //Clear interrupt register
     fps1196_clear_interrupt(g_fps1196);
    }while(!kthread_should_stop());
  
 return 0;
}

//Timer interrupt processing function
static void int_timer_handle(unsigned long arg)
{
   int_count = 0;
}


// Register interrupt device
static int fps1196_create_inputdev(void)
{
  fps1196_inputdev = input_allocate_device();
  if (!fps1196_inputdev)
   {
    printk("fps1196_inputdev create faile!\n");
    return -ENOMEM;
   }
   __set_bit(EV_KEY,fps1196_inputdev->evbit);
   __set_bit(KEY_INTERRUPT,fps1196_inputdev->keybit);
   
   fps1196_inputdev->id.bustype = BUS_HOST;
   fps1196_inputdev->name = "fps1196_inputdev";
   if (input_register_device(fps1196_inputdev))
    {
      printk("register inputdev failed\n");
      input_free_device(fps1196_inputdev);
      return -ENOMEM;
    }
   //Create and run the thread
  fps1196_thread = kthread_run(fps1196_thread_func,0,"fps1196_thread");
  if (IS_ERR(fps1196_thread))
    {
     printk("kthread_run is faile\n");
     return -(PTR_ERR(fps1196_thread));
    }
 
}

static void destroy_fps1196_device (struct fps1196_data * fps1196)
{
  // Destruction of character equipment
 device_destroy(fps1196->class,fps1196->devno);
  // Destruction of class structure
 if (fps1196->class)
   {
     class_destroy(fps1196->class);
   }
 // Cancel character device area
 unregister_chrdev_region (fps1196->devno,1);
 list_del(&fps1196->device_entry);
}

static int  mt_spi_init(void)
{
     int ret=0;
     ret=spi_register_board_info(spi_board_fps1196,ARRAY_SIZE(spi_board_fps1196));
     ret=spi_register_driver(&fps1196_driver);

   return ret; 
}


static void  mt_spi_exit(void)
{
  spi_unregister_driver(&fps1196_driver);
  destroy_fps1196_device (g_fps1196);
}

module_init(mt_spi_init);
module_exit(mt_spi_exit);

MODULE_DESCRIPTION("MT6572 SPI Controller Driver");
MODULE_AUTHOR("liwandong");
MODULE_LICENSE("GPL");
MODULE_ALIAS("fps1196");
