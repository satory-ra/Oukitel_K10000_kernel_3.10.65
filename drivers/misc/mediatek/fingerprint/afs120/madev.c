/* MicroArray Fingerprint
 * madev.c
 * date: 2015-08-15
 * version: v2.0
 * Author: czl
 */

#include <asm/memory.h>
#include <asm/uaccess.h>
#include <asm/dma.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/earlysuspend.h>
#include <linux/kobject.h> 
#include <linux/platform_device.h>
#include <linux/delay.h>
#include "print.h"
#include "madev.h"
//#include "plat-general.h"
#include "plat-mtk.h"

struct fprint_dev {
	dev_t idd;
	int major;
	int minor;
	struct cdev *chd;
	struct class *cls;
};
static struct fprint_dev *sdev = NULL;

struct fprint_spi {
	volatile u8 f_irq;		//中断标志
	volatile u8 f_repo;	//上报开关		
	spinlock_t spi_lock;
	struct spi_device *spi;
	struct list_head dev_entry;
	struct spi_message msg;
	struct spi_transfer xfer;
	struct input_dev *input;
	struct work_struct work;
	struct workqueue_struct *workq;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend suspend;
#endif
};
static struct fprint_spi *smas = NULL;
static u8 *stxb = NULL;
static u8 *srxb = NULL;

static LIST_HEAD(dev_list);
static DEFINE_MUTEX(dev_lock);
static DEFINE_MUTEX(drv_lock);

int mas_sync(u8 *txb, u8 *rxb, int len);

static void mas_work(struct work_struct *pws) {
	printd("%s: start.\n", __func__);

	input_report_key(smas->input, KEY_F1, 1);
	input_sync(smas->input);
	input_report_key(smas->input, KEY_F1, 0);
	input_sync(smas->input);
	
	printd("%s: end. report key F1\n", __func__);
}

static irqreturn_t mas_interrupt(int irq, void *dev_id) {
	printw("%s: start. f_irq=%d\n", __func__, smas->f_irq);

	plat_enable_irq(smas->spi, 0);
	if(smas->f_irq==0) {
		smas->f_irq = 1;
		if(smas->f_repo==REPORT_ON) queue_work(smas->workq, &smas->work);
	}

	printw("%s: end. f_irq=%d\n", __func__, smas->f_irq);

	return IRQ_HANDLED;
}

/*---------------------------------- fops ------------------------------------*/
static int mas_open(struct inode *inode, struct file *filp) {
	int ret = -ENXIO;

	printd("%s: start\n", __func__);

	mutex_lock(&dev_lock);
	list_for_each_entry(smas, &dev_list, dev_entry) {
		if (sdev->idd == inode->i_rdev) {
			ret = 0;
			break;
		}
	}
	if (!ret) nonseekable_open(inode, filp);
	else printw("%s: nothing for minor\n", __func__);
	mutex_unlock(&dev_lock);
		
	printd("%s: end ret=%d\n", __func__, ret);

	return ret;
}

/* 读写数据
 * @buf 数据
 * @len 长度
 * @返回值：0成功，否则失败
 */
int mas_sync(u8 *txb, u8 *rxb, int len) {
	int ret;

	//printd("%s: start. cmd=0x%.2x len=%d\n", __func__, buf[0], len);

	mutex_lock(&drv_lock);
	plat_tansfer(smas->spi, len);
	smas->xfer.tx_buf = txb;
	smas->xfer.rx_buf = rxb;
	smas->xfer.delay_usecs = 1;
	smas->xfer.len = len;
	smas->xfer.bits_per_word = 8;
	smas->xfer.speed_hz = smas->spi->max_speed_hz;
	spi_message_init(&smas->msg);
	spi_message_add_tail(&smas->xfer, &smas->msg);
	ret = spi_sync(smas->spi, &smas->msg);
	mutex_unlock(&drv_lock);

	//printd("%s: end. ret=%d\n", __func__, ret);

	return ret;
}

/* 读数据
 * @return 成功:count, -1count太大，-2通讯失败, -3拷贝失败
 */
static ssize_t mas_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
	int val, ret = 0;

	printd("%s: start. count=%d\n", __func__, count);
	
	//mutex_lock(&drv_lock);	
	if(count>FBUF) {
		printw("%s: too long. count=%dn", __func__, count);
		return -1;
	}
	ret = mas_sync(stxb, srxb, count);
	if(ret) {
		printw("%s: mas_sync failed.\n", __func__);
		return -2;
	}
	ret = copy_to_user(buf, srxb, count);
	if(!ret) val = count;
	else {
		val = -3;
		printw("%s: copy_to_user failed.\n", __func__);
	}	
	//mutex_unlock(&drv_lock);

	printd("%s: end. ret=%d\n", __func__, ret);

	return val;
}

int mas_ioctl(int cmd, int arg) {
	int ret = 0;

	printw("%s: start cmd=0x%.3x arg=%d\n", __func__, cmd, arg);

	switch (cmd) {
	case IOCTL_DEBUG:	
		sdeb = (u8) arg;
		break;
	case IOCTL_IRQ_ENABLE:
		plat_enable_irq(smas->spi, arg);
		break;
	case IOCTL_SPI_SPEED: //设置SPI速度
		smas->spi->max_speed_hz = (u32) arg;
		spi_setup(smas->spi);
		break;	
	case IOCTO_NUM_STUFF: //获取材料编号: 1:陶瓷，2:塑封
		ret = SENSOR_TYPE;
		break;
	case IOCTL_GET_VDATE:
		ret = 20150930;
		break;
	case IOCTL_RES_INTF:
		smas->f_irq = 0;
		break;
	case IOCTL_GET_INTF:
		ret = smas->f_irq;
		break;
	case IOCTL_KEY_REPO:
		smas->f_repo = arg;
		break;	
	}

	printw("%s: end. ret=%d f_irq=%d\n", __func__, ret, smas->f_irq);

	return ret;
}

/* 写数据
 * @return 成功:count, -1count太大，-2拷贝失败
 */
static ssize_t mas_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	int val, ret;

	printd("%s: start. count=%d\n", __func__, count);

	if(count==6) { //命令
		int cmd, arg;
		u8 tmp[6];
		ret = copy_from_user(tmp, buf, count);
		cmd = tmp[0];
		cmd <<= 8;
		cmd += tmp[1];
		arg = tmp[2];
		arg <<= 8;
		arg += tmp[3];
		arg <<= 8;
		arg += tmp[4];
		arg <<= 8;
		arg += tmp[5];
		printd("%s: cmd=0x%.3x arg=%d\n", __func__, cmd, arg);
		val = mas_ioctl(cmd, arg);
	} else {		
		if(count>FBUF) {
			printw("%s: write data too long.\n", __func__);
			return -1;
		}
		//mutex_lock(&drv_lock);				
		memset(stxb, 0, FBUF);
		ret = copy_from_user(stxb, buf, count);		
		if(ret) {
			printw("%s: copy_from_user failed.\n", __func__);
			val = -2;
		} else {
			val = count;
		//	printn(__func__, stxb, count);
		}	
		//mutex_unlock(&drv_lock);
	}

	printd("%s: end. ret =%d\n", __func__, ret);

	return val;
}

static int mas_release(struct inode *inode, struct file *filp) {
	int ret = 0;

	printd("%s: start\n", __func__);

	printd("%s: end\n", __func__);

	return ret;
}

static const struct file_operations sfops = {
	.owner = THIS_MODULE,
	.write = mas_write,
	.read = mas_read,
	.open = mas_open,
	.release = mas_release,
};

static void mas_set_input(void) {
	struct input_dev *input = NULL;
	int ret = 0;

	printd("%s: start.\n", __func__);

	input = input_allocate_device();
	if (!input) {
		printw("%s: input_allocate_device failed.\n", __func__);
		return ;
	}
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_SYN, input->evbit);
	set_bit(KEY_F1, input->keybit); //单触
	set_bit(KEY_F2, input->keybit);
	set_bit(KEY_F3, input->keybit);
	set_bit(KEY_F4, input->keybit);
	set_bit(KEY_F5, input->keybit);
	set_bit(KEY_POWER, input->keybit);
	set_bit(BTN_TOUCH, input->keybit);

	input->name = "madev";
    	input->id.bustype = BUS_SPI;
	ret = input_register_device(input);
    if (ret) {
        input_free_device(input);
        printw("%s: failed to register input device.\n",__func__);
        return;
    }
	smas->input  = input;

	printd("%s: end.\n", __func__);
}
#ifdef CONFIG_HAS_EARLYSUSPEND
//void mas_suspend(struct device *dev, pm_message_t message)
void mas_suspend(struct early_suspend *handler) {
	printd("%s: start.\n", __func__);
	
	//mas_set_worker(FP_DRV);

	printd("%s: end.\n", __func__);
}

//void mas_resume(struct device *dev)
void mas_resume(struct early_suspend *handler) {
	printd("%s: start.\n", __func__);

	//mas_set_worker(FP_APP);

	printd("%s: end.\n", __func__);
}
#endif

int mas_probe(struct spi_device *spi) {
	int ret = 0;

	printd("%s: start\n", __func__);

	smas->spi = spi;
	smas->spi->max_speed_hz = SPI_SPEED;
	spi_setup(spi);
	spin_lock_init(&smas->spi_lock);
	INIT_LIST_HEAD(&smas->dev_entry);
	mutex_lock(&dev_lock);

	//device_init_wakeup(&spi->dev, 1);
#ifdef CONFIG_HAS_EARLYSUSPEND
	smas->suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	smas->suspend.suspend = mas_suspend;
	smas->suspend.resume = mas_resume;
	register_early_suspend(&smas->suspend);
#endif
	plat_power(1);
	spi_set_drvdata(spi, smas);
	mas_set_input();

	INIT_WORK(&smas->work, mas_work);
	ret = plat_request_irq(spi->irq, mas_interrupt, IRQF_TRIGGER_RISING,
		dev_name(&spi->dev), &spi->dev);
	printd("%s: request_irq ret=%d\n", __func__, ret);
	if (ret) {
		printw("%s: request_irq failed.\n", __func__);
		goto fail;
	}
	plat_enable_irq(spi, 0);
	mutex_unlock(&dev_lock);

	printw("%s: insmod successfully. version:2.0 time:0919_am11\n", __func__);
	return ret;

fail:
	printw("%s: insmod failed.\n", __func__);

	return ret;
}

int mas_remove(struct spi_device *spi) {
	printd("%s: start.\n", __func__);

	// make sure ops on existing fds can abort cleanly
	spin_lock_irq(&smas->spi_lock);
	plat_enable_irq(spi, 0);
	free_irq(spi->irq, &spi->dev);
	smas->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&smas->spi_lock);

	// prevent new opens
	mutex_lock(&dev_lock);
	input_unregister_device(smas->input);
	//device_init_wakeup(&spi->dev, 0);
	mutex_unlock(&dev_lock);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&smas->suspend);
#endif
	plat_power(0);

	printd("%s: end\n", __func__);

	return 0;
}

/*---------------------------------- module ------------------------------------*/
static int __init mas_init(void) {
	int ret;
	struct device *dev;
	
	printd("%s: start\n", __func__);

	sdev = kmalloc(sizeof(struct fprint_dev), GFP_KERNEL);
	smas = kmalloc(sizeof(struct fprint_spi), GFP_KERNEL);
	srxb = kmalloc(FBUF*sizeof(u8), GFP_KERNEL);
	stxb = kmalloc(FBUF*sizeof(u8), GFP_KERNEL);
	if (sdev==NULL || smas==NULL || srxb==NULL || stxb==NULL) {
		printw("%s: smas kmalloc failed.\n", __func__);
		if(sdev!=NULL) kfree(sdev);
		if(smas!=NULL) kfree(smas);
		if(stxb!=NULL) kfree(stxb);
		if(srxb!=NULL) kfree(srxb);
		return -ENOMEM;
	}
	
	//初始化cdev
	sdev->chd = cdev_alloc();
	cdev_init(sdev->chd, &sfops);
	sdev->chd->owner = THIS_MODULE;

	//动态获取主设备号(dev_t idd中包含"主设备号"和"次设备号"信息)
	alloc_chrdev_region(&sdev->idd, 0, 1, "madev");
	sdev->major = MAJOR(sdev->idd);
	sdev->minor = MINOR(sdev->idd);
	printd("%s: major=%d minor=%d\n", __func__, sdev->major, sdev->minor);

	//注册字符设备 (1)
	ret = cdev_add(sdev->chd, sdev->idd, 1);
	if (ret) {
		printw("%s: cdev_add failed. ret=%d\n", ret);
		return -1;
	}
	sdev->cls = class_create(THIS_MODULE, "madev");
	if (IS_ERR(sdev->cls)) {
		printw("%s: class_create failed.\n", __func__);
		return -1;
	}
	dev = device_create(sdev->cls, NULL, sdev->idd, NULL, "madev0");
	ret = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (ret == 0) {
		list_add(&smas->dev_entry, &dev_list);
	} else {
		printw("%s: device_create failed. ret=%d\n", __func__, ret);
		return -1;
	}
	smas->workq = create_singlethread_workqueue("mas_workqueue");
	if (!smas->workq) {
		printw("%s: create_single_workqueue failed\n", __func__);
		return -ENOMEM;
	}
	ret = plat_register_driver();
	if (ret < 0) {
		printw("%s: spi_register_driver failed. ret=%d\n", __func__, ret);
		class_destroy(sdev->cls);
		unregister_chrdev_region(sdev->idd, 1);
	}
	printd("%s: end\n", __func__);

	return ret;
}

static void __exit mas_exit(void) {
	printd("%s: start\n", __func__);

	plat_unregister_driver();
	unregister_chrdev_region(sdev->idd, 1);
	cdev_del(sdev->chd);
	device_destroy(sdev->cls, sdev->idd);
	class_destroy(sdev->cls);
	if(smas->workq) destroy_workqueue(smas->workq);

	if(sdev!=NULL) kfree(sdev);
	if(smas!=NULL) kfree(smas);
	if(stxb!=NULL) kfree(stxb);
	if(srxb!=NULL) kfree(srxb);

	printd("%s: end\n", __func__);
}

module_init(mas_init);
module_exit(mas_exit);

MODULE_AUTHOR("czl");
MODULE_DESCRIPTION("for microarray fprint driver");
MODULE_LICENSE("GPL");

