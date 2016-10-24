/* MicroArray Fingerprint
 * madev.h
 * date: 2015-08-15
 * version: v2.0
 * Author: czl
 */

#ifndef MADEV_H
#define MADEV_H

#define W   	120   // 宽
#define H   	120   // 高
#define WBUF	121
#define FIMG	(W*H)
#define FBUF  	(1024*15)		 // 读写长度

//指纹类型
#define AFS120T 		1	//陶瓷
#define AFS120N 		2	//塑封
#define AFS120M 		3	//塑封+
#define SENSOR_TYPE   	AFS120M 	//指纹类型

//接口命令
#define IOCTL_DEBUG		0x100	// 调试信息
#define IOCTL_IRQ_ENABLE	0x101	// 中断使能
#define IOCTL_SPI_SPEED   	0x102	// SPI速度
#define IOCTL_READ_FLEN		0x103	// 读帧长度(保留)
#define IOCTL_LINK_DEV		0x104	// 连接设备(保留)
#define IOCTO_NUM_STUFF		0x105	// 材料编号
#define IOCTL_GET_VDATE		0x106	// 版本日期

#define IOCTL_RES_INTF		0x110	// 复位中断标志
#define IOCTL_GET_INTF		0x111	// 获取中断标志
#define IOCTL_KEY_REPO		0x112 	// 键值开关

#define REPORT_ON	1		// 上报开
#define REPORT_OFF  0	    // 上报关

#define SPI_SPEED 	(10*1000000)

#endif /* MADEV_H */



