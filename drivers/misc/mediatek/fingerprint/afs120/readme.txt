/**
  * microarray-hct  驱动包
  * support：microarray fae
  * email：guq@microarray.com.cn
  * tel: 0512-62761291-818
  **/
驱动指导：
1.该驱动包解压到fingerprint目录下，修改同级目录下Makefile添加
obj-y	+= afs120/

2.fingerprint上级目录Kconfig添加
source "drivers/misc/mediatek/fingerprint/afs120/Kconfig"

3.编译下载
4.进入adb shell 使用命令/*针对android 5.x*/
set enforce 0
chmod 0666 /dev/madev0
5.运行apk查看效果
