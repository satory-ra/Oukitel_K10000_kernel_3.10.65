include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_LEDS) += leds_drv.o

ifeq ($(CONFIG_AW9106B_BREATH_SUPPORT),y)
obj-y	+= aw9106b.o
endif

ifeq ($(CONFIG_AW2103_BREATH_SUPPORT),y)
obj-y	+= aw2103.o
endif

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/

