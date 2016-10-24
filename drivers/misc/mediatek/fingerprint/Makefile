include $(srctree)/drivers/misc/mediatek/Makefile.custom


ifeq ($(CONFIG_FINGERPRINT_ET300),y)
obj-y	+=  et300/
endif

ifeq ($(CONFIG_FINGERPRINT_ET310),y)
obj-y	+=  et310/
endif

ifeq ($(CONFIG_FINGERPRINT_SILEADE),y)
obj-y	+=  sileade_sensor/
endif

ifeq ($(CONFIG_FINGERPRINT_SILEADE_6162),y)
obj-y	+=  sileade_sensor_6162/
endif

ifeq ($(CONFIG_FINGERPRINT_NEWSCHIP),y)
obj-y	+= newschip_sensor/
endif

ifeq ($(CONFIG_FINGERPRINT_JMT103),y)
obj-y	+= jmt103/
endif

ifeq ($(CONFIG_FINGERPRINT_SF115),y)
obj-y	+= sf115_syno/
endif

ifeq ($(CONFIG_FINGERPRINT_SF115_OLD),y)
obj-y	+= sf115_syno_old/
endif

ifeq ($(CONFIG_FINGERPRINT_GF318M_GOODIX),y)
obj-y	+= gf318m_goodix/
endif

ifeq ($(CONFIG_FINGERPRINT_FPS1196),y)
obj-y	+=  cdfinger_fps1196/
endif

ifeq ($(CONFIG_FINGERPRINT_AFS120),y)
obj-y	+= afs120/
endif
