#
# Makefile for Bosch bmc156 sensor driver.
#
obj-y 	+= bstclass.o
obj-y	+= bma2x2.o
obj-y   += bmm050_driver.o bmm050.o
EXTRA_CFLAGS += -DBMM_USE_BASIC_I2C_FUNC -DCONFIG_BMM_USE_PLATFORM_DATA

ifeq ($(CONFIG_SENSORS_BMA2X2_ENABLE_INT1),y)
	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT1
endif

ifeq ($(CONFIG_BOSCH_BMA2X2_ENABLE_INT2),y)
	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT2
endif
