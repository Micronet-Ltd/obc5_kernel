/*
 focaltelech tp firmware update infomations
 
 Date              Author         Module    vendor id    Old_ver    New_ver
 2014.08.01    zengguang   rongna         0x11        NULL        0x12
 2014.08.18    chenchen     rongna         0x11        null          0x10
 2014.09.05    chenchen     rongna         0xd1        0x15        0x16
 2014.09.09    chenchen  rongna(ctyon-f8926)  0x11     null      0x13 
 2014.10.14    zengguang  shenyue(QL600_platform)  0x11    null      0x17 
 */
#ifndef __FOCALTECH_VENDOR_H__
#define __FOCALTECH_VENDOR_H__

#include "ft5x36_vendor_id.h"

#if defined(CONFIG_QL1006_PLATFORM)
static unsigned char FT6X06_FIRMWARE0x80_EACH[] = {
#if defined(CONFIG_QL1006_PLATFORM)
	#include "ft5x36_firmware/QL1006_PLATFORM_Ft6x06_EACH0x80_Ver10_20140619.h"
#endif
}; 
static unsigned char FT6X06_FIRMWARE0xD7_TONGHAO[] = {
#if defined(CONFIG_QL1006_PLATFORM)
	#include "ft5x36_firmware/QL1006_PLATFORM_Ft6x06_TONGHAO0xd7_Ver0x10_20140329.h"
#endif
}; 
#endif

/*add for hualu rongna ctp  vendor id begin by zengguang 2014.07.30*/
//midify by chenchen for shijitianyuan rongna 20140909 begin
#if defined(CONFIG_QL602_CHL) || defined(CONFIG_QL602_CTYON) || defined(CONFIG_QL602_BENWEE) || defined(CONFIG_QL602_CTYON_F8926)||defined(CONFIG_QL600_FACTORY)
static unsigned char FT6X06_FIRMWARE0x11_RONGNA[] = {
#if defined(CONFIG_QL602_CHL)
	#include "ft5x36_firmware/QL602_HUALU_Ft6306_RONGNA0x11_Ver0x15_20140729.h"
#elif defined(CONFIG_QL602_CTYON)
	#include "ft5x36_firmware/QL602_SHIJITIANYUAN_ft6306_RONGNA0x11_Ver0x12_20140804.h"
#elif defined(CONFIG_QL602_BENWEE)
	#include "ft5x36_firmware/QL602_BENWEE_V5_FT6306_RONHNA0x11_Ver0x10_20140812.h"
#elif defined(CONFIG_QL602_CTYON_F8926)
	#include "ft5x36_firmware/QL602_CTYON_F8926_FT6306_RONGNA0x11_Ver0x13_20140902.h"
#elif defined(CONFIG_QL600_FACTORY)	
	#include "ft5x36_firmware/QL600_Platform_FT6x36_Toptouch0x11_Ver0x0F_20141008.h"
#endif
//midify by chenchen for shijitianyuan rongna 20140909 end
}; 
#endif

#if defined(CONFIG_QL602_CHL)
static unsigned char FT6X06_FIRMWARE0xd1_RONGNA[] = {
#if defined(CONFIG_QL602_CHL)
	#include "ft5x36_firmware/QL602_HUALU_FT6306_RONGNA0xd1_Ver0x16_20140901.h"
#endif
}; 
#endif
/*add for hualu rongna ctp  vendor id  end by zengguang 2014.07.30*/
//added by pangle for ONUS 20140911 begin
#if defined(CONFIG_QL1001_ONUS_P18B)
static unsigned char FT5X36_FIRMWARE_EKEY[] = {
	#include "ft5x36_firmware/QL1001_ONUS_P18B_FT5336_EKEY0x51_Ver0x03_20140814.h"
};
#endif
//added by pangle for ONUS 20140911 end

#if defined(CONFIG_PRODUCT_A2001)
static unsigned char FT5526_FIRMWARE_EKEY[] = {
	#include "ft5x36_firmware/A2001_FT5x46_V02_D01_20151022_app.h"
};
static unsigned char FT5526_FIRMWARE_JUNDA[] = {
	#include "ft5x36_firmware/A2001_FT5x46_V04_JD_20160106_app.i"
};
#endif

#if defined(CONFIG_PRODUCT_A3001)
static unsigned char FT5526_FIRMWARE_EKEY[] = {
	#include "ft5x36_firmware/A3001_FT5x46_V02_D01_20151022_app.i"
};
#endif

#if defined(CONFIG_QL1001_J20) || defined(CONFIG_QL1001_J20TMP)
static unsigned char FT5526_FIRMWARE_LEAD[] = {
	#include "ft5x36_firmware/FT5526_Ref_V06_LEAD_SZTOPDAY+J20_20150205_app.h"
};
#endif
/*added by jiao.shp for p2170 5526 tp fw 20141023 end*/

#endif
