#ifndef __FOCALTECH_INFO_H__
#define __FOCALTECH_INFO_H__
 
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include "ft5x36_vendor_id.h"


typedef struct{  
u8 vendor_id;  
char ven_name[16] ;
} ft5x36_ven_info;

#ifdef CONFIG_GET_HARDWARE_INFO
/*add vendor name in vendor name array*/
static ft5x36_ven_info ft5x36_name[]=
{
//modify by chenchen for hardware info 20140820 begin
{FT5X36_VENDOR_BYD,"BYD0x59_"},
{FT5X36_VENDOR_CHAOSHENG,"ChaoSheng0x57_"},
{FT5X36_VENDOR_TOPTOUCH,"Toptouch0xA0_"},
{FT5X36_VENDOR_JTOUCH,"J-Touch0x8B_"},
{FT5X36_VENDOR_DAWOSI,"Dawos0x8A_"},
{FT5X36_VENDOR_TRULY,"Truly0x5A_"},
{FT6X06_VENDOR0x86_EKEY,"Ekey0x86_"},
{FT5X46_VENDOR0x79_EKEY,"Ekey0x79_"},
{FT5X46_VENDOR0xD8_LEAD,"Lead0xD8"},
//modify by chenchen for benwee rongna 20140818 begin
{FT6X06_VENDOR0x11_RONGNA,"RongNa0x11_"},
{FT6X06_VENDOR0xD1_RONGNA,"RongNa0xD1_"},
//modify by chenchen for benwee rongna 20140818 end
{FT6X06_VENDOR0xD2_HONGZHAN,"HongZhan0xD2_"},
{FT6X06_VENDOR0x80_EACH,"EACH0x80_"},
{FT6X06_VENDOR0xD7_TONGHAO,"TongHao0xD7_"},
{FT6X06_VENDOR0xF5_JINCHEN,"JinChen0xF5_"},
//modify by chenchen for hardware info 20140820 end
};
#endif
#endif
