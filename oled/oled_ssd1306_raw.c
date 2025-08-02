#define pr_fmt(fmt) "[SSD1306_RAW] " fmt
#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "oled_font.h"
#include "oled_ssd1306_def.h"




MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("SSD1306 OLED MyI2C Character Device Driver");