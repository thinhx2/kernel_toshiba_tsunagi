/* Copyright (c) 2011 Markinus
 * Display driver for the Toshiba TG01
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>

#include "board-tsunagi.h"
#include "devices.h"

static int spi_cs;
static int spi_sclk;
static int spi_mosi;
static int spi_miso;

struct toshiba_state_type{
	int disp_initialized;
	int display_on;
	int disp_powered_up;
};


void sleepnop(void) {
	nop();
	nop();
	nop();
	nop();
	nop();
	nop();
	nop();
	nop();
	nop();
	nop();
}

static struct toshiba_state_type toshiba_state = { 0 };

static void toshiba_spi_write_byte(char dc, uint8_t data)
{
	uint32_t bit;
	int bnum;

	gpio_set_value(spi_sclk, 0); /* clk low */
	/* dc: 0 for command, 1 for parameter */
	sleepnop();
	gpio_set_value(spi_mosi, dc);
	sleepnop();	
	gpio_set_value(spi_sclk, 1); /* clk high */
	sleepnop();	
	gpio_set_value(spi_sclk, 0); /* clk low */
	sleepnop();
	bnum = 8;	/* 8 data bits */
	bit = 0x80;
	while (bnum) {
		gpio_set_value(spi_sclk, 0); /* clk low */
		sleepnop();
		if (data & bit)
			gpio_set_value(spi_mosi, 1);
		else
			gpio_set_value(spi_mosi, 0);
		sleepnop();
		gpio_set_value(spi_sclk, 1); /* clk high */
		sleepnop();
		gpio_set_value(spi_sclk, 0); /* clk low */
		sleepnop();	/* at least 20 ns */
		bit >>= 1;
		bnum--;
	}
}

static void toshiba_spi_write(char cmd, uint32_t data, int num)
{
	char *bp;

	gpio_set_value(spi_cs, 1);	/* cs high */

	/* command byte first */
	toshiba_spi_write_byte(0, cmd);

	/* followed by parameter bytes */
	if (num) {
		bp = (char *)&data;;
		bp += (num - 1);
		while (num) {
			toshiba_spi_write_byte(1, *bp);
			num--;
			bp--;
		}
	}

	gpio_set_value(spi_cs, 0);	/* cs low */
	sleepnop();
}

void toshiba_spi_read_bytes(char cmd, uint32_t *data, int num)
{
	uint32_t dbit, bits;
	int bnum;

	gpio_set_value(spi_cs, 1);	/* cs high */

	/* command byte first */
	toshiba_spi_write_byte(0, cmd);

	if (num > 1) {
		/* extra dc bit */
		gpio_set_value(spi_sclk, 0); /* clk low */
		sleepnop();
		dbit = gpio_get_value(spi_miso);/* dc bit */
		sleepnop();
		gpio_set_value(spi_sclk, 1); /* clk high */
	}

	/* followed by data bytes */
	bnum = num * 8;	/* number of bits */
	bits = 0;
	while (bnum) {
		bits <<= 1;
		gpio_set_value(spi_sclk, 0); /* clk low */
		sleepnop();
		dbit = gpio_get_value(spi_miso);
		sleepnop();
		gpio_set_value(spi_sclk, 1); /* clk high */
		bits |= dbit;
		bnum--;
	}

	*data = bits;

	sleepnop();
	gpio_set_value(spi_cs, 0);	/* cs low */
	sleepnop();
}

static void spi_pin_assign(void)
{
	printk("%s\n", __func__);
	/* Setting the Default GPIO's */
	spi_sclk = 156;
	spi_cs   = 148;
	spi_mosi  = 155;
	spi_miso  = 154;
}

static void toshiba_disp_powerup(void)
{
	int ret;
	printk("%s\n", __func__);
	if (!toshiba_state.disp_powered_up && !toshiba_state.display_on) {
		/* Reset the hardware first */
		/* Include DAC power up implementation here */
		printk("%s\n", __func__);
		ret = gpio_request(155, "DISPI1");
		ret = gpio_request(156, "DISPI2");
		ret = gpio_request(148, "DISPI3");
		ret = gpio_request(154, "DISPI4");
		ret = gpio_request(98, "DISPI5");
		ret = gpio_request(100, "DISPI6");
		
		gpio_direction_output(155, 0);
		gpio_direction_output(156, 0);
		gpio_direction_output(148, 0);
		gpio_direction_input(154);
	
		gpio_set_value(98, 0x1);
		gpio_set_value(100, 0x1);		
		mdelay(50);	      
		toshiba_state.disp_powered_up = 1;
	}
}

static void toshiba_disp_on(void)
{
	uint32_t	data;
	printk("%s\n", __func__);

	gpio_set_value(spi_cs, 0);	/* low */
	gpio_set_value(spi_sclk, 1);	/* high */
	gpio_set_value(spi_mosi, 0);
	gpio_set_value(spi_miso, 0);

	if (toshiba_state.disp_powered_up && !toshiba_state.display_on) {
		toshiba_spi_write(0, 0, 0);
		mdelay(6);
		toshiba_spi_write(0, 0, 0);
		mdelay(6);
		toshiba_spi_write(0, 0, 0);
		mdelay(6);
		
		toshiba_spi_write(0xba, 0x11, 1);
	
		toshiba_spi_write(0x2B, 0x0000018F, 4);
	
		toshiba_spi_write(0x36, 0xD0, 1);		
	
		toshiba_spi_write(0x3a, 0x60, 1);
		toshiba_spi_write(0xb1, 0x5d, 1);
	
		toshiba_spi_write(0xb2, 0x33, 1);
		toshiba_spi_write(0xb3, 0x22, 1);
	
		toshiba_spi_write(0xb4, 0x02, 1);
		toshiba_spi_write(0xb5, 0x21, 1); /* vcs -- adjust brightness */
	
		toshiba_spi_write(0xb6, 0x2F, 1);
		toshiba_spi_write(0xb7, 0x03, 1);
	
		toshiba_spi_write(0xb9, 0x24, 1);
		toshiba_spi_write(0xbd, 0xa1, 1);
	
		toshiba_spi_write(0xbe, 0x00, 1);
		toshiba_spi_write(0xbb, 0x00, 1);
	
		toshiba_spi_write(0xbf, 0x01, 1);
		toshiba_spi_write(0xc0, 0x11, 1);
	
		toshiba_spi_write(0xc1, 0x11, 1);
		toshiba_spi_write(0xc2, 0x11, 1);
	
		toshiba_spi_write(0xc3, 0x3232, 2);
	
		toshiba_spi_write(0xc4, 0x3232, 2);
	
		toshiba_spi_write(0xc5, 0x3232, 2);
	
		toshiba_spi_write(0xc6, 0x3232, 2);
	
		toshiba_spi_write(0xc7, 0x6445, 2);
	
		toshiba_spi_write(0xc8, 0x44, 1);
		toshiba_spi_write(0xc9, 0x52, 1);
	
		toshiba_spi_write(0xca, 0x00, 1);
	
		toshiba_spi_write(0xec, 0x0200, 2);	
	
		toshiba_spi_write(0xcf, 0x01, 1);
	
		toshiba_spi_write(0xd0, 0x1004, 2);	
	
		toshiba_spi_write(0xd1, 0x01, 1);
	
		toshiba_spi_write(0xd2, 0x001A, 2);
	
		toshiba_spi_write(0xd3, 0x001A, 2);
	
		toshiba_spi_write(0xd4, 0x207A, 2);
	
		toshiba_spi_write(0xd5, 0x18, 1);
	
		toshiba_spi_write(0xe2, 0x00, 1);
	
		toshiba_spi_write(0xe3, 0x36, 1);
	
		toshiba_spi_write(0xe4, 0x0003, 2);
	
		toshiba_spi_write(0xe5, 0x0003, 2);
	
		toshiba_spi_write(0xe6, 0x04, 1);
	
		toshiba_spi_write(0xe7, 0x030C, 2);
	
		toshiba_spi_write(0xe8, 0x03, 1);
	
		toshiba_spi_write(0xe9, 0x20, 1);
	
		toshiba_spi_write(0xea, 0x0404, 2);
	
		toshiba_spi_write(0xef, 0x3200, 2);
	
		toshiba_spi_write(0xbc, 0x80, 1);	/* wvga pass through */
		toshiba_spi_write(0x3b, 0x00, 1);
	
		toshiba_spi_write(0xb9, 0x24, 1);
	
		toshiba_spi_write(0xb0, 0x16, 1);
	
		toshiba_spi_write(0xb8, 0xfff5, 2);
	
		toshiba_spi_write(0x11, 0, 0);
	
		toshiba_spi_write(0x29, 0, 0);
	
		toshiba_state.display_on = 1;
	}

	data = 0;
	toshiba_spi_read_bytes(0x04, &data, 3);
	printk(KERN_INFO "toshiba_disp_on: id=%x\n", data);

}

static int lcdc_toshiba_panel_on(struct msm_lcdc_panel_ops *ops)
{
	printk("%s\n", __func__);
	if (!toshiba_state.disp_initialized) {
		toshiba_disp_powerup();
		toshiba_disp_on();
		toshiba_state.disp_initialized = 1;
	}
	return 0;
}

static int lcdc_toshiba_panel_off(struct msm_lcdc_panel_ops *ops)
{
	printk("%s\n", __func__);
	if (toshiba_state.disp_powered_up && toshiba_state.display_on) {
		/* Main panel power off (Deep standby in) */

		toshiba_spi_write(0x28, 0, 0);	/* display off */
		mdelay(1);
		toshiba_spi_write(0xb8, 0x8002, 2);	/* output control */
		mdelay(1);
		toshiba_spi_write(0x10, 0x00, 1);	/* sleep mode in */
		mdelay(85);							/* wait 85 msec */
		toshiba_spi_write(0xb0, 0x00, 1);	/* deep standby in */
		mdelay(1);
		toshiba_state.display_on = 0;
		toshiba_state.disp_initialized = 0;
	}
	return 0;
}

static int lcdc_toshiba_panel_init(struct msm_lcdc_panel_ops *ops)
{
	pr_info("%s\n", __func__);
	return 0;
}

static struct resource resources_msm_fb[] =
{
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct msm_lcdc_panel_ops tsunagi_lcdc_panel_ops =
{
	.init		= lcdc_toshiba_panel_init,
	.blank		= lcdc_toshiba_panel_off,
	.unblank	= lcdc_toshiba_panel_on,
};

static struct msm_lcdc_timing tsunagi_lcdc_timing =
{
	.clk_rate		= 24576000,
	.hsync_pulse_width	= 0x8,
	.hsync_back_porch	= 0x10,
	.hsync_front_porch	= 0x8,
	.hsync_skew		= 0,
	.vsync_pulse_width	= 0x2,
	.vsync_back_porch	= 0x4,
	.vsync_front_porch	= 0x4,
	.vsync_act_low		= 1,
	.hsync_act_low		= 1,
	.den_act_low		= 0,
};

static struct msm_fb_data tsunagi_lcdc_fb_data =
{
	.xres		= 480,
	.yres		= 800,
	.width		= 48,
	.height		= 80,
	.output_format	= 0,
};

static struct msm_lcdc_platform_data tsunagi_lcdc_platform_data =
{
	.panel_ops	= &tsunagi_lcdc_panel_ops,
	.timing		= &tsunagi_lcdc_timing,
	.fb_id		= 0,
	.fb_data	= &tsunagi_lcdc_fb_data,
	.fb_resource= &resources_msm_fb[0],
};

static struct platform_device tsunagi_lcdc_device =
{
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	=
	{
		.platform_data = &tsunagi_lcdc_platform_data,
	},
};


int __init tsunagi_init_panel(void)
{
	int ret=0;
	pr_info("Init Panel\n");
	if (!machine_is_tsunagi())
		return 0;
	spi_pin_assign();
	
	ret = platform_device_register(&msm_device_mdp);
	if (ret != 0)
		return ret;

	ret = platform_device_register(&tsunagi_lcdc_device);
	if (ret != 0)
		return ret;

	return 0;
}

