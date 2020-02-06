/* linux/arch/arm/mach-msm/board-tsunagi.c
 *
 * Copyright (C) 2010 Markinus
 * Author: Markinus
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/regulator/machine.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>
#include <linux/gpio_keys.h>
#include <linux/power_supply.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/msm_iomap.h>
#include <mach/perflock.h>
#include <mach/htc_usb.h>
#include <mach/msm_serial_hs.h>
#include <mach/perflock.h>
#include <mach/msm_ts.h>
#include "board-tsunagi.h"
#include "devices.h"
#include "proc_comm.h"


extern int __init tsunagi_init_mmc(unsigned debug_uart);
extern int __init tsunagi_init_panel(void);
extern void __init tsunagi_audio_init(void);

void msm_init_pmic_vibrator(void);

///////////////////////////////////////////////////////////////////////
// Regulator
///////////////////////////////////////////////////////////////////////

static struct regulator_consumer_supply tps65023_dcdc1_supplies[] =
{
    {
        .supply = "acpu_vcore",
    },
};

static struct regulator_init_data tps65023_data[5] =
{
    {
        .constraints =
		{
            .name = "dcdc1", /* VREG_MSMC2_1V29 */
            .min_uV = 1000000,
            .max_uV = 1300000,
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
        },
        .consumer_supplies = tps65023_dcdc1_supplies,
        .num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_supplies),
    },
    /* dummy values for unused regulators to not crash driver: */
    {
        .constraints = {
            .name = "dcdc2", /* VREG_MSMC1_1V26 */
            .min_uV = 1260000,
            .max_uV = 1260000,
        },
    },
    {
        .constraints = {
            .name = "dcdc3", /* unused */
            .min_uV = 800000,
            .max_uV = 3300000,
        },
    },
    {
        .constraints = {
            .name = "ldo1", /* unused */
            .min_uV = 1000000,
            .max_uV = 3150000,
        },
    },
    {
        .constraints = {
            .name = "ldo2", /* V_USBPHY_3V3 */
            .min_uV = 3300000,
            .max_uV = 3300000,
        },
    },
};


static struct i2c_board_info base_i2c_devices[] =
{
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
};



///////////////////////////////////////////////////////////////////////
// SPI
///////////////////////////////////////////////////////////////////////

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start  = INT_SPI_INPUT,
		.end    = INT_SPI_INPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start  = INT_SPI_OUTPUT,
		.end    = INT_SPI_OUTPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start  = INT_SPI_ERROR,
		.end    = INT_SPI_ERROR,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start  = 0xA1200000,
		.end    = 0xA1200000 + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_clk",
		.start  = 17,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_mosi",
		.start  = 18,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_miso",
		.start  = 19,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_cs0",
		.start  = 20,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_pwr",
		.start  = 21,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_cs0",
		.start  = 22,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct spi_platform_data tsunagi_spi_pdata = {
	.clk_rate	= 4800000,
};

static struct platform_device qsd_device_spi = {
	.name           = "spi_qsd",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(qsd_spi_resources),
	.resource       = qsd_spi_resources,
	.dev		= {
		.platform_data = &tsunagi_spi_pdata
	},
};


///////////////////////////////////////////////////////////////////////
// Memory 
///////////////////////////////////////////////////////////////////////

static struct android_pmem_platform_data mdp_pmem_pdata = {
	.name		= "pmem",
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.start		= MSM_PMEM_ADSP_BASE,
	.size		= MSM_PMEM_ADSP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};


static struct android_pmem_platform_data android_pmem_venc_pdata = {
	.name		= "pmem_venc",
	.start		= MSM_PMEM_VENC_BASE,
	.size		= MSM_PMEM_VENC_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct platform_device android_pmem_mdp_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &mdp_pmem_pdata
	},
};

static struct platform_device android_pmem_adsp_device = {
	.name		= "android_pmem",
	.id		= 4,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_venc_device = {
	.name		= "android_pmem",
	.id		= 5,
	.dev		= {
		.platform_data = &android_pmem_venc_pdata,
	},
};

///////////////////////////////////////////////////////////////////////
// USB 
///////////////////////////////////////////////////////////////////////

static uint32_t usb_phy_3v3_table[] =
{
 //   PCOM_GPIO_CFG(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
};

//static int tsunagi_phy_init_seq[] ={0x0C, 0x31, 0x30, 0x32, 0x1D, 0x0D, 0x1D, 0x10, -1};

#ifdef CONFIG_USB_ANDROID

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
//	.phy_init_seq		= tsunagi_phy_init_seq,
	.phy_reset		= msm_hsusb_8x50_phy_reset,
	.accessory_detect = 0, /* detect by ID pin gpio */
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c02,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
static void tsunagi_add_usb_devices(void)
{
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
//	config_gpio_table(usb_phy_3v3_table, ARRAY_SIZE(usb_phy_3v3_table));
//	gpio_set_value(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 1);
	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
}

#endif

///////////////////////////////////////////////////////////////////////
// KGSL (HW3D support)#include <linux/android_pmem.h>
///////////////////////////////////////////////////////////////////////

static struct resource msm_kgsl_resources[] =
{
	{
		.name	= "kgsl_reg_memory",
		.start	= MSM_GPU_REG_PHYS,
		.end	= MSM_GPU_REG_PHYS + MSM_GPU_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "kgsl_phys_memory",
		.start	= MSM_GPU_PHYS_BASE,
		.end	= MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
	},
};

static int tsunagi_kgsl_power_rail_mode(int follow_clk)
{
	int mode = follow_clk ? 0 : 1;
	int rail_id = 0;
	return msm_proc_comm(PCOM_CLK_REGIME_SEC_RAIL_CONTROL, &rail_id, &mode);
}

static int tsunagi_kgsl_power(bool on)
{
	int cmd;
	int rail_id = 0;

    	cmd = on ? PCOM_CLK_REGIME_SEC_RAIL_ENABLE : PCOM_CLK_REGIME_SEC_RAIL_DISABLE;
    	return msm_proc_comm(cmd, &rail_id, 0);
}

static struct platform_device msm_kgsl_device =
{
	.name		= "kgsl",
	.id		= -1,
	.resource	= msm_kgsl_resources,
	.num_resources	= ARRAY_SIZE(msm_kgsl_resources),
};

///////////////////////////////////////////////////////////////////////
// RAM-Console
///////////////////////////////////////////////////////////////////////

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

///////////////////////////////////////////////////////////////////////
// Hardware Buttons
///////////////////////////////////////////////////////////////////////

static struct gpio_keys_button tsunagi_button_table[] = {
	//KEY 			GPIO			ACTIVE_LOW DESCRIPTION		type		wakeup	debounce
	{KEY_VOLUMEUP,		TSUNAGI_GPIO_KP_VOLUP,		1, "Volume Up",		EV_KEY,		0, 	0},
	{KEY_VOLUMEDOWN,	TSUNAGI_GPIO_KP_VOLDOWN,	1, "Volume Down",	EV_KEY,		0,	0},
	{KEY_CAMERA,	       	TSUNAGI_GPIO_KP_CAMERA,		1, "Camera button",	EV_KEY,		1,	0},
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons=tsunagi_button_table,
	.nbuttons=ARRAY_SIZE(tsunagi_button_table),
};
static struct platform_device gpio_keys = {
        .name = "gpio-keys",
        .dev  = {
                .platform_data = &gpio_keys_data,
        },
        .id   = -1,
};

///////////////////////////////////////////////////////////////////////
// Touchscreen
///////////////////////////////////////////////////////////////////////

struct ts_virt_key ts_keys_y[] = {
	//KEY,		MIN,	MAX
	{KEY_HOME,	65,	200},
	{KEY_MENU,	250,	750},
	{KEY_BACK,	800,	970},
};

static struct msm_ts_virtual_keys tsunagi_keys_y = {
	.keys		= &ts_keys_y,
	.num_keys 	= 3,
};

static struct msm_ts_platform_data tsunagi_ts_pdata = {
	.min_x          = 65,
	.max_x          = 950,
	.min_y          = 30,
	.max_y          = 910,
	.min_press      = 0,
	.max_press      = 256,
	.inv_x          = 0,
	.inv_y          = 0,
	.inv_axes	= 1,
	.vkeys_y	= &tsunagi_keys_y,
	.virt_y_start	= 915,
};

///////////////////////////////////////////////////////////////////////
// Real Time Clock
///////////////////////////////////////////////////////////////////////

struct platform_device msm_device_rtc = {
	.name = "msm_rtc",
	.id = -1,
};

///////////////////////////////////////////////////////////////////////
// Power and battery
///////////////////////////////////////////////////////////////////////

static struct platform_device tsunagi_power = {
	.name 		    = "tsunagi_power",
	.id		    = -1,
};

static struct platform_device tsunagi_battery = {
	.name 		    = "tsunagi_battery",
	.id		    = -1,
};

///////////////////////////////////////////////////////////////////////
// Headset
///////////////////////////////////////////////////////////////////////

static struct platform_device tsunagi_hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = "8k_handset",
	},
};


//////////////////////////////////////////////////////////////////////
// Platform Devices
///////////////////////////////////////////////////////////////////////


static struct platform_device *devices[] __initdata =
{
	&ram_console_device,
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
	&msm_device_smd,
	&msm_device_rtc,
	&tsunagi_hs_device,
	&android_pmem_mdp_device,
	&android_pmem_adsp_device,
	&android_pmem_venc_device,
	&msm_device_i2c,
	&msm_kgsl_device,
	&qsd_device_spi,
	&msm_device_touchscreen,
	&gpio_keys,
//	&tsunagi_power,
	&tsunagi_battery,
};

///////////////////////////////////////////////////////////////////////
// I2C
///////////////////////////////////////////////////////////////////////

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock = 400000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_8MA,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}
///////////////////////////////////////////////////////////////////////
// Clocks
///////////////////////////////////////////////////////////////////////

static struct msm_acpu_clock_platform_data tsunagi_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 245000,
	.wait_for_irq_khz	= 245000,
//	.wait_for_irq_khz	= 19200,   // TCXO
};

static unsigned tsunagi_perf_acpu_table[] = {
	245000000,
	576000000,
	998400000,
};

static struct perflock_platform_data tsunagi_perflock_data = {
	.perf_acpu_table = tsunagi_perf_acpu_table,
	.table_size = ARRAY_SIZE(tsunagi_perf_acpu_table),
};
///////////////////////////////////////////////////////////////////////
// Reset
///////////////////////////////////////////////////////////////////////

static void tsunagi_reset(void)
{

}

static void do_grp_reset(void)
{
   	writel(0x20000, MSM_CLK_CTL_BASE + 0x214);
}


///////////////////////////////////////////////////////////////////////
// Init
///////////////////////////////////////////////////////////////////////

static void __init tsunagi_init(void)
{
	printk("tsunagi_init()\n");

	msm_hw_reset_hook = tsunagi_reset;

	do_grp_reset();

	msm_acpu_clock_init(&tsunagi_clock_data);
	
	perflock_init(&tsunagi_perflock_data);

	msm_device_i2c_init();
	
	/* set the gpu power rail to manual mode so clk en/dis will not
	* turn off gpu power, and hang it on resume */

	tsunagi_kgsl_power_rail_mode(0);
	tsunagi_kgsl_power(false);
	mdelay(100);
	tsunagi_kgsl_power(true);

	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_device_touchscreen.dev.platform_data = &tsunagi_ts_pdata;
	tsunagi_init_panel();
	tsunagi_audio_init();

	i2c_register_board_info(0, base_i2c_devices, ARRAY_SIZE(base_i2c_devices));
	

#ifdef CONFIG_USB_ANDROID
	tsunagi_add_usb_devices();
#endif
	tsunagi_init_mmc(0);

	msm_init_pmic_vibrator();
}

///////////////////////////////////////////////////////////////////////
// Bootfunctions
///////////////////////////////////////////////////////////////////////

static void __init tsunagi_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = MSM_EBI1_BANK0_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_EBI1_BANK0_BASE);
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
}

#if defined(CONFIG_VERY_EARLY_CONSOLE)
#if defined(CONFIG_HTC_FB_CONSOLE)
int __init htc_fb_console_init(void);
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
int __init ram_console_early_init(void);
#endif
#endif

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static void __init pmem_sf_size_setup(char **p)
{
	pmem_sf_size = memparse(*p, p);
}
__early_param("pmem_sf_size=", pmem_sf_size_setup);

static void __init tsunagi_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		mdp_pmem_pdata.start = __pa(addr);
		mdp_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}

}

static void __init tsunagi_map_io(void)
{
	msm_map_common_io();
	tsunagi_allocate_memory_regions();
	msm_clock_init();
	
#if defined(CONFIG_VERY_EARLY_CONSOLE)
// Init our consoles _really_ early
#if defined(CONFIG_HTC_FB_CONSOLE)
	htc_fb_console_init();
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
	ram_console_early_init();
#endif
#endif

}

extern struct sys_timer msm_timer;

MACHINE_START(TSUNAGI, "tsunagi")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= (CONFIG_PHYS_OFFSET + 0x00000100),
	.fixup		= tsunagi_fixup,
	.map_io		= tsunagi_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= tsunagi_init,
	.timer		= &msm_timer,
MACHINE_END
