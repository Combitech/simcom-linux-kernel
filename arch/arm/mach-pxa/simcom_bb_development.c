/*
 * linux/arch/arm/mach-pxa/simcom_bb_development.c
 *
 * Combitech SimCoM Module support, based on cm-x2xx.c
 *
 * Copyright (C) 2009 Combitech AB
 * David Kiland <david.kiland(at)combitech.se>
 * Tobias Knutsson <tobias.knutsson(at)combitech.se>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/pm.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <linux/dm9000.h>
#include <linux/i2c/pca953x.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/userspace-consumer.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#include <mach/pxa2xx-regs.h>
#include <mach/mfp-pxa27x.h>
#include <mach/pxafb.h>
#include <mach/ohci.h>
#include <mach/mmc.h>
#include <mach/bitfield.h>

#include <linux/gpio.h>

#include <plat/i2c.h>
#include <linux/i2c-dev.h>

#include "generic.h"
#include "devices.h"

/* SimCom device physical addresses */
#define SIMCOM_CS1_PHYS		(PXA_CS1_PHYS)

/* GPIO related definitions */
#define SIMCOM_ETHIRQ		IRQ_GPIO(20)

#define DM9000_PHYS_BASE	(PXA_CS2_PHYS)
#define NOR_PHYS_BASE		(PXA_CS0_PHYS)
#define NAND_PHYS_BASE		(PXA_CS1_PHYS)



static unsigned long simcom_pin_config[] = {

	/* BTUART */
	GPIO42_BTUART_RXD,
	GPIO43_BTUART_TXD,
	GPIO44_BTUART_CTS,
	GPIO45_BTUART_RTS,

	/* STUART */
	GPIO46_STUART_RXD,
	GPIO47_STUART_TXD,

	/* LCD */
	GPIO58_LCD_LDD_0,
	GPIO59_LCD_LDD_1,
	GPIO60_LCD_LDD_2,
	GPIO61_LCD_LDD_3,
	GPIO62_LCD_LDD_4,
	GPIO63_LCD_LDD_5,
	GPIO64_LCD_LDD_6,
	GPIO65_LCD_LDD_7,
	GPIO66_LCD_LDD_8,
	GPIO67_LCD_LDD_9,
	GPIO68_LCD_LDD_10,
	GPIO69_LCD_LDD_11,
	GPIO70_LCD_LDD_12,
	GPIO71_LCD_LDD_13,
	GPIO72_LCD_LDD_14,
	GPIO73_LCD_LDD_15,
	GPIO74_LCD_FCLK,
	GPIO75_LCD_LCLK,
	GPIO76_LCD_PCLK,
	GPIO77_LCD_BIAS,

	/* I2C */
	GPIO117_I2C_SCL,
	GPIO118_I2C_SDA,

	/* I2S */
	GPIO28_I2S_BITCLK_OUT,
	GPIO29_I2S_SDATA_IN,
	GPIO30_I2S_SDATA_OUT,
	GPIO31_I2S_SYNC,
	GPIO113_I2S_SYSCLK,

	/* SSP1 */
	GPIO23_SSP1_SCLK,
	GPIO24_SSP1_SFRM,
	GPIO25_SSP1_TXD,
	GPIO26_SSP1_RXD,

	/* SSP2 */
	GPIO19_SSP2_SCLK,
	GPIO14_SSP2_SFRM,
	GPIO87_SSP2_TXD,
	GPIO88_SSP2_RXD,

	/* USB */
	GPIO89_USBH1_PEN,
	GPIO120_USBH2_PEN,

	/* SDRAM and local bus */
	GPIO15_nCS_1,
	GPIO78_nCS_2,
	GPIO79_nCS_3,
	GPIO80_nCS_4,
	GPIO33_nCS_5,
	GPIO48_nPOE,
	GPIO49_nPWE,
	GPIO18_RDY,

	/* DM9000 */
	GPIO21_nSDCS_3,

};


/* NAND FLASH */
#if defined(CONFIG_MTD_NAND_SIMCOM)

static struct mtd_partition simcom_nand_partitions[] = {
	{
		.name		= "rootfs",
		.offset		= 0,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct physmap_flash_data simcom_nand_data = {
	.width		= 4,
	.parts		= simcom_nand_partitions,
	.nr_parts	= ARRAY_SIZE(simcom_nand_partitions),
};

static struct resource simcom_nand_resource = {
	.start		= NAND_PHYS_BASE,
	.end		= NAND_PHYS_BASE + SZ_128M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device simcom_nand_device = {
	.name		= "simcom-nand",
	.id		= 0,
	.dev		= {
		.platform_data	= &simcom_nand_data,
	},
	.num_resources	= 1,
	.resource	= &simcom_nand_resource,
};
static void __init simcom_init_nand(void)
{
	platform_device_register(&simcom_nand_device);
}
#else
static inline void simcom_init_nand(void) {}
#endif //NAND


/* DM9000 Ethernet */
#if defined(CONFIG_DM9000) || defined(CONFIG_DM9000_MODULE)
static struct resource simcom_dm9000_resource[] = {
	[0] = {
		.start = DM9000_PHYS_BASE,
		.end   = DM9000_PHYS_BASE + 3,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = DM9000_PHYS_BASE + 4,
		.end   = DM9000_PHYS_BASE + 7,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = SIMCOM_ETHIRQ,
		.end   = SIMCOM_ETHIRQ,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct platform_device simcom_dm9000_device = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(simcom_dm9000_resource),
	.resource	= simcom_dm9000_resource,
};

static void __init simcom_init_dm9000(void)
{
	platform_device_register(&simcom_dm9000_device);
}
#else
static inline void simcom_init_dm9000(void) {}
#endif //DM9000


/* USB OHCI controller */
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct pxaohci_platform_data simcom_ohci_platform_data = {
	.port_mode	= PMM_PERPORT_MODE,
	.flags		= ENABLE_PORT_ALL
};

static void __init simcom_init_ohci(void)
{
	pxa_set_ohci_info(&simcom_ohci_platform_data);
}
#else
static inline void simcom_init_ohci(void) {}
#endif // USB


/* PXA Frame Buffer */
#if defined(CONFIG_FB_PXA) || defined(CONFIG_FB_PXA_MODULE)
static struct pxafb_mode_info generic_crt_800x600_mode = {
	.pixclock	= 28846,
	.bpp		= 16,
	.xres		= 800,
	.yres	  	= 600,
	.hsync_len	= 63,
	.vsync_len	= 2,
	.left_margin	= 26,
	.upper_margin	= 21,
	.right_margin	= 26,
	.lower_margin	= 11,
	.sync		= (FB_SYNC_HOR_HIGH_ACT |
			   FB_SYNC_VERT_HIGH_ACT),
	.cmap_greyscale = 0,
};

static struct pxafb_mach_info generic_crt_800x600 = {
	.modes		= &generic_crt_800x600_mode,
	.num_modes	= 1,
	.lccr0		= (LCCR0_PAS),
	.lccr3		= (LCCR3_PixClkDiv(0x02) |
			   LCCR3_Acb(0xff)),
	.cmap_inverse	= 0,
	.cmap_static	= 0,
};

static struct pxafb_mode_info generic_crt_640x480_mode = {
	.pixclock	= 38461,
	.bpp		= 16,
	.xres		= 640,
	.yres		= 480,
	.hsync_len	= 63,
	.vsync_len	= 2,
	.left_margin	= 81,
	.upper_margin	= 33,
	.lower_margin	= 10,
	.sync		= (FB_SYNC_HOR_HIGH_ACT |
			   FB_SYNC_VERT_HIGH_ACT),
	.cmap_greyscale = 0,
};

static struct pxafb_mach_info generic_crt_640x480 = {
	.modes		= &generic_crt_640x480_mode,
	.num_modes	= 1,
	.lccr0		= (LCCR0_PAS),
	.lccr3		= (LCCR3_PixClkDiv(0x01) |
			   LCCR3_Acb(0xff)),
	.cmap_inverse	= 0,
	.cmap_static	= 0,
};

static struct pxafb_mode_info generic_tft_640x480_mode = {
	.pixclock	= 38461,
	.bpp		= 16,
	.xres		= 640,
	.yres		= 480,
	.hsync_len	= 60,
	.vsync_len	= 2,
	.left_margin	= 70,
	.upper_margin	= 10,
	.right_margin	= 70,
	.lower_margin	= 5,
	.sync		= 0,
	.cmap_greyscale = 0,
};

static struct pxafb_mach_info generic_tft_640x480 = {
	.modes		= &generic_tft_640x480_mode,
	.num_modes	= 1,
	.lccr0		= (LCCR0_PAS),
	.lccr3		= (LCCR3_PixClkDiv(0x01) |
			   LCCR3_Acb(0xff) |
			   LCCR3_PCP),

	.cmap_inverse	= 0,
	.cmap_static	= 0,
};

static struct pxafb_mach_info *simcom_display = &generic_tft_640x480;
static void __init simcom_init_display(void)
{
	set_pxa_fb_info(simcom_display);
}
#else
static inline void simcom_init_display(void) {}
#endif // PXAFB

/* Development baseboard */
#if defined(CONFIG_SIMCOM_BB_DEV)
static struct pca953x_platform_data gpio_exp = {
		.gpio_base	= 128,
		.invert = 0,
};
static struct i2c_board_info simcom_bb_dev_i2c_info[] = {
	{	/* I2C Switch */
		I2C_BOARD_INFO("pca9546a", 0x70),
	},
	{	/* GPIO Expander */
		I2C_BOARD_INFO("pca9539", 0x74),
		.platform_data = &gpio_exp,
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
};

static void __init simcom_init_baseboard(void)
{
	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(simcom_bb_dev_i2c_info));
}
#else
static inline void simcom_init_baseboard(void) {}
#endif // DEV BB

/* CONSUMERS */
#define simcom_user_consumer(_name, _id)	\
static struct regulator_bulk_data simcom_##_name##_consumer_supply = {	\
	.supply		= #_name "_vcc",										\
};																		\
static struct regulator_userspace_consumer_data			\
		simcom_##_name##_consumer_data = {				\
	.name		= #_name "_vcc",						\
	.num_supplies	= 1,								\
	.supplies	= &simcom_##_name##_consumer_supply,	\
};														\
static struct platform_device simcom_##_name##_consumer = {	\
	.name = "reg-userspace-consumer",						\
	.id = _id,												\
	.dev = {												\
		.platform_data = &simcom_##_name##_consumer_data,	\
	},														\
}

simcom_user_consumer(VDCDC1,0);
simcom_user_consumer(VDCDC2,1);
simcom_user_consumer(VDCDC3,2);


static struct platform_device *simcom_consumers[] = {
		&simcom_VDCDC1_consumer,
		&simcom_VDCDC2_consumer,
		&simcom_VDCDC3_consumer,
};


static int __init simcom_init_consumers(void)
{
	//regulator_has_full_constraints();
	return platform_add_devices(simcom_consumers, ARRAY_SIZE(simcom_consumers));
}
/* CONSUMERS */

/* REGULATOR */
#define simcom_regulator(_name, _min, _max, _boot_on) 	\
{											\
	.constraints = {						\
		.min_uV = (_min)*1000,				\
		.max_uV = (_max)*1000,				\
		.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS),\
		.boot_on = _boot_on,				\
	},										\
	.num_consumer_supplies = ARRAY_SIZE(simcom_##_name##_consumers),	\
	.consumer_supplies = simcom_##_name##_consumers,					\
}

#define simcom_supply_single(_name)										\
static struct regulator_consumer_supply simcom_##_name##_consumers[] = {	\
	{	.supply = #_name "_vcc",	},									\
}

simcom_supply_single(VDCDC1);
simcom_supply_single(VDCDC2);
simcom_supply_single(VDCDC3);
simcom_supply_single(LDO1);
simcom_supply_single(LDO2);



static struct regulator_init_data simcom_regulators[] = {
	simcom_regulator(VDCDC1, 3300, 3300, 1),
	simcom_regulator(VDCDC2, 1800, 1800, 1),
	simcom_regulator(VDCDC3, 800, 1600, 1),
	simcom_regulator(LDO1, 1000, 3300, 1),
	simcom_regulator(LDO2, 1000, 3300, 1),
};


/* REGULATOR */

/* POWER I2C */
static struct i2c_board_info simcom_bb_dev_pwr_i2c_info[] = {
	{	/* PMIC */
		.type			= "tps65020",
		.addr			= 0x48,
		.platform_data	= &simcom_regulators,
	},
};



static void __init simcom_init_pwr_i2c(void)
{
	pxa27x_set_i2c_power_info(NULL);
	i2c_register_board_info(1, ARRAY_AND_SIZE(simcom_bb_dev_pwr_i2c_info));
}

/* POWER I2C */


static void __init simcom_init(void)
{
	simcom_init_pwr_i2c();
	simcom_init_nand();
	simcom_init_dm9000();
	simcom_init_ohci();
	simcom_init_display();
	simcom_init_baseboard();
	simcom_init_consumers();

	pxa2xx_mfp_config(ARRAY_AND_SIZE(simcom_pin_config));
}

static void __init simcom_init_irq(void)
{
	pxa27x_init_irq();
}

static void __init simcom_map_io(void)
{
	pxa_map_io();
}


MACHINE_START(SIMCOM, "Combitech SimCoM")
	.boot_params	= 0xa0000100,
	.phys_io	= 0x40000000,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= simcom_map_io,
	.init_irq	= simcom_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= simcom_init,
MACHINE_END
