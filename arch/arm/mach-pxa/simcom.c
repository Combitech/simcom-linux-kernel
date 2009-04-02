/*
 * linux/arch/arm/mach-pxa/simcom.c
 *
 * Copyright (C) 200 Combitech AB
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

#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/pxa2xx-regs.h>
#include <asm/arch/pxa2xx-gpio.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/ohci.h>
#include <asm/arch/mmc.h>
#include <asm/arch/bitfield.h>

#include "generic.h"

/* SimCom device physical addresses */
#define SIMCOM_CS1_PHYS		(PXA_CS1_PHYS)

/* GPIO related definitions */
#define SIMCOM_ETHIRQ		IRQ_GPIO(20)

#define DM9000_PHYS_BASE	(PXA_CS2_PHYS)
#define NOR_PHYS_BASE		(PXA_CS0_PHYS)
#define NAND_PHYS_BASE		(PXA_CS1_PHYS)

static struct mtd_partition simcom_partitions[] = {
	{
		.name		= "simcom",
		.offset		= 0,
		.size		= SZ_16M,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
};

static struct physmap_flash_data simcom_flash_data = {
	.width		= 4,
	.parts		= simcom_partitions,
	.nr_parts	= ARRAY_SIZE(simcom_partitions),
};

static struct resource simcom_dm9k_resource[] = {
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

/* Ethernet device */
static struct platform_device simcom_device_dm9k = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(simcom_dm9k_resource),
	.resource	= simcom_dm9k_resource,
};


static struct resource simcom_flash_resource = {
	.start		= NAND_PHYS_BASE,
	.end		= NAND_PHYS_BASE + SZ_16M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device simcom_flash_device = {
	.name		= "simcom-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &simcom_flash_data,
	},
	.num_resources	= 1,
	.resource	= &simcom_flash_resource,
};



/* audio device */
static struct platform_device simcom_audio_device = {
	.name		= "pxa2xx-ac97",
	.id		= -1,
};


/* platform devices */
static struct platform_device *platform_devices[] __initdata = {
	&simcom_device_dm9k,
	&simcom_audio_device,
	&simcom_flash_device,
};


static struct pxafb_mode_info generic_crt_800x600_mode = {
	.pixclock	= 28846,
	.bpp		= 8,
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
	.bpp		= 8,
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
	.bpp		= 8,
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

static struct pxafb_mach_info *simcom_display = &generic_tft_640x480;//&generic_crt_800x600;

static int simcom_ohci_init(struct device *dev)
{
	/* Set the Power Control Polarity Low */
	UHCHR = (UHCHR | UHCHR_PCPL) &
		~(UHCHR_SSEP1 | UHCHR_SSEP2 | UHCHR_SSE);

	return 0;
}

static struct pxaohci_platform_data simcom_ohci_platform_data = {
	.port_mode	= PMM_PERPORT_MODE,
	.init		= simcom_ohci_init,
};


static void __init simcom_init(void)
{
	set_pxa_fb_info(simcom_display);

	/* Register SimCom platform devices */
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));

	pxa_set_ohci_info(&simcom_ohci_platform_data);

	/* Enables the STUART */
	pxa_gpio_mode(GPIO46_STRXD_MD);
	pxa_gpio_mode(GPIO47_STTXD_MD);

	/* Enables the BTUART  */
	pxa_gpio_mode(GPIO42_BTRXD_MD);
	pxa_gpio_mode(GPIO43_BTTXD_MD);
	pxa_gpio_mode(GPIO44_BTCTS_MD);
	pxa_gpio_mode(GPIO45_BTRTS_MD);

	/* Enables the DM9000 */
	pxa_gpio_mode(GPIO78_nCS_2_MD);
	pxa_gpio_mode(21 | GPIO_OUT);

	/* Enables the NAND */
	pxa_gpio_mode(GPIO15_nCS_1_MD);
	pxa_gpio_mode(GPIO49_nPWE_MD);
	pxa_gpio_mode(GPIO48_nPOE_MD);
	pxa_gpio_mode(GPIO18_RDY_MD);
}

static void __init simcom_init_irq(void)
{
	pxa27x_init_irq();

	/* Setup interrupt for dm9000 */
	pxa_gpio_mode(IRQ_TO_GPIO(SIMCOM_ETHIRQ));
	set_irq_type(SIMCOM_ETHIRQ, IRQT_RISING);
}

static void __init simcom_map_io(void)
{
	pxa_map_io();
}


MACHINE_START(SIMCOM, "Combitech SimCom Module")
	.boot_params	= 0xa0000100,
	.phys_io	= 0x40000000,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= simcom_map_io,
	.init_irq	= simcom_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= simcom_init,
MACHINE_END
