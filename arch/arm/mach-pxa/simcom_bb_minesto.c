/*
 * linux/arch/arm/mach-pxa/simcom_bb_minesto.c
 *
 * Combitech SimCoM Module support, based on cm-x2xx.c
 *
 * Copyright (C) 2010 Combitech AB
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
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <linux/dm9000.h>
#include <linux/i2c/pca953x.h>
#include <linux/spi/mcp2515.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#include <mach/pxa2xx-regs.h>
#include <mach/mfp-pxa27x.h>
#include <mach/pxafb.h>
#include <mach/ohci.h>
#include <mach/mmc.h>
#include <mach/pxa2xx_spi.h>
#include <mach/bitfield.h>

#include <plat/i2c.h>

#include "generic.h"
#include "devices.h"

/* SimCom device physical addresses */
#define SIMCOM_CS1_PHYS		(PXA_CS1_PHYS)
#define SIMCOM_MCP2515IRQ	83

/* GPIO related definitions */
#define SIMCOM_ETHIRQ		IRQ_GPIO(20)

#define DM9000_PHYS_BASE	(PXA_CS2_PHYS)
#define NOR_PHYS_BASE		(PXA_CS0_PHYS)
#define NAND_PHYS_BASE		(PXA_CS1_PHYS)



/***************************************************************/
/*                           GPIO                              */
/***************************************************************/
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

	/* PWM */
	GPIO16_PWM0_OUT,
	GPIO17_PWM1_OUT,
	GPIO11_PWM2_OUT,
	GPIO12_PWM3_OUT,

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


/***************************************************************/
/*                           NAND                              */
/***************************************************************/
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


/***************************************************************/
/*                            USB                              */
/***************************************************************/
static struct pxaohci_platform_data simcom_ohci_platform_data = {
	.port_mode	= PMM_PERPORT_MODE,
	.flags		= ENABLE_PORT_ALL
};



/***************************************************************/
/*                         Ethernet                            */
/***************************************************************/
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


/****************************************************************/
/*                            SPI							    */
/****************************************************************/
static struct pxa2xx_spi_master simcom_spi_port1_info = {
	.num_chipselect	= 3,
	.enable_dma		= 0,
};

static struct pxa2xx_spi_master simcom_spi_port2_info = {
	.num_chipselect	= 3,
	.enable_dma		= 0,
};


struct mcp2515_spi_platform_data simcom_mcp2515_pdata = {
	.cs_gpio = 24,
	.reset_gpio = 94,
};


static struct spi_board_info simcom_spi_devices[] __initdata = {
	{
		.modalias		= "mcp2515",
		.irq 			= SIMCOM_MCP2515IRQ,
		.max_speed_hz	= 6000000,
		.bus_num		= 1,
		.chip_select	= 0,
		.mode			= SPI_MODE_0,
		.platform_data	= &simcom_mcp2515_pdata,
	},
	{
		.modalias		= "adxl346",
		.max_speed_hz	= 6000000,
		.bus_num		= 1,
		.chip_select	= 1,
		.mode			= SPI_MODE_0,
	},
	{
		.modalias		= "mcp3001",
		.max_speed_hz	= 6000000,
		.bus_num		= 1,
		.chip_select	= 2,
		.mode			= SPI_MODE_0,
	},
	{
		.modalias		= "mcp3008",
		.max_speed_hz	= 6000000,
		.bus_num		= 2,
		.chip_select	= 0,
		.mode			= SPI_MODE_0,
	},
	{
		.modalias		= "adis16135",
		.max_speed_hz	= 6000000,
		.bus_num		= 2,
		.chip_select	= 1,
		.mode			= SPI_MODE_0,
	},
};

/***************************************************************/
/*                           I2C                               */
/***************************************************************/
static struct i2c_board_info simcom_minesto_i2c_info[] = {
	{
		I2C_BOARD_INFO("m41t65", 0x68) /* Realtime Clock */
	},
};


static void __init simcom_init(void)
{
	/* Initialize NAND */
	platform_device_register(&simcom_nand_device);
	/* Initialize DM9000 */
	platform_device_register(&simcom_dm9000_device);
	/* Initialize SPI interfaces */
	pxa2xx_set_spi_info(1, &simcom_spi_port1_info);
	pxa2xx_set_spi_info(2, &simcom_spi_port2_info);
	spi_register_board_info(ARRAY_AND_SIZE(simcom_spi_devices));

	/* Initialize USB */
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	pxa_set_ohci_info(&simcom_ohci_platform_data);
#endif

	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(simcom_minesto_i2c_info));

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
