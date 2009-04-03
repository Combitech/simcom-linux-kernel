/*
 *  linux/drivers/mtd/nand/simcom-nand.c
 *
 *  Copyright (C) 2008 Combitech AB.
 *  David Kiland <david.kiland@combitech.se>
 *	Tobias Knutsson <tobias.knutsson@combitech.se>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   Simcom board.
 */

#include <linux/mtd/nand.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/pxa2xx-regs.h>

#define	MASK_ALE	(1 << 21)	/* our ALE is AD21 */
#define	MASK_CLE	(1 << 20)	/* our CLE is AD20 */

/* MTD structure for Simcom board */
static struct mtd_info *simcom_nand_mtd;

/* Re-mapped IO address of the device */
static void __iomem *simcom_nand_io;


/* Fall-back partition table */
static struct mtd_partition partition_info[] = {
	[0] = {
		.name	= "default",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL
	}
};



/* Low-level driver */

static void simcom_hwcontrol(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
       struct nand_chip* this = mtd->priv;
       unsigned int nandaddr = (unsigned int)this->IO_ADDR_W;

       if (ctrl & NAND_CTRL_CHANGE) {
               nandaddr &= ~(MASK_ALE|MASK_CLE);
               if ( ctrl & NAND_ALE )
                       nandaddr |=  MASK_ALE;
               else
                       nandaddr &= ~MASK_ALE;
               if ( ctrl & NAND_CLE )
                       nandaddr |=  MASK_CLE;
               else
                       nandaddr &= ~MASK_CLE;
       }

       this->IO_ADDR_W = (void __iomem*)nandaddr;

       if(dat != NAND_CMD_NONE)
               writeb(dat, this->IO_ADDR_W);
}


/* Platform driver */

static int simcom_nand_probe(struct platform_device *pdev)
{
	struct nand_chip *this;
	struct physmap_flash_data *mtd_data = (struct physmap_flash_data*)pdev->dev.platform_data;
	struct mtd_partition *mtd_parts = mtd_data->parts;
	int mtd_parts_nb = mtd_data->nr_parts;
	int ret;

	dev_info(&pdev->dev, "Probing flash..\n");
	/* Allocate memory for MTD device structure and private data */
	simcom_nand_mtd = kzalloc(sizeof(struct mtd_info) +
				  sizeof(struct nand_chip),
				  GFP_KERNEL);
	if (!simcom_nand_mtd) {
		dev_err(&pdev->dev, "Unable to allocate SimCom NAND MTD device structure.\n");
		return -ENOMEM;
	}

	// Address register will be at MASK_ALE, and 4 bytes long.
	//simcom_nand_io = ioremap(PXA_CS1_PHYS | (MASK_CLE-1), 12);
	simcom_nand_io = ioremap(PXA_CS1_PHYS, 0x01000000);
	if (!simcom_nand_io) {
		dev_err(&pdev->dev, "Unable to ioremap NAND device\n");
		ret = -EINVAL;
		goto err1;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *)(&simcom_nand_mtd[1]);

	/* Link the private data with the MTD structure */
	simcom_nand_mtd->owner = THIS_MODULE;
	simcom_nand_mtd->priv = this;

	/* insert callbacks */
	this->IO_ADDR_R = simcom_nand_io;
	this->IO_ADDR_W = simcom_nand_io;
	this->cmd_ctrl = simcom_hwcontrol;
	this->dev_ready = NULL;

	/* 25 us command delay time */
	this->chip_delay = 25;
	this->ecc.mode = NAND_ECC_SOFT;

	/* Scan to find existence of the device */
	if (nand_scan (simcom_nand_mtd, 1)) {
		dev_err(&pdev->dev, "No NAND device found\n");
		ret = -ENXIO;
		goto err2;
	}

	if (!mtd_parts_nb) {
		mtd_parts = partition_info;
		mtd_parts_nb = 1;
		dev_info(&pdev->dev, "Using default partition definition\n");
	}

	/* Register the partitions */
	ret = add_mtd_partitions(simcom_nand_mtd, mtd_parts, mtd_parts_nb);
	if (ret)
		goto err2;

	/* Return happy */
	return 0;

err2:
	iounmap(simcom_nand_io);
err1:
	kfree(simcom_nand_mtd);

	return ret;

}

static int simcom_nand_remove(struct platform_device *pdev)
{
	/* Release resources, unregister device */
	nand_release(simcom_nand_mtd);
	iounmap(simcom_nand_io);

	/* Free the MTD device structure */
	kfree (simcom_nand_mtd);

	return 0;
}

static struct platform_driver simcom_nand_driver = {
	.probe		= simcom_nand_probe,
	.remove		= simcom_nand_remove,
	.driver		= {
		.name	= "simcom-nand",
		.owner	= THIS_MODULE,
	},
};


/* Module interface */

static int __init simcom_nand_init(void)
{
	return platform_driver_register(&simcom_nand_driver);
}
module_init(simcom_nand_init);

static void __exit simcom_nand_exit(void)
{
	platform_driver_unregister(&simcom_nand_driver);
}
module_exit(simcom_nand_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");
MODULE_DESCRIPTION("NAND flash driver for Combitech SimCom Module");
