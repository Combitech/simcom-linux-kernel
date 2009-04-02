/*
 *  linux/drivers/mtd/nand/simcom-nand.c
 *
 *  Copyright (C) 2008 Combitech AB.
 *  David Kiland <david.kiland@combitech.se>
 *
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
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <asm/arch/hardware.h>
#include <asm/arch/pxa-regs.h>

#define	MASK_ALE	(1 << 21)	/* our ALE is AD21 */
#define	MASK_CLE	(1 << 20)	/* our CLE is AD20 */

/* MTD structure for Simcom board */
static struct mtd_info *simcom_nand_mtd;

/* remaped IO address of the device */
static void __iomem *simcom_nand_io;

/*
 * Define static partitions for flash device
 */
static struct mtd_partition partition_info[] = {
	[0] = {
		.name	= "simcom-0",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL
	}
};


/*
 *	hardware specific access to control-lines
 */
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


/*
 * Main initialization routine	this->chip_delay = 100;
 */
static int simcom_init(void)
{
	struct nand_chip *this;
	const char *part_type;
	struct mtd_partition *mtd_parts;
	int mtd_parts_nb = 0;
	int ret;

	printk("Loading  simcom flash driver\n");
	/* Allocate memory for MTD device structure and private data */
	simcom_nand_mtd = kzalloc(sizeof(struct mtd_info) +
				  sizeof(struct nand_chip),
				  GFP_KERNEL);
	if (!simcom_nand_mtd) {
		printk("Unable to allocate SimCom NAND MTD device structure.\n");
		return -ENOMEM;
	}


	// Address register will be at MASK_ALE, and 4 bytes long.
	//simcom_nand_io = ioremap(PXA_CS1_PHYS | (MASK_CLE-1), 12);
	simcom_nand_io = ioremap(PXA_CS1_PHYS, 0x01000000);
	if (!simcom_nand_io) {
		printk("Unable to ioremap NAND device\n");
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

	/* 20 us command delay time */
	this->chip_delay = 25;
	this->ecc.mode = NAND_ECC_SOFT;

	/* Scan to find existence of the device */
	if (nand_scan (simcom_nand_mtd, 1)) {
		printk(KERN_NOTICE "No NAND device\n");
		ret = -ENXIO;
		goto err2;
	}

	if (!mtd_parts_nb) {
		mtd_parts = partition_info;
		mtd_parts_nb = 1;
		part_type = "static";
	}

	/* Register the partitions */
	printk(KERN_NOTICE "Using %s partition definition\n", part_type);
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
module_init(simcom_init);

/*
 * Clean up routine
 */
static void simcom_cleanup(void)
{
	/* Release resources, unregister device */
	nand_release(simcom_nand_mtd);

	iounmap(simcom_nand_io);

	/* Free the MTD device structure */
	kfree (simcom_nand_mtd);
}
module_exit(simcom_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");
MODULE_DESCRIPTION("NAND flash driver for Combitech SimCom Module");
