/*
 * adis16135.h
 *
 *  Created on: Sep 17, 2010
 *      Author: dakila
 */

#ifndef ADIS16135_H_
#define ADIS16135_H_

/* register definitions */
#define FLASH_CNT		0x00
#define TEMP_OUT		0x02
#define GYRO_OUT2		0x04
#define GYRO_OUT		0x06
#define GYRO_OFF2		0x08
#define GYRO_OFF		0x0a
#define ALM_MAG1		0x10
#define ALM_MAG2		0x12
#define ALM_SMPL1		0x14
#define ALM_SMPL2		0x16
#define ALM_CTRL		0x18
#define GPIO_CTRL		0x1a
#define MSC_CTRL		0x1c
#define SMPL_PRD		0x1e
#define AVG_CNT			0x20
#define DEC_RATE		0x22
#define SLP_CTRL		0x24
#define DIAG_STAT		0x26
#define GLOB_CMD		0x28
#define LOT_ID1			0x32
#define LOT_ID2			0x34
#define LOT_ID3			0x36
#define PROD_ID			0x38
#define SERIAL_NUM		0x3a

#define ADIS16135_WRITE_REGISTER	1
#define ADIS16135_READ_REGISTER		2


#endif /* ADIS16135_H_ */
