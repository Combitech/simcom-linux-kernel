/*
 * nacelle_sensors.c
 *
 *  Created on: Feb 17, 2011
 *      Author: dakila
 */

/*
 * adxl34x.c
 *
 *  Created on: Sep 16, 2010
 *      Author: dakila
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <mach/ssp.h>
#include <mach/regs-ssp.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>


MODULE_DESCRIPTION("Analog Devices adxl345/adxl346 3-axis Digital Accelerometer");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");


/* Command to recieve measures */
#define	CMD_GET 0xAB
#define GET_ACK 0xCD


/*
 * IOCTL Commands
 */
#define AD77XX_WRITE_REGISTER		1
#define AD77XX_READ_REGISTER		2

/*
 * Registers
 */
#define AD77XX_STATUS_REG 			0x0
#define AD77XX_MODE_REG   			0x1
#define AD77XX_CONFIG_REG 			0x2
#define AD77XX_DATA_REG   			0x3
#define AD77XX_ID_REG     			0x4
#define AD77XX_IO_REG     			0x5
#define AD77XX_OFFSET_REG 			0x6
#define AD77XX_SCALE_REG  			0x7


/*
 * Status
 */
#define AD77XX_STATUS_RDY       	0x80
#define AD77XX_STATUS_ERROR     	0x40
#define AD77XX_STATUS_NOREF     	0x20
#define AD77XX_STATUS_IS_AD7799 	0x08
#define AD77XX_STATUS_CHAN_MASK 	0x07



/*
 * 	Configurations
 */
enum {	AD77XX_CONTINUOUS_CONVERSION_MODE = 0,
		AD77XX_SINGLE_CONVERSION_MODE,
		AD77XX_IDLE_MODE, AD77XX_POWERDOWN_MODE,
		AD77XX_INTERNAL_OFFSET_CAL_MODE,
		AD77XX_INTERNAL_SCALE_CAL_MODE,
		AD77XX_SYSTEM_OFFSET_CAL_MODE,
		AD77XX_SYSTEM_SCALE_CAL_MODE
};

enum {	AD77XX_470_HZ = 1,
		AD77XX_242_HZ,
		AD77XX_123_HZ,
		AD77XX_62_HZ,
		AD77XX_50_HZ,
		AD77XX_39_HZ,
		AD77XX_33_2_HZ,
		AD77XX_19_6_HZ,
		AD77XX_16_7_1_HZ,
		AD77XX_16_7_2_HZ,
		AD77XX_12_5_HZ,
		AD77XX_10_HZ,
		AD77XX_8_33_HZ,
		AD77XX_6_25_HZ,
		AD77XX_4_17_HZ
};

enum {
		AD77XX_1_GAIN = 0,
		AD77XX_2_GAIN,
		AD77XX_4_GAIN,
		AD77XX_8_GAIN,
		AD77XX_16_GAIN,
		AD77XX_32_GAIN,
		AD77XX_64_GAIN,
		AD77XX_128_GAIN
};

enum { 	AD77XX_AIN1_CHAN = 0,
		AD77XX_AIN2_CHAN,
		AD77XX_AIN3_CHAN,
		AD77XX_AIN11_CHAN,
		AD77XX_AVDD_CHAN
};



/* ADXL345/6 Register Map */
#define DEVID			0x00	/* R   Device ID */
#define THRESH_TAP		0x1D	/* R/W Tap threshold */
#define OFSX			0x1E	/* R/W X-axis offset */
#define OFSY			0x1F	/* R/W Y-axis offset */
#define OFSZ			0x20	/* R/W Z-axis offset */
#define DUR				0x21	/* R/W Tap duration */
#define LATENT			0x22	/* R/W Tap latency */
#define WINDOW			0x23	/* R/W Tap window */
#define THRESH_ACT		0x24	/* R/W Activity threshold */
#define THRESH_INACT	0x25	/* R/W Inactivity threshold */
#define TIME_INACT		0x26	/* R/W Inactivity time */
#define ACT_INACT_CTL	0x27	/* R/W Axis enable control for activity and */

/* inactivity detection */
#define THRESH_FF		0x28	/* R/W Free-fall threshold */
#define TIME_FF			0x29	/* R/W Free-fall time */
#define TAP_AXES		0x2A	/* R/W Axis control for tap/double tap */
#define ACT_TAP_STATUS	0x2B	/* R   Source of tap/double tap */
#define BW_RATE			0x2C	/* R/W Data rate and power mode control */
#define POWER_CTL		0x2D	/* R/W Power saving features control */
#define INT_ENABLE		0x2E	/* R/W Interrupt enable control */
#define INT_MAP			0x2F	/* R/W Interrupt mapping control */
#define INT_SOURCE		0x30	/* R   Source of interrupts */
#define DATA_FORMAT		0x31	/* R/W Data format control */
#define DATAX0			0x32	/* R   X-Axis Data 0 */
#define DATAX1			0x33	/* R   X-Axis Data 1 */
#define DATAY0			0x34	/* R   Y-Axis Data 0 */
#define DATAY1			0x35	/* R   Y-Axis Data 1 */
#define DATAZ0			0x36	/* R   Z-Axis Data 0 */
#define DATAZ1			0x37	/* R   Z-Axis Data 1 */
#define FIFO_CTL		0x38	/* R/W FIFO control */
#define FIFO_STATUS		0x39	/* R   FIFO status */
#define TAP_SIGN		0x3A	/* R   Sign and source for tap/double tap */

/* Orientation ADXL346 only */
#define ORIENT_CONF		0x3B	/* R/W Orientation configuration */
#define ORIENT			0x3C	/* R   Orientation status */

/* DEVIDs */
#define ID_ADXL345		0xE5
#define ID_ADXL346		0xE6

/* INT_ENABLE/INT_MAP/INT_SOURCE Bits */
#define DATA_READY		(1 << 7)
#define SINGLE_TAP		(1 << 6)
#define DOUBLE_TAP		(1 << 5)
#define ACTIVITY		(1 << 4)
#define INACTIVITY		(1 << 3)
#define FREE_FALL		(1 << 2)
#define WATERMARK		(1 << 1)
#define OVERRUN			(1 << 0)

/* ACT_INACT_CONTROL Bits */
#define ACT_ACDC		(1 << 7)
#define ACT_X_EN		(1 << 6)
#define ACT_Y_EN		(1 << 5)
#define ACT_Z_EN		(1 << 4)
#define INACT_ACDC		(1 << 3)
#define INACT_X_EN		(1 << 2)
#define INACT_Y_EN		(1 << 1)
#define INACT_Z_EN		(1 << 0)

/* TAP_AXES Bits */
#define SUPPRESS		(1 << 3)
#define TAP_X_EN		(1 << 2)
#define TAP_Y_EN		(1 << 1)
#define TAP_Z_EN		(1 << 0)

/* ACT_TAP_STATUS Bits */
#define ACT_X_SRC		(1 << 6)
#define ACT_Y_SRC		(1 << 5)
#define ACT_Z_SRC		(1 << 4)
#define ASLEEP			(1 << 3)
#define TAP_X_SRC		(1 << 2)
#define TAP_Y_SRC		(1 << 1)
#define TAP_Z_SRC		(1 << 0)

/* BW_RATE Bits */
#define LOW_POWER		(1 << 4)
#define RATE(x)			((x) & 0xF)

/* POWER_CTL Bits */
#define PCTL_LINK		(1 << 5)
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE	(1 << 3)
#define PCTL_SLEEP		(1 << 2)
#define PCTL_WAKEUP(x)	((x) & 0x3)

/* DATA_FORMAT Bits */
#define SELF_TEST		(1 << 7)
#define SPI				(1 << 6)
#define INT_INVERT		(1 << 5)
#define FULL_RES		(1 << 3)
#define JUSTIFY			(1 << 2)
#define RANGE(x)		((x) & 0x3)
#define RANGE_PM_2g		0
#define RANGE_PM_4g		1
#define RANGE_PM_8g		2
#define RANGE_PM_16g	3

/*
 * Maximum value our axis may get in full res mode for the input device
 * (signed 13 bits)
 */
#define ADXL_FULLRES_MAX_VAL 4096

/*
 * Maximum value our axis may get in fixed res mode for the input device
 * (signed 10 bits)
 */
#define ADXL_FIXEDRES_MAX_VAL 512

/* FIFO_CTL Bits */
#define FIFO_MODE(x)	(((x) & 0x3) << 6)
#define FIFO_BYPASS		0
#define FIFO_FIFO		1
#define FIFO_STREAM		2
#define FIFO_TRIGGER	3
#define TRIGGER			(1 << 5)
#define SAMPLES(x)		((x) & 0x1F)

/* FIFO_STATUS Bits */
#define FIFO_TRIG		(1 << 7)
#define ENTRIES(x)		((x) & 0x3F)

/* TAP_SIGN Bits ADXL346 only */
#define XSIGN			(1 << 6)
#define YSIGN			(1 << 5)
#define ZSIGN			(1 << 4)
#define XTAP			(1 << 3)
#define YTAP			(1 << 2)
#define ZTAP			(1 << 1)

/* ORIENT_CONF ADXL346 only */
#define ORIENT_DEADZONE(x)	(((x) & 0x7) << 4)
#define ORIENT_DIVISOR(x)	((x) & 0x7)

/* ORIENT ADXL346 only */
#define ADXL346_2D_VALID			(1 << 6)
#define ADXL346_2D_ORIENT(x)		(((x) & 0x3) >> 4)
#define ADXL346_3D_VALID			(1 << 3)
#define ADXL346_3D_ORIENT(x)		((x) & 0x7)
#define ADXL346_2D_PORTRAIT_POS		0	/* +X */
#define ADXL346_2D_PORTRAIT_NEG		1	/* -X */
#define ADXL346_2D_LANDSCAPE_POS	2	/* +Y */
#define ADXL346_2D_LANDSCAPE_NEG	3	/* -Y */

#define ADXL346_3D_FRONT			3	/* +X */
#define ADXL346_3D_BACK				4	/* -X */
#define ADXL346_3D_RIGHT			2	/* +Y */
#define ADXL346_3D_LEFT				5	/* -Y */
#define ADXL346_3D_TOP				1	/* +Z */
#define ADXL346_3D_BOTTOM			6	/* -Z */

#undef ADXL_DEBUG

#define ADXL_X_AXIS		0
#define ADXL_Y_AXIS		1
#define ADXL_Z_AXIS		2

#define WRITE_SINGLE	0x00
#define WRITE_MULT		0x40
#define READ_SINGLE		0x80
#define READ_MULT		0xc0


struct nacelle_sensors_priv {
	struct ssp_dev spi_dev;
	int cs_adxl;
	int cs_mcp3008;
	int cs_mcp3001_0;
	int cs_mcp3001_1;
	int cs_adis16135;
	int cs_ad7799;
	int cs_pic;
	int type;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};


struct nacelle_sensors_data {
	short adxl_x;
	short adxl_y;
	short adxl_z;
	short adis16135_gyro;
	short adis16135_temp;
	short mcp_3008[8];
	short mcp_3001_0;
	short mcp_3001_1;
	int ad7799;
	short pwm_value[2];
	short pwm_freq[2];
	short pump_counter;
};


static struct class *nacelle_sensors_class = 0;



static int spi_read_byte(struct ssp_dev *dev, u8 output, u8 *data)
{
	u32 d = 0;

	ssp_write_word(dev, output);
	ssp_read_word(dev, &d);
	ssp_flush(dev);
	*data = d;
	return 0;
}

static int spi_write_byte(struct ssp_dev *dev, u8 data)
{
	ssp_write_word(dev, data);
	ssp_flush(dev);
	return 0;
}



/****************************************************************************/
/*                                 PIC										*/
/****************************************************************************/


static int pic_read(struct nacelle_sensors_priv *priv, struct nacelle_sensors_data *data)
{

	u8 rx[11];
	u8 tries = 0;

	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(128));

	ssp_enable(&priv->spi_dev);



	gpio_set_value(priv->cs_pic, 0);

	spi_write_byte(&priv->spi_dev, CMD_GET);

	for(; tries < 3; tries++)
	{
		spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);
		if(rx[0] == GET_ACK)
			break;
	}

	spi_read_byte(&priv->spi_dev, 0xff, &rx[1]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[2]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[3]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[4]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[5]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[6]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[7]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[8]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[9]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[10]);

	gpio_set_value(priv->cs_pic, 1);

	data->pwm_value[0] = (rx[1]<<8) + rx[2];
	data->pwm_freq[0] = (rx[3]<<8) + rx[4];
	data->pwm_value[0] = (rx[5]<<8) + rx[6];
	data->pwm_freq[1] = (rx[7]<<8) + rx[8];
	data->pump_counter = (rx[9]<<8) + rx[10];

	return 0;
}




/****************************************************************************/
/*                                 mcp300x									*/
/****************************************************************************/


static int mcp300x_read(struct nacelle_sensors_priv *priv, struct nacelle_sensors_data *data)
{
	u8 i;
	u8 rx[3];


	/* read from cmp3008 */
	for(i=0; i<8; i++) {
		gpio_set_value(priv->cs_mcp3008, 0);
			spi_write_byte(&priv->spi_dev, 0x01);
			spi_read_byte(&priv->spi_dev, 0x80 | (i<<4), &rx[0]);
			spi_read_byte(&priv->spi_dev, 0xff, &rx[1]);
			data->mcp_3008[i] = ((rx[0]&0x3)<<8) | rx[1];
		gpio_set_value(priv->cs_mcp3008, 1);
	}

	/* read from mcp3001 0 */
	gpio_set_value(priv->cs_mcp3001_0, 0);
		spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);
		spi_read_byte(&priv->spi_dev, 0xff, &rx[1]);
		data->mcp_3001_0 = ((rx[0]&0x1f)<<5) | (((rx[1]&0xf8)>>3)&0x1f);
	gpio_set_value(priv->cs_mcp3001_0, 1);

	/* read from mcp3001 0 */
	gpio_set_value(priv->cs_mcp3001_1, 0);
		spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);
		spi_read_byte(&priv->spi_dev, 0xff, &rx[1]);
		data->mcp_3001_1 = ((rx[0]&0x1f)<<5) | (((rx[1]&0xf8)>>3)&0x1f);
	gpio_set_value(priv->cs_mcp3001_1, 1);



	return 0;
}




/****************************************************************************/
/*                                 AD7799									*/
/****************************************************************************/


static inline void ad77xx_comm(struct nacelle_sensors_priv *priv, u8 reg, u8 read, u8 cont)
{
	u8 byte = (read ? 0x40 : 0x00) | (reg << 3) | (cont ? 0x04 : 0x00);
	spi_write_byte(&priv->spi_dev, byte);
}


/*
 * Read Status Register
 */
static u8 ad77xx_status(struct nacelle_sensors_priv *priv)
{
	u8 rx;

	ad77xx_comm(priv, AD77XX_STATUS_REG, 1, 0);

	spi_read_byte(&priv->spi_dev, 0xff, &rx);

	return rx;
}


/*
 * Read Status Register
 */
static u8 ad77xx_read_id(struct nacelle_sensors_priv *priv)
{
	u8 rx;

	ad77xx_comm(priv, AD77XX_ID_REG, 1, 0);

	spi_read_byte(&priv->spi_dev, 0xff, &rx);

	printk("ad77xx device id is 0x%x\n", rx);

	return rx;
}


/*
 * Read Status Register
 */
static void ad77xx_reset(struct nacelle_sensors_priv *priv)
{

	spi_write_byte(&priv->spi_dev, 0xff);
	spi_write_byte(&priv->spi_dev, 0xff);
	spi_write_byte(&priv->spi_dev, 0xff);
	spi_write_byte(&priv->spi_dev, 0xff);
	spi_write_byte(&priv->spi_dev, 0xff);

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(HZ/2);

}


/*
 * Read Offset Register
 */
u32 ad77xx_read_offset(struct nacelle_sensors_priv *priv)
{
	u8 rx;
	u32 val = 0;

	ad77xx_comm(priv, AD77XX_OFFSET_REG, 1, 0);

	spi_read_byte(&priv->spi_dev, 0xff, &rx);
	val |= rx << 16;

	spi_read_byte(&priv->spi_dev, 0xff, &rx);
	val |= rx << 8;

	spi_read_byte(&priv->spi_dev, 0xff, &rx);
	val |= rx;


	return val;
}

/*
 * Read Full Scale Register
 */
u32 ad77xx_read_scale(struct nacelle_sensors_priv *priv)
{
	u32 val = 0;
	u8 rx;

	ad77xx_comm(priv, AD77XX_SCALE_REG, 1, 0);

	spi_read_byte(&priv->spi_dev, 0xff, &rx);
	val |= rx << 16;

	spi_read_byte(&priv->spi_dev, 0xff, &rx);
	val |= rx << 8;

	spi_read_byte(&priv->spi_dev, 0xff, &rx);
	val |= rx;


	return val;
}

/*
 * Set Mode
 *
 * @param mode Mode (see Configuration above)
 * @param pwd Power Switch bit
 * @param rate Update rate
 */
static void ad77xx_set_mode(struct nacelle_sensors_priv *priv, u8 mode, u8 psw, u8 rate)
{
	u8 tx;

    ad77xx_comm(priv, AD77XX_MODE_REG, 0, 0);

	tx = mode << 5 | (psw ? 0x10 : 0x00);
    spi_write_byte(&priv->spi_dev, tx);
	spi_write_byte(&priv->spi_dev, rate);
}

/*
 * Write to Configuration Register
 *
 * @param burnout Burn out current enable bit
 * @param unipolar Unipolar/Bipolar enable bit
 * @param gain Gain select bits
 * @param ref_det Reference detect bit
 * @param buf ADC Buffered mode
 * @param chan Channel select bit
 */
void ad77xx_write_config(	struct nacelle_sensors_priv *priv,
							u8 burnout, u8 unipolar, u8 gain,
							u8 ref_det, u8 buf, u8 chan)
{
	u8 tx;

	ad77xx_comm(priv, AD77XX_CONFIG_REG, 0, 1);

	tx = ((burnout ? 0x20 : 0x00) | (unipolar ? 0x10 : 0x00) | gain);
	spi_write_byte(&priv->spi_dev, tx);

	tx = ((ref_det ? 0x20 : 0x00) | (buf ? 0x10 : 0x00) | chan);
	spi_write_byte(&priv->spi_dev, tx);
}


/*
 *  Request a read from the data register
 *
 *@param continuous Continuous reading
 */
void ad77xx_request_data(struct nacelle_sensors_priv *priv, u8 continuous)
{
	ad77xx_comm(priv, AD77XX_DATA_REG, 1, continuous);
}



/*
 * Determine if data is ready to be read
 *
 */
u8 ad77xx_data_ready(struct nacelle_sensors_priv *priv)
{
	u8 status;

	status = ad77xx_status(priv);
	status &= AD77XX_STATUS_RDY;

	return !status;
}


/* Read from data register, it should be previously requested
 *   from ad77xx_request_data. The value is signed!!
 *
 */
s32 ad77xx_read_data(struct nacelle_sensors_priv *priv)
{
	s32 val = 0;
	u8 rx;

	spi_read_byte(&priv->spi_dev, 0xff, &rx);
	val = rx;
	val <<= 8;

	spi_read_byte(&priv->spi_dev, 0xff, &rx);
	val |= rx;
	val <<= 8;

	spi_read_byte(&priv->spi_dev, 0xff, &rx);
	val |= rx;

	return val;

}



/*
 * Calibrate device
 */
void ad77xx_calibrate(struct nacelle_sensors_priv *priv)
{
	u32 off = ad77xx_read_offset(priv);
	printk("ad77xx: offset: 0x%x\n", off);

	/* cal */
	ad77xx_set_mode(priv, AD77XX_INTERNAL_OFFSET_CAL_MODE, 0, AD77XX_470_HZ);


	off = ad77xx_read_offset(priv);
	printk("offset: 0x%x\n", off);
}

/*
 * Initialize AD77XX
 */
static void ad77xx_init(struct nacelle_sensors_priv *priv)
{

	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(6));

	ssp_enable(&priv->spi_dev);

	gpio_set_value(priv->cs_ad7799, 0);

		ad77xx_reset(priv);
		ad77xx_read_id(priv);
		ad77xx_set_mode(priv, AD77XX_CONTINUOUS_CONVERSION_MODE, 0, AD77XX_470_HZ);
		ad77xx_write_config(priv,
			0,					/* No burn out */
			1,					/* Unipolar */
			AD77XX_1_GAIN,		/* 1 Gain */
			0,					/* No reference detection */
			1,					/* Buffered */
			AD77XX_AIN1_CHAN	/* AIN1 Channel */
		);

	gpio_set_value(priv->cs_ad7799, 1);

}


static int ad77xx_read(struct nacelle_sensors_priv *priv, struct nacelle_sensors_data *data)
{
	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(6));

	ssp_enable(&priv->spi_dev);

	gpio_set_value(priv->cs_ad7799, 0);

		ad77xx_request_data(priv, 1);

		while(gpio_get_value(41));

		data->ad7799 = ad77xx_read_data(priv);

	gpio_set_value(priv->cs_ad7799, 1);


	return 0;
}



/****************************************************************************/
/*                                 ADIS16135								*/
/****************************************************************************/


static int adis16135_read(struct nacelle_sensors_priv *priv, struct nacelle_sensors_data *data)
{
	u8 values[6];
	volatile int k;


	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(16));

	ssp_enable(&priv->spi_dev);


	gpio_set_value(priv->cs_adis16135, 0);
		spi_read_byte(&priv->spi_dev, 0x06, &values[0]);
		spi_read_byte(&priv->spi_dev, 0x00, &values[0]);
	gpio_set_value(priv->cs_adis16135, 1);

	for(k=0; k<1000; k++) {}

	gpio_set_value(priv->cs_adis16135, 0);
		spi_read_byte(&priv->spi_dev, 0x02, &values[0]);
		spi_read_byte(&priv->spi_dev, 0x00, &values[1]);
	gpio_set_value(priv->cs_adis16135, 1);

	for(k=0; k<1000; k++) {}

	gpio_set_value(priv->cs_adis16135, 0);
		spi_read_byte(&priv->spi_dev, 0xff, &values[2]);
		spi_read_byte(&priv->spi_dev, 0xff, &values[3]);
	gpio_set_value(priv->cs_adis16135, 1);


	data->adis16135_gyro = (values[0]<<8) +  values[1];
	data->adis16135_temp = (values[2]<<8) +  values[3];

	return 6;
}


/****************************************************************************/
/*                                 ADXL345									*/
/****************************************************************************/

static int adxl34x_read_id(struct nacelle_sensors_priv *priv)
{
	u8 tx[1];
	u8 rx[1];

	tx[0] = READ_SINGLE | DEVID;

	gpio_set_value(priv->cs_adxl, 0);

	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);

	gpio_set_value(priv->cs_adxl, 1);

	return rx[0];
}


static int adxl34x_read_reg(struct nacelle_sensors_priv *priv, u8 reg)
{
	u8 tx[1];
	u8 rx[1];

	tx[0] = READ_SINGLE | reg;

	gpio_set_value(priv->cs_adxl, 0);

	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);

	gpio_set_value(priv->cs_adxl, 1);
	return rx[0];
}


static void adxl34x_write_reg(struct nacelle_sensors_priv *priv, u8 reg, u8 value)
{
	u8 tx[1];

	tx[0] = WRITE_SINGLE | reg;

	gpio_set_value(priv->cs_adxl, 0);

	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_write_byte(&priv->spi_dev, value);

	gpio_set_value(priv->cs_adxl, 1);
}

static int adxl_init(struct nacelle_sensors_priv *priv)
{
	u8 id;

	ssp_disable(&priv->spi_dev);
	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(6));

	ssp_enable(&priv->spi_dev);

	id = adxl34x_read_id(priv);

	printk("ADXL345 device ID is: 0x%02x\n", id);

	adxl34x_write_reg(priv, POWER_CTL, PCTL_MEASURE);
	return 0;
}


static int adxl_read(struct nacelle_sensors_priv *priv, struct nacelle_sensors_data *data)
{
	u8 rx[6];
	u8 tx[1];


	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(6));

	ssp_enable(&priv->spi_dev);


	tx[0] = READ_MULT | DATAX0;

	gpio_set_value(priv->cs_adxl, 0);
	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[0]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[1]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[2]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[3]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[4]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[5]);
	gpio_set_value(priv->cs_adxl, 1);

	data->adxl_x = (rx[1]<<8) + rx[0];
	data->adxl_y = (rx[3]<<8) + rx[2];
	data->adxl_z = (rx[5]<<8) + rx[4];


	return 0;
}


static ssize_t nacelle_sensors_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct nacelle_sensors_data data;

	struct nacelle_sensors_priv *priv = file->private_data;

	if(count < sizeof(struct nacelle_sensors_data)) {
		return 0;
	}

	adxl_read(priv, &data);
	adis16135_read(priv, &data);
	ad77xx_read(priv, &data);
	mcp300x_read(priv, &data);
	pic_read(priv, &data);

	if(copy_to_user(buf, &data, sizeof(struct nacelle_sensors_data))) {
		return 0;
	}

	return sizeof(struct nacelle_sensors_data);
}

static ssize_t nacelle_sensors_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct nacelle_sensors_priv *priv = file->private_data;


	return count;
}


int nacelle_sensors_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct adxl34x_priv *priv = file->private_data;
	u16 data[2];
	int ret;


	switch (cmd) {
	case 1:
		if(copy_from_user(data, (u16 *)arg, 2)) {
			ret = -ENOMEM;
		}
		ret = adxl34x_read_reg(priv, data[0]);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}


static int nacelle_sensors_open(struct inode *inode, struct file *file)
{
	struct adxl34x_priv *priv = container_of(inode->i_cdev, struct nacelle_sensors_priv, cdev);

	file->private_data = priv;

	return 0;
}

static int nacelle_sensors_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations nacelle_sensors_ops = {
	.owner		= THIS_MODULE,
	.read		= nacelle_sensors_read,
	.write		= nacelle_sensors_write,
	.open		= nacelle_sensors_open,
	.release	= nacelle_sensors_release,
	.ioctl		= nacelle_sensors_ioctl,
};

static int __devinit nacelle_sensors_probe(struct platform_device *pdev)
{
	struct nacelle_sensors_priv *priv;
	int err;
	struct resource *r;
	u8 id;

	dev_info(&pdev->dev, "Probing nacelle_sensors..\n");

	priv = kzalloc(sizeof(struct nacelle_sensors_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}

	/* Initialize spi and read device id */
	dev_set_drvdata(&pdev->dev, priv);

	if(ssp_init(&priv->spi_dev, 3, SSP_NO_IRQ) == -ENODEV) {
		printk("Could not allocate device\n");
		goto exit_free;
	}

	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(6));

	ssp_enable(&priv->spi_dev);



	/* Get chip select for adxl */
	r = platform_get_resource(pdev, IORESOURCE_IO, 0);
	priv->cs_adxl = r->start;

	if(gpio_request(priv->cs_adxl, "adxl346 chipselect") < 0) {
		printk("Could not request chipselect pin for adxl34x\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_adxl, 1);


	/* Get chip select for adxl */
	r = platform_get_resource(pdev, IORESOURCE_IO, 1);
	priv->cs_adis16135 = r->start;

	if(gpio_request(priv->cs_adis16135, "adis16135 chipselect") < 0) {
		printk("Could not request chipselect pin for adis16135\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_adis16135, 1);


	/* Get chip select for ad77xx */
	r = platform_get_resource(pdev, IORESOURCE_IO, 2);
	priv->cs_ad7799 = r->start;

	if(gpio_request(priv->cs_ad7799, "ad77xx chipselect") < 0) {
		printk("Could not request chipselect pin for ad77xx\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_ad7799, 1);


	/* Get chip select for mcp3008 */
	r = platform_get_resource(pdev, IORESOURCE_IO, 3);
	priv->cs_mcp3008 = r->start;

	if(gpio_request(priv->cs_mcp3008, "mcp3008 chipselect") < 0) {
		printk("Could not request chipselect pin for mcp3008\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_mcp3008, 1);


	/* Get chip select for mcp3001 */
	r = platform_get_resource(pdev, IORESOURCE_IO, 4);
	priv->cs_mcp3001_0 = r->start;

	if(gpio_request(priv->cs_mcp3001_0, "mcp3001_0 chipselect") < 0) {
		printk("Could not request chipselect pin for mcp3001_0\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_mcp3001_0, 1);


	/* Get chip select for mcp3001 */
	r = platform_get_resource(pdev, IORESOURCE_IO, 5);
	priv->cs_mcp3001_1 = r->start;

	if(gpio_request(priv->cs_mcp3001_1, "mcp3001_1 chipselect") < 0) {
		printk("Could not request chipselect pin for mcp3001_1\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_mcp3001_1, 1);


	/* Get chip select for pic */
	r = platform_get_resource(pdev, IORESOURCE_IO, 6);
	priv->cs_pic = r->start;

	if(gpio_request(priv->cs_pic, "pic chipselect") < 0) {
		printk("Could not request chipselect pin for pic\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_pic, 1);

	/* Initialize all sensors */
	adxl_init(priv);
	ad77xx_init(priv);


	alloc_chrdev_region(&priv->dev, 0, 1, "nacelle_sensors");
	cdev_init(&priv->cdev, &nacelle_sensors_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &nacelle_sensors_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(nacelle_sensors_class, NULL, priv->dev, NULL, "nacelle_sensors");

	dev_info(&pdev->dev, "nacelle_sensors device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}




static int __devexit nacelle_sensors_remove(struct platform_device *pdev)
{
	struct nacelle_sensors_priv *priv = dev_get_drvdata(&pdev->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	class_destroy(nacelle_sensors_class);

	return 0;
}

static struct platform_driver nacelle_sensors_driver = {
	.probe	= nacelle_sensors_probe,
	.remove	= nacelle_sensors_remove,
	.driver = {
		.name	= "nacelle_sensors",
		.owner	= THIS_MODULE,
	},
};

static __init int nacelle_sensors_init_module(void)
{
	nacelle_sensors_class = class_create(THIS_MODULE, "nacelle_sensors");

	if(nacelle_sensors_class == NULL) {
		printk("Could not create class counter\n");
		return -ENOMEM;
	}

	return platform_driver_register(&nacelle_sensors_driver);
}

static __exit void nacelle_sensors_cleanup_module(void)
{
	platform_driver_unregister(&nacelle_sensors_driver);
}

module_init(nacelle_sensors_init_module);
module_exit(nacelle_sensors_cleanup_module);

MODULE_ALIAS("nacelle_sensors");
