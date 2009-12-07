/*
 * ssd1322.h
 *
 *  Created on: Dec 4, 2009
 *      Author: dakila
 */

#ifndef SSD1322_H_
#define SSD1322_H_


struct spi_device;

struct ssd1322_spi_platform_data {
	int reg_iopin;
	int reset_iopin;
};


#endif /* SSD1322_H_ */
