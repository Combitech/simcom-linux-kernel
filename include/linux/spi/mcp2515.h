/*
 * mcp2515.h
 *
 *  Created on: Dec 9, 2009
 *      Author: dakila
 */

#ifndef MCP2515_H_
#define MCP2515_H_


struct spi_device;

struct mcp2515_spi_platform_data {
	int cs_gpio;
	int reset_gpio;
};


#endif /* MCP2515_H_ */
