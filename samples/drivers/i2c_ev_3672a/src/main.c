/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>

/**
 * @file Sample app using the Fujitsu MB85RC256V FRAM through I2C.
 */

#define INITIALIZATION_REGISTER_1	0x0F

static int read_bytes(const struct device *i2c_dev, uint16_t addr,
		      uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, INITIALIZATION_REGISTER_1);
}

void main(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	uint8_t cmp_data[16];
	uint8_t data[16];
	int i, ret;

	if (!device_is_ready(i2c_dev)) {
		printk("I2C: Device is not ready.\n");
		return;
	}

	data[0] = 0x00;
	ret = read_bytes(i2c_dev, 0x00, &data[0], 1);
	if (ret) {
		printk("Error reading from EV 3672A! error code (%d)\n", ret);
		return;
	} else {
		printk("Read 0x%X from address 0x00.\n", data[0]);
	}

}
