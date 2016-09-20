/*
 * Copyright (c) 2016 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <stdio.h>
#include <device.h>
#include <zephyr.h>
#include <gpio.h>
#include "soc.h"
#include <i2c.h>

#include "arduino/arduino.h"
#include "sharedmemory_com.h"
#include "soc_ctrl.h"
#include "cdcacm_serial.h"
#include "curie_shared_mem.h"

#include "arduino/libraries/Wire.h"

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#define SOFTRESET_INTERRUPT_PIN		0

#define MB85RC_DEFAULT_ADDRESS        (0x50) /* 1010 + A2 + A1 + A0 = 0x50 default */
#define MB85RC_SLAVE_ID       (0xF8)
#define FRAM_I2C_ADDR	0x50

struct gpio_callback cb;

void softResetButton();
int fram_write_bytes(struct device *i2c_dev, uint16_t addr,
		uint8_t *data, uint32_t num_bytes);

int fram_read_bytes(struct device *i2c_dev, uint16_t addr,
	       uint8_t *data, uint32_t num_bytes);

static void softReset_button_callback(struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	soft_reboot();
}

void main(void)
{
	init_cdc_acm();
	softResetButton();
	init_sharedMemory_com();
	variantInit();

	// start ARC core
	uint32_t *reset_vector;
	reset_vector = (uint32_t *)RESET_VECTOR;
	start_arc(*reset_vector);
	
	task_start(QUARK_SKETCH);
	task_start(CDCACM_SETUP);
	task_start(BAUDRATE_RESET);
	task_start(USB_SERIAL);
}


extern "C" void quark_sketch(void)
{
	//setup
	I2C0.begin();
	//I2C0.setClock(I2C_SPEED_FAST);
	//loop
	while(1)
	{
		PRINT("I2C_0 Test\r\n");
		
		uint8_t a[3] = { 0, 0, 0 };
		uint8_t results;
		
		//FRAM Write8
		
		I2C0.beginTransmission(0x50);
		I2C0.write(0 >> 8);
		I2C0.write(0 & 0xFF);
		I2C0.write(23);
		I2C0.endTransmission();
		
		
		//FRAM Read8
		
		PRINT("I2C_0 Write\r\n");
		I2C0.beginTransmission(0x50);
		I2C0.write(0 >> 8);
  		I2C0.write(0 & 0xFF);
		results = I2C0.endTransmission();
		
		PRINT("I2C_0 Read\r\n");
		I2C0.requestFrom(0x50, 1);
		a[0] = I2C0.read();

		
		//FRAM ID
		
		PRINT("I2C FRAM_ID\r\n");		
		I2C0.beginTransmission(MB85RC_SLAVE_ID >> 1);
		I2C0.write(0x50 << 1);
		results = I2C0.endTransmission(false);

		I2C0.requestFrom(MB85RC_SLAVE_ID >> 1, 3);
		a[0] = I2C0.read();
		a[1] = I2C0.read();
		a[2] = I2C0.read();
		
		int manufacturerID = (a[0] << 4) + (a[1]  >> 4);
		int productID = ((a[1] & 0x0F) << 8) + a[2];

		PRINT("FRAM manufacturerID: %d\r\n", manufacturerID);
		PRINT("FRAM productID: %d\r\n", productID);
		
		delay(100);
		
		task_yield();
	}

}

void softResetButton()
{
	struct device *aon_gpio;
	char* gpio_aon_0 = (char*)"GPIO_AON_0";
	aon_gpio = device_get_binding(gpio_aon_0);
	if (!aon_gpio) 
	{
		return;
	}

	gpio_init_callback(&cb, softReset_button_callback, BIT(SOFTRESET_INTERRUPT_PIN));
	gpio_add_callback(aon_gpio, &cb);

	gpio_pin_configure(aon_gpio, SOFTRESET_INTERRUPT_PIN,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE);

	gpio_pin_enable_callback(aon_gpio, SOFTRESET_INTERRUPT_PIN);
}

int fram_read_bytes(struct device *i2c_dev, uint16_t addr,
	       uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to read */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. RESTART as neededm and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, FRAM_I2C_ADDR);
}

int fram_write_bytes(struct device *i2c_dev, uint16_t addr,
		uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to write */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, FRAM_I2C_ADDR);
}
