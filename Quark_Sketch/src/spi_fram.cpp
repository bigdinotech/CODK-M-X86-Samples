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

#include "arduino/arduino.h"
#include "sharedmemory_com.h"
#include "soc_ctrl.h"
#include "cdcacm_serial.h"
#include "curie_shared_mem.h"

#include "arduino/libraries/SPI_LIB.h"

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#define SOFTRESET_INTERRUPT_PIN		0

typedef enum opcodes_e
{
  OPCODE_WREN   = 0b0110,     /* Write Enable Latch */
  OPCODE_WRDI   = 0b0100,     /* Reset Write Enable Latch */
  OPCODE_RDSR   = 0b0101,     /* Read Status Register */
  OPCODE_WRSR   = 0b0001,     /* Write Status Register */
  OPCODE_READ   = 0b0011,     /* Read Memory */
  OPCODE_WRITE  = 0b0010,     /* Write Memory */
  OPCODE_RDID   = 0b10011111  /* Read Device ID */
} opcodes_t;

struct gpio_callback cb;

uint8_t fram_read8 (uint16_t addr);
void fram_write8 (uint16_t addr, uint8_t value);
void fram_writeEnable (bool enable);
void fram_getDeviceID();

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
	SPI1.begin();
	SPI1.setClockDivider(SPI_CLOCK_DIV2);
	pinMode(10, OUTPUT);
	//loop
	while(1)
	{
		fram_getDeviceID();
		uint16_t addr = 0x3CC3;
		
		PRINT("write: 0xAA\r\n");
		fram_writeEnable(true);
		fram_write8(addr, 0xAA);
		fram_writeEnable(false);		
		
		int data = fram_read8(addr);
		PRINT("read: 0x%X\r\n", data);

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


void fram_getDeviceID()
{
  digitalWrite(10, LOW);
  SPI1.transfer(OPCODE_RDID);

  uint32_t data = SPI1.transfer32(0);
  digitalWrite(10, HIGH);
  int manufacturerID = (data >> 24);
  int productID = (data & 0x0000FF00) + (data & 0xFF);

  PRINT("FRAM manufacturerID: 0x%x\r\n", manufacturerID);
  PRINT("FRAM productID: 0x%x\r\n", productID);
}

uint8_t fram_read8 (uint16_t addr)
{
  digitalWrite(10, LOW);
  SPI1.transfer(OPCODE_READ);
  SPI1.transfer16(addr);
  uint8_t x = SPI1.transfer(0);
  digitalWrite(10, HIGH);
  return x;
}

void fram_write8 (uint16_t addr, uint8_t value)
{
  digitalWrite(10, LOW);
  SPI1.transfer(OPCODE_WRITE);
  SPI1.transfer16(addr);
  SPI1.transfer(value);
  /* CS on the rising edge commits the WRITE */
  digitalWrite(10, HIGH);
}

void fram_writeEnable (bool enable)
{
  digitalWrite(10, LOW);
  if (enable)
  {
    SPI1.transfer(OPCODE_WREN);
  }
  else
  {
    SPI1.transfer(OPCODE_WRDI);
  }
  digitalWrite(10, HIGH);
}

