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

#include <zephyr.h>

#include <stdint.h>
#include <device.h>

#include <i2c.h>

#define BUFFER_LENGTH	32

class TwoWire {

public:
	TwoWire(int devID);
	void begin();
	void setClock(int speed);
	void beginTransmission(uint8_t addr);
	void beginTransmission(int addr);
	uint8_t endTransmission(uint8_t sendStop);
	uint8_t endTransmission(void);

	uint8_t requestFrom(uint8_t addr, uint8_t quantity, uint8_t sendStop);
	uint8_t requestFrom(uint8_t addr, uint8_t quantity);
	uint8_t requestFrom(int addr, int quantity, int sendStop);
	uint8_t requestFrom(int addr, int quantity);
	
	virtual size_t write(uint8_t data);
	virtual size_t write(const uint8_t *data, size_t length);
	virtual int available(void);
	virtual int read(void);
	virtual int peek(void);
	virtual void flush(void);

private:
	//Buffers
	uint8_t rxBuff[BUFFER_LENGTH];
	uint8_t rxBuffIndex;
	uint8_t rxBuffLength;
	uint8_t txBuff[BUFFER_LENGTH];
	uint8_t txBuffLength;

	uint8_t txAddress;
	
	uint8_t dev_id;
	struct device *i2c_dev;
	union dev_config cfg;
	int init_status;

	void i2c_setslave(uint8_t addr);
	int i2c_writebytes(uint8_t *bytes, uint8_t length, bool no_stop);
	int i2c_readbytes(uint8_t *buf, int length, bool no_stop);
};


extern TwoWire I2C0;
extern TwoWire I2C1;
