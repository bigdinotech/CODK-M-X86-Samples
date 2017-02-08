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
#include "arduino101_services.h"


/* size of stack area used by each thread */
#define MAIN_STACKSIZE  1024
#define STACKSIZE256    256
#define STACKSIZE128    128


/* scheduling priority used by each thread */
#define PRIORITY 7
#define TASK_PRIORITY 8

char __noinit __stack sketch_stack_area[2048];
char __noinit __stack cdcacm_setup_stack_area[STACKSIZE256];
char __noinit __stack baudrate_reset_stack_area[STACKSIZE128];
char __noinit __stack usb_serial_stack_area[STACKSIZE256];

void threadMain(void *dummy1, void *dummy2, void *dummy3)
{
	init_cdc_acm();
	init_sharedMemory_com();

	// start ARC core
	uint32_t *reset_vector;
	reset_vector = (uint32_t *)RESET_VECTOR;
	start_arc(*reset_vector);
	

	k_thread_spawn(sketch_stack_area, 2048, sketch, NULL, NULL,
			NULL, TASK_PRIORITY, 0, K_NO_WAIT);

	k_thread_spawn(cdcacm_setup_stack_area, STACKSIZE256, cdcacm_setup, NULL, NULL,
			NULL, TASK_PRIORITY, 0, K_NO_WAIT);
	k_thread_spawn(baudrate_reset_stack_area, STACKSIZE128, baudrate_reset, NULL, NULL,
			NULL, TASK_PRIORITY, 0, K_NO_WAIT);
	k_thread_spawn(usb_serial_stack_area, STACKSIZE256, usb_serial, NULL, NULL,
			NULL, TASK_PRIORITY, 0, K_NO_WAIT);
}

K_THREAD_DEFINE(threadMain_id, MAIN_STACKSIZE, threadMain, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);

