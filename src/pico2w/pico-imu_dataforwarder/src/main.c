/*
 * Written by Alex Bucknall.
 * Modified to include MPU-6050 sensor reading.
 *
 * Copyright (c) 2022 Alex Bucknall. MIT License. Use of this source code is
 * governed by licenses granted by the copyright holder including that found in
 * the LICENSE file.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define STANDARD_GRAVITY 9.80665

/* Check overlay exists for CDC UART console */ 
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	    "Console device is not ACM CDC UART device");

/* Get the MPU-6050 device from the device tree */
static const struct device *const mpu6050 = DEVICE_DT_GET_ANY(invensense_mpu6050);

static void setup_mpu6050(void)
{
	if (!device_is_ready(mpu6050)) {
		printk("MPU-6050: Device is not ready.\n");
		return;
	}
	printk("MPU-6050: Device is ready.\n");
}

void main(void)
{
	/* Configure to set Console output to USB Serial */ 
	const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

    /* Check if USB can be initialised, bails out if fail is returned */
	if (usb_enable(NULL) != 0) {
		return;
	}

	/* Wait for a console connection */
	while (!dtr) {
		uart_line_ctrl_get(usb_device, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

    /* Set up the MPU-6050 sensor */
    setup_mpu6050();

	while (1) {
		struct sensor_value accel[3];
		int ret;

		/* Fetch the latest sample from the sensor */
		ret = sensor_sample_fetch(mpu6050);
		if (ret < 0) {
			printk("Failed to fetch sample from MPU-6050 (%d)\n", ret);
		} else {
			/* Get accelerometer data (X, Y, Z) */
			sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_XYZ, accel);

			printk("% 8.4f\t% 8.4f\t% 8.4f\r\n",
				   sensor_value_to_double(&accel[0])/ STANDARD_GRAVITY,
				   sensor_value_to_double(&accel[1])/ STANDARD_GRAVITY,
				   sensor_value_to_double(&accel[2])/ STANDARD_GRAVITY);
		}
		
		k_usleep(9600); // 104Hz
	}
}