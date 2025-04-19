/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "environment.hpp"
#include "imu.hpp"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

/* 1000 msec = 1 sec */
#define sleepTime_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED3_NODE DT_ALIAS(led3)

#define IMU_STACK_SIZE 1000
#define IMU_PRIORITY 5
#define ENVIRONMENT_STACK_SIZE 500
#define ENVIRONMENT_PRIORITY 6

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

LOG_MODULE_REGISTER(app);

K_THREAD_DEFINE(imu_tid, IMU_STACK_SIZE,
				Imu::thread, NULL, NULL, NULL,
				IMU_PRIORITY, 0, 0);

K_THREAD_DEFINE(environment_tid, ENVIRONMENT_STACK_SIZE,
				Env::thread, NULL, NULL, NULL,
				ENVIRONMENT_PRIORITY, 0, 0);

int main(void)
{
	int ret;
	bool ledState = true;

	if (!gpio_is_ready_dt(&led))
	{
		LOG_ERR("LED GPIO not ready.");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		LOG_ERR("Failed to configure LED GPIO as output.");
		return 0;
	}

	while (1)
	{
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0)
		{
			LOG_ERR("Failed to toggle LED GPIO.");
			return 0;
		}

		ledState = !ledState;

		k_msleep(sleepTime_MS);
	}

	return 0;
}
