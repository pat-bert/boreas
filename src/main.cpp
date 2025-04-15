/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <cmath>

/* 1000 msec = 1 sec */
#define sleepTime_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED3_NODE DT_ALIAS(led3)

constexpr double standardLapseRate{-0.0065};

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

#define IMU_STACK_SIZE 500
#define IMU_PRIORITY 5

constexpr int64_t imuTaskDurationMs{1000 / 119};

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

void imu_processing(void *, void *, void *)
{
	const struct device *const imu = DEVICE_DT_GET_ONE(st_lsm9ds1);
	if (!device_is_ready(imu))
	{
		printk("%s: device not ready.\n", imu->name);
		return;
	}

	unsigned int count{0U};
	while (true)
	{
		int64_t startTime = k_uptime_get();
		struct sensor_value x{}, y{}, z{};

		/* lsm6dso accel */
		sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ);
		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &x);
		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &y);
		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &z);

		/* lsm6dso gyro */
		sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &x);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &y);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &z);

		++count;

		if (count % 100 == 0)
		{
			printk("accel x:%f ms/2 y:%f ms/2 z:%f ms/2\n", (double)out_ev(&x), (double)out_ev(&y), (double)out_ev(&z));
			printk("gyro x:%f rad/s y:%f rad/s z:%f rad/s\n", (double)out_ev(&x), (double)out_ev(&y), (double)out_ev(&z));
		}

		int64_t elapsedTime = k_uptime_get() - startTime;
		int64_t sleepTime = imuTaskDurationMs - elapsedTime; // Calculate remaining time

		if (sleepTime > 0)
		{
			k_msleep(sleepTime); // Sleep until next cycle
		}
		else{
			printk("Deadline missed. Elapsed time %lld ms\n", elapsedTime);
		}
	}
}

K_THREAD_DEFINE(imu_tid, IMU_STACK_SIZE,
				imu_processing, NULL, NULL, NULL,
				IMU_PRIORITY, 0, 0);

static bool process_sample_pressure(const struct device *dev, struct sensor_value *pressure)
{
	if (sensor_sample_fetch(dev) < 0)
	{
		printk("LPS22HB sample update error\n");
		return false;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, pressure) < 0)
	{
		printk("Cannot read LPS22HB pressure channel\n");
		return false;
	}

	return true;
}

static bool process_sample_humidity(const struct device *dev, struct sensor_value *temperature, struct sensor_value *humidity)
{
	if (sensor_sample_fetch(dev) < 0)
	{
		printk("HTS221 sample update error\n");
		return false;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, temperature) < 0)
	{
		printk("Cannot read HTS221 temperature channel\n");
		return false;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, humidity) < 0)
	{
		printk("Cannot read HTS221 humidity channel\n");
		return false;
	}

	return true;
}

constexpr double celsius2kelvin(double celsius)
{
	return celsius + 273.15;
}

int main(void)
{
	int ret;
	bool ledState = true;

	const double pressureReference{0.0};

	const struct device *const pressureSensor = DEVICE_DT_GET_ONE(st_lps22hb_press);
	const struct device *const humiditySensor = DEVICE_DT_GET_ONE(st_hts221);

	if (!device_is_ready(pressureSensor))
	{
		printk("%s: device not ready.\n", pressureSensor->name);
		return 0;
	}

	if (!device_is_ready(humiditySensor))
	{
		printk("%s: device not ready.\n", humiditySensor->name);
		return 0;
	}

	if (!gpio_is_ready_dt(&led))
	{
		printk("LED GPIO not ready.\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 0;
	}

	while (1)
	{
		struct sensor_value pressure{}, temperature{}, humidity{};

		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0)
		{
			return 0;
		}

		ledState = !ledState;

		const bool successPressure{process_sample_pressure(pressureSensor, &pressure)};
		if (successPressure)
		{
			printk("Pressure:%.1f kPa\n", sensor_value_to_double(&pressure));
		}

		const bool successTempAndHum{process_sample_humidity(humiditySensor, &temperature, &humidity)};
		if (successTempAndHum)
		{
			// Temperature seems to be inaccurate when powered via USB due to self-heating
			printk("Temperature:%.1f C\n", sensor_value_to_double(&temperature));
			printk("Relative Humidity:%.1f%%\n", sensor_value_to_double(&humidity));
		}

		if (successPressure && successTempAndHum)
		{
			const double pressureDouble{sensor_value_to_double(&pressure)};
			const double temperatureDouble{sensor_value_to_double(&temperature)};

			const double height = ((std::pow(pressureReference / pressureDouble, 5.257) - 1.0) * celsius2kelvin(temperatureDouble)) / -standardLapseRate;

			printk("Relative height:%.1f m\n", height);
		}

		k_msleep(sleepTime_MS);
	}

	return 0;
}
