/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED3_NODE DT_ALIAS(led3)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

static void process_sample_imu(const struct device *dev)
{
	struct sensor_value x, y, z;

	/* lsm6dso accel */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &z);

	printk("%s\t - accel x:%f ms/2 y:%f ms/2 z:%f ms/2\n", dev->name, (double)out_ev(&x), (double)out_ev(&y), (double)out_ev(&z));

	/* lsm6dso gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &z);

	printk("%s\t - gyro x:%f rad/s y:%f rad/s z:%f rad/s\n", dev->name, (double)out_ev(&x), (double)out_ev(&y), (double)out_ev(&z));
}

static int set_sampling_freq_imu(const struct device *dev)
{
	int ret = 0;
	struct sensor_value odr_attr;

	/* set accel/gyro sampling frequency to 12.5 Hz */
	odr_attr.val1 = 12.5;
	odr_attr.val2 = 0;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
						  SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret != 0)
	{
		printk("Cannot set sampling frequency for accelerometer.\n");
		return ret;
	}

	ret = sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
						  SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret != 0)
	{
		printk("Cannot set sampling frequency for gyro.\n");
		return ret;
	}

	return 0;
}

static void process_sample_pressure(const struct device *dev)
{
	struct sensor_value pressure, temp;

	if (sensor_sample_fetch(dev) < 0)
	{
		printk("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0)
	{
		printk("Cannot read LPS22HB pressure channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0)
	{
		printk("Cannot read LPS22HB temperature channel\n");
		return;
	}

	/* display pressure */
	printk("%s\t - Pressure:%.1f kPa\n", dev->name, sensor_value_to_double(&pressure));

	/* display temperature */
	printk("%s\t - Temperature:%.1f C\n", dev->name, sensor_value_to_double(&temp));
}

static void process_sample_humidity(const struct device *dev)
{
	struct sensor_value temp, hum;
	if (sensor_sample_fetch(dev) < 0)
	{
		printk("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0)
	{
		printk("Cannot read HTS221 temperature channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum) < 0)
	{
		printk("Cannot read HTS221 humidity channel\n");
		return;
	}

	/* display temperature */
	printk("%s\t - Temperature:%.1f C\n", dev->name, sensor_value_to_double(&temp));

	/* display humidity */
	printk("%s\t - Relative Humidity:%.1f%%\n", dev->name, sensor_value_to_double(&hum));
}

int main(void)
{
	int ret;
	bool ledState = true;

	const struct device *const imu = DEVICE_DT_GET_ONE(st_lsm9ds1);
	const struct device *const pressureSensor = DEVICE_DT_GET_ONE(st_lps22hb_press);
	const struct device *const humiditySensor = DEVICE_DT_GET_ONE(st_hts221);

	if (!device_is_ready(imu))
	{
		printk("%s: device not ready.\n", imu->name);
		return 0;
	}

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

	if (set_sampling_freq_imu(imu) != 0)
	{
		return 0;
	}

	while (1)
	{
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0)
		{
			return 0;
		}

		ledState = !ledState;
		printk("LED state: %s\n", ledState ? "ON" : "OFF");
		process_sample_imu(imu);
		process_sample_pressure(pressureSensor);
		process_sample_humidity(humiditySensor);
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
