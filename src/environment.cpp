#include "environment.hpp"

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <cmath>

namespace Env
{
    constexpr int64_t environmentTaskDurationMs{1000};

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

    void thread(void *, void *, void *)
    {
        const double pressureReference{0.0};

        const struct device *const pressureSensor = DEVICE_DT_GET_ONE(st_lps22hb_press);
        const struct device *const humiditySensor = DEVICE_DT_GET_ONE(st_hts221);

        if (!device_is_ready(pressureSensor))
        {
            printk("%s: device not ready.\n", pressureSensor->name);
            return;
        }

        if (!device_is_ready(humiditySensor))
        {
            printk("%s: device not ready.\n", humiditySensor->name);
            return;
        }

        while (true)
        {
            int64_t startTime = k_uptime_get();

            struct sensor_value pressure{}, temperature{}, humidity{};
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

                constexpr double standardLapseRate{-0.0065};
                const double height = ((std::pow(pressureReference / pressureDouble, 5.257) - 1.0) * celsius2kelvin(temperatureDouble)) / -standardLapseRate;

                printk("Relative height:%.1f m\n", height);
            }

            int64_t elapsedTime = k_uptime_get() - startTime;
            int64_t sleepTime = environmentTaskDurationMs - elapsedTime; // Calculate remaining time

            if (sleepTime > 0)
            {
                k_msleep(sleepTime); // Sleep until next cycle
            }
            else
            {
                printk("Deadline missed. Elapsed time %lld ms\n", elapsedTime);
            }
        }
    }
}