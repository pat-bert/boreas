#include "environment.hpp"

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <cmath>

LOG_MODULE_REGISTER(environment);

namespace Env
{
    constexpr int64_t environmentTaskDurationMs{1000};

    static bool process_sample_pressure(const struct device *dev, struct sensor_value *pressure)
    {
        if (sensor_sample_fetch(dev) < 0)
        {
            LOG_ERR("LPS22HB sample update error");
            return false;
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, pressure) < 0)
        {
            LOG_ERR("Cannot read LPS22HB pressure channel");
            return false;
        }

        return true;
    }

    static bool process_sample_humidity(const struct device *dev, struct sensor_value *temperature, struct sensor_value *humidity)
    {
        if (sensor_sample_fetch(dev) < 0)
        {
            LOG_ERR("HTS221 sample update error");
            return false;
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, temperature) < 0)
        {
            LOG_ERR("Cannot read HTS221 temperature channel");
            return false;
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, humidity) < 0)
        {
            LOG_ERR("Cannot read HTS221 humidity channel");
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
            LOG_ERR("%s: device not ready.", pressureSensor->name);
            return;
        }

        if (!device_is_ready(humiditySensor))
        {
            LOG_ERR("%s: device not ready.", humiditySensor->name);
            return;
        }

        while (true)
        {
            int64_t startTime = k_uptime_get();

            struct sensor_value pressure{}, temperature{}, humidity{};
            const bool successPressure{process_sample_pressure(pressureSensor, &pressure)};
            if (successPressure)
            {
                LOG_INF("Pressure:%.1f kPa", sensor_value_to_double(&pressure));
            }

            const bool successTempAndHum{process_sample_humidity(humiditySensor, &temperature, &humidity)};
            if (successTempAndHum)
            {
                // Temperature seems to be inaccurate when powered via USB due to self-heating
                LOG_INF("Temperature:%.1f C", sensor_value_to_double(&temperature));
                LOG_INF("Relative Humidity:%.1f%%", sensor_value_to_double(&humidity));
            }

            if (successPressure && successTempAndHum)
            {
                const double pressureDouble{sensor_value_to_double(&pressure)};
                const double temperatureDouble{sensor_value_to_double(&temperature)};

                constexpr double standardLapseRate{-0.0065};
                const double height = ((std::pow(pressureReference / pressureDouble, 5.257) - 1.0) * celsius2kelvin(temperatureDouble)) / -standardLapseRate;

                LOG_INF("Relative height:%.1f m", height);
            }

            int64_t elapsedTime = k_uptime_get() - startTime;
            int64_t sleepTime = environmentTaskDurationMs - elapsedTime; // Calculate remaining time

            if (sleepTime > 0)
            {
                k_msleep(sleepTime); // Sleep until next cycle
            }
            else
            {
                LOG_ERR("Deadline missed. Elapsed time %lld ms", elapsedTime);
            }
        }
    }
}