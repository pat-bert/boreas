#include "environment.hpp"

#include "universalConstants.hpp"

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

    constexpr float celsius2kelvin(float celsius)
    {
        return celsius + 273.15F;
    }

    float calculateAltitude(float pressureReference, float pressure, float temperature)
    {
        constexpr float barometricExponent{universalGasConstant * standardLapseRate / (earthAcceleration * molarMassAir)};
        return ((std::pow(pressureReference / pressure, barometricExponent) - 1.0F) * celsius2kelvin(temperature)) / standardLapseRate;
    }

    void thread(void *, void *, void *)
    {
        float pressureReference{0.0};

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

        int i{0};
        while (i < 10)
        {
            struct sensor_value pressureRaw{};
            const bool successPressure{process_sample_pressure(pressureSensor, &pressureRaw)};
            k_msleep(100);

            if (!successPressure)
            {
                continue;
            }

            const float currentPressure{sensor_value_to_float(&pressureRaw)};
            pressureReference += (currentPressure - pressureReference) / (static_cast<float>(i) + 1.0F);
            ++i;
        }

        LOG_INF("Reference pressure: %.5f kPa", pressureReference);

        while (true)
        {
            int64_t startTime = k_uptime_get();

            struct sensor_value pressureRaw{}, temperatureRaw{}, humidityRaw{};
            const bool successPressure{process_sample_pressure(pressureSensor, &pressureRaw)};
            if (successPressure)
            {
                LOG_INF("Pressure:%.5f kPa", sensor_value_to_double(&pressureRaw));
            }

            const bool successTempAndHum{process_sample_humidity(humiditySensor, &temperatureRaw, &humidityRaw)};
            if (successTempAndHum)
            {
                // Temperature seems to be inaccurate when powered via USB due to self-heating
                LOG_INF("Temperature:%.1f C", sensor_value_to_double(&temperatureRaw));
                LOG_INF("Relative Humidity:%.1f%%", sensor_value_to_double(&humidityRaw));
            }

            if (successPressure && successTempAndHum)
            {
                const float pressure{sensor_value_to_float(&pressureRaw)};
                const float temperature{sensor_value_to_float(&temperatureRaw)};
                const float altitude{calculateAltitude(pressureReference, pressure, temperature)};
                LOG_INF("Relative altitude:%.2f m", altitude);
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