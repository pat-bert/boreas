#include "imu.hpp"

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu);

namespace Imu
{
    constexpr int64_t imuTaskDurationMs{1000 / 119};

    static inline float out_ev(struct sensor_value *val)
    {
        return (val->val1 + (float)val->val2 / 1000000);
    }

    void thread(void *, void *, void *)
    {
        const struct device *const imu = DEVICE_DT_GET_ONE(st_lsm9ds1);
        if (!device_is_ready(imu))
        {
            LOG_ERR("%s: device not ready.", imu->name);
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
                LOG_INF("accel x:%f ms/2 y:%f ms/2 z:%f ms/2", (double)out_ev(&x), (double)out_ev(&y), (double)out_ev(&z));
                LOG_INF("gyro x:%f rad/s y:%f rad/s z:%f rad/s", (double)out_ev(&x), (double)out_ev(&y), (double)out_ev(&z));
            }

            int64_t elapsedTime = k_uptime_get() - startTime;
            int64_t sleepTime = imuTaskDurationMs - elapsedTime; // Calculate remaining time

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