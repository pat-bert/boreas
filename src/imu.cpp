#include "imu.hpp"

#include "universalConstants.hpp"

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <cmath>

LOG_MODULE_REGISTER(imu);

namespace Imu
{
    constexpr int64_t imuTaskDurationMs{1000 / 119};
    constexpr float imuTaskDuration_f{static_cast<float>(imuTaskDurationMs) / 1000.0F};

    float wrapToPi(float angle)
    {
        angle = std::fmod(angle + pi, 2.0F * pi);
        if (angle < 0)
        {
            angle += 2.0F * pi;
        }
        return angle - pi;
    }

    void thread(void *, void *, void *)
    {
        const struct device *const imu = DEVICE_DT_GET_ONE(st_lsm9ds1);
        if (!device_is_ready(imu))
        {
            LOG_ERR("%s: device not ready.", imu->name);
            return;
        }

        // Initial guess for system state
        float rollAngleEstimate{0.0F};
        float pitchAngleEstimate{0.0F};
        float yawAngleEstimate{0.0F};

        float rollAnglePredicted{0.0F};
        float pitchAnglePredicted{0.0F};
        float yawAnglePredicted{0.0F};

        constexpr float varianceAccelerometerNoise{0.09F * earthAcceleration}; // Zero-g level offset 90 mg
        constexpr float varianceGyrometerNoise{30.0F / 180.0F * pi}; // Zero-rate output 30 dps
        float varianceOrientationEstimate{1.0F};

        float kalmanGain{0.0F};

        unsigned int count{0U};
        while (true)
        {
            int64_t startTime = k_uptime_get();
            struct sensor_value axRaw{}, ayRaw{}, azRaw{};
            struct sensor_value rollRateRaw{}, pitchRateRaw{}, yawRateRaw{};

            // 1. Measure
            sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ);
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &axRaw);
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ayRaw);
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &azRaw);

            sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &rollRateRaw);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &pitchRateRaw);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &yawRateRaw);

            float axMeasured{sensor_value_to_float(&axRaw)};
            float ayMeasured{sensor_value_to_float(&ayRaw)};
            float azMeasured{sensor_value_to_float(&azRaw)};
            float rollRateMeasured{sensor_value_to_float(&rollRateRaw)};
            float pitchRateMeasured{sensor_value_to_float(&pitchRateRaw)};
            float yawRateMeasured{sensor_value_to_float(&yawRateRaw)};

            float gravity{std::sqrt(axMeasured * axMeasured + ayMeasured * ayMeasured + azMeasured * azMeasured)};

            // Cannot use data from accelerometer in zero gravity (or init state)
            if (std::abs(gravity) > 1e-7F)
            {
                // If pitch angle is +-90°, ay and az are close to zero and roll angle becomes unstable
                // This can be improved by accounting ax via an alpha
                constexpr float alpha{0.01F};
                float rollAngleMeasured{std::atan2(ayMeasured, azMeasured + axMeasured * alpha)};
                float pitchAngleMeasured{std::asin(axMeasured / gravity)};

                // Yaw angle can only be measured based on magnetometer
                float yawAngleMeasured{0.0F};

                // 2.1 Calculate gain
                kalmanGain = varianceOrientationEstimate / (varianceOrientationEstimate + varianceAccelerometerNoise);

                // 2.2 Covariance update
                varianceOrientationEstimate = (1.0F - kalmanGain) * varianceOrientationEstimate;

                // 2.3 State update
                rollAngleEstimate += kalmanGain * (rollAngleMeasured - rollAnglePredicted);
                pitchAngleEstimate += kalmanGain * (pitchAngleMeasured - pitchAnglePredicted);
                yawAngleEstimate += kalmanGain * (yawAngleMeasured - yawAnglePredicted);

                // Clamp to [-pi, +pi]
                rollAngleEstimate = wrapToPi(rollAngleEstimate);
                pitchAngleEstimate = wrapToPi(pitchAngleEstimate);
                yawAngleEstimate = wrapToPi(yawAngleEstimate);
            }

            // 3.1 State prediction
            rollAnglePredicted = rollAngleEstimate + rollRateMeasured * imuTaskDuration_f;
            pitchAnglePredicted = pitchAngleEstimate + pitchRateMeasured * imuTaskDuration_f;
            yawAnglePredicted = yawAngleEstimate + yawRateMeasured * imuTaskDuration_f;

            // 3.2 Covariance prediction
            varianceOrientationEstimate += imuTaskDuration_f * imuTaskDuration_f * varianceGyrometerNoise;

            // Clamp to [-pi, +pi]
            rollAnglePredicted = wrapToPi(rollAnglePredicted);
            pitchAnglePredicted = wrapToPi(pitchAnglePredicted);
            yawAnglePredicted = wrapToPi(yawAnglePredicted);

            ++count;
            if (count % 50 == 0)
            {
                LOG_INF("accel x:%f ms/2 y:%f ms/2 z:%f ms/2", axMeasured, ayMeasured, azMeasured);
                LOG_INF("gyro x:%f rad/s y:%f rad/s z:%f rad/s", rollRateMeasured, pitchRateMeasured, yawRateMeasured);
                LOG_INF("orientation x:%f rad y:%f rad z:%f rad", rollAngleEstimate, pitchAngleEstimate, yawAngleEstimate);
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
