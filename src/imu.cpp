#include "imu.hpp"

#include "universalConstants.hpp"

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <zsl/zsl.h>
#include <zsl/orientation/orientation.h>

#include <cmath>

LOG_MODULE_REGISTER(imu);

namespace Imu
{
    constexpr int64_t imuTaskDurationMs{1000 / 119};
    constexpr float imuTaskDuration_f{static_cast<float>(imuTaskDurationMs) / 1000.0F};

    float wrapToPi(float angle)
    {
        angle = std::fmod(angle + ZSL_PI, 2.0F * ZSL_PI);
        if (angle < 0)
        {
            angle += 2.0F * ZSL_PI;
        }
        return angle - ZSL_PI;
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

        constexpr float varianceAccelerometerNoise{0.09F * ZSL_GRAV_EARTH}; // Zero-g level offset 90 mg
        constexpr float varianceGyrometerNoise{30.0F * ZSL_DEG_TO_RAD};     // Zero-rate output 30 dps
        float varianceOrientationEstimate{1.0F};

        float kalmanGain{0.0F};

        unsigned int count{0U};
        while (true)
        {
            int64_t startTime = k_uptime_get();
            struct sensor_value axRaw{}, ayRaw{}, azRaw{};
            struct sensor_value rollRateRaw{}, pitchRateRaw{}, yawRateRaw{};

            // 1. Measure
            int success{sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ)};
            if (success != 0)
            {
                LOG_ERR("Error fetching accelerometer data.");
                continue; // Skip this iteration if the fetch fails
            }

            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &axRaw);
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ayRaw);
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &azRaw);

            success = sensor_sample_fetch_chan(imu, SENSOR_CHAN_GYRO_XYZ);
            if (success != 0)
            {
                LOG_ERR("Error fetching gyroscope data.");
                continue; // Skip this iteration if the fetch fails
            }

            sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &rollRateRaw);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &pitchRateRaw);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &yawRateRaw);

            zsl_real_t accelerationMeasured[3] = {
                sensor_value_to_float(&axRaw),
                sensor_value_to_float(&ayRaw),
                sensor_value_to_float(&azRaw)};
            float rollRateMeasured{sensor_value_to_float(&rollRateRaw)};
            float pitchRateMeasured{sensor_value_to_float(&pitchRateRaw)};
            float yawRateMeasured{sensor_value_to_float(&yawRateRaw)};

            float absoluteAcceleration{std::sqrt(
                accelerationMeasured[0] * accelerationMeasured[0] +
                accelerationMeasured[1] * accelerationMeasured[1] +
                accelerationMeasured[2] * accelerationMeasured[2])};

            // Cannot use data from accelerometer with high disturbances away from earth acceleration
            if ((absoluteAcceleration >= 0.8F * ZSL_GRAV_EARTH) && (absoluteAcceleration <= 1.2F * ZSL_GRAV_EARTH))
            {
                // TODO Estimate gyro bias covariance also in Kalman filter, assuming constant prediction model
                float axNormalized{accelerationMeasured[0] / absoluteAcceleration};
                float ayNormalized{accelerationMeasured[1] / absoluteAcceleration};
                float azNormalized{accelerationMeasured[2] / absoluteAcceleration};

                // Cross product between gravity vector [0 0 -1] and normalized acceleration vector
                float rotationAxisX{-ayNormalized};
                float rotationAxisY{axNormalized};
                float rotationAxisZ{0.0F}; // single rotation cannot happen around gravity vector to align with measured acceleration
                float rotationAngle{std::acos(-azNormalized)};

                // Convert to quaternion
                float qw{std::cos(0.5F * rotationAngle)};
                float sinThetaHalf{std::sin(0.5F * rotationAngle)};
                float qx{rotationAxisX * sinThetaHalf};
                float qy{rotationAxisY * sinThetaHalf};
                float qz{rotationAxisZ * sinThetaHalf};

                struct zsl_vec accel_vec{};
                accel_vec.sz = 3;
                accel_vec.data = accelerationMeasured;

                // Yaw angle can only be measured based on magnetometer
                zsl_attitude attitudeAccelerometer{};
                const int ret{zsl_att_from_accel(&accel_vec, &attitudeAccelerometer)};
                if (ret != 0)
                {
                    LOG_ERR("Error calculating roll and pitch from accelerometer data.");
                    continue; // Skip this iteration if the calculation fails
                }

                // 2.1 Calculate gain
                kalmanGain = varianceOrientationEstimate / (varianceOrientationEstimate + varianceAccelerometerNoise);

                // 2.2 Covariance update
                varianceOrientationEstimate = (1.0F - kalmanGain) * varianceOrientationEstimate;

                // 2.3 State update
                rollAngleEstimate += kalmanGain * (ZSL_DEG_TO_RAD * attitudeAccelerometer.roll - rollAnglePredicted);
                pitchAngleEstimate += kalmanGain * (ZSL_DEG_TO_RAD * attitudeAccelerometer.pitch - pitchAnglePredicted);
                yawAngleEstimate += kalmanGain * (ZSL_DEG_TO_RAD * attitudeAccelerometer.yaw - yawAnglePredicted);

                // Clamp to [-pi, +pi]
                rollAngleEstimate = wrapToPi(rollAngleEstimate);
                pitchAngleEstimate = wrapToPi(pitchAngleEstimate);
                yawAngleEstimate = wrapToPi(yawAngleEstimate);
            }
            else
            {
                LOG_WRN("High disturbance detected. Skipping accelerometer data.");
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
                LOG_INF("orientation fused x:%f° y:%f° z:%f°", ZSL_RAD_TO_DEG * rollAngleEstimate, ZSL_RAD_TO_DEG * pitchAngleEstimate, ZSL_RAD_TO_DEG * yawAngleEstimate);
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
