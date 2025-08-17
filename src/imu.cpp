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
    constexpr float pi_f{ZSL_PI};
    constexpr float degToRad_f{ZSL_DEG_TO_RAD};
    constexpr float radToDeg_f{ZSL_RAD_TO_DEG};

    constexpr int64_t imuTaskDurationMs{static_cast<int64_t>(1000.0 / 59.5)};
    constexpr float imuTaskDuration_f{static_cast<float>(imuTaskDurationMs) / 1000.0F};

    constexpr float varianceAccelerometerNoise{0.09F * static_cast<float>(ZSL_GRAV_EARTH)}; // Zero-g level offset 90 mg
    constexpr float varianceGyrometerNoise{30.0F * static_cast<float>(ZSL_RAD_TO_DEG)};     // Zero-rate output 30 dps

    constexpr float absoluteSquareAccelerationLowerThreshold{0.8F * static_cast<float>(ZSL_GRAV_EARTH) * 0.8F * static_cast<float>(ZSL_GRAV_EARTH)};
    constexpr float absoluteSquareAccelerationUpperThreshold{1.2F * static_cast<float>(ZSL_GRAV_EARTH) * 1.2F * static_cast<float>(ZSL_GRAV_EARTH)};

    float wrapToPi(float angle)
    {
        angle = std::fmod(angle + pi_f, 2.0F * pi_f);
        if (angle < 0)
        {
            angle += 2.0F * pi_f;
        }
        return angle - pi_f;
    }

    void thread(void *, void *, void *)
    {
        const struct device *const imu = DEVICE_DT_GET_ONE(st_lsm9ds1);
        if (!device_is_ready(imu))
        {
            LOG_ERR("%s: device not ready.", imu->name);
            return;
        }

        const struct device *const mag = DEVICE_DT_GET_ONE(st_lis3mdl_magn);
        if (!device_is_ready(mag))
        {
            LOG_ERR("%s: device not ready.", mag->name);
            return;
        }

        // Initial guess for system state
        float rollAngleEstimate{0.0F};
        float pitchAngleEstimate{0.0F};
        float yawAngleEstimate{0.0F};

        float rollAnglePredicted{0.0F};
        float pitchAnglePredicted{0.0F};
        float yawAnglePredicted{0.0F};

        float varianceOrientationEstimate{1.0F};

        float kalmanGain{0.0F};

        unsigned int count{0U};
        while (true)
        {
            int64_t startTime = k_uptime_get();
            struct sensor_value axRaw{}, ayRaw{}, azRaw{};
            struct sensor_value rollRateRaw{}, pitchRateRaw{}, yawRateRaw{};
            struct sensor_value magXRaw{}, magYRaw{}, magZRaw{};

            // 1. Measure
            int64_t accelGyroFetchStart = k_uptime_get();
            int success{sensor_sample_fetch_chan(imu, SENSOR_CHAN_ALL)};
            if (success != 0)
            {
                LOG_ERR("Error fetching accelerometer and gyroscope data.");
                continue; // Skip this iteration if the fetch fails
            }

            int64_t accelGyroFetchEnd = k_uptime_get();

            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &axRaw);
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ayRaw);
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &azRaw);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &rollRateRaw);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &pitchRateRaw);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &yawRateRaw);

            int64_t magFetchStart = k_uptime_get();
            success = sensor_sample_fetch_chan(mag, SENSOR_CHAN_MAGN_XYZ);
            if (success != 0)
            {
                LOG_ERR("Error fetching magnetometer data.");
                continue; // Skip this iteration if the fetch fails
            }

            int64_t magFetchEnd = k_uptime_get();

            sensor_channel_get(mag, SENSOR_CHAN_MAGN_X, &magXRaw);
            sensor_channel_get(mag, SENSOR_CHAN_MAGN_Y, &magYRaw);
            sensor_channel_get(mag, SENSOR_CHAN_MAGN_Z, &magZRaw);

            zsl_real_t accelerationMeasured[3] = {
                sensor_value_to_float(&axRaw),
                sensor_value_to_float(&ayRaw),
                sensor_value_to_float(&azRaw)};

            zsl_real_t magneticFieldMeasured[3] = {
                sensor_value_to_float(&magXRaw),
                sensor_value_to_float(&magYRaw),
                sensor_value_to_float(&magZRaw)};

            float rollRateMeasured{sensor_value_to_float(&rollRateRaw)};
            float pitchRateMeasured{sensor_value_to_float(&pitchRateRaw)};
            float yawRateMeasured{sensor_value_to_float(&yawRateRaw)};

            float absoluteAccelerationSquared{
                accelerationMeasured[0] * accelerationMeasured[0] +
                accelerationMeasured[1] * accelerationMeasured[1] +
                accelerationMeasured[2] * accelerationMeasured[2]};

            // Cannot use data from accelerometer with high disturbances away from earth acceleration
            int64_t attitudeStart{0}, attitudeEnd{0};
            if ((absoluteAccelerationSquared >= absoluteSquareAccelerationLowerThreshold) && (absoluteAccelerationSquared <= absoluteSquareAccelerationUpperThreshold))
            {
                struct zsl_vec accelVec{};
                accelVec.sz = 3;
                accelVec.data = accelerationMeasured;

                struct zsl_vec magVec{};
                magVec.sz = 3;
                magVec.data = magneticFieldMeasured;

                // Yaw angle can only be measured based on magnetometer
                zsl_attitude attitudeAccelMag{};
                attitudeStart = k_uptime_get();
                if (zsl_att_from_accelmag(&accelVec, &magVec, &attitudeAccelMag) != 0)
                {
                    LOG_ERR("Error calculating roll and pitch from accelerometer data.");
                    continue; // Skip this iteration if the calculation fails
                }
                attitudeEnd = k_uptime_get();

                // 2.1 Calculate gain
                kalmanGain = varianceOrientationEstimate / (varianceOrientationEstimate + varianceAccelerometerNoise);

                // 2.2 Covariance update
                varianceOrientationEstimate = (1.0F - kalmanGain) * varianceOrientationEstimate;

                // 2.3 State update
                rollAngleEstimate += kalmanGain * (degToRad_f * attitudeAccelMag.roll - rollAnglePredicted);
                pitchAngleEstimate += kalmanGain * (degToRad_f * attitudeAccelMag.pitch - pitchAnglePredicted);
                yawAngleEstimate += kalmanGain * (degToRad_f * attitudeAccelMag.yaw - yawAnglePredicted);

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
                LOG_INF("orientation fused x:%f° y:%f° z:%f°", static_cast<double>(radToDeg_f * rollAngleEstimate), static_cast<double>(radToDeg_f * pitchAngleEstimate), static_cast<double>(radToDeg_f * yawAngleEstimate));
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
                LOG_INF("Accelerometer/Gyroscope fetch time: %lld ms", accelGyroFetchEnd - accelGyroFetchStart);
                LOG_INF("Magnetometer fetch time: %lld ms", magFetchEnd - magFetchStart);
                LOG_INF("Attitude calculation time: %lld ms", attitudeEnd - attitudeStart);
            }
        }
    }
}
