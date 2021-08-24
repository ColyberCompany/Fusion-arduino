/**
 * @file FusionAhrs.h
 * @author Seb Madgwick
 * @brief The AHRS sensor fusion algorithm to combines gyroscope, accelerometer,
 * and magnetometer measurements into a single measurement of orientation
 * relative to the Earth (NWU convention).
 *
 * The algorithm behaviour is governed by a gain.  A low gain will decrease the
 * influence of the accelerometer and magnetometer so that the algorithm will
 * better reject disturbances causes by translational motion and temporary
 * magnetic distortions.  However, a low gain will also increase the risk of
 * drift due to gyroscope calibration errors.  A typical gain value suitable for
 * most applications is 0.5.
 *
 * The algorithm allows the application to define a minimum and maximum valid
 * magnetic field magnitude.  The algorithm will ignore magnetic measurements
 * that fall outside of this range.  This allows the algorithm to reject
 * magnetic measurements that do not represent the direction of magnetic North.
 * The typical magnitude of the Earth's magnetic field is between 20 uT and
 * 70 uT.
 *
 * The algorithm can be used without a magnetometer.  Measurements of
 * orientation obtained using only gyroscope and accelerometer measurements
 * can be expected to drift in the yaw component of orientation only.  The
 * application can reset the drift in yaw by setting the yaw to a specified
 * angle at any time.
 *
 * The algorithm provides the measurement of orientation as a quaternion.  The
 * library includes functions for converting this quaternion to a rotation
 * matrix and Euler angles.
 *
 * The algorithm also provides a measurement of linear acceleration and Earth
 * acceleration.  Linear acceleration is equal to the accelerometer  measurement
 * with the 1 g of gravity removed.  Earth acceleration is a measurement of
 * linear acceleration in the Earth coordinate frame.
 */

#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

#include "FusionTypes.h"


namespace Fusion
{
    /**
     * @brief AHRS algorithm class.
     */
    class FusionAhrs
    {
        float gain;
        float minimumMagneticFieldSquared;
        float maximumMagneticFieldSquared;
        Quaternion quaternion; // describes the Earth relative to the sensor
        Vector3 linearAcceleration;
        float rampedGain;
        bool zeroYawPending;

        const float SamplePeriod; //Sample period in seconds.  This is the difference in time between the current and previous gyroscope measurements.

        static const float InitialGain;
        static const float InitializationPeriod;


    public:
        /**
         * @brief Initialises the AHRS algorithm structure.
         * @param gain AHRS algorithm gain.
         */
        FusionAhrs(const float gain, const float samplePeriod);

        /**
         * @brief Sets the AHRS algorithm gain.  The gain must be equal or greater than
         * zero.
         * @param gain AHRS algorithm gain.
         */
        void setGain(const float gain);

        /**
         * @brief Sets the minimum and maximum valid magnetic field magnitudes in uT.
         * @param minimumMagneticField Minimum valid magnetic field magnitude.
         * @param maximumMagneticField Maximum valid magnetic field magnitude.
         */
        void setMagneticField(const float minimumMagneticField, const float maximumMagneticField);

        /**
         * @brief Updates the AHRS algorithm.  This function should be called for each
         * new gyroscope measurement.
         * @param gyroscope Gyroscope measurement in degrees per second.
         * @param accelerometer Accelerometer measurement in g.
         * @param magnetometer Magnetometer measurement in uT.
         */
        void update(const Vector3 gyroscope, const Vector3 accelerometer, const Vector3 magnetometer);

        /**
         * @brief Updates the AHRS algorithm.  This function should be called for each
         * new gyroscope measurement.
         * @param gyroscope Gyroscope measurement in degrees per second.
         * @param accelerometer Accelerometer measurement in g.
         */
        void updateWithoutMagnetometer(const Vector3 gyroscope, const Vector3 accelerometer);

        /**
         * @brief Gets the quaternion describing the sensor relative to the Earth.
         * @return Quaternion describing the sensor relative to the Earth.
         */
        Quaternion getQuaternion();

        /**
         * @brief Gets the linear acceleration measurement equal to the accelerometer
         * measurement with the 1 g of gravity removed.
         * @return Linear acceleration measurement.
         */
        Vector3 getLinearAcceleration();

        /**
         * @brief Gets the Earth acceleration measurement equal to linear acceleration
         * in the Earth coordinate frame.
         * @return Earth acceleration measurement.
         */
        Vector3 getEarthAcceleration();

        /**
         * @brief Reinitialise the AHRS algorithm.
         */
        void reinitialise();

        /**
         * @brief Returns true while the AHRS algorithm is initialising.
         * @return True while the AHRS algorithm is initialising.
         */
        bool isInitialising();

        /**
         * @brief Sets the yaw component of the orientation measurement provided by the
         * AHRS algorithm.  This function can be used to reset drift in yaw when the
         * AHRS algorithm is being used without a magnetometer.
         * @param yaw Yaw angle in degrees.
         */
        void setYaw(const float yaw);


        /**
         * @brief Gyroscope and accelerometer calibration model.
         * @param uncalibrated Uncalibrated gyroscope or accelerometer measurement in
         * lsb.
         * @param misalignment Misalignment matrix (may not be a true rotation matrix).
         * @param sensitivity Sensitivity in g per lsb for an accelerometer and degrees
         * per second per lsb for a gyroscope.
         * @param bias Bias in lsb.
         * @return Calibrated gyroscope or accelerometer measurement.
         */
        static Vector3 calibrationInertial(const Vector3 uncalibrated, const RotationMatrix misalignment, const Vector3 sensitivity, const Vector3 bias)
        {
            return rotationMatrixMultiplyVector(misalignment, vectorHadamardProduct(vectorSubtract(uncalibrated, bias), sensitivity));
        }

        /**
         * @brief Magnetometer calibration model.
         * @param magnetometer Uncalibrated magnetometer measurement in uT.
         * @param softIronMatrix Soft-iron matrix (may not be a true rotation matrix).
         * @param hardIronBias Hard-iron bias in uT.
         * @return Calibrated magnetometer measurement.
         */
        static Vector3 calibrationMagnetic(const Vector3 uncalibrated, const RotationMatrix softIronMatrix, const Vector3 hardIronBias)
        {
            return vectorSubtract(rotationMatrixMultiplyVector(softIronMatrix, uncalibrated), hardIronBias);
        }
    };
}


#endif
