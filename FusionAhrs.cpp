/**
 * @file FusionAhrs.cpp
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

#include "FusionAhrs.h"
#include <math.h>
#include <float.h>


/**
 * @brief Initial gain used during the initialisation period.  The gain used by
 * each algorithm iteration will ramp down from this initial gain to the
 * specified algorithm gain over the initialisation period.
 */
const float Fusion::FusionAhrs::InitialGain = 10.f;

const float Fusion::FusionAhrs::InitializationPeriod = 3.f;


Fusion::FusionAhrs::FusionAhrs(const float gain, const float samplePeriod)
    : SamplePeriod(samplePeriod)
{
    this->gain = gain;

    minimumMagneticFieldSquared = 0.0f;
    maximumMagneticFieldSquared = FLT_MAX;
    quaternion = FUSION_QUATERNION_IDENTITY;
    linearAcceleration = FUSION_VECTOR3_ZERO;
    rampedGain = InitialGain;
    zeroYawPending = false;
}


void Fusion::FusionAhrs::setGain(const float gain)
{
    this->gain = gain;
}


void Fusion::FusionAhrs::setMagneticField(const float minimumMagneticField, const float maximumMagneticField)
{
    minimumMagneticFieldSquared = minimumMagneticField * minimumMagneticField;
    maximumMagneticFieldSquared = maximumMagneticField * maximumMagneticField;
}


void Fusion::FusionAhrs::update(const Vector3 gyroscope, const Vector3 accelerometer, const Vector3 magnetometer)
{
#define Q quaternion // define shorthand label for more readable code

    // Calculate feedback error
    Vector3 halfFeedbackError = FUSION_VECTOR3_ZERO; // scaled by 0.5 to avoid repeated multiplications by 2
    do {
        // Abandon feedback calculation if accelerometer measurement invalid
        if ((accelerometer.x == 0.0f) && (accelerometer.y == 0.0f) && (accelerometer.z == 0.0f)) {
            break;
        }

        // Calculate direction of gravity assumed by quaternion
        const Vector3 halfGravity = {
            Q.x * Q.z - Q.w * Q.y,          // x
            Q.w * Q.x + Q.y * Q.z,          // y
            Q.w * Q.w - 0.5f + Q.z * Q.z    // z
        }; // equal to 3rd column of rotation matrix representation scaled by 0.5

        // Calculate accelerometer feedback error
        halfFeedbackError = vectorCrossProduct(vectorFastNormalise(accelerometer), halfGravity);

        // Abandon magnetometer feedback calculation if magnetometer measurement invalid
        const float magnetometerMagnitudeSquared = vectorMagnitudeSquared(magnetometer);
        if ((magnetometerMagnitudeSquared < minimumMagneticFieldSquared) || (magnetometerMagnitudeSquared > maximumMagneticFieldSquared)) {
            break;
        }

        // Compute direction of 'magnetic west' assumed by quaternion
        const Vector3 halfWest = {
            Q.x * Q.y + Q.w * Q.z,          // x
            Q.w * Q.w - 0.5f + Q.y * Q.y,   // y
            Q.y * Q.z - Q.w * Q.x           // z
        }; // equal to 2nd column of rotation matrix representation scaled by 0.5

        // Calculate magnetometer feedback error
        halfFeedbackError = vectorAdd(halfFeedbackError, vectorCrossProduct(vectorFastNormalise(vectorCrossProduct(accelerometer, magnetometer)), halfWest));

    } while (false);

    // Ramp down gain until initialisation complete
    if (gain == 0) {
        rampedGain = 0; // skip initialisation if gain is zero
    }
    float feedbackGain = gain;
    if (rampedGain > gain) {
        rampedGain -= (InitialGain - gain) * SamplePeriod / InitializationPeriod;
        feedbackGain = rampedGain;
    }

    // Convert gyroscope to radians per second scaled by 0.5
    Vector3 halfGyroscope = vectorMultiplyScalar(gyroscope, 0.5f * degreesToRadians(1.0f));

    // Apply feedback to gyroscope
    halfGyroscope = vectorAdd(halfGyroscope, vectorMultiplyScalar(halfFeedbackError, feedbackGain));

    // Integrate rate of change of quaternion
    quaternion = quaternionAdd(quaternion, quaternionMultiplyVector(quaternion, vectorMultiplyScalar(halfGyroscope, SamplePeriod)));

    // Normalise quaternion
    quaternion = quaternionFastNormalise(quaternion);

    // Calculate linear acceleration
    const Vector3 gravity = {
        2.0f * (Q.x * Q.z - Q.w * Q.y),         // x
        2.0f * (Q.w * Q.x + Q.y * Q.z),         // y
        2.0f * (Q.w * Q.w - 0.5f + Q.z * Q.z),  // z
    }; // equal to 3rd column of rotation matrix representation
    linearAcceleration = vectorSubtract(accelerometer, gravity);

#undef Q // undefine shorthand label
}


void Fusion::FusionAhrs::updateWithoutMagnetometer(const Vector3 gyroscope, const Vector3 accelerometer)
{
    // Update AHRS algorithm
    update(gyroscope, accelerometer, FUSION_VECTOR3_ZERO);

    // Zero yaw once initialisation complete
    if (isInitialising() == true) {
        zeroYawPending = true;
    } else {
        if (zeroYawPending == true) {
            setYaw(0.0f);
            zeroYawPending = false;
        }
    }
}


Fusion::Quaternion Fusion::FusionAhrs::getQuaternion()
{
    return quaternionConjugate(quaternion);
}


Fusion::Vector3 Fusion::FusionAhrs::getLinearAcceleration()
{
    return linearAcceleration;
}


Fusion::Vector3 Fusion::FusionAhrs::getEarthAcceleration()
{
#define Q quaternion // define shorthand labels for more readable code
#define A linearAcceleration
    const float qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;
    const Vector3 earthAcceleration = {
        2.0f * ((qwqw - 0.5f + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
        2.0f * ((qxqy + qwqz) * A.x + (qwqw - 0.5f + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
        2.0f * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5f + Q.z * Q.z) * A.z),
    }; // transpose of a rotation matrix representation of the quaternion multiplied with the linear acceleration
    return earthAcceleration;
#undef Q // undefine shorthand label
#undef A
}


void Fusion::FusionAhrs::reinitialise()
{
    quaternion = FUSION_QUATERNION_IDENTITY;
    linearAcceleration = FUSION_VECTOR3_ZERO;
    rampedGain = InitialGain;
}


bool Fusion::FusionAhrs::isInitialising()
{
    return rampedGain > gain;
}


void Fusion::FusionAhrs::setYaw(const float yaw)
{
#define Q quaternion // define shorthand label for more readable code
    quaternion = quaternionNormalise(quaternion); // quaternion must be normalised accurately (approximation not sufficient)
    const float inverseYaw = atan2f(Q.x * Q.y + Q.w * Q.z, Q.w * Q.w - 0.5f + Q.x * Q.x); // Euler angle of conjugate
    const float halfInverseYawMinusOffset = 0.5f * (inverseYaw - degreesToRadians(yaw));
    const Quaternion inverseYawQuaternion = {
        cosf(halfInverseYawMinusOffset),            // w
        0.0f,                                       // x
        0.0f,                                       // y
        -1.0f * sinf(halfInverseYawMinusOffset),    // z
    };
    quaternion = quaternionMultiply(inverseYawQuaternion, quaternion);
#undef Q // undefine shorthand label
}
