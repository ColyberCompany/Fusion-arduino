/**
 * @file FusionBias.cpp
 * @author Seb Madgwick
 * @brief The gyroscope bias correction algorithm achieves run-time calibration
 * of the gyroscope bias.  The algorithm will detect when the gyroscope is
 * stationary for a set period of time and then begin to sample gyroscope
 * measurements to calculate the bias as an average.
 */


#include "FusionBias.h"

const float Fusion::FusionBias::StationaryPeriod = 5.f;
const float Fusion::FusionBias::CornerFrequency = 0.02f;


Fusion::FusionBias::FusionBias(const float threshold, const float samplePeriod)
{
    this->threshold = threshold;
    this->samplePeriod = samplePeriod;
    filterCoefficient = (2.0f * M_PI * CornerFrequency) * samplePeriod;
    stationaryTimer = 0.0f;
    gyroscopeBias = FUSION_VECTOR3_ZERO;
}


Fusion::Vector3 Fusion::FusionBias::update(Vector3 gyroscope)
{
    // Subtract bias from gyroscope measurement
    gyroscope = vectorSubtract(gyroscope, gyroscopeBias);

    // Reset stationary timer if gyroscope not stationary
    if ((fabs(gyroscope.axis.x) > threshold) || (fabs(gyroscope.axis.y) > threshold) || (fabs(gyroscope.axis.z) > threshold)) {
        stationaryTimer = 0.0f;
        return gyroscope;
    }

    // Increment stationary timer while gyroscope stationary
    if (stationaryTimer < StationaryPeriod) {
        stationaryTimer += samplePeriod;
        return gyroscope;
    }

    // Adjust bias if stationary timer has elapsed
    gyroscopeBias = vectorAdd(gyroscopeBias, vectorMultiplyScalar(gyroscope, filterCoefficient));
    return gyroscope;
}


bool Fusion::FusionBias::isActive()
{
    return stationaryTimer >= StationaryPeriod;
}
