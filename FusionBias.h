/**
 * @file FusionBias.h
 * @author Seb Madgwick
 * @brief The gyroscope bias correction algorithm achieves run-time calibration
 * of the gyroscope bias.  The algorithm will detect when the gyroscope is
 * stationary for a set period of time and then begin to sample gyroscope
 * measurements to calculate the bias as an average.
 */

#ifndef FUSION_BIAS_H
#define FUSION_BIAS_H

#include "FusionTypes.h"


namespace Fusion
{
    /**
     * @brief Gyroscope bias correction algorithm class.
     */
    class FusionBias
    {
        float threshold;
        float samplePeriod;
        float filterCoefficient;
        float stationaryTimer;
        Vector3 gyroscopeBias;

        static const float StationaryPeriod; // Minimum stationary period (in seconds) after which the the algorithm becomes active and begins sampling gyroscope measurements.
        static const float CornerFrequency;  // Corner frequency (in Hz) of the high-pass filter used to sample the gyroscope bias.

    public:
        /**
         * @brief Initialises the gyroscope bias correction algorithm.
         * @param threshold Gyroscope threshold (in degrees per second) below which the
         * gyroscope is detected stationary.
         * @param samplePeriod Nominal sample period (in seconds) corresponding the rate
         * at which the application will update the algorithm.
         */
        FusionBias(const float threshold, const float samplePeriod);

        /**
         * @brief Updates the gyroscope bias correction algorithm and returns the
         * corrected gyroscope measurement.
         * @param gyroscope Gyroscope measurement in degrees per second.
         * @return Corrected gyroscope measurement in degrees per second.
         */
        Vector3 update(Vector3 gyroscope);

        /**
         * @brief Returns true if the gyroscope bias correction algorithm is active.
         * @return True if the gyroscope bias correction algorithm is active.
         */
        bool isActive();
    };
}


#endif
