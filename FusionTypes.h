/**
 * @file FusionTypes.h
 * @author Seb Madgwick
 * @brief Common types and their associated operations.
 *
 * Static inline implementations are used to optimise for increased execution
 * speed.
 */

#ifndef FUSION_TYPES_H
#define FUSION_TYPES_H

#include <math.h> // M_PI, sqrtf, atan2f, asinf
#include <stdint.h> // int32_t


namespace Fusion
{
    /**
     * @brief Three-dimensional spacial vector.
     */
    struct Vector3 {
        float x;
        float y;
        float z;
    };

    /**
     * @brief Quaternion. This library uses the conversion of placing the 'w'
     * element as the first element. Other implementations may place the 'w'
     * element as the last element.
     */
    struct Quaternion {
        float w;
        float x;
        float y;
        float z;
    };

    /**
     * @brief Rotation matrix in row-major order.
     * See http://en.wikipedia.org/wiki/Row-major_order
     */
    struct RotationMatrix {
        float xx;
        float xy;
        float xz;
        float yx;
        float yy;
        float yz;
        float zx;
        float zy;
        float zz;
    };

    /**
     * @brief The Euler angles are in the Aerospace sequence also known as the ZYX sequence.
     */
    struct EulerAngles {
        float roll;
        float pitch;
        float yaw;
    };

    /**
     * @brief Zero-length vector definition.
     */
    #define FUSION_VECTOR3_ZERO ((Vector3){ 0.0f, 0.0f, 0.0f })

    /**
     * @brief Quaternion identity definition to represent an aligned of
     * orientation.
     */
    #define FUSION_QUATERNION_IDENTITY ((Quaternion){ 1.0f, 0.0f, 0.0f, 0.0f })

    /**
     * @brief Rotation matrix identity definition to represent an aligned of
     * orientation.
     */
    #define FUSION_ROTATION_MATRIX_IDENTITY ((RotationMatrix){ 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f })

    /**
     * @brief Euler angles zero definition to represent an aligned of orientation.
     */
    #define FUSION_EULER_ANGLES_ZERO ((EulerAngles){ 0.0f, 0.0f, 0.0f })

    /**
     * @brief Definition of M_PI.  Some compilers may not define this in math.h.
     */
    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif

    //------------------------------------------------------------------------------
    // Inline functions - Degrees and radians conversion

    /**
     * @brief Converts degrees to radians.
     * @param degrees Degrees.
     * @return Radians.
     */
    static inline __attribute__((always_inline)) float degreesToRadians(const float degrees) {
        return degrees * ((float) M_PI / 180.0f);
    }

    /**
     * @brief Converts radians to degrees.
     * @param radians Radians.
     * @return Degrees.
     */
    static inline __attribute__((always_inline)) float radiansToDegrees(const float radians) {
        return radians * (180.0f / (float) M_PI);
    }

    //------------------------------------------------------------------------------
    // Inline functions - Fast inverse square root

    /**
     * @brief Calculates the reciprocal of the square root.
     * See http://en.wikipedia.org/wiki/Fast_inverse_square_root
     * @param x Operand.
     * @return Reciprocal of the square root of x.
     */
    static inline __attribute__((always_inline)) float fastInverseSqrt(const float x) {
        float halfx = 0.5f * x;
        float y = x;
        int32_t i = *(int32_t*) & y;
        i = 0x5f3759df - (i >> 1);
        y = *(float*) &i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }

    //------------------------------------------------------------------------------
    // Inline functions - Vector operations

    /**
     * @brief Adds two vectors.
     * @param vectorA First vector of the operation.
     * @param vectorB Second vector of the operation.
     * @return Sum of vectorA and vectorB.
     */
    static inline __attribute__((always_inline)) Vector3 vectorAdd(const Vector3 vectorA, const Vector3 vectorB) {
        Vector3 result;
        result.x = vectorA.x + vectorB.x;
        result.y = vectorA.y + vectorB.y;
        result.z = vectorA.z + vectorB.z;
        return result;
    }

    /**
     * @brief Subtracts two vectors.
     * @param vectorA First vector of the operation.
     * @param vectorB Second vector of the operation.
     * @return vectorB subtracted from vectorA.
     */
    static inline __attribute__((always_inline)) Vector3 vectorSubtract(const Vector3 vectorA, const Vector3 vectorB) {
        Vector3 result;
        result.x = vectorA.x - vectorB.x;
        result.y = vectorA.y - vectorB.y;
        result.z = vectorA.z - vectorB.z;
        return result;
    }

    /**
     * @brief Multiplies vector by a scalar.
     * @param vector Vector to be multiplied.
     * @param scalar Scalar to be multiplied.
     * @return Vector multiplied by scalar.
     */
    static inline __attribute__((always_inline)) Vector3 vectorMultiplyScalar(const Vector3 vector, const float scalar) {
        Vector3 result;
        result.x = vector.x * scalar;
        result.y = vector.y * scalar;
        result.z = vector.z * scalar;
        return result;
    }

    /**
     * @brief Calculates the Hadamard product (element-wise multiplication) of two
     * vectors.
     * @param vectorA First vector of the operation.
     * @param vectorB Second vector of the operation.
     * @return Hadamard product of vectorA and vectorB.
     */
    static inline __attribute__((always_inline)) Vector3 vectorHadamardProduct(const Vector3 vectorA, const Vector3 vectorB) {
        Vector3 result;
        result.x = vectorA.x * vectorB.x;
        result.y = vectorA.y * vectorB.y;
        result.z = vectorA.z * vectorB.z;
        return result;
    }

    /**
     * @brief Calculates the cross-product of two vectors.
     * @param vectorA First vector of the operation.
     * @param vectorB Second vector of the operation.
     * @return Cross-product of vectorA and vectorB.
     */
    static inline __attribute__((always_inline)) Vector3 vectorCrossProduct(const Vector3 vectorA, const Vector3 vectorB) {
    #define A vectorA // define shorthand labels for more readable code
    #define B vectorB
        Vector3 result;
        result.x = A.y * B.z - A.z * B.y;
        result.y = A.z * B.x - A.x * B.z;
        result.z = A.x * B.y - A.y * B.x;
        return result;
    #undef A // undefine shorthand labels
    #undef B
    }

    /**
     * @brief Calculates the vector magnitude squared.
     * @param vector Vector of the operation.
     * @return Vector magnitude squared.
     */
    static inline __attribute__((always_inline)) float vectorMagnitudeSquared(const Vector3 vector) {
    #define V vector // define shorthand label for more readable code
        return V.x * V.x + V.y * V.y + V.z * V.z;
    #undef V // undefine shorthand label
    }

    /**
     * @brief Calculates the magnitude of a vector.
     * @param vector Vector to be used in calculation.
     * @return Vector magnitude.
     */
    static inline __attribute__((always_inline)) float vectorMagnitude(const Vector3 vector) {
        return sqrtf(vectorMagnitudeSquared(vector));
    }

    /**
     * @brief Normalises a vector to be of unit magnitude.
     * @param vector Vector to be normalised.
     * @return Normalised vector.
     */
    static inline __attribute__((always_inline)) Vector3 vectorNormalise(const Vector3 vector) {
        const float magnitudeReciprocal = 1.0f / sqrtf(vectorMagnitudeSquared(vector));
        return vectorMultiplyScalar(vector, magnitudeReciprocal);
    }

    /**
     * @brief Normalises a vector to be of unit magnitude using the fast inverse
     * square root approximation.
     * @param vector Vector to be normalised.
     * @return Normalised vector.
     */
    static inline __attribute__((always_inline)) Vector3 vectorFastNormalise(const Vector3 vector) {
        const float magnitudeReciprocal = fastInverseSqrt(vectorMagnitudeSquared(vector));
        return vectorMultiplyScalar(vector, magnitudeReciprocal);
    }

    //------------------------------------------------------------------------------
    // Inline functions - Quaternion operations

    /**
     * @brief Adds two quaternions.
     * @param quaternionA First quaternion of the operation.
     * @param quaternionB Second quaternion of the operation.
     * @return Sum of quaternionA and quaternionB.
     */
    static inline __attribute__((always_inline)) Quaternion quaternionAdd(const Quaternion quaternionA, const Quaternion quaternionB) {
        Quaternion result;
        result.w = quaternionA.w + quaternionB.w;
        result.x = quaternionA.x + quaternionB.x;
        result.y = quaternionA.y + quaternionB.y;
        result.z = quaternionA.z + quaternionB.z;
        return result;
    }

    /**
     * @brief Multiplies two quaternions.
     * @param quaternionA First quaternion of the operation.
     * @param quaternionB Second quaternion of the operation.
     * @return quaternionA multiplied by quaternionB.
     */
    static inline __attribute__((always_inline)) Quaternion quaternionMultiply(const Quaternion quaternionA, const Quaternion quaternionB) {
    #define A quaternionA // define shorthand labels for more readable code
    #define B quaternionB
        Quaternion result;
        result.w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z;
        result.x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y;
        result.y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x;
        result.z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w;
        return result;
    #undef A // undefine shorthand labels
    #undef B
    }

    /**
     * @brief Multiplies quaternion by a vector.  This is a normal quaternion
     * multiplication where the vector is treated a quaternion with a 'w' element
     * value of 0.  The quaternion is post multiplied by the vector.
     * @param quaternion Quaternion to be multiplied.
     * @param vector Vector to be multiplied.
     * @return Quaternion multiplied by vector.
     */
    static inline __attribute__((always_inline)) Quaternion quaternionMultiplyVector(const Quaternion quaternion, const Vector3 vector) {
    #define Q quaternion // define shorthand labels for more readable code
    #define V vector
        Quaternion result;
        result.w = -Q.x * V.x - Q.y * V.y - Q.z * V.z;
        result.x = Q.w * V.x + Q.y * V.z - Q.z * V.y;
        result.y = Q.w * V.y - Q.x * V.z + Q.z * V.x;
        result.z = Q.w * V.z + Q.x * V.y - Q.y * V.x;
        return result;
    #undef Q // undefine shorthand labels
    #undef V
    }

    /**
     * @brief Returns the quaternion conjugate.
     * @param quaternion Quaternion to be conjugated.
     * @return Conjugated quaternion.
     */
    static inline __attribute__((always_inline)) Quaternion quaternionConjugate(const Quaternion quaternion) {
        Quaternion conjugate;
        conjugate.w = quaternion.w;
        conjugate.x = -1.0f * quaternion.x;
        conjugate.y = -1.0f * quaternion.y;
        conjugate.z = -1.0f * quaternion.z;
        return conjugate;
    }

    /**
     * @brief Normalises a quaternion to be of unit magnitude.
     * @param quaternion Quaternion to be normalised.
     * @return Normalised quaternion.
     */
    static inline __attribute__((always_inline)) Quaternion quaternionNormalise(const Quaternion quaternion) {
    #define Q quaternion // define shorthand label for more readable code
        const float magnitudeReciprocal = 1.0f / sqrtf(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
        Quaternion normalisedQuaternion;
        normalisedQuaternion.w = Q.w * magnitudeReciprocal;
        normalisedQuaternion.x = Q.x * magnitudeReciprocal;
        normalisedQuaternion.y = Q.y * magnitudeReciprocal;
        normalisedQuaternion.z = Q.z * magnitudeReciprocal;
        return normalisedQuaternion;
    #undef Q // undefine shorthand label
    }

    /**
     * @brief Normalises a quaternion to be of unit magnitude using the fast inverse
     * square root approximation.
     * @param quaternion Quaternion to be normalised.
     * @return Normalised quaternion.
     */
    static inline __attribute__((always_inline)) Quaternion quaternionFastNormalise(const Quaternion quaternion) {
    #define Q quaternion // define shorthand label for more readable code
        const float magnitudeReciprocal = fastInverseSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
        Quaternion normalisedQuaternion;
        normalisedQuaternion.w = Q.w * magnitudeReciprocal;
        normalisedQuaternion.x = Q.x * magnitudeReciprocal;
        normalisedQuaternion.y = Q.y * magnitudeReciprocal;
        normalisedQuaternion.z = Q.z * magnitudeReciprocal;
        return normalisedQuaternion;
    #undef Q // undefine shorthand label
    }

    //------------------------------------------------------------------------------
    // Inline functions - Rotation matrix operations

    /**
     * @brief Multiplies two rotation matrices.
     * @param rotationMatrixA First rotation matrix of the operation.
     * @param rotationMatrixB Second rotation matrix of the operation.
     * @return rotationMatrixA with rotationMatrixB.
     */
    static inline __attribute__((always_inline)) RotationMatrix rotationMatrixMultiply(const RotationMatrix rotationMatrixA, const RotationMatrix rotationMatrixB) {
    #define A rotationMatrixA // define shorthand label for more readable code
    #define B rotationMatrixB
        RotationMatrix result;
        result.xx = A.xx * B.xx + A.xy * B.yx + A.xz * B.zx;
        result.xy = A.xx * B.xy + A.xy * B.yy + A.xz * B.zy;
        result.xz = A.xx * B.xz + A.xy * B.yz + A.xz * B.zz;
        result.yx = A.yx * B.xx + A.yy * B.yx + A.yz * B.zx;
        result.yy = A.yx * B.xy + A.yy * B.yy + A.yz * B.zy;
        result.yz = A.yx * B.xz + A.yy * B.yz + A.yz * B.zz;
        result.zx = A.zx * B.xx + A.zy * B.yx + A.zz * B.zx;
        result.zy = A.zx * B.xy + A.zy * B.yy + A.zz * B.zy;
        result.zz = A.zx * B.xz + A.zy * B.yz + A.zz * B.zz;
        return result;
    #undef A // undefine shorthand label
    #undef B
    }

    /**
     * @brief Multiplies rotation matrix with scalar.
     * @param rotationMatrix Rotation matrix to be multiplied.
     * @param vector Vector to be multiplied.
     * @return Rotation matrix multiplied with scalar.
     */
    static inline __attribute__((always_inline)) Vector3 rotationMatrixMultiplyVector(const RotationMatrix rotationMatrix, const Vector3 vector) {
    #define R rotationMatrix // define shorthand label for more readable code
        Vector3 result;
        result.x = R.xx * vector.x + R.xy * vector.y + R.xz * vector.z;
        result.y = R.yx * vector.x + R.yy * vector.y + R.yz * vector.z;
        result.z = R.zx * vector.x + R.zy * vector.y + R.zz * vector.z;
        return result;
    #undef R // undefine shorthand label
    }

    //------------------------------------------------------------------------------
    // Inline functions - Conversion operations

    /**
     * @brief Converts a quaternion to a rotation matrix.
     * @param quaternion Quaternion to be converted.
     * @return Rotation matrix.
     */
    static inline __attribute__((always_inline)) RotationMatrix quaternionToRotationMatrix(const Quaternion quaternion) {
    #define Q quaternion // define shorthand label for more readable code
        const float qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
        const float qwqx = Q.w * Q.x;
        const float qwqy = Q.w * Q.y;
        const float qwqz = Q.w * Q.z;
        const float qxqy = Q.x * Q.y;
        const float qxqz = Q.x * Q.z;
        const float qyqz = Q.y * Q.z;
        RotationMatrix rotationMatrix;
        rotationMatrix.xx = 2.0f * (qwqw - 0.5f + Q.x * Q.x);
        rotationMatrix.xy = 2.0f * (qxqy + qwqz);
        rotationMatrix.xz = 2.0f * (qxqz - qwqy);
        rotationMatrix.yx = 2.0f * (qxqy - qwqz);
        rotationMatrix.yy = 2.0f * (qwqw - 0.5f + Q.y * Q.y);
        rotationMatrix.yz = 2.0f * (qyqz + qwqx);
        rotationMatrix.zx = 2.0f * (qxqz + qwqy);
        rotationMatrix.zy = 2.0f * (qyqz - qwqx);
        rotationMatrix.zz = 2.0f * (qwqw - 0.5f + Q.z * Q.z);
        return rotationMatrix;
    #undef Q // undefine shorthand label
    }

    /**
     * @brief Converts a quaternion to Euler angles in degrees.
     * @param quaternion Quaternion to be converted.
     * @return Euler angles in degrees.
     */
    static inline __attribute__((always_inline)) EulerAngles quaternionToEulerAngles(const Quaternion quaternion) {
    #define Q quaternion // define shorthand label for more readable code
        const float qwqwMinusHalf = Q.w * Q.w - 0.5f; // calculate common terms to avoid repeated operations
        EulerAngles eulerAngles;
        eulerAngles.roll = radiansToDegrees(atan2f(Q.y * Q.z - Q.w * Q.x, qwqwMinusHalf + Q.z * Q.z));
        eulerAngles.pitch = radiansToDegrees(-1.0f * asinf(2.0f * (Q.x * Q.z + Q.w * Q.y)));
        eulerAngles.yaw = radiansToDegrees(atan2f(Q.x * Q.y - Q.w * Q.z, qwqwMinusHalf + Q.x * Q.x));
        return eulerAngles;
    #undef Q // undefine shorthand label
    }
}


#endif
