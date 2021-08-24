#include <Tasker.h>
#include <SimpleMPU6050.h>
#include <FusionAhrs.h>
#include <FusionBias.h>

using namespace Fusion;

const float MainFreq = 200.f;
const float samplePeriod = 1.f / MainFreq;


SimpleMPU6050 mpu;
Tasker tasker(5);

//FusionBias bias(0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second
FusionAhrs ahrs(0.5f, samplePeriod); // gain = 0.5


Vector3 gyroscopeSensitivity = {
    0.0152671f, 0.0152671f, 0.0152671f
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

Vector3 accelerometerSensitivity = {
    0.000244141f, 0.000244141f, 0.000244141f
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet


class : public IExecutable
{
    float velocity = 0.f;
    float position = 0.f;

    void execute() override
    {
        mpu.readRawData();

        auto rawAcc = mpu.getRawAcceleration();
        auto rawGyro = mpu.getRawRotation();

        // Calibrate gyroscope
        Vector3 uncalibratedGyroscope = {
            rawGyro.x,
            rawGyro.y,
            rawGyro.z
        };
        Vector3 calibratedGyroscope = FusionAhrs::calibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate accelerometer
        Vector3 uncalibratedAccelerometer = {
            rawAcc.x,
            rawAcc.y,
            rawAcc.z
        };
        Vector3 calibratedAccelerometer = FusionAhrs::calibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

        // Update gyroscope bias correction algorithm
        //calibratedGyroscope = bias.update(calibratedGyroscope);

        // Update AHRS algorithm
        ahrs.updateWithoutMagnetometer(calibratedGyroscope, calibratedAccelerometer);



        // Vector3 earthAccel = ahrs.getEarthAcceleration();
        // float az = earthAccel.axis.z * 9.81f;
        // velocity += az * samplePeriod;
        // position += velocity * samplePeriod;

        // Serial.println(velocity);
    }
} ahrsTask;


class : public IExecutable
{
    void execute() override
    {
        EulerAngles eulerAngles = quaternionToEulerAngles(ahrs.getQuaternion());

        Serial.print(eulerAngles.angle.pitch);
        Serial.print('\t');
        Serial.print(eulerAngles.angle.roll);
        Serial.print('\t');
        Serial.println(eulerAngles.angle.yaw);


        // Vector3 earthAccel = ahrs.getEarthAcceleration();

        // Serial.print(earthAccel.axis.x);
        // Serial.print('\t');
        // Serial.print(earthAccel.axis.y);
        // Serial.print('\t');
        // Serial.println(earthAccel.axis.z);
    }
} printingTask;


void setup()
{
    Serial.begin(115200);
    Serial.println("Program has started");

    Wire.begin();
    delay(100);

    mpu.initialize();
    mpu.setGyroOffset(-142, 124, 5);

    Wire.setClock(400000L);


    tasker.addTask_Hz(&ahrsTask, MainFreq);
    tasker.addTask_Hz(&printingTask, 50.f);
}


void loop()
{
    tasker.loop();
}
