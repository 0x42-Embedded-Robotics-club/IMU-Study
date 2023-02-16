#include "BufferedSerial.h"
#include "PinNameAliases.h"
#include "PinNames.h"
#include "mbed.h"
#include <cmath>
#include <cstdio>
#include <string>

#include "MPU9250.hpp"

int main()
{
    MPU9250 mpu9250(D14, D15);
    int16_t accelData[3], gyroData[3], magData[3];

    mpu9250.resetMPU9250();
    printf("MPU9250 > reset\n");
    mpu9250.MPU9250SelfTest(mpu9250.selfTest);
    printf("MPU9250 > x-axis self test: acceleration trim within: %f pct of factory value\n", mpu9250.selfTest[0]);
	printf("MPU9250 > y-axis self test: acceleration trim within: %f pct of factory value\n", mpu9250.selfTest[1]);
	printf("MPU9250 > z-axis self test: acceleration trim within: %f pct of factory value\n", mpu9250.selfTest[2]);
	printf("MPU9250 > x-axis self test: gyration trim within: %f pct of factory value\n", mpu9250.selfTest[3]);
	printf("MPU9250 > y-axis self test: gyration trim within: %f pct of factory value\n", mpu9250.selfTest[4]);
	printf("MPU9250 > z-axis self test: gyration trim within: %f pct of factory value\n", mpu9250.selfTest[5]);
    
    float calibrateGyro[3]; // -> need fix to use bias on mpu class
    float calibrateAccel[3];
    mpu9250.calibrateMPU9250(calibrateGyro, calibrateAccel);
    printf("MPU9250 > gyro x-axis bias: %f\n", calibrateGyro[0]);
    printf("MPU9250 > gyro y-axis bias: %f\n", calibrateGyro[1]);
    printf("MPU9250 > gyro z-axis bias: %f\n", calibrateGyro[2]);
    printf("MPU9250 > accel x-axis bias: %f\n", calibrateAccel[0]);
    printf("MPU9250 > accel y-axis bias: %f\n", calibrateAccel[1]);
    printf("MPU9250 > accel z-axis bias: %f\n", calibrateAccel[2]);

    wait_us(2000);
    mpu9250.initMPU9250();
    printf("MPU9250 > init\n");
    mpu9250.initAK8963(mpu9250.magCalibration);
    printf("MPU9250 > AK8963 init\n");
    
    mpu9250.getAccelRes();
    mpu9250.getGyroRes();
    mpu9250.getMagRes();
    printf("MPU9250 > Accelerometer sensitivity is %f LSB/g \n", 1.0f / mpu9250.aRes);
	printf("MPU9250 > Gyroscope sensitivity is %f LSB/deg/s \n", 1.0f / mpu9250.gRes);
	printf("MPU9250 > Magnetometer sensitivity is %f LSB/G \n", 1.0f / mpu9250.mRes);

    BufferedSerial pc(USBTX, USBRX);
    pc.set_baud(115200);
    pc.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );

    std::string cummBuf;

    while (1)
    {
        mpu9250.readAccelData(accelData);
        mpu9250.readGyroData(gyroData);
        mpu9250.readMagData(magData);
        
        float ax = (float)accelData[0] * mpu9250.aRes - calibrateAccel[0];
        float ay = (float)accelData[1] * mpu9250.aRes - calibrateAccel[1];
        float az = (float)accelData[2] * mpu9250.aRes - calibrateAccel[2];

        float gx = (float)gyroData[0] * mpu9250.gRes - calibrateGyro[0];
        float gy = (float)gyroData[1] * mpu9250.gRes - calibrateGyro[1];
        float gz = (float)gyroData[2] * mpu9250.gRes - calibrateGyro[2];

        float mx = (float)magData[0] * mpu9250.mRes * mpu9250.magCalibration[0];
        float my = (float)magData[1] * mpu9250.mRes * mpu9250.magCalibration[1];
        float mz = (float)magData[2] * mpu9250.mRes * mpu9250.magCalibration[2];

        // printf("accel(g/s): %f, %f, %f (length: %f)\n", ax, ay, az, sqrtf(pow(ax, 2) + pow(ay, 2) + pow(az, 2)));
        // printf("gyro(Degree/second): %f, %f, %f\n", gx, gy, gz);
        // printf("mag(milligauss): %d, %d, %d\n\n", magData[0], magData[1], magData[2]);

        cummBuf = 
            to_string(ax) + "," + to_string(ay) + "," + to_string(az) + "," + 
            to_string(gx) + "," + to_string(gy) + "," + to_string(gz) + "," + 
            to_string(mx) + "," + to_string(my) + "," + to_string(mz) + "," + 
            to_string(0) + "," + to_string(0) + "," + to_string(0);

        printf("%s\n", cummBuf.c_str());

        wait_us(1000);
    }
}