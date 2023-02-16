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

    mpu9250.resetMPU9250();
    printf("MPU9250 > reset\n");
    mpu9250.MPU9250SelfTest();
    printf("MPU9250 > x-axis self test: acceleration trim within: %f pct of factory value\n", mpu9250.selfTest[0]);
	printf("MPU9250 > y-axis self test: acceleration trim within: %f pct of factory value\n", mpu9250.selfTest[1]);
	printf("MPU9250 > z-axis self test: acceleration trim within: %f pct of factory value\n", mpu9250.selfTest[2]);
	printf("MPU9250 > x-axis self test: gyration trim within: %f pct of factory value\n", mpu9250.selfTest[3]);
	printf("MPU9250 > y-axis self test: gyration trim within: %f pct of factory value\n", mpu9250.selfTest[4]);
	printf("MPU9250 > z-axis self test: gyration trim within: %f pct of factory value\n", mpu9250.selfTest[5]);
    
    mpu9250.calibrateMPU9250();
    printf("MPU9250 > gyro x-axis bias: %f\n", mpu9250.calibrateGyro[0]);
    printf("MPU9250 > gyro y-axis bias: %f\n", mpu9250.calibrateGyro[1]);
    printf("MPU9250 > gyro z-axis bias: %f\n", mpu9250.calibrateGyro[2]);
    printf("MPU9250 > accel x-axis bias: %f\n", mpu9250.calibrateAccel[0]);
    printf("MPU9250 > accel y-axis bias: %f\n", mpu9250.calibrateAccel[1]);
    printf("MPU9250 > accel z-axis bias: %f\n", mpu9250.calibrateAccel[2]);

    wait_us(2000);
    mpu9250.initMPU9250();
    printf("MPU9250 > init\n");
    mpu9250.initAK8963();
    printf("MPU9250 > AK8963 init\n");
    
    mpu9250.getAccelRes();
    mpu9250.getGyroRes();
    mpu9250.getMagRes();
    printf("MPU9250 > Accelerometer sensitivity is %f LSB/g \n", 1.0f / mpu9250.aRes);
	printf("MPU9250 > Gyroscope sensitivity is %f LSB/deg/s \n", 1.0f / mpu9250.gRes);
	printf("MPU9250 > Magnetometer sensitivity is %f LSB/G \n", 1.0f / mpu9250.mRes);

    BufferedSerial pc(USBTX, USBRX);
    // pc.set_baud(115200);
    pc.set_baud(9600);
    pc.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );

    std::string cummBuf;

    while (1)
    {
        mpu9250.readAccelData();
        mpu9250.readGyroData();
        mpu9250.readMagData();

        // printf("accel(g/s): %f, %f, %f (length: %f)\n\n", mpu9250.accel[X], mpu9250.accel[Y], mpu9250.accel[Z], sqrtf(pow(mpu9250.accel[X], 2) + pow(mpu9250.accel[Y], 2) + pow(mpu9250.accel[Z], 2)));
        // printf("gyro(Degree/second): %f, %f, %f\n\n", mpu9250.gyro[X], mpu9250.gyro[Y], mpu9250.gyro[Z]);
        // printf("mag(milligauss): %f, %f, %f\n\n", mpu9250.mag[X], mpu9250.mag[Y], mpu9250.mag[Z]);

        cummBuf = 
            to_string(mpu9250.accel[X]) + "," + to_string(mpu9250.accel[Y]) + "," + to_string(mpu9250.accel[Z]) + "," + 
            to_string(mpu9250.gyro[X]) + "," + to_string(mpu9250.gyro[Y]) + "," + to_string(mpu9250.gyro[Z]) + "," + 
            to_string(mpu9250.mag[X]) + "," + to_string(mpu9250.mag[Y]) + "," + to_string(mpu9250.mag[Z]) + "," + 
            to_string(0) + "," + to_string(0) + "," + to_string(0);

        printf("%s\n", cummBuf.c_str());

        wait_us(1000);
    }
}