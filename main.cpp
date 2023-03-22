#include "BufferedSerial.h"
#include "I2Cdev.h"
#include "PinNameAliases.h"
#include "PinNames.h"
#include "mbed.h"
#include <cmath>
#include <cstdio>
#include <string>
#include "MPU9250.hpp"

MPU9250 accelgyro;
I2Cdev I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
#define sample_num_mdate  5000
volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;

void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    wait_us(10000);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}

void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}

int main()
{
    // Serial set
    BufferedSerial pc(USBTX, USBRX);
    pc.set_baud(115200);
    pc.set_format(8, BufferedSerial::None, 1);

    // MPU9250 initialize
    printf("Initializing I2C devices...\n");
    accelgyro.initialize();
    printf("Testing device connections...\n");
    if (accelgyro.testConnection())
        printf("MPU9250 connection successful\n");
    else
        printf("MPU9250 connection failed\n");
    
    wait_us(1000000);
    printf("     \n");

    while (1)
    {
        getAccel_Data();
        getGyro_Data();
        getCompassDate_calibrated(); 
        getHeading();               
        getTiltHeading();

        printf("calibration parameter: \n");
        printf("%f", mx_centre);
        printf("         ");
        printf("%f", my_centre);
        printf("         ");
        printf("%f\n", mz_centre);
        printf("     \n");


        printf("Acceleration(g) of X,Y,Z:\n");
        printf("%f", Axyz[0]);
        printf(",");
        printf("%f", Axyz[1]);
        printf(",");
        printf("%f\n", Axyz[2]);
        printf("Gyro(degress/s) of X,Y,Z:\n");
        printf("%f", Gxyz[0]);
        printf(",");
        printf("%f", Gxyz[1]);
        printf(",");
        printf("%f\n", Gxyz[2]);
        printf("Compass Value of X,Y,Z:\n");
        printf("%f", Mxyz[0]);
        printf(",");
        printf("%f", Mxyz[1]);
        printf(",");
        printf("%f\n", Mxyz[2]);
        printf("The clockwise angle between the magnetic north and X-Axis:\n");
        printf("%f", heading);
        printf(" \n");
        printf("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:\n");
        printf("%f", tiltheading);
        printf("   \n");
        printf("\n");
        wait_us(1000000);
    }

    return 0;
}