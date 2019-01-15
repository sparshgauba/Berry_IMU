/*
    This program  reads the angles from the accelerometer and gyroscope
    on a BerryIMU connected to a Raspberry Pi.


    Both the BerryIMUv1 and BerryIMUv2 are supported

    Feel free to do whatever you like with this code
    Distributed as-is; no warranty is given.

    http://ozzmaker.com/
*/



#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include "IMU.c"
#include "MadgwickAHRS.h"


#define DT 0.02         // [s/loop] loop period. 20ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573    // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846

#define THRES_A 15
#define THRES_G 15

void  INThandler(int sig)// Used to do a nice clean exit when Ctrl-C is pressed
{
    signal(sig, SIG_IGN);
    exit(0);
}

int mymillis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;
    return (diff<0);
}

long long * calibrate_acc()
{
    long long *ret = malloc(4*sizeof(long long));
    ret[0] = ret[1] = ret[2] = ret[3] = 0;
    int accRaw[3];
    int start = mymillis();
    for (int i = 0; i < 100; i++)
    {
        readACC(accRaw);
        ret[0]+=accRaw[0];
        ret[1]+=accRaw[1];
        ret[2]+=accRaw[2];
    }
    printf("*********************************    Loop Time %d     ************************\n", mymillis()- start);
    ret[0] = ret[0]/100;
    ret[1] = ret[1]/100;
    ret[2] = ret[2]/100;
    //ret[3] is the calculated raw value that corresponds to 1 G
    ret[3] = (long long)sqrt( pow(ret[0],2) + pow(ret[1],2) + pow(ret[2],2));
    return ret;
}

long long *calibrate_gyr()
{
    long long *ret = malloc(3*sizeof(long long));
    ret[0] = ret[1] = ret[2] = 0;
    int gyrRaw[3];
    int start = mymillis();
    for (int i = 0; i < 100; i++)
    {
        readGYR(gyrRaw);
        ret[0]+=gyrRaw[0];
        ret[1]+=gyrRaw[1];
        ret[2]+=gyrRaw[2];
    }
    printf("*********************************    Loop Time %d     ************************\n", mymillis()- start);
    ret[0] = ret[0]/100;
    ret[1] = ret[1]/100;
    ret[2] = ret[2]/100;
    return ret;
}

int main(int argc, char *argv[])
{


    float rate_gyr_y = 0.0;   // [deg/s]
    float rate_gyr_x = 0.0;   // [deg/s]
    float rate_gyr_z = 0.0;   // [deg/s]

    int  accRaw[3];
    int  magRaw[3];
    int  gyrRaw[3];

    //Previous states for low threshold filtering

    float gyroXangle = 0.0;
    float gyroYangle = 0.0;
    float gyroZangle = 0.0;
    float AccYangle = 0.0;
    float AccXangle = 0.0;
    float CFangleX = 0.0;
    float CFangleY = 0.0;

    //Calibrated Acc Values with prev values for filtering;
    int ca_x[2] = {0};
    int ca_y[2] = {0};
    int ca_z[2] = {0};
    //Calibrated Gyr Values
    int cg_x[2] = {0};
    int cg_y[2] = {0};
    int cg_z[2] = {0};

    int startInt  = mymillis();
    struct  timeval tvBegin, tvEnd,tvDiff;


    signal(SIGINT, INThandler);

    detectIMU();
    enableIMU();

    //Get Calibration offset
    long long *ca = calibrate_acc();
    long long *cg = calibrate_gyr();

    int G_raw = (int)ca[3];
    gettimeofday(&tvBegin, NULL);


    while(1)
    {
        startInt = mymillis();

        //Slide last values back
        ca_x[1] = ca_x[0];
        ca_y[1] = ca_y[0];
        ca_z[1] = ca_z[0];
        cg_x[1] = cg_x[0];
        cg_y[1] = cg_y[0];
        cg_z[1] = cg_z[0];


        //read ACC and GYR data
        readACC(accRaw);
        readGYR(gyrRaw);
        readMAG(magRaw);

        //Subtracted calibration values
        ca_x[0] = accRaw[0] - ca[0];
        ca_y[0] = accRaw[1] - ca[1];
        ca_z[0] = accRaw[2];
        cg_x[0] = gyrRaw[0] - cg[0];
        cg_y[0] = gyrRaw[1] - cg[1];
        cg_z[0] = gyrRaw[2] - cg[2];

        // If new val is less than |15| apart from last, keep last value
        // Otherwise, new val is the average of the cur and prev
        ca_x[0] = abs(ca_x[0]) < THRES_A ? 0 : (int)(ca_x[1] + ca_x[0]) / 2;
        ca_y[0] = abs(ca_y[0]) < THRES_A ? 0 : (int)(ca_y[1] + ca_y[0]) / 2;
        ca_z[0] = abs(ca_z[0]) < THRES_A ? 0 : (int)(ca_z[1] + ca_z[0]) / 2;
        cg_x[0] = abs(cg_x[0]) < THRES_G ? 0 : (int)(cg_x[1] + cg_x[0]) / 2;
        cg_y[0] = abs(cg_y[0]) < THRES_G ? 0 : (int)(cg_y[1] + cg_y[0]) / 2;
        cg_z[0] = abs(cg_z[0]) < THRES_G ? 0 : (int)(cg_z[1] + cg_z[0]) / 2;

        //Print Acc Values after Calibration
        //printf("AccX: %4d\tAccY: %4d\tAccZ: %4d\t", ca_x, ca_y, ca_z);

        //Print Gyr Values after Calibration
        //printf("GyrX: %4d\tGyrY: %4d\tGyrZ: %4d\t", cg_x, cg_y, cg_z);

        //Print Mag Values after Calibration
        printf("MagX: %4d\tMagY: %4d\tMagZ: %4d\t", magRaw[0], magRaw[1], magRaw[2]);

        //Convert to G values
        float acc_x = ((float)ca_x[0]/G_raw);
        float acc_y = ((float)ca_y[0]/G_raw);
        float acc_z = ((float)ca_z[0]/G_raw);

        //Print G values
        //printf("AccX: %5.2f\tAccY: %5.2f\tAccZ: %5.2f\t", acc_x, acc_y, acc_z);


        //Convert Gyro raw to degrees per second
        rate_gyr_x = (float) cg_x[0]  * G_GAIN;
        rate_gyr_y = (float) cg_y[0]  * G_GAIN;
        rate_gyr_z = (float) cg_z[0]  * G_GAIN;


        //PRINT GYR DEG PER SEC
        //printf("GyrX: %10.5f\tGyrY: %10.5f\tGyrZ: %10.5f\t", rate_gyr_x, rate_gyr_y, rate_gyr_z);


        //Calculate the angles from the gyro
        //gyroXangle+=rate_gyr_x*DT;
        //gyroYangle+=rate_gyr_y*DT;
        //gyroZangle+=rate_gyr_z*DT;

        //PRINT ANGLES BASED ON GYRO
        //printf("GyrX: %10.5f\tGyrY: %10.5f\tGyrZ: %10.5f\t", gyroXangle, gyroYangle, gyroZangle);


        //Convert Accelerometer values to degrees
        AccXangle = (float) ( atan2( ca_y[0], ca_z[0] ) + M_PI ) * RAD_TO_DEG; //Roll about X axis
        AccYangle = (float) ( atan2( ca_z[0], ca_x[0] ) + M_PI ) * RAD_TO_DEG; //Pitch about Y axis

        //PRINT ACC BASED PITCH AND ROLL
        printf("Roll:  %5.1f\tPitch:  %5.1f\t", AccXangle, AccYangle);

        //Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up.
        //Two different pieces of code are used depending on how your IMU is mounted.
        //If IMU is upside down
        /*
        if (AccXangle >180)
            AccXangle -= (float)360.0;

        AccYangle-=90;
        if (AccYangle >180)
            AccYangle -= (float)360.0;
        */

        //If IMU is up the correct way, use these lines
        /**********************************************************
        AccXangle -= (float)180.0;
        if (AccYangle > 90)
            AccYangle -= (float)270;
        else
            AccYangle += (float)90;


        //Complementary filter used to combine the accelerometer and gyro values.
        CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
        CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;


        printf ("   GyroX  %7.3f \t AccXangle \e[m %7.3f \t \033[22;31mCFangleX %7.3f\033[0m\t 
                    GyroY  %7.3f \t AccYangle %7.3f \t \033[22;36mCFangleY %7.3f\t\033[0m\n",
                gyroXangle,AccXangle,CFangleX,gyroYangle,AccYangle,CFangleY);
        ***********************************************************/
        //Each loop should be at least 20ms.
        while(mymillis() - startInt < (DT*1000))
        {
            usleep(100);
        }

        printf("Loop Time %d\t", mymillis()- startInt);

        //End with newline:
        printf("\n");
    }
}

