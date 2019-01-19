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
//#include "MadgwickAHRS.h"
#include <sys/time.h>
#include "MahonyAHRS.h"

#define DT 0.006         // [s/loop] loop period in ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573    // [deg/LSB]
#define G_GAIN 0.017     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846

#define THRES_A 10
#define THRES_G 10


///////////////////////////////////////////////////////
///////////////MODIFY FOR EVERY USE////////////////////
///////////////////////////////////////////////////////
//Mag Calibration Values
#define magXmax 2008
#define magYmax 1493
#define magZmax 886
#define magXmin -374
#define magYmin -707
#define magZmin -1243
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

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


// Returns the offets for raw acc values
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


// Returns offests for raw gyr values
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
    //Raw sensor values
    int  accRaw[3];
    int  magRaw[3];
    int  gyrRaw[3];


     //Calibrated Acc Values with prev values for filtering;
    int ca_x[2] = {0};
    int ca_y[2] = {0};
    int ca_z[2] = {0};

    //Calibrated Gyr Values
    int cg_x[2] = {0};
    int cg_y[2] = {0};
    int cg_z[2] = {0};


    //Values to send to MadgwickAHRS
    float gyr_rate_rad[3] = {0}; // Converted to Radians per second
    float acc_G[3] = {0}; // Converted to G
    float scaledMag[3] = {0}; //After hard iron and soft iron calibration


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

        // If new val is less than THRES apart from last, keep last value
        // Otherwise, new val is the average of the cur and prev
        ca_x[0] = abs(ca_x[0]) < THRES_A ? 0 : (int)(ca_x[1] + ca_x[0]) / 2;
        ca_y[0] = abs(ca_y[0]) < THRES_A ? 0 : (int)(ca_y[1] + ca_y[0]) / 2;
        ca_z[0] = abs(ca_z[0]) < THRES_A ? 0 : (int)(ca_z[1] + ca_z[0]) / 2;
        cg_x[0] = abs(cg_x[0]) < THRES_G ? 0 : (int)(cg_x[1] + cg_x[0]) / 2;
        cg_y[0] = abs(cg_y[0]) < THRES_G ? 0 : (int)(cg_y[1] + cg_y[0]) / 2;
        cg_z[0] = abs(cg_z[0]) < THRES_G ? 0 : (int)(cg_z[1] + cg_z[0]) / 2;

        //Apply hard iron calibration
        magRaw[0] -= (magXmin + magXmax) /2 ;
        magRaw[1] -= (magYmin + magYmax) /2 ;
        magRaw[2] -= (magZmin + magZmax) /2 ;

        //Apply soft iron calibration
        scaledMag[0]  = (float)(magRaw[0] - magXmin) / (magXmax - magXmin) * 2 - 1;
        scaledMag[1]  = (float)(magRaw[1] - magYmin) / (magYmax - magYmin) * 2 - 1;
        scaledMag[2]  = (float)(magRaw[2] - magZmin) / (magZmax - magZmin) * 2 - 1;

        //Convert to G values
        acc_G[0] = ((float)ca_x[0]/G_raw);
        acc_G[1] = ((float)ca_y[0]/G_raw);
        acc_G[2] = ((float)ca_z[0]/G_raw);

        //Convert Gyro raw to RADIANS per second
        gyr_rate_rad[0] = (float) cg_x[0]  * G_GAIN * M_PI / 180;
        gyr_rate_rad[1] = (float) cg_y[0]  * G_GAIN * M_PI / 180;
        gyr_rate_rad[2] = (float) cg_z[0]  * G_GAIN * M_PI / 180;

        //Print Acc Values after Calibration
        //printf("AccX: %5d\tAccY: %5d\tAccZ: %5d\t", ca_x[0], ca_y[0], ca_z[0]);

        //Print Gyr Values after Calibration
        //printf("GyrX: %5d\tGyrY: %5d\tGyrZ: %5d\t", cg_x[0], cg_y[0], cg_z[0]);

        //Print Raw Mag Values
        //printf("MagX: %4d\tMagY: %4d\tMagZ: %4d\t", magRaw[0], magRaw[1], magRaw[2]);

        //Print G values
        //printf("AccX: %5.2f\tAccY: %5.2f\tAccZ: %5.2f\t", acc_G[0], acc_G[1], acc_G[2]);

        //PRINT GYR DEG PER SEC
        //printf("GyrX: %5.3f\tGyrY: %5.3f\tGyrZ: %5.3f\t", gyr_rate_rad[0], gyr_rate_rad[1], gyr_rate_rad[2]);



	//Madgwick Filter with magneto fused
	//MadgwickAHRSupdate(gyr_rate_rad[0], gyr_rate_rad[1], gyr_rate_rad[2], acc_G[0], acc_G[1], acc_G[2], scaledMag[0], scaledMag[1], scaledMag[2]);
        //MahonyAHRSupdate(gyr_rate_rad[0], gyr_rate_rad[1], gyr_rate_rad[2], acc_G[0], acc_G[1], acc_G[2], scaledMag[0], scaledMag[1], scaledMag[2]);

	//Without magneto
	//MadgwickAHRSupdateIMU(gyr_rate_rad[0], gyr_rate_rad[1], gyr_rate_rad[2], acc_G[0], acc_G[1], acc_G[2]);
        MahonyAHRSupdateIMU(gyr_rate_rad[0], gyr_rate_rad[1], gyr_rate_rad[2], acc_G[0], acc_G[1], acc_G[2]);
	computeAngles();

	//printf("q0: %5.3f   q1: %5.3f   q2: %5.3f   q3: %5.3f\t", q0, q1, q2, q3); 
	printf("Roll: %8.3f    Pitch: %8.3f    Yaw %8.3f\t", madAngles[0], madAngles[1], madAngles[2]);



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

