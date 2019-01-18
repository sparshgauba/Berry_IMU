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

#define DT 0.02         // [s/loop] loop period in ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573    // [deg/LSB]
#define G_GAIN 0.017     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846

#define THRES_A 30
#define THRES_G 30


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


int accOff[4];
int gyrOff[3];
int accRaw[3];
int gyrRaw[3];
int magRaw[3];


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
void calibrate_acc()
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
    accOff[0] = (int)(ret[0]/100);
    accOff[1] = (int)(ret[1]/100);
    accOff[2] = (int)(ret[2]/100);
    //ret[3] is the calculated raw value that corresponds to 1 G
    accOff[3] = (long long)sqrt( pow(ret[0],2) + pow(ret[1],2) + pow(ret[2],2));
}

// Returns offests for raw gyr values
void calibrate_gyr(int gyr[])
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
    gyrOff[0] = (int)(ret[0]/100);
    gyrOff[1] = (int)(ret[1]/100);
    gyrOff[2] = (int)(ret[2]/100);

}





void getACC(int acc[], uint n)
{
    acc[0] = acc[1] = acc[2] = 0;
    for (int i = 0; i < n; i++)
    {
      readACC(accRaw);
      acc[0] += accRaw[0];
      acc[1] += accRaw[1];
      acc[2] += accRaw[2];
      //usleep(5000);                                                                                                                                         
    }
    acc[0]/=n;
    acc[1]/=n;
    acc[2]/=n;
    acc[0]-=accOff[0];
    acc[1]-=accOff[1];
    acc[2]-=accOff[2];
}

void getGYR(int gyr[], uint n)
{
    gyr[0] = gyr[1] = gyr[2] = 0;
    for (int i = 0; i < n; i++)
    {
      readGYR(gyrRaw);
      gyr[0] += gyrRaw[0];
      gyr[1] += gyrRaw[1];
      gyr[2] += gyrRaw[2];
      //usleep(5000);                                                                                                                                         
    }
    gyr[0]/=n;
    gyr[1]/=n;
    gyr[2]/=n;
    gyr[0]-=gyrOff[0];
    gyr[1]-=gyrOff[1];
    gyr[2]-=gyrOff[2];
}

void getMAG(int mag[], uint n)
{
    mag[0] = mag[1] = mag[2] = 0;
    for (int i = 0; i < n; i++)
    {
      readMAG(magRaw);
      mag[0] += magRaw[0];
      mag[1] += magRaw[1];
      mag[2] += magRaw[2];
      //usleep(5000);                                                                                                                                         
    }
    mag[0]/=n;
    mag[1]/=n;
    mag[2]/=n;
}




int main(int argc, char *argv[])
{
    int acc[3];
    int gyr[3];
    int mag[3];
    //Raw sensor values


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
    calibrate_acc(accOff);
    calibrate_gyr(gyrOff);
    float G_raw = accOff[4];

    gettimeofday(&tvBegin, NULL);


    while(1)
    {
        startInt = mymillis();

        //Slide last values back
        //read ACC and GYR data

        getACC(acc,5);
        getGYR(gyr,5);
        getMAG(mag,5);

        //Apply hard iron calibration
        mag[0] -= (magXmin + magXmax) /2 ;
        mag[1] -= (magYmin + magYmax) /2 ;
        mag[2] -= (magZmin + magZmax) /2 ;

        //Apply soft iron calibration
        scaledMag[0]  = (float)(mag[0] - magXmin) / (magXmax - magXmin) * 2 - 1;
        scaledMag[1]  = (float)(mag[1] - magYmin) / (magYmax - magYmin) * 2 - 1;
        scaledMag[2]  = (float)(mag[2] - magZmin) / (magZmax - magZmin) * 2 - 1;

        //Convert to G values
        acc_G[0] = ((float)acc[0]/G_raw);
        acc_G[1] = ((float)acc[0]/G_raw);
        acc_G[2] = ((float)acc[0]/G_raw);

        //Convert Gyro raw to RADIANS per second
        gyr_rate_rad[0] = (float) gyr[0]  * G_GAIN * M_PI / 180;
        gyr_rate_rad[1] = (float) gyr[0]  * G_GAIN * M_PI / 180;
        gyr_rate_rad[2] = (float) gyr[0]  * G_GAIN * M_PI / 180;

        //Print Acc Values after Calibration
        //printf("AccX: %4d\tAccY: %4d\tAccZ: %4d\t", ca_x, ca_y, ca_z);

        //Print Gyr Values after Calibration
        //printf("GyrX: %4d\tGyrY: %4d\tGyrZ: %4d\t", cg_x, cg_y, cg_z);

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

