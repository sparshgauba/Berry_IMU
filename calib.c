#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "IMU.c"

#define DT 0.02         // [s/loop] loop period in ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573    // [deg/LSB]
#define G_GAIN 0.017*2     // [deg/s/LSB] //For 500 deg/s Precision
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846

#define THRES_A 10
#define THRES_G 10

// System constants
#define deltat 0.02f // sampling period in seconds (shown as 20 ms)
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
// Global system variables


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


float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions

float madAngles[3] = {0.0f};


//----------------------------------------------------------------------------------------
//////////              FUNCTION DECLARATIONS               //////////////////////////////
//----------------------------------------------------------------------------------------


void  INThandler(int sig);// Used to do a nice clean exit when Ctrl-C is pressed

int mymillis();

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1);

long long * calibrate_acc();

long long *calibrate_gyr();

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);

void filterUpdateAHRS(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z);

void computeAngles();


int main(int argc, char *argv[])
{
  //Raw sensor values
  int  accRaw[3];
  int  magRaw[3];
  int  gyrRaw[3];
  //Calibrated Acc Values with prev values for filtering;
  uint64_t ca_x_full = 0;
  uint64_t ca_y_full = 0;
  uint64_t ca_z_full = 0;
  //Calibrated Gyr Values
  uint64_t cg_x_full = 0;
  uint64_t cg_y_full = 0;
  uint64_t cg_z_full = 0;

  int32_t *ca_x = (int32_t *)&ca_x_full;
  int32_t *ca_y = (int32_t *)&ca_y_full;
  int32_t *ca_z = (int32_t *)&ca_z_full;

  int32_t *cg_x = (int32_t *)&cg_x_full;
  int32_t *cg_y = (int32_t *)&cg_y_full;
  int32_t *cg_z = (int32_t *)&cg_z_full;

  //Values to send to MadgwickAHRS
  float gyr_rate_rad[3] = {0.0f}; // Converted to Radians per second
  float acc_G[3] = {0.0f}; // Converted to G
  float scaledMag[3] = {0.0f}; //After hard iron and soft iron calibration


  int startInt;
  struct  timeval tvBegin, tvEnd,tvDiff;

  signal(SIGINT, INThandler);

  detectIMU();
  enableIMU();


  //Get Calibration offset
  int64_t *ca = calibrate_acc();
  int64_t *cg = calibrate_gyr();
  int G_raw = ca[3];
  printf("OffAccX: %6lld\tOffAccY: %6lld\tOffAccZ: %6lld\n", ca[0], ca[1], ca[2]);
  gettimeofday(&tvBegin, NULL);
  while(1)
    {
      startInt  = mymillis();
      //Move current values to last values;
      ca_x_full <<= 32;
      ca_y_full <<= 32;
      ca_z_full <<= 32;
      cg_x_full <<= 32;
      cg_y_full <<= 32;
      cg_z_full <<= 32;


      //Get Raw Values
      readACC(accRaw);
      readGYR(gyrRaw);
      readMAG(magRaw);


      //Subtract offset values;
      ca_x[0] = (((accRaw[0]-(int)ca[0]) + ca_x[1])/2);
      ca_y[0] = (((accRaw[1]-(int)ca[1]) + ca_y[1])/2);
      ca_z[0] = -((accRaw[2] + ca_z[1])/2);
      cg_x[0] = (((gyrRaw[0]-(int)cg[0]) + cg_x[1])/2);
      cg_y[0] = (((gyrRaw[1]-(int)cg[1]) + cg_y[1])/2);
      cg_z[0] = (((gyrRaw[2]-(int)cg[2]) + cg_z[1])/2);

      //Apply hard iron calibration
        magRaw[0] -= (magXmin + magXmax) /2 ;
        magRaw[1] -= (magYmin + magYmax) /2 ;
        magRaw[2] -= (magZmin + magZmax) /2 ;

      //Convert acc to G's
      acc_G[0] = ((float)ca_x[0])/((float)G_raw);
      acc_G[1] = ((float)ca_y[0])/((float)G_raw);
      acc_G[2] = ((float)ca_z[0])/((float)G_raw);

      //Convert gyr to Rad/s
      gyr_rate_rad[0] = (float)cg_x[0]  * G_GAIN * M_PI / 180.0f;
      gyr_rate_rad[1] = (float)cg_y[0]  * G_GAIN * M_PI / 180.0f;
      gyr_rate_rad[2] = (float)cg_z[0]  * G_GAIN * M_PI / 180.0f;

      //Apply soft iron calibration
        scaledMag[0]  = (float)(magRaw[0] - magXmin) / (magXmax - magXmin) * 2 - 1;
        scaledMag[1]  = (float)(magRaw[1] - magYmin) / (magYmax - magYmin) * 2 - 1;
        scaledMag[2]  = (float)(magRaw[2] - magZmin) / (magZmax - magZmin) * 2 - 1;

      //printf("AccX: %4d\tAccY: %4d\tAccZ: %4d\t", ca_x[0], ca_y[0], ca_z[0]);

      //printf("GyrX: %4d\tGyrY: %4d\tGyrZ: %4d\t", cg_x[0], cg_y[0], cg_z[0]);

      //printf("AccX: %5.2f\tAccY: %5.2f\tAccZ: %5.2f\t", acc_G[0], acc_G[1], acc_G[2]);

      filterUpdate(gyr_rate_rad[0], gyr_rate_rad[1], gyr_rate_rad[2], acc_G[0], acc_G[1], acc_G[2]);
      //filterUpdateAHRS(gyr_rate_rad[0], gyr_rate_rad[1], gyr_rate_rad[2], acc_G[0], acc_G[1], acc_G[2], scaledMag[0], scaledMag[1], scaledMag[2]);
      computeAngles();

      printf("Roll: %5.2f\t Pitch: %5.2f\t Yaw:Z %5.2f", madAngles[0], madAngles[1], madAngles[2]);

      //Each loop should be at least 20ms.
      while(mymillis() - startInt < (DT*1000))
        {
	  usleep(100);
        }

      printf("Loop Time %d\n", mymillis()- startInt);

    }

}


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
  int64_t *ret = malloc(4*sizeof(int64_t));
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
  ret[3] = (int64_t)sqrt( pow(ret[0],2) + pow(ret[1],2) + pow(ret[2],2));
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




void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z)
{
  // Local system variables
  float norm; // vector norm
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
  float f_1, f_2, f_3; // objective function elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
  // Axulirary variables to avoid reapeated calcualtions
  float halfSEq_1 = 0.5f * SEq_1;
  float halfSEq_2 = 0.5f * SEq_2;
  float halfSEq_3 = 0.5f * SEq_3;
  float halfSEq_4 = 0.5f * SEq_4;
  float twoSEq_1 = 2.0f * SEq_1;
  float twoSEq_2 = 2.0f * SEq_2;
  float twoSEq_3 = 2.0f * SEq_3;
  // Normalise the accelerometer measurement
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_x /= norm;
  a_y /= norm;
  a_z /= norm;
  // Compute the objective function and Jacobian
  f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
  f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
  J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * SEq_4;
  J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21; // negated in matrix multiplication
  J_33 = 2.0f * J_11or24; // negated in matrix multiplication
  // Compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
  // Normalise the gradient
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 /= norm;
  SEqHatDot_2 /= norm;
  SEqHatDot_3 /= norm;
  SEqHatDot_4 /= norm;
  // Compute the quaternion derrivative measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
  SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
  SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
  SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
  // Compute then integrate the estimated quaternion derrivative
  SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
  SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
  SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
  SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
  // Normalise quaternion
  norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
  SEq_1 /= norm;
  SEq_2 /= norm;
  SEq_3 /= norm;
  SEq_4 /= norm;
}

/*
void filterUpdateAHRS(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z)
{
    // local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
    float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
    J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
    float h_x, h_y, h_z; // computed flux in the earth frame
    // axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;
    float twoSEq_4 = 2.0f * SEq_4;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xSEq_1 = 2.0f * b_x * SEq_1;
    float twob_xSEq_2 = 2.0f * b_x * SEq_2;
    float twob_xSEq_3 = 2.0f * b_x * SEq_3;
    float twob_xSEq_4 = 2.0f * b_x * SEq_4;
    float twob_zSEq_1 = 2.0f * b_z * SEq_1;
    float twob_zSEq_2 = 2.0f * b_z * SEq_2;
    float twob_zSEq_3 = 2.0f * b_z * SEq_3;
    float twob_zSEq_4 = 2.0f * b_z * SEq_4;
    float SEq_1SEq_2;
    float SEq_1SEq_3 = SEq_1 * SEq_3;
    float SEq_1SEq_4;
    float SEq_2SEq_3;
    float SEq_2SEq_4 = SEq_2 * SEq_4;
    float SEq_3SEq_4;
    float twom_x = 2.0f * m_x;
    float twom_y = 2.0f * m_y;
    float twom_z = 2.0f * m_z;
    // normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // normalise the magnetometer measurement
    norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    m_x /= norm;
    m_y /= norm;
    m_z /= norm;
    // compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
    f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    J_41 = twob_zSEq_3; // negated in matrix multiplication
    J_42 = twob_zSEq_4;
    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
    J_64 = twob_xSEq_2;
    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
    // compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
    // compute and remove the gyroscope baises
    w_bx += w_err_x * deltat * zeta;
    w_by += w_err_y * deltat * zeta;
    w_bz += w_err_z * deltat * zeta;
    w_x -= w_bx;
    w_y -= w_by;
    w_z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // compute then integrate the estimated quaternion rate
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    // normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
    // compute flux in the earth frame
    SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
    SEq_1SEq_3 = SEq_1 * SEq_3;
    SEq_1SEq_4 = SEq_1 * SEq_4;
    SEq_3SEq_4 = SEq_3 * SEq_4;
    SEq_2SEq_3 = SEq_2 * SEq_3;
    SEq_2SEq_4 = SEq_2 * SEq_4;
    h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
}
*/


void computeAngles()
{
  madAngles[0] = atan2f(SEq_1*SEq_2 + SEq_3*SEq_4, 0.5f - SEq_2*SEq_2 - SEq_3*SEq_3);
  madAngles[1] = asinf(-2.0f * (SEq_2*SEq_4 - SEq_1*SEq_3));
  madAngles[2] = atan2f(SEq_2*SEq_3 + SEq_1*SEq_4, 0.5f - SEq_3*SEq_3 - SEq_4*SEq_4);
}



















