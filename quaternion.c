#include "quaternion.h"

void quaternion_create(float vx, float vy, float vz, float angle, float qr, float qi, float qj, float qk)
{
  qr = cos(angle/2);
  qi = vx * sin(angle/2);
  qj = vy * sin(angle/2);
  qk = vz * sin(angle/2);
}

void quaternion_rotate( float vx, float vy ,float vz, float qr, float qi, float qj, float qk, float rx, float ry, float rz)
{
  rx = vx * (1 - 2*(pow(qj, 2) + pow(qk, 2))) + vy * 2*(qi * qj - qk * qr) + vz * 2*(qi * qk + qj * qr);
  ry = vx * 2*(qi * qj + qk * qr) + vy * (1 - 2*(pow(qi, 2) + pow(qk, 2))) + vz * 2*(qj * qk - qi * qr);
  rz = vx * 2*(qi * qk - qj * qr) + vy * 2*(qj * qk + qi * qr) + vz * (1 - 2*(pow(qi, 2) + pow(qj, 2)));
}

void quaternion_multiply(float q1r, float q1i, float q1j, float q1k, float q2r, float q2i, float q2j, float q2k, float rr, float ri, float rj, float rk)
{
  rr = q1r * q2r - q1i * q2i - q1j * q2j - q1k * q2k;
  ri = q1r * q2i + q1i * q2r + q1j * q2k - q1k * q2j;
  rj = q1r * q2j - q1i * q2k + q1j * q2r + q1k * q2i;
  rk = q1r * q2k + q1i * q2j - q1j * q2i + q1k * q2r;
}
