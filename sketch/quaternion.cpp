#include "quaternion.h"

Quaternion::Quaternion()
{
  w = 1.0f;
  x = 0.0f;
  y = 0.0f;
  z = 0.0f; 
}

Quaternion::Quaternion(float w, float x, float y, float z)
{
  this->w = w;
  this->x = x;
  this->y = y;
  this->z = z;
}

Quaternion Quaternion::fromRotationVector(float vx, float vy, float vz)
{
  float v_norm = sqrt(vx*vx + vy*vy + vz*vz);
  
  float s = sin(v_norm * 0.5) / v_norm;
  
  return Quaternion(
    cos(v_norm * 0.5),
    vx * s,
    vy * s,
    vz * s
  );
}
   
void Quaternion::setByAngles(float phi, float theta, float psi)
{
  //translation to rad/2
  float a = phi * M_PI / 360.0;  // Phi / 2.0
  float b = theta * M_PI / 360.0; // Theta / 2.0
  float c = psi * M_PI / 360.0;   // Psi / 2.0
  
  w = cos(a)*cos(b)*cos(c) + sin(a)*sin(b)*sin(c);
  x = sin(a)*cos(b)*cos(c) - cos(a)*sin(b)*sin(c);
  y = cos(a)*sin(b)*cos(c) + sin(a)*cos(b)*sin(c);
  z = cos(a)*cos(b)*sin(c) - sin(a)*sin(b)*cos(c);
}
    
void Quaternion::getAngles(float *phi, float *theta, float *psi)
{
  (*phi) = atan(2.0 * (w * x + y * z) / (1.0 - 2.0 * (x*x + y*y)));
  (*theta) = asin(2.0 * (w * y - z * x));
  (*psi) = atan(2.0 * (w * z + x * y) / (1.0 - 2.0 * (y*y + z*z)));
   
  (*phi) *= (180.0 / M_PI);
  (*theta) *= (180.0 / M_PI);
  (*psi) *= (180.0 / M_PI);
}

Quaternion Quaternion::operator*(Quaternion q)
{
  return Quaternion(
    w*q.w - x*q.x - y*q.y - z*q.z,  // new w
    w*q.x + x*q.w + y*q.z - z*q.y,  // new x
    w*q.y - x*q.z + y*q.w + z*q.x,  // new y (+ + -)?
    w*q.z + x*q.y - y*q.x + z*q.w); // new z (- + +)?
}

Quaternion Quaternion::rotateByAngularVelocity(float vx, float vy, float vz)
{
  Quaternion q_rot = fromRotationVector(vx, vy, vz);
  return q_rot * (*this);
}
