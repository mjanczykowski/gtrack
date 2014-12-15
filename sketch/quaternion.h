#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

class Quaternion
{
  public:
    float w;
    float x;
    float y;
    float z;
    Quaternion();
    Quaternion(float w, float x, float y, float z);
    //maps a rotation vector to a unit quaternion
    static Quaternion fromRotationVector(float vx, float vy, float vz);
    
    Quaternion operator*(Quaternion q);
    
    void setByAngles(float phi, float theta, float psi);
    void getPRYAngles(float *phi, float *theta, float *psi);   
    Quaternion rotateByAngularVelocity(float vx, float vy, float vz);
    void getGravity(float *gx, float *gy, float *gz);
};

#endif /* QUATERNION_H */
