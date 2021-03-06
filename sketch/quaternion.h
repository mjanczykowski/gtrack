/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */
 
#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include "Arduino.h"

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
    static Quaternion fromThetaAndVector(float theta, float x, float y, float z);
    
    Quaternion operator*(Quaternion q);
    static Quaternion slerp(Quaternion q, Quaternion p, float t);
    
    void setByAngles(float phi, float theta, float psi);
    void getAngles(float *phi, float *theta, float *psi);
    void getYawPitchRoll(float *yaw, float *pitch, float *roll);
    void getPRYAngles(float *phi, float *theta, float *psi);
    Quaternion rotateByAngles(float vx, float vy, float vz);
    void getGravity(float *gx, float *gy, float *gz);
    void normalize();
    void printQuaternion(char *text, int enter);
    void setValues(float w, float x, float y, float z);
};

#endif /* QUATERNION_H */  

