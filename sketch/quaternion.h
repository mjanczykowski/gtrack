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
    
    void setByAngles(float phi, float theta, float psi);  //Euler
    void getAngles(float *phi, float *theta, float *psi); //Euler   
    Quaternion rotateByAngularVelocity(float vx, float vy, float vz);
};

#endif /* QUATERNION_H */
