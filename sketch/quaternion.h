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
    void setByAngles(float phi, float theta, float psi);
    void getAngles(float *phi, float *theta, float *psi);
};

#endif /* QUATERNION_H */
