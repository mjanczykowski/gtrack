/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */
 
#ifndef DCM_H
#define DCM_H

#include <math.h>
#include "Arduino.h"

class Vector
{
  public:
    float x;
    float y;
    float z;

    Vector(float x, float y, float z);
    Vector();
    
    void setXYZ(float x, float y, float z);
    
    float dot(Vector v);
    Vector cross(Vector v);
    Vector operator*(float s);
    Vector operator+(Vector v);
    Vector operator-(Vector v);
    void normalize();
    
    void printDeg();
    void print();
};

class DCM
{
  public:
    float R[3][3];
    
    DCM();
    
    DCM rotateByVector(Vector w); //w = angular_velocity * dt
    Vector getXRow();
    Vector getYRow();
    Vector getZRow();    
    void setXRow(Vector X);
    void setYRow(Vector Y);
    void setZRow(Vector Z);
    
    void normalize();
    
    Vector getYawPitchRoll();
};

#endif /* DCM_H */

