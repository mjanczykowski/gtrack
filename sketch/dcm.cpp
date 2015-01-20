#include "dcm.h"

Vector::Vector(float x, float y, float z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

Vector::Vector()
{
  x = 0.;
  y = 0.;
  z = 0.;
}

void Vector::setXYZ(float x, float y, float z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

float Vector::dot(Vector v)
{
  return x * v.x + y * v.y + z * v.z;
}

Vector Vector::cross(Vector v)
{
  return Vector(
    y * v.z - z * v.y,
    z * v.x - x * v.z,
    x * v.y - y * v.x
  );
}

Vector Vector::operator*(float s)
{
  return Vector(
    x * s,
    y * s,
    z * s
  );
}

Vector Vector::operator+(Vector v)
{
  return Vector(
    x + v.x,
    y + v.y,
    z + v.z
  );
}

Vector Vector::operator-(Vector v)
{
  return Vector(
    x - v.x,
    y - v.y,
    z - v.z
  );
}

void Vector::normalize()
{
  float m = sqrt(x*x + y*y + z*z);
  x /= m;
  y /= m;
  z /= m;
}

void Vector::printDeg()
{
  Serial.print(x * RAD_TO_DEG, 6); Serial.print("\t");
  Serial.print(y * RAD_TO_DEG, 6); Serial.print("\t");
  Serial.print(z * RAD_TO_DEG, 6); Serial.print("\t");
}

void Vector::print()
{
  Serial.print(x, 6); Serial.print("\t");
  Serial.print(y, 6); Serial.print("\t");
  Serial.print(z, 6); Serial.print("\t");
}

/* ---------------- DCM --------------------------*/

DCM::DCM()
{
  R[0][0] = 1.;
  R[1][1] = 1.;
  R[2][2] = 1.;
}

DCM DCM::rotateByVector(Vector w)
{
  DCM result;

  float U[3][3] = {
    { 1., -w.z, w.y},
    { w.z, 1., -w.x},
    { -w.y, w.x, 1.}
  };
  
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      float val = 0.0;
      
      for(int k = 0; k < 3; k++)
      {
        val += R[i][k] * U[k][j];
      }
      
      result.R[i][j] = val;
    }
  }
  
  return result;
}

Vector DCM::getXRow()
{
  return Vector(R[0][0], R[0][1], R[0][2]);
}

Vector DCM::getYRow()
{
  return Vector(R[1][0], R[1][1], R[1][2]);
}

Vector DCM::getZRow()
{
  return Vector(R[2][0], R[2][1], R[2][2]);
}

void DCM::setXRow(Vector X)  
{
  R[0][0] = X.x;
  R[0][1] = X.y;
  R[0][2] = X.z;
}

void DCM::setYRow(Vector Y)  
{
  R[1][0] = Y.x;
  R[1][1] = Y.y;
  R[1][2] = Y.z;
}

void DCM::setZRow(Vector Z)  
{
  R[2][0] = Z.x;
  R[2][1] = Z.y;
  R[2][2] = Z.z;
}

void DCM::normalize()
{
  Vector X = getXRow(), Y = getYRow();
  float error = X.dot(Y);
  float sc = -error * 0.5;
  
  Vector Xorth = X + Y * sc;
  Vector Yorth = Y + X * sc;
  Vector Zorth = Xorth.cross(Yorth);
  
  Vector Xnorm = Xorth * 0.5 * (3. - Xorth.dot(Xorth));
  Vector Ynorm = Yorth * 0.5 * (3. - Yorth.dot(Yorth));
  Vector Znorm = Zorth * 0.5 * (3. - Zorth.dot(Zorth));
  
  setXRow(Xnorm);
  setYRow(Ynorm);
  setZRow(Znorm);
}

Vector DCM::getYawPitchRoll()
{
  return Vector(
    atan2(R[2][1], R[2][2]),
    -asin(R[2][0]),
    atan2(R[1][0], R[0][0])
  );
}
