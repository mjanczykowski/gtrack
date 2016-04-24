/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */
 
 #include "quaternion.h"

Quaternion::Quaternion()
{
  w = 1.0f;
  x = 0.0f;
  y = 0.0f;
  z = 0.0f; 
}

Quaternion Quaternion::fromThetaAndVector(float theta, float x, float y, float z)
{
  float m = sqrt(x*x + y*y + z*z), sintheta2 = sin(theta/2.0);
  
  float aw = cos(theta/2.0);
  float ax = x / m * sintheta2;
  float ay = y / m * sintheta2;
  float az = z / m * sintheta2;
  
  return Quaternion(aw, ax, ay, az);
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
  Quaternion qx(
    cos(vx * 0.5),
    sin(vx * 0.5),
    0,
    0
  );
  
  Quaternion qy(
    cos(vy * 0.5),
    0,
    sin(vy * 0.5),
    0
  );
  
  Quaternion qz(
    cos(vz * 0.5),
    0,
    0,
    sin(vz * 0.5)
  );
  
  return qx * qy * qz;
}

void Quaternion::setByAngles(float pitch, float roll, float yaw)
{
  //translation to rad/2
  float a = roll * M_PI / 360.0;  // Phi / 2.0
  float b = pitch * M_PI / 360.0; // Theta / 2.0
  float c = yaw * M_PI / 360.0;   // Psi / 2.0

  float c1 = cos(a), c2 = cos(b), c3 = cos(c);
  float s1 = sin(a), s2 = sin(b), s3 = sin(c);

  w = c1*c2*c3 - s1* s2*s3;
  x = c1*c3*s2 - s1*c2*s3;
  y = c1*s2*s3 + c2*c3*s1;
  z = c1*c2*s3 + s1*c3*s2;
}

void Quaternion::getAngles(float *phi, float *theta, float *psi)
{
  (*theta) = atan2(-2*x*z + 2*w*y, z*z - y*y - x*x + w*w);
  (*phi) = asin(2*y*z + 2*w*x);
  (*psi) = atan2(-2*x*y + 2*w*z, y*y - z*z + w*w - x*x);
  
  (*phi) *= RAD_TO_DEG;
  (*theta) *= RAD_TO_DEG;
  (*psi) *= RAD_TO_DEG;
}

void Quaternion::getGravity(float *gx, float *gy, float *gz)
{
  *gx = 2 * (x*z - w*y);
  *gy = 2 * (w*x + y*z);
  *gz = w*w - x*x - y*y + z*z;
}
    
void Quaternion::getYawPitchRoll(float *yaw, float *pitch, float *roll)
{
//  *yaw = -atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
//  *pitch = atan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z);
//  *roll = asin(-2.0 * (x * z - w * y));
  *pitch = atan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z);
  *roll = -asin(2.0 * (x * z - w * y));
  *yaw = atan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z);
}


void Quaternion::getPRYAngles(float *phi, float *theta, float *psi)
{
  float gx, gy, gz;
  getGravity(&gx, &gy, &gz);
  // yaw: (about Z axis)
  *psi = atan2(2*x*y - 2*w*z, 2*w*w + 2*x*x - 1);
  // pitch: (nose up/down, about Y axis)
  *theta = atan(gx / sqrt(gy*gy + gz*gz));
  // roll: (tilt left/right, about X axis)
  *phi = atan(gy / sqrt(gx*gx + gz*gz));
   
  (*phi) *= RAD_TO_DEG;
  (*theta) *= RAD_TO_DEG;
  (*psi) *= RAD_TO_DEG;
}

Quaternion Quaternion::operator*(Quaternion q)
{
  return Quaternion(
    w*q.w - x*q.x - y*q.y - z*q.z,  // new w
    w*q.x + x*q.w + y*q.z - z*q.y,  // new x
    w*q.y - x*q.z + y*q.w + z*q.x,  // new y (+ + -)?
    w*q.z + x*q.y - y*q.x + z*q.w); // new z (- + +)?
}

/* spherical interpolation between q and p by amount of t of range (0, 1)
 * none of input quaternions can be zero-length */
Quaternion Quaternion::slerp(Quaternion q, Quaternion p, float t)
{
  float cosHalfAngle = q.w * p.w + q.x * p.x + q.y * p.y + q.z * p.z;

  if (cosHalfAngle >= 1.0f || cosHalfAngle <= -1.0f)
  {
    // angle = 0.0f, so just return q.
    return q;
  }
  else if (cosHalfAngle < 0.0f)
  {
    p.w = -p.w;
    p.x = -p.x;
    p.y = -p.y;
    p.z = -p.z;
    cosHalfAngle = -cosHalfAngle;
  }

  float q_weight, p_weight;

  if(cosHalfAngle < 0.99f)
  {
    float halfAngle = acos(cosHalfAngle);
    float sinHalfAngle = sin(halfAngle);
    float oneOverSinHalfAngle = 1.0f / sinHalfAngle;
    q_weight = sin(halfAngle * (1.0f - t)) * oneOverSinHalfAngle;
    p_weight = sin(halfAngle * t) * oneOverSinHalfAngle;
  }
  else
  {
    q_weight = 1.0f - t;
    p_weight = t;
  }

  Quaternion result(
    q_weight * q.w + p_weight * p.w,
    q_weight * q.x + p_weight * p.x,
    q_weight * q.y + p_weight * p.y,
    q_weight * q.z + p_weight * p.z
  );

  result.normalize();

  return result;
}

Quaternion Quaternion::rotateByAngles(float vx, float vy, float vz)
{
  Quaternion q_rot = fromRotationVector(vx, vy, vz);
  return (*this) * q_rot;
}

void Quaternion::normalize()
{
  float m = sqrt(w*w + x*x + y*y + z*z);
  w /= m;
  x /= m;
  y /= m;
  z /= m;
}

void Quaternion::printQuaternion(char *text, int enter)
{
  Serial.print("Quaternion ");
  Serial.print(text); Serial.print(": w=");
  Serial.print(w); Serial.print("\tx=");
  Serial.print(x); Serial.print("\ty=");
  Serial.print(y); Serial.print("\tz=");
  Serial.print(z); 
  if(enter){
    Serial.print("\n");
  } else {
    Serial.print("\t");
  }
}

void Quaternion::setValues(float w, float x, float y, float z) {
  this -> w = w;
  this -> x = x;
  this -> y = y;
  this -> z = z;
}

