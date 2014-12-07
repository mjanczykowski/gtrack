#include "i2c.h"
#include "quaternion.h"
#include <math.h>
#include <inttypes.h>

#define I2C_BITRATE                         100000

#define MPU6050_I2C_ADDRESS                 0x68
#define MPU6050_IDLE_REGISTER               0x6b

#define MPU6050_ACCEL_X_AXIS_HIGH_REGISTER  0x3b
#define MPU6050_ACCEL_X_AXIS_LOW_REGISTER   0x3c
#define MPU6050_ACCEL_Y_AXIS_HIGH_REGISTER  0x3d
#define MPU6050_ACCEL_Y_AXIS_LOW_REGISTER   0x3e
#define MPU6050_ACCEL_Z_AXIS_HIGH_REGISTER  0x3f
#define MPU6050_ACCEL_Z_AXIS_LOW_REGISTER   0x40

#define MPU6050_GYRO_X_AXIS_HIGH_REGISTER   0x43
#define MPU6050_GYRO_X_AXIS_LOW_REGISTER    0x44
#define MPU6050_GYRO_Y_AXIS_HIGH_REGISTER   0x45
#define MPU6050_GYRO_Y_AXIS_LOW_REGISTER    0x46
#define MPU6050_GYRO_Z_AXIS_HIGH_REGISTER   0x47
#define MPU6050_GYRO_Z_AXIS_LOW_REGISTER    0x48

#define MPU6050_GYROSCOPE_CONTROL_REGISTER  0x1b

#define MPU6050_GYROSCOPE_SCALE_FACTOR      131.0

#define MIN_DT                              100      //in us
#define CALIBRATION_TIME                    8000000  //in us

//#define READABLE
//#define DEBUG

//Device - accel/gyro MPU-6050 (GY-521)
I2CDevice *dev;

//We send teapot packet to Processing MPU DPM demo
uint8_t teapotPacket[14] = {
  '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, '\r', '\n'};

//Time related values
unsigned long currentTime, newTime, dt;

//Angles
//float gyroAngleX = 0.0, gyroAngleY = 0.0, gyroAngleZ = 0.0;
//float compAngleX = 0.0, compAngleY = 0.0;
float accAngleX = 0.0, accAngleY = 0.0;

//angular velocities
float gyro_v_x = 0.0, gyro_v_y = 0.0, gyro_v_z = 0.0;
float acc_v_x = 0.0, acc_v_y = 0.0, acc_v_z = 0.0;
float comp_v_x = 0.0, comp_v_y = 0.0;

//Quaternion to send to computer and converted values
Quaternion q;
uint16_t w, x, y, z;

//Values from MPU
float ax, ay, az, gx, gy, gz;

//Drift
float drift_x = 0.0, drift_y = 0.0, drift_z = 0.0;

//Function to get current values from MPU
void getCurrentValuesFromMPU(float *a_x, float *a_y, float *a_z, float *g_x, float *g_y, float *g_z);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  
  dev = new I2CDevice(MPU6050_I2C_ADDRESS, I2C_BITRATE);
  dev -> writeRegister(MPU6050_IDLE_REGISTER, 0);
  dev -> writeRegister(MPU6050_GYROSCOPE_CONTROL_REGISTER, 0);

  getCurrentValuesFromMPU(&ax, &ay, &az, &gx, &gy, &gz);
  
  float prev_gz = gz;
  
  Serial.println("Calibrating...");
  
  float drift_angle_x = 0.0, drift_angle_y = 0.0, drift_angle_z = 0.0;
  
  unsigned long startTime;
  
  startTime = currentTime = micros();
  
  while((newTime = micros()) - startTime < CALIBRATION_TIME)
  {
    dt = newTime - currentTime;

    if(dt < MIN_DT) //Too small dt detection
    {
      continue;
    }
    
    getCurrentValuesFromMPU(&ax, &ay, &az, &gx, &gy, &gz);
    
    drift_angle_x += gx * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR;
    drift_angle_y += gy * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR;
    drift_angle_z += gz * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR;
    
    currentTime = newTime;
  }

  drift_x = drift_angle_x / CALIBRATION_TIME;
  drift_y = drift_angle_y / CALIBRATION_TIME;  
  drift_z = drift_angle_z / CALIBRATION_TIME;
  
  Serial.print("Done. Drift:\t");
  Serial.print(drift_z * 10000000.0);
  Serial.print("e-7\tDrift angle:\t");
  Serial.println(drift_angle_z);
  Serial.println();
  
  getCurrentValuesFromMPU(&ax, &ay, &az, &gx, &gy, &gz);
  
  #ifndef READABLE
//  gyroAngleX = gx;
//  gyroAngleY = gy;
  #endif
  
//  compAngleX = gx;
//  compAngleY = gy;

  currentTime = micros();
  newTime = 0;
}

void loop()
{
  newTime = micros();

  if(newTime < currentTime)  //overflow detection
  {
    currentTime = newTime;
    return;
  }

  dt = newTime - currentTime;

  if(dt < MIN_DT) //Too small dt detection
  {
    return;
  }

  currentTime = newTime;

  getCurrentValuesFromMPU(&ax, &ay, &az, &gx, &gy, &gz);

  float gyroDeltaX = gx * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_x * dt;
  float gyroDeltaY = gy * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_y * dt;
  float gyroDeltaZ = gz * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_z * dt;

//  gyroAngleX += gyroDeltaX;
//  gyroAngleY += gyroDeltaY;
//  gyroAngleZ += gz * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR - drift * dt;

//  gyro_v_x = gx / MPU6050_GYROSCOPE_SCALE_FACTOR;
//  gyro_v_y = gy / MPU6050_GYROSCOPE_SCALE_FACTOR;
//  gyro_v_z = gz / MPU6050_GYROSCOPE_SCALE_FACTOR;

  accAngleX = atan2(ay, az) * RAD_TO_DEG;
  accAngleY = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  
#ifdef DEBUG
  Serial.print(accAngleX); 
  Serial.print("\t");
  Serial.println(accAngleY);
#endif

//  compAngleX = 0.96 * (compAngleX + gyroDeltaX) + 0.04 * accAngleX; // Calculate the angle using a Complimentary filter
//  compAngleY = 0.96 * (compAngleY + gyroDeltaY) + 0.04 * accAngleY;

  //  q.setByAngles(gyroAngleX, gyroAngleY, gyroAngleZ);
//  q.setByAngles(compAngleX, compAngleY, gyroAngleZ);
  q = q.rotateByAngularVelocity(gyroDeltaX * DEG_TO_RAD, gyroDeltaY * DEG_TO_RAD, gyroDeltaZ * DEG_TO_RAD);
  
  float tempx, tempy, tempz;
  q.getAngles(&tempx, &tempy, &tempz);
  
#ifdef DEBUG

  Serial.print(tempx); Serial.print("\t"); Serial.print(tempy); Serial.print("\t"); Serial.print(tempz); Serial.print("\n\n");
  
#endif

#ifdef READABLE

#ifdef DEBUG

  Serial.print(ax); 
  Serial.print("\t");
  Serial.print(ay); 
  Serial.print("\t");
  Serial.print(az); 
  Serial.print("\t");
  Serial.print(gx); 
  Serial.print("\t");
  Serial.print(gy); 
  Serial.print("\t");
  Serial.print(gz); 
  Serial.print("\n");

#endif //DEBUG

  Serial.print(gyroDeltaX); 
  Serial.print("\t");
  Serial.print(gyroDeltaY); 
  Serial.print("\t");
  Serial.print(gyroDeltaZ); 
  Serial.println("\t");

//  Serial.print(q.w); 
//  Serial.print("\t");
//  Serial.print(q.x); 
//  Serial.print("\t");
//  Serial.print(q.y); 
//  Serial.print("\t");
//  Serial.print(q.z); 
//  Serial.print("\n");

#else

  w = q.w * 16384.0;
  x = q.x * 16384.0;
  y = q.y * 16384.0;
  z = q.z * 16384.0;

  teapotPacket[2] = ((uint8_t)(w >> 8));
  teapotPacket[3] = ((uint8_t)(w & 0xFF));
  teapotPacket[4] = ((uint8_t)(x >> 8));
  teapotPacket[5] = ((uint8_t)(x & 0xFF));
  teapotPacket[6] = ((uint8_t)(y >> 8));
  teapotPacket[7] = ((uint8_t)(y & 0xFF));
  teapotPacket[8] = ((uint8_t)(z >> 8));
  teapotPacket[9] = ((uint8_t)(z & 0xFF));
  
#ifndef DEBUG

  Serial.write(teapotPacket, 14);

#else

  Serial.print(q.w * 16384.0); 
  Serial.print("\t");
  Serial.print(w); 
  Serial.print("\t");
  Serial.print(teapotPacket[2]); 
  Serial.print("\t");
  Serial.print(teapotPacket[3]); 
  Serial.print("\n");
  Serial.print((((uint16_t)teapotPacket[2])<<8) + teapotPacket[3]); 
  Serial.print("\n");
  Serial.print("------------------------------------------------------------------\n");
  delay(10);

#endif //DEBUG

  teapotPacket[11]++;

#endif //READABLE
}

void getCurrentValuesFromMPU(float *a_x, float *a_y, float *a_z, float *g_x, float *g_y, float *g_z){
  int8_t ax_h = dev -> readRegister(MPU6050_ACCEL_X_AXIS_HIGH_REGISTER);
  int8_t ax_l = dev -> readRegister(MPU6050_ACCEL_X_AXIS_LOW_REGISTER);
  int16_t ax = (((int16_t)ax_h) << 8) + ax_l;

  int8_t ay_h = dev -> readRegister(MPU6050_ACCEL_Y_AXIS_HIGH_REGISTER);
  int8_t ay_l = dev -> readRegister(MPU6050_ACCEL_Y_AXIS_LOW_REGISTER);
  int16_t ay = (((int16_t)ay_h) << 8) + ay_l;

  int8_t az_h = dev -> readRegister(MPU6050_ACCEL_Z_AXIS_HIGH_REGISTER);
  int8_t az_l = dev -> readRegister(MPU6050_ACCEL_Z_AXIS_LOW_REGISTER);
  int16_t az = (((int16_t)az_h) << 8) + az_l;

  int8_t gx_h = dev -> readRegister(MPU6050_GYRO_X_AXIS_HIGH_REGISTER);
  int8_t gx_l = dev -> readRegister(MPU6050_GYRO_X_AXIS_LOW_REGISTER);
  int16_t gx = (((int16_t)gx_h) << 8) + gx_l;

  int8_t gy_h = dev -> readRegister(MPU6050_GYRO_Y_AXIS_HIGH_REGISTER);
  int8_t gy_l = dev -> readRegister(MPU6050_GYRO_Y_AXIS_LOW_REGISTER);
  int16_t gy = (((int16_t)gy_h) << 8) + gy_l;

  int8_t gz_h = dev -> readRegister(MPU6050_GYRO_Z_AXIS_HIGH_REGISTER);
  int8_t gz_l = dev -> readRegister(MPU6050_GYRO_Z_AXIS_LOW_REGISTER);
  int16_t gz = (((int16_t)gz_h) << 8) + gz_l;
  
  (*a_x) = (float) ax;
  (*a_y) = (float) ay;
  (*a_z) = (float) az;
  (*g_x) = (float) gx;
  (*g_y) = (float) gy;
  (*g_z) = (float) gz;
}
