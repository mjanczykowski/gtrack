#include "i2c.h"
#include "quaternion.h"
#include "kalman_filter.h"
#include "dcm.h"
#include <math.h>
#include <inttypes.h>

#define I2C_BITRATE                         200000

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

#define MIN_DT                              100   //in us
#define CALIBRATION_TIME                    8000000  //in us

#define KALMAN_Q_ANGLE                      0.001
#define KALMAN_Q_GYROBIAS                   0.003
#define KALMAN_R_ANGLE                      0.03

//correction consts
#define W_RP                                1.0
#define W_Y                                 0.1
#define K_P                                 1.0
#define K_I                                 0.0

//magnetometer
#define HMC5883L_I2C_ADDRESS                0x1e
#define HMC5883L_RA_CONFIG_A                0x00
#define HMC5883L_RA_CONFIG_B                0x01
#define HMC5883L_RA_MODE                    0x02

#define HMC5883L_RA_DATAX_H                 0x03
#define HMC5883L_RA_DATAX_L                 0x04
#define HMC5883L_RA_DATAZ_H                 0x05
#define HMC5883L_RA_DATAZ_L                 0x06
#define HMC5883L_RA_DATAY_H                 0x07
#define HMC5883L_RA_DATAY_L                 0x08

#define HMC5883L_OFFSET_X                   -122.5
#define HMC5883L_OFFSET_Y                   226.5
#define HMC5883L_OFFSET_Z                   46.5
#define HMC5883L_SPREAD_X                   765.0
#define HMC5883L_SPREAD_Y                   785.0
#define HMC5883L_SPREAD_Z                   793.0

#define READABLE
//#define DEBUG

//Device - accel/gyro MPU-6050 (GY-521)
I2CDevice *dev, *mgn;

//Kalman filters - X and Y axis
KalmanFilter *kalmanX, *kalmanY;

//We send teapot packet to Processing MPU DPM demo
uint8_t teapotPacket[14] = {
  '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, '\r', '\n'};

//Time related values
unsigned long currentTime, newTime, dt;

//Angles
//float gyroAngleX = 0.0, gyroAngleY = 0.0, gyroAngleZ = 0.0;
float compAngleX = 0.0, compAngleY = 0.0;
float accAngleX = 0.0, accAngleY = 0.0;

//angular velocities
float gyro_v_x = 0.0, gyro_v_y = 0.0, gyro_v_z = 0.0;
float acc_v_x = 0.0, acc_v_y = 0.0, acc_v_z = 0.0;
float comp_v_x = 0.0, comp_v_y = 0.0;

//Quaternion to send to computer and converted values
Quaternion q, q2;
uint16_t w, x, y, z;

//DCM
DCM R, R_aux;

Vector rollPitchCorrectionPlane, totalCorrection, angVel_Pcorr, angVel_Icorr(0., 0., 0.), angVel_corr, rotation, angVel;

int i = 0;
long time = 0;

//Values from MPU
float ax, ay, az, gx, gy, gz;

//Values from magnetometer
float mx, my, mz;

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
  
  mgn = new I2CDevice(HMC5883L_I2C_ADDRESS, I2C_BITRATE);
  mgn -> writeRegister(HMC5883L_RA_CONFIG_A, 0x70);
  mgn -> writeRegister(HMC5883L_RA_CONFIG_B, 0x20);
  mgn -> writeRegister(HMC5883L_RA_MODE, 0x01);

  //gyro calibration
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

  drift_x = drift_angle_x / CALIBRATION_TIME * 1000000.0;
  drift_y = drift_angle_y / CALIBRATION_TIME * 1000000.0;  
  drift_z = drift_angle_z / CALIBRATION_TIME * 1000000.0;

  Serial.print("Done.\nOX: Drift:\t");
  Serial.print(drift_x);
  Serial.print("\tDrift angle:\t");
  Serial.println(drift_angle_x);
  Serial.print("OY: Drift:\t");
  Serial.print(drift_y);
  Serial.print("\tDrift angle:\t");
  Serial.println(drift_angle_y);
  Serial.print("OZ: Drift:\t");
  Serial.print(drift_z);
  Serial.print("\t\tDrift angle:\t");
  Serial.println(drift_angle_z);
  Serial.println();

  getCurrentValuesFromMPU(&ax, &ay, &az, &gx, &gy, &gz);

  currentTime = micros();
  newTime = 0;

  time = micros();
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
  getCurrentValuesFromMagnetometer(&mx, &my, &mz);
  
//  float gyroDeltaX = gx * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_x * dt;
//  float gyroDeltaY = gy * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_y * dt;
//  float gyroDeltaZ = gz * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_z * dt;

  float gyroAngVel_X = (gx / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_x) * DEG_TO_RAD;
  float gyroAngVel_Y = (gy / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_y) * DEG_TO_RAD;
  float gyroAngVel_Z = (gz / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_z) * DEG_TO_RAD;

  angVel.setXYZ(gyroAngVel_X, gyroAngVel_Y, gyroAngVel_Z);
  rotation = angVel * (dt / 1000000.0);
//  rotation.setXYZ(gyroDeltaX * DEG_TO_RAD, gyroDeltaY * DEG_TO_RAD, gyroDeltaZ * DEG_TO_RAD);

  R_aux = R.rotateByVector(rotation);
  R_aux.normalize();
  
  //correction
  Vector acc(ax, ay, az);
  acc.normalize();
  
  rollPitchCorrectionPlane = R_aux.getZRow().cross(acc);
  
  totalCorrection = rollPitchCorrectionPlane * W_RP; // + W_Y * yawCorrection 
  angVel_Pcorr = totalCorrection * K_P;
  angVel_Icorr = angVel_Icorr + totalCorrection * (K_I * (dt / 1000000.0));
  angVel_corr = angVel_Pcorr + angVel_Icorr;
  
  angVel = angVel - angVel_corr;
  rotation = angVel * (dt / 1000000.0);
  
  R = R.rotateByVector(rotation);
  R.normalize();
  
  //  gyroAngleX += gyroDeltaX;
  //  gyroAngleY += gyroDeltaY;
  //  gyroAngleZ += gz * dt / 1000000.0 / MPU6050_GYROSCOPE_SCALE_FACTOR - drift * dt;

//  gyro_v_x = gx / MPU6050_GYROSCOPE_SCALE_FACTOR;
//  gyro_v_y = gy / MPU6050_GYROSCOPE_SCALE_FACTOR;
  //  gyro_v_z = gz / MPU6050_GYROSCOPE_SCALE_FACTOR;

  //  accAngleX = atan2(-ay, -az) * RAD_TO_DEG;
  //  accAngleY = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

//  accAngleX = atan2(ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG;
//  accAngleY = atan2(-ax, az) * RAD_TO_DEG;



  //float accTheta = acos(-az);


#ifdef DEBUG
  Serial.print(accAngleX); 
  Serial.print("\t");
  Serial.println(accAngleY);
#endif

  float yaw, pitch, roll;

  //q is previous value updated with gyro readouts
  //  q.getAngles(&pitch, &roll, &yaw);

  //  compAngleX = 0.96 * (pitch + gyroDeltaX) + 0.04 * accAngleX; // Calculate the angle using a Complimentary filter
  //  compAngleY = 0.96 * (roll + gyroDeltaY) + 0.04 * accAngleY;

  //  q = q.rotateByAngles(gyroDeltaX * DEG_TO_RAD, gyroDeltaY * DEG_TO_RAD, gyroDeltaZ * DEG_TO_RAD);

  //  q.getAngles(&pitch, &roll, &yaw); //is it necessary? just need updated yaw

  //q.printQuaternion("q");

  //q2 is current rotation from acc, the Z axis is taken from q
  //  q2.setByAngles(accAngleX, accAngleY, yaw);
  //  q2=q2.rotateByAngles(accAngleX * DEG_TO_RAD, accAngleY * DEG_TO_RAD, yaw * DEG_TO_RAD);

  float azz = az;

  if(az < -17128.0){
    azz = - 17128.0; 
  }
  if(az > 17128.0){
    azz = 17128.0;
  }

  float theta = acos(-azz/17128.0);
  float vx = -ay/17128.0, vy = ax/17128.0, vz = 0;

  Vector ypr = R.getYawPitchRoll();

  q.setByAngles(ypr.x * RAD_TO_DEG, ypr.y * RAD_TO_DEG, ypr.z * RAD_TO_DEG);

  //  q2 = Quaternion::fromThetaAndVector(theta, vx, vy, vz);

  //complimentary filter - weighted average of both results: 
  //  q=Quaternion::average(q, 0.99, q2, 0.01);
  //  q2 = Quaternion::fromRotationVector(pitch * DEG_TO_RAD, roll * DEG_TO_RAD, yaw * DEG_TO_RAD);

#ifdef READABLE

  if(i % 11 == 10) {
    i = 0;
  } 
  else {
    i++;
    return;
  }
  
//  R.getXRow().print();
//  R.getYRow().print();
//  R.getZRow().print();
//  ypr.printDeg();

  /*Serial.print("mag:\t");
  Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t");
  Serial.print(mz); Serial.print("\t");

  float heading = atan2(my, mx);
  if(heading < 0)
    heading += 2 * M_PI;
  Serial.print("heading:\t");
  Serial.println(heading * 180/M_PI);*/
  
//  acc.print();

  /*Serial.print(gyroDeltaX, 8); 
   Serial.print("\t");
   Serial.print(gyroDeltaY, 8); 
   Serial.print("\t");
   Serial.print(gyroDeltaZ, 8); 
   Serial.print("\t");
   */
  /*Serial.print(pitch, 8); 
   Serial.print("\t");
   Serial.print(roll, 8); 
   Serial.print("\t");
   Serial.print(yaw, 8); 
   Serial.print("\t");
   Serial.print(ax/17128.0, 8); 
   Serial.print("\t");
   Serial.print(ay/17128.0, 8); 
   Serial.print("\t");
   Serial.print(az/17128.0, 8); 
   Serial.print("\t");
   Serial.print(acos(-azz/17128.0)); 
   Serial.print("\t");*/

  /*Serial.print(accAngleX, 8);
   Serial.print("\t");
   Serial.print(accAngleY, 8);
   Serial.print("\t");*/

  Serial.print("\n");

  time = micros();

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

  Serial.print(q.w * 16384.0, 8); 
  Serial.print("\t");
  Serial.print(w, 8); 
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
  int16_t ax = (((int16_t)ax_h) << 8) | (ax_l & 0xff);

  int8_t ay_h = dev -> readRegister(MPU6050_ACCEL_Y_AXIS_HIGH_REGISTER);
  int8_t ay_l = dev -> readRegister(MPU6050_ACCEL_Y_AXIS_LOW_REGISTER);
  int16_t ay = (((int16_t)ay_h) << 8) | (ay_l & 0xff);

  int8_t az_h = dev -> readRegister(MPU6050_ACCEL_Z_AXIS_HIGH_REGISTER);
  int8_t az_l = dev -> readRegister(MPU6050_ACCEL_Z_AXIS_LOW_REGISTER);
  int16_t az = (((int16_t)az_h) << 8) | (az_l & 0xff);

  int8_t gx_h = dev -> readRegister(MPU6050_GYRO_X_AXIS_HIGH_REGISTER);
  int8_t gx_l = dev -> readRegister(MPU6050_GYRO_X_AXIS_LOW_REGISTER);
  int16_t gx = (((int16_t)gx_h) << 8) | (gx_l & 0xff);

  int8_t gy_h = dev -> readRegister(MPU6050_GYRO_Y_AXIS_HIGH_REGISTER);
  int8_t gy_l = dev -> readRegister(MPU6050_GYRO_Y_AXIS_LOW_REGISTER);
  int16_t gy = (((int16_t)gy_h) << 8) | (gy_l & 0xff);

  int8_t gz_h = dev -> readRegister(MPU6050_GYRO_Z_AXIS_HIGH_REGISTER);
  int8_t gz_l = dev -> readRegister(MPU6050_GYRO_Z_AXIS_LOW_REGISTER);
  int16_t gz = (((int16_t)gz_h) << 8) | (gz_l & 0xff);

  (*a_x) = (float) ax;
  (*a_y) = (float) ay;
  (*a_z) = (float) az;
  (*g_x) = (float) gx;
  (*g_y) = (float) gy;
  (*g_z) = (float) gz;
}


void getCurrentValuesFromMagnetometer(float *m_x, float *m_y, float *m_z){
  int8_t mx_h = mgn -> readRegister(HMC5883L_RA_DATAX_H);
  int8_t mx_l = mgn -> readRegister(HMC5883L_RA_DATAX_L);
  int16_t mx = (((int16_t)mx_h) << 8) | (mx_l & 0xff);
  
  int8_t my_h = mgn -> readRegister(HMC5883L_RA_DATAY_H);
  int8_t my_l = mgn -> readRegister(HMC5883L_RA_DATAY_L);
  int16_t my = (((int16_t)my_h) << 8) | (my_l & 0xff);
  
  int8_t mz_h = mgn -> readRegister(HMC5883L_RA_DATAZ_H);
  int8_t mz_l = mgn -> readRegister(HMC5883L_RA_DATAZ_L);
  int16_t mz = (((int16_t)mz_h) << 8) | (mz_l & 0xff);
  
  Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t");
  Serial.print(mz); Serial.print("\n");
  
  (*m_x) = (float) mx + HMC5883L_OFFSET_X;
  (*m_y) = (float) (my + HMC5883L_OFFSET_Y) / HMC5883L_SPREAD_Y * HMC5883L_SPREAD_X;
  (*m_z) = (float) (mz + HMC5883L_OFFSET_Z) / HMC5883L_SPREAD_Z * HMC5883L_SPREAD_X;
  
  //set mode to SINGLE (some bits are overriden?)
  mgn -> writeRegister(HMC5883L_RA_MODE, 0x01);
}
