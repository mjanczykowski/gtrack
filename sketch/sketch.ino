#include "i2c.h"
#include "quaternion.h"
#include "dcm.h"
#include "gamecontroller.h"
#include <math.h>
#include <inttypes.h>

#define I2C_BITRATE                         200000

//Gyroscope and accelerometer - MPU6050
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
#define CALIBRATION_TIME                    2000000  //in us

//correction consts
#define W_RP                                1.0
#define W_Y                                 1.0
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
#define HMC5883L_OFFSET_Y                   230.5
#define HMC5883L_OFFSET_Z                   46.5
#define HMC5883L_SPREAD_X                   813.0
#define HMC5883L_SPREAD_Y                   831.0
#define HMC5883L_SPREAD_Z                   753.0

//#define READABLE
//#define DEBUG
#define VJOY

//Device - accel/gyro MPU-6050 (GY-521) and magnetometer HMC5833L
I2CDevice *dev, *mgn;

//HID Game Controller
GameController controller;

#ifdef VJOY

//packet with 3x short for virtual joystick
uint8_t teapotPacket[10] = {'$', 0x02, 0, 0, 0, 0, 0, 0, '\r', '\n'};

#else

//We send teapot packet to Processing MPU DPM demo
uint8_t teapotPacket[14] = {
  '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, '\r', '\n'};
  
#endif

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
Quaternion q;
uint16_t w, x, y, z;

//DCM
DCM R, R_aux;

Vector rollPitchCorrectionPlane, yawCorrectionPlane, totalCorrection, angVel_Pcorr, angVel_Icorr(0., 0., 0.), angVel_corr, rotation, angVel;

int i = 0;
long time = 0;

//Values from MPU
float ax, ay, az, gx, gy, gz;

//Values from magnetometer
float mx, my, mz;
float initialHeading, heading;

//Drift
float drift_x = 0.0, drift_y = 0.0, drift_z = 0.0;

//Function to get current values from MPU
void getCurrentValuesFromMPU(float *a_x, float *a_y, float *a_z, float *g_x, float *g_y, float *g_z);

//Function to get current values from magnetometer
void getCurrentValuesFromMagnetometer(float *m_x, float *m_y, float *m_z);

//I2C buffers
char mpu_buffer[14], magnetometer_buffer[6];

void setup()
{
  Serial.begin(115200);
  Serial.println();

  controller.start();

  dev = new I2CDevice(MPU6050_I2C_ADDRESS, I2C_BITRATE);
  dev -> writeRegister(MPU6050_IDLE_REGISTER, 0);
  dev -> writeRegister(MPU6050_GYROSCOPE_CONTROL_REGISTER, 0);
  
  mgn = new I2CDevice(HMC5883L_I2C_ADDRESS, I2C_BITRATE);
  mgn -> writeRegister(HMC5883L_RA_CONFIG_A, 0x78);
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
  Serial.print("\tDrift angle:\t");
  Serial.println(drift_angle_z);
  Serial.println();

  getCurrentValuesFromMPU(&ax, &ay, &az, &gx, &gy, &gz);
  
  //heading
  getCurrentValuesFromMagnetometer(&mx, &my, &mz);
  
  //TODO - read heading according to current gravity direction
  
  initialHeading = atan2(my, mx);
  if(initialHeading < 0)
    initialHeading += 2 * M_PI;
  //

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
  
  float gyroAngVel_X = (gx / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_x) * DEG_TO_RAD;
  float gyroAngVel_Y = (gy / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_y) * DEG_TO_RAD;
  float gyroAngVel_Z = (gz / MPU6050_GYROSCOPE_SCALE_FACTOR - drift_z) * DEG_TO_RAD;

  angVel.setXYZ(gyroAngVel_X, gyroAngVel_Y, gyroAngVel_Z);
  rotation = angVel * (dt / 1000000.0);

  R_aux = R.rotateByVector(rotation);
  R_aux.normalize();
  
  Vector ypr = R.getYawPitchRoll();
  
  //correction

  //magnetometer axes are rotated 90 CCW around Z axis
  float cosx = cos(-ypr.y), sinx = sin(-ypr.y), cosy = cos(ypr.x), siny = sin(ypr.x);
  
  float mxg = mx*cosy + my*siny*sinx + mz*siny*cosx;
  float myg = my*cosx - mz*sinx;

  heading = atan2(myg, mxg);
  if(heading < 0)
    heading += 2 * M_PI;
    
  heading -= initialHeading;
  heading *= -1.;
    
  float headingX_ground = cos(heading);
  float headingY_ground = sin(heading);
  float yawCorrectionGround = - R.R[0][0]*headingY_ground + R.R[1][0]*headingX_ground;
  
  yawCorrectionPlane = R.getZRow() * yawCorrectionGround;
  
  //accelerometer
  Vector acc(ax, ay, az);
  acc.normalize();
  
  rollPitchCorrectionPlane = R_aux.getZRow().cross(acc);
  
  totalCorrection = rollPitchCorrectionPlane * W_RP + yawCorrectionPlane * W_Y;
  angVel_Pcorr = totalCorrection * K_P;
  angVel_Icorr = angVel_Icorr + totalCorrection * (K_I * (dt / 1000000.0));
  angVel_corr = angVel_Pcorr + angVel_Icorr;
  
  angVel = angVel - angVel_corr;
  rotation = angVel * (dt / 1000000.0);
  
  R = R.rotateByVector(rotation);
  R.normalize();
  
  ypr = R.getYawPitchRoll();
  
  float yaw, pitch, roll;

  q.setByAngles(ypr.x * RAD_TO_DEG, ypr.y * RAD_TO_DEG, ypr.z * RAD_TO_DEG);
  
#ifdef VJOY
  //prepare data for v-joy
  float newX =  atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
  float newY = asin(-2.0 * (q.x * q.z - q.w * q.y));
  float newZ = -atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
  
  // scale to range -32767 to 32767
  newX = newX * 10430.06; //  = 64k / (2*M_PI)
  newY = newY * 10430.06;
  newZ = newZ * 10430.06;

  // scale to range -127 to 127
//  newX = newX * 40.74367; //  = 256 / (2*M_PI)
//  newY = newY * 40.74367;
//  newZ = newZ * 40.74367;
  
  //clamp at 90 degrees left and right (what for?)
//  newX = constrain(newX, -16383.0, 16383.0);
//  newY = constrain(newY, -16383.0, 16383.0);
//  newZ = constrain(newZ, -16383.0, 16383.0);
  
  short joyX = constrain((long)newX, -32767, 32767);
  short joyY = constrain((long)newY, -32767, 32767);
  short joyZ = constrain((long)newZ, -32767, 32767);

//  byte joyX = constrain((long)newX, -127, 127);
//  byte joyY = constrain((long)newY, -127, 127);
//  byte joyZ = constrain((long)newZ, -127, 127);
  
#endif

#ifdef READABLE

  if(i % 11 == 10) {
    i = 0;
  } 
  else {
    i++;
    return;
  }

#ifndef VJOY
  
  ypr.printDeg();

//  Serial.print(mx); Serial.print("\t");
//  Serial.print(my); Serial.print("\t");
//  Serial.print(mz); Serial.print("\t");

//  Serial.print("heading:\t");
//  Serial.println(heading * 180/M_PI);
  
#else
  
  Serial.print(joyX);
  Serial.print("\t");
  Serial.print(joyY);
  Serial.print("\t");
  Serial.print(joyZ);

#endif
  Serial.print("\n");

  time = micros();

#else

#ifdef VJOY

//  teapotPacket[2] = ((uint8_t)(joyX >> 8));
//  teapotPacket[3] = ((uint8_t)(joyX & 0xFF));
//  teapotPacket[4] = ((uint8_t)(joyY >> 8));
//  teapotPacket[5] = ((uint8_t)(joyY & 0xFF));
//  teapotPacket[6] = ((uint8_t)(joyZ >> 8));
//  teapotPacket[7] = ((uint8_t)(joyZ & 0xFF));

  

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
  
#endif /*VJOY*/

#ifndef DEBUG

#ifdef VJOY

  //v-joy packet is smaller
//  Serial.write(teapotPacket, 10);
  controller.setXAxisRotation(joyX);
  controller.setYAxisRotation(joyY);
  controller.setZAxisRotation(joyZ);
  controller.sendReport();
  
#else

//  Serial.write(teapotPacket, 14);

#endif /* VJOY */

#else

/*  Serial.print(q.w * 16384.0, 8); 
  Serial.print("\t");
  Serial.print(w, 8); 
  Serial.print("\t");
  Serial.print(teapotPacket[2]); 
  Serial.print("\t");
  Serial.print(teapotPacket[3]); 
  Serial.print("\n");
  Serial.print((((uint16_t)teapotPacket[2])<<8) + teapotPacket[3]); 
  Serial.print("\n");
  Serial.print("------------------------------------------------------------------\n");*/
  delay(10);

#endif //DEBUG

#ifndef VJOY

  teapotPacket[11]++;

#endif

#endif //READABLE
  delay(10);
}

void getCurrentValuesFromMPU(float *a_x, float *a_y, float *a_z, float *g_x, float *g_y, float *g_z){
  dev -> readNRegisters(MPU6050_ACCEL_X_AXIS_HIGH_REGISTER, 14, mpu_buffer);
  int8_t ax_h = /*dev -> readRegister(MPU6050_ACCEL_X_AXIS_HIGH_REGISTER)/**/ /**/mpu_buffer[0]/**/ ;
  int8_t ax_l = /*dev -> readRegister(MPU6050_ACCEL_X_AXIS_LOW_REGISTER)/**/ /**/mpu_buffer[1]/**/ ;
  int16_t ax = (((int16_t)ax_h) << 8) | (ax_l & 0xff);

  int8_t ay_h = /*dev -> readRegister(MPU6050_ACCEL_Y_AXIS_HIGH_REGISTER)/**/ /**/mpu_buffer[2]/**/ ;
  int8_t ay_l = /*dev -> readRegister(MPU6050_ACCEL_Y_AXIS_LOW_REGISTER)/**/ /**/mpu_buffer[3]/**/ ;
  int16_t ay = (((int16_t)ay_h) << 8) | (ay_l & 0xff);

  int8_t az_h = /*dev -> readRegister(MPU6050_ACCEL_Z_AXIS_HIGH_REGISTER)/**/ /**/mpu_buffer[4]/**/ ;
  int8_t az_l = /*dev -> readRegister(MPU6050_ACCEL_Z_AXIS_LOW_REGISTER)/**/ /**/mpu_buffer[5]/**/ ;
  int16_t az = (((int16_t)az_h) << 8) | (az_l & 0xff);

  int8_t gx_h = /*dev -> readRegister(MPU6050_GYRO_X_AXIS_HIGH_REGISTER)/**/ /**/mpu_buffer[8]/**/ ;
  int8_t gx_l = /*dev -> readRegister(MPU6050_GYRO_X_AXIS_LOW_REGISTER)/**/ /**/mpu_buffer[9]/**/ ;
  int16_t gx = (((int16_t)gx_h) << 8) | (gx_l & 0xff);

  int8_t gy_h = /*dev -> readRegister(MPU6050_GYRO_Y_AXIS_HIGH_REGISTER)/**/ /**/mpu_buffer[10]/**/ ;
  int8_t gy_l = /*dev -> readRegister(MPU6050_GYRO_Y_AXIS_LOW_REGISTER)/**/ /**/mpu_buffer[11]/**/ ;
  int16_t gy = (((int16_t)gy_h) << 8) | (gy_l & 0xff);

  int8_t gz_h = /*dev -> readRegister(MPU6050_GYRO_Z_AXIS_HIGH_REGISTER)/**/ /**/mpu_buffer[12]/**/ ;
  int8_t gz_l = /*dev -> readRegister(MPU6050_GYRO_Z_AXIS_LOW_REGISTER)/**/ /**/mpu_buffer[13]/**/ ;
  int16_t gz = (((int16_t)gz_h) << 8) | (gz_l & 0xff);

  (*a_x) = (float) ax;
  (*a_y) = (float) ay;
  (*a_z) = (float) az;
  (*g_x) = (float) gx;
  (*g_y) = (float) gy;
  (*g_z) = (float) gz;
}


void getCurrentValuesFromMagnetometer(float *m_x, float *m_y, float *m_z){
  //mgn -> readNRegisters(HMC5883L_RA_DATAX_H, 6, magnetometer_buffer);
  
  int8_t mx_h = mgn -> readRegister(HMC5883L_RA_DATAX_H)/**/ /*magnetometer_buffer[0]/**/ ;
  int8_t mx_l = mgn -> readRegister(HMC5883L_RA_DATAX_L)/**/ /*magnetometer_buffer[1]/**/ ;
  int16_t mx = (((int16_t)mx_h) << 8) | (mx_l & 0xff);
  
  int8_t my_h = mgn -> readRegister(HMC5883L_RA_DATAY_H)/**/ /*magnetometer_buffer[2]/**/ ;
  int8_t my_l = mgn -> readRegister(HMC5883L_RA_DATAY_L)/**/ /*magnetometer_buffer[3]/**/ ;
  int16_t my = (((int16_t)my_h) << 8) | (my_l & 0xff);
  
  int8_t mz_h = mgn -> readRegister(HMC5883L_RA_DATAZ_H)/**/ /*magnetometer_buffer[4]/**/ ;
  int8_t mz_l = mgn -> readRegister(HMC5883L_RA_DATAZ_L)/**/ /*magnetometer_buffer[5]/**/ ;
  int16_t mz = (((int16_t)mz_h) << 8) | (mz_l & 0xff);

  (*m_x) = (float) mx + HMC5883L_OFFSET_X;
  (*m_y) = (float) (my + HMC5883L_OFFSET_Y) / HMC5883L_SPREAD_Y * HMC5883L_SPREAD_X;
  (*m_z) = (float) (mz + HMC5883L_OFFSET_Z) / HMC5883L_SPREAD_Z * HMC5883L_SPREAD_X;
  
  //set mode to SINGLE (some bits are overriden?)
  mgn -> writeRegister(HMC5883L_RA_MODE, 0x01);
}
