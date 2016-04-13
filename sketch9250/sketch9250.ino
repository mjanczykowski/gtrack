/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */

#include "mpu9250.h"
#include "quaternion.h"
#include "dcm.h"
#include <math.h>
#include <inttypes.h>
#include <Wire.h>

//=========================================================================================================================================================
// CONSTANTS
//=========================================================================================================================================================

#define MIN_DT                              100      //in us
#define W_RP                                1.0
#define W_Y                                 1.0
#define K_P                                 1.0
#define K_I                                 0.0
#define TWBR_I2C_CLOCKRATE                  12       //= 12400 kHz (200 kHz if 8 MHz processor)

//=========================================================================================================================================================
// GLOBAL VARIABLES
//=========================================================================================================================================================

//Packet to send to PC
uint8_t teapotPacket[10] = {'$', 0x02, 0, 0, 0, 0, 0, 0, '\r', '\n'};

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

//Values from MPU
float accel[3];
float mag[3];
float angles[3];
float initialHeading, heading;

//Drift
float drift_x = 0.0, drift_y = 0.0, drift_z = 0.0;

//FIFO waiting
short newValues = 0;

//MPU 9250
MPU9250Device mpuDev;

//=========================================================================================================================================================
// MPU RELATED CODE
//=========================================================================================================================================================

void enable_mpu() {
  EICRB |= (1 << ISC60) | (1 << ISC61); // sets the interrupt type for EICRB (INT6)
  EIMSK |= (1 << INT6); // activates the interrupt. 6 for 6, etc
  mpuDev.enable();
}

void disable_mpu() {
  mpuDev.disable();
  EIMSK &= ~(1 << INT6);      //deactivate interupt
}

ISR(INT6_vect) {
  newValues = 1;
}

//=========================================================================================================================================================
// SETUP CODE
//=========================================================================================================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("GTRACK v. 1.0");
  Serial.println("(C) 2016 Michal Ciolczyk, Michal Janczykowski");
  Serial.println();
  Wire.begin();
  TWBR = TWBR_I2C_CLOCKRATE;
  mpuDev.init();
  enable_mpu();
}

//=========================================================================================================================================================
// MAIN LOOP
//=========================================================================================================================================================

void loop() {
  while(!newValues) ;
  newValues = 0;
  newTime = micros();
  dt = newTime - currentTime;
  currentTime = newTime;
  
  mpuDev.getAnglesAndAccelerometer(angles, accel);
  mpuDev.getMagnetometer(mag);

  rotation = Vector(angles[0] * DEG_TO_RAD, angles[1] * DEG_TO_RAD, angles[2] * DEG_TO_RAD);

  R_aux = R.rotateByVector(rotation);
  R_aux.normalize();
  
  Vector ypr = R.getYawPitchRoll();
  
  //correction
  //magnetometer axes are rotated 90 CCW around Z axis
  float cosx = cos(-ypr.y), sinx = sin(-ypr.y), cosy = cos(ypr.x), siny = sin(ypr.x);
  
  float mxg = mag[0]*cosy + mag[1]*siny*sinx + mag[2]*siny*cosx;
  float myg = mag[1]*cosx - mag[2]*sinx;

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
  Vector acc(accel[0], accel[1], accel[2]);
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

  //prepare data for v-joy
  float newX =  atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
  float newY = asin(-2.0 * (q.x * q.z - q.w * q.y));
  float newZ = -atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
  
  // scale to range -32767 to 32767
  newX = newX * 10430.06; //  = 64k / (2*M_PI)
  newY = newY * 10430.06;
  newZ = newZ * 10430.06;
    
  short joyX = constrain((long)newX, -32767, 32767);
  short joyY = constrain((long)newY, -32767, 32767);
  short joyZ = constrain((long)newZ, -32767, 32767);

  teapotPacket[2] = ((uint8_t)(joyX >> 8));
  teapotPacket[3] = ((uint8_t)(joyX & 0xFF));
  teapotPacket[4] = ((uint8_t)(joyY >> 8));
  teapotPacket[5] = ((uint8_t)(joyY & 0xFF));
  teapotPacket[6] = ((uint8_t)(joyZ >> 8));
  teapotPacket[7] = ((uint8_t)(joyZ & 0xFF));

  Serial.write(teapotPacket, 10);
}

