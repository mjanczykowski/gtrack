/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */

#include "mpu9250.h"
#include "quaternion.h"
#include "gamecontroller.h"
#include "median.h"
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

#define MAG_OFFSET_X                        -33.0
#define MAG_OFFSET_Y                        -56.0
#define MAG_OFFSET_Z                        60.0
#define MAG_SPREAD_X                        94.0
#define MAG_SPREAD_Y                        96.0
#define MAG_SPREAD_Z                        96.0

//=========================================================================================================================================================
// GLOBAL VARIABLES
//=========================================================================================================================================================

//Time related values
unsigned long currentTime, newTime, dt;

//Quaternion to send to computer and converted values
Quaternion q;
uint16_t w, x, y, z;

//Values from MPU
float mag[3];
float initialHeading, heading;

//Drift
float drift_correction = 0.0;           // current drift correction applied in each iteration
int drift_correction_counter = 0;       // how many steps of yaw drift compensation have been done in a row in same direction
                                        // after switching direction, the variable is set to 0
                                        // the value is either positive or negative according to direction

//FIFO waiting
volatile bool newValues = false;

//MPU 9250
MPU9250Device mpuDev;

//USB HID Game Controller
GameController controller;

//Median filters
MedianFilter yawFilter(3), pitchFilter(3), rollFilter(3), headingFilter(49);

//=========================================================================================================================================================
// MPU RELATED CODE
//=========================================================================================================================================================

void enable_mpu() {
  mpuDev.enable();
}

void disable_mpu() {
  mpuDev.disable();
}

//=========================================================================================================================================================
// SETUP CODE
//=========================================================================================================================================================

void setup() {
  Wire.begin();
  TWBR = TWBR_I2C_CLOCKRATE;
  mpuDev.init();
  enable_mpu();
  controller.start();

  //set initial heading
  mpuDev.getQuaternion(&q, &newValues);
  
  float yaw, pitch, roll;
  q.getYawPitchRoll(&yaw, &pitch, &roll);

  int i;
  for(i = 0; i < 50; i++) {
    mpuDev.getMagnetometer(mag);
    rescale_mag(mag); //compensate hard-iron
    initialHeading = atan2(mag[2] * sin(roll) - mag[1] * cos(roll), mag[0] * cos(pitch) + mag[1] * sin(pitch) * sin(roll) + mag[2] * sin(pitch) * cos(roll)); //result in range (-M_PI, +M_PI)
  
    headingFilter.addMeasurement((short)(initialHeading * RAD_TO_DEG * 100.0));
    delay(10);
  }
  short headingFiltered;
  headingFilter.getFilteredMeasurement(&headingFiltered);
  initialHeading = (float)headingFiltered * 0.01 *DEG_TO_RAD;
}

//=========================================================================================================================================================
// MAIN LOOP
//=========================================================================================================================================================

void loop() {
  newTime = micros();
  dt = newTime - currentTime;
  currentTime = newTime;

  if(!mpuDev.getQuaternion(&q, &newValues)) {
    return;
  }
  
  float yaw, pitch, roll;
  q.getYawPitchRoll(&yaw, &pitch, &roll);
  
  mpuDev.getMagnetometer(mag);
  rescale_mag(mag); //compensate hard-iron

  heading = atan2(mag[2] * sin(roll) - mag[1] * cos(roll), mag[0] * cos(pitch) + mag[1] * sin(pitch) * sin(roll) + mag[2] * sin(pitch) * cos(roll)); //result in range (-M_PI, +M_PI)
  
  heading -= initialHeading;

  if(heading < -M_PI)
    heading += 2 * M_PI;
  else if(heading > M_PI)
    heading -= 2 * M_PI;

  headingFilter.addMeasurement((short)(heading * RAD_TO_DEG * 100.0));
  short headingFiltered;
  headingFilter.getFilteredMeasurement(&headingFiltered);

  heading = (float)headingFiltered * 0.01 *DEG_TO_RAD;
  
  yaw += drift_correction;

  if(heading > yaw)
  {
    if(drift_correction_counter < 0)
    {
      drift_correction_counter = 0;
    }
    else
    {
      drift_correction_counter=1;
    }
  }
  else
  {
    if(drift_correction_counter > 0)
    {
      drift_correction_counter = 0;
    }
    else
    {
      drift_correction_counter = -1;
    }
  }

  drift_correction += drift_correction_counter * 0.00001;
  
  float newX, newY, newZ;

  // scale to range -32767 to 32767
  newX = pitch * 10430.06; //  = 64k / (2*M_PI)
  newY = roll * 10430.06;
  newZ = yaw * 10430.06;
    
  short joyX = constrain((long)newX, -32767, 32767);
  short joyY = constrain((long)newY, -32767, 32767);
  short joyZ = constrain((long)newZ, -32767, 32767);

//  yawFilter.addMeasurement(joyX);
//  pitchFilter.addMeasurement(joyY);
//  rollFilter.addMeasurement(joyZ);
//  yawFilter.getFilteredMeasurement(&joyX);
//  pitchFilter.getFilteredMeasurement(&joyY);
//  rollFilter.getFilteredMeasurement(&joyZ);

  controller.setXAxisRotation(joyX);
  controller.setYAxisRotation(joyY);
  controller.setZAxisRotation(joyZ);
  controller.sendReport();
}

inline void rescale_mag(float *mag) {
  mag[0] += MAG_OFFSET_X;
  mag[1] = (mag[1] + MAG_OFFSET_Y) / MAG_SPREAD_Y * MAG_SPREAD_X;
  mag[2] = (mag[2] + MAG_OFFSET_Z) / MAG_SPREAD_Z * MAG_SPREAD_Y;
}
