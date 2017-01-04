/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */

#include "mpu9250.h"
#include "quaternion.h"
#include "gamecontroller.h"
#include "median.h"
#include <inttypes.h>
#include <Wire.h>
#include <MatrixMath.h>

//=========================================================================================================================================================
// CONSTANTS
//=========================================================================================================================================================

#define TWBR_I2C_CLOCKRATE                  12       //= 12400 kHz (200 kHz if 8 MHz processor)

#define CALIBRATION_TIME_US                 30000000 //30s
#define CALIBRATION_SAMPLE_DT_MS            30       //30ms

#define HEADING_FILTER_MULTIPLIER           ( RAD_TO_DEG * 100.0 )   //filter stores short values - degrees * 100

//=========================================================================================================================================================
// GLOBAL VARIABLES
//=========================================================================================================================================================

// calibration mode
boolean calibrationMode = false;
unsigned long calibrationStartTime;
float calib_XtX[4][4];
float calib_XtY[4];
//

//Time related values
unsigned long currentTime;

//Values from MPU
float initialHeading;

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
MedianFilter headingFilter(49);

//=========================================================================================================================================================
// MPU RELATED CODE
//=========================================================================================================================================================

inline void enable_mpu() {
  mpuDev.enable();
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
  Serial.begin(115200);

  Quaternion q;
  //set initial heading
  mpuDev.getQuaternion(&q, &newValues);
  
  float yaw, pitch, roll;
  q.getYawPitchRoll(&yaw, &pitch, &roll);

  float mag[3];
  for(int i = 0; i < 50; ++i) {
    mpuDev.getMagnetometer(mag);
    initialHeading = atan2(mag[2] * sin(roll) - mag[1] * cos(roll), mag[0] * cos(pitch) + mag[1] * sin(pitch) * sin(roll) + mag[2] * sin(pitch) * cos(roll)); //result in range (-M_PI, +M_PI)
  
    headingFilter.addMeasurement((short)(initialHeading * HEADING_FILTER_MULTIPLIER));
    delay(10);
  }
  
  short headingFiltered;
  headingFilter.getFilteredMeasurement(&headingFiltered);
  initialHeading = (float)headingFiltered / HEADING_FILTER_MULTIPLIER;
}

//=========================================================================================================================================================
// MAIN LOOP
//=========================================================================================================================================================

void loop() {
  currentTime = micros();
  
  if(Serial.available() > 0) {
    char command = Serial.read();
    if('c' == command && !calibrationMode) {
      Serial.println("Starting calibration. Rotate device in all possible directions for 30 seconds to collect as diverse samples as possible.");
      calibrationStartTime = currentTime;
      calibrationMode = true;
    }
  }

  if(calibrationMode) {
    if(currentTime - calibrationStartTime > CALIBRATION_TIME_US) {
      calibrationMode = false;
      Serial.println("Calibration finished.");

      Matrix.Invert((float*)calib_XtX, 4);

      float beta[4];
      Matrix.Multiply((float*)calib_XtX, (float*)calib_XtY, 4, 4, 1, (float*) beta);
      
      SVector V;
      V.x = (short)(beta[0] * 0.5);
      V.y = (short)(beta[1] * 0.5);
      V.z = (short)(beta[2] * 0.5);
      
      mpuDev.updateCorrectionVector(V);
      return;
    }
    
    delay(CALIBRATION_SAMPLE_DT_MS);

    short raw_mag[4];
    raw_mag[3] = 1; //used for calibration
    
    mpuDev.getRawMagnetometer(raw_mag); 

    // update calib_XtX matrix and calib_XtY vector
    long mag_dot_mag = (long)raw_mag[0]*(long)raw_mag[0] + (long)raw_mag[1]*(long)raw_mag[1] + (long)raw_mag[2]*(long)raw_mag[2];
    
    for(int i = 0; i < 4; ++i) {
      for(int j = 0; j < 4; ++j) {
        calib_XtX[i][j] += (long)raw_mag[i] * (long)raw_mag[j];
      }
      calib_XtY[i] += (long)raw_mag[i] * mag_dot_mag;
    }
    return;
  }

  //end calibration

  Quaternion q;
  if(!mpuDev.getQuaternion(&q, &newValues)) {
    return;
  }
  
  float yaw, pitch, roll;
  q.getYawPitchRoll(&yaw, &pitch, &roll);

  float mag[3];
  mpuDev.getMagnetometer(mag);

  float heading = atan2(mag[2] * sin(roll) - mag[1] * cos(roll), mag[0] * cos(pitch) + mag[1] * sin(pitch) * sin(roll) + mag[2] * sin(pitch) * cos(roll)); //result in range (-M_PI, +M_PI)
  
  heading -= initialHeading;

  if(heading < -M_PI)
    heading += 2 * M_PI;
  else if(heading > M_PI)
    heading -= 2 * M_PI;

  headingFilter.addMeasurement((short)(heading * HEADING_FILTER_MULTIPLIER));
  short headingFiltered;
  headingFilter.getFilteredMeasurement(&headingFiltered);

  heading = (float)headingFiltered / HEADING_FILTER_MULTIPLIER;

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
  
  long newX, newY, newZ;

  // scale to range -32767 to 32767
  newX = pitch * 10430.06; //  = 64k / (2*M_PI)
  newY = roll * 10430.06;
  newZ = yaw * 10430.06;

  short joyX = constrain(newX, -32767, 32767);
  short joyY = constrain(newY, -32767, 32767);
  short joyZ = constrain(newZ, -32767, 32767);

  controller.setXAxisRotation(joyX);
  controller.setYAxisRotation(joyY);
  controller.setZAxisRotation(joyZ);
  controller.sendReport();
}

