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

#define CALIBRATION_TIME_US                 30000000 //30s
#define CALIBRATION_SAMPLE_DT_MS            30       //30ms

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
//GameController controller;

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
  Quaternion q;
  float mag[4];
//  float mag[3];

  Wire.begin();
  TWBR = TWBR_I2C_CLOCKRATE;
  mpuDev.init();
  enable_mpu();
//  controller.start();
  Serial.begin(115200);
  
  //set initial heading
  mpuDev.getQuaternion(&q, &newValues);
  
  float yaw, pitch, roll;
  q.getYawPitchRoll(&yaw, &pitch, &roll);

  for(int i = 0; i < 50; ++i) {
    mpuDev.getMagnetometer(mag);
//    rescale_mag(mag); //compensate hard-iron
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
  currentTime = micros();
  
  float mag[3];
   
  if(Serial.available() > 0) {
    char command = Serial.read();
    if('c' == command && !calibrationMode) {
      calibrationStartTime = currentTime;
      calibrationMode = true;
    }
  }

  if(calibrationMode) {
    if(currentTime - calibrationStartTime > CALIBRATION_TIME_US) {
      calibrationMode = false;

      Matrix.Invert((float*)calib_XtX, 4);

      float beta[4];
      Matrix.Multiply((float*)calib_XtX, (float*)calib_XtY, 4, 4, 1, (float*) beta);
      
      float Vx = 0.5 * beta[0];
      float Vy = 0.5 * beta[1];
      float Vz = 0.5 * beta[2];


      Serial.println(Vx);
      Serial.println(Vy);
      Serial.println(Vz);

      SVector V;
      V.x = (short)(beta[0] * 0.5);
      V.y = (short)(beta[1] * 0.5);
      V.z = (short)(beta[2] * 0.5);
      
      mpuDev.updateCorrectionVector(V);
//      float B = sqrt(beta[3] + Vx*Vx + Vy*Vy + Vz*Vz);

//      Serial.println(V.x);
//      Serial.println(V.y);
//      Serial.println(V.z);
//      Serial.println(B);
      return;
    }
    
    delay(CALIBRATION_SAMPLE_DT_MS);

    float raw_mag[4];
    raw_mag[3] = 1; //used for calibration
    
    mpuDev.getRawMagnetometer(raw_mag); 

    // update calib_XtX matrix and calib_XtY vector
//    long mag_dot_mag = (long)raw_mag[0]*(long)raw_mag[0] + (long)raw_mag[1]*(long)raw_mag[1] + (long)raw_mag[2]*(long)raw_mag[2];
    float mag_dot_mag = raw_mag[0]*raw_mag[0] + raw_mag[1]*raw_mag[1] + raw_mag[2]*raw_mag[2];
    
    for(int i = 0; i < 4; ++i) {
      for(int j = 0; j < 4; ++j) {
//         calib_XtX[i][j] += (long)raw_mag[i] * (long)raw_mag[j];
        calib_XtX[i][j] += raw_mag[i] * raw_mag[j];
      }
      calib_XtY[i] += raw_mag[i] * mag_dot_mag;
    }
    return;
  }

  Quaternion q;
  if(!mpuDev.getQuaternion(&q, &newValues)) {
    return;
  }
  
  float yaw, pitch, roll;
  q.getYawPitchRoll(&yaw, &pitch, &roll);
  
  mpuDev.getMagnetometer(mag);
//  rescale_mag(mag); //compensate hard-iron

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

//  Serial.println(heading * RAD_TO_DEG);

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

//  controller.setXAxisRotation(joyX);
//  controller.setYAxisRotation(joyY);
//  controller.setZAxisRotation(joyZ);
//  controller.sendReport();
}

