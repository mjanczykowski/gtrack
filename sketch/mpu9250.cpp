/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */

#include <I2Cdev.h>
#include <EEPROM.h>

extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

#include "mpu9250.h"



MPU9250Device::MPU9250Device() {
  // load hard-iron correction vector from EEPROM
  SVector cv;
  EEPROM.get(CORRECTION_VECTOR_EEPROM_ADDRESS, cv);
  this->correctionVector = cv;
}

void MPU9250Device::init() {
  mpu_init(&this->revision);
  mpu_set_compass_sample_rate(100); // defaults to 100 in the libs

  /* Get/set hardware configuration. Start gyro. Wake up all sensors. */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  //  mpu_set_gyro_fsr (2000);//250
  //  mpu_set_accel_fsr(2);//4

  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);

  dmp_load_motion_driver_firmware();
  dmp_set_orientation(GYRO_ORIENTATION);

  //dmp_register_tap_cb(&tap_cb);

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
                                DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;

  //dmp_features = dmp_features |  DMP_FEATURE_TAP ;

  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
}

void MPU9250Device::enable() {
  mpu_set_dmp_state(1);
}

void MPU9250Device::disable() {
  mpu_set_dmp_state(0);
}

bool MPU9250Device::getQuaternion(Quaternion *quaternion, volatile bool *hasMoreMeasurements) {
  if (!checkFIFO()) {
    *hasMoreMeasurements = false;
    return false;
  }
  long unsigned int sensor_data;
  long quat[4];
  short gyro[3], accel[3], sensors;
  unsigned char more;
  dmp_read_fifo(gyro, accel, quat, &sensor_data, &sensors, &more);
  quaternion -> setValues((float)quat[0] / QUAT_SCALEFACTOR, (float)quat[1] / QUAT_SCALEFACTOR,
                          (float)quat[2] / QUAT_SCALEFACTOR, (float)quat[3] / QUAT_SCALEFACTOR);
  quaternion -> normalize();
  *hasMoreMeasurements = (more != 0);
  return true;
}

void MPU9250Device::getMagnetometer(float *values) {
  short mag[3];
  unsigned char magSampled = mpu_get_compass_reg(mag);
  if(magSampled == 0) {
    values[0] = (float)(mag[0] - this->correctionVector.x) / MAG_SCALEFACTOR;
    values[1] = (float)(mag[1] - this->correctionVector.y) / MAG_SCALEFACTOR;
    values[2] = (float)(mag[2] - this->correctionVector.z) / MAG_SCALEFACTOR;
  }
}

void MPU9250Device::getRawMagnetometer(short *values) {
  short mag[3];
  unsigned char magSampled = mpu_get_compass_reg(mag);
  if(magSampled == 0) {
    values[0] = mag[0];
    values[1] = mag[1];
    values[2] = mag[2];
  }
}

void MPU9250Device::updateCorrectionVector(SVector newValues) {
  EEPROM.put(CORRECTION_VECTOR_EEPROM_ADDRESS, newValues);
  this->correctionVector = newValues;
}

void MPU9250Device::resetFIFO() {
  I2Cdev::writeBit(MPU_ADDRESS, MPU_USERCTRL_REG, MPU_USERCTRL_FIFO_RESET_BIT, true);
}

uint8_t MPU9250Device::getMPUStatus() {
  I2Cdev::readByte(MPU_ADDRESS, MPU_INT_STATUS_REG, buffer);
  return buffer[0];
}

uint16_t MPU9250Device::getFIFOCount() {
  I2Cdev::readBytes(MPU_ADDRESS, MPU_FIFO_COUNTH_REG, MPU_FIFO_COUNT_LEN, buffer);
  return (((uint16_t) buffer[0]) << 8) | buffer[1];
}

bool MPU9250Device::checkFIFO() {
  uint8_t mpuIntStatus = getMPUStatus();
  uint16_t fifoCount = getFIFOCount();
  if ((mpuIntStatus & MPU_OVERFLOW_VALUE) || fifoCount >= MAX_FIFO_COUNT) {
    resetFIFO();
    return false;
  }
  if(fifoCount == 0) {
    return false;
  }
  return true;
}

