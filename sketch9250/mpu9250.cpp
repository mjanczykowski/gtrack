/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */

#include <I2Cdev.h>

extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

#include "mpu9250.h"
#include "quaternion.h"

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
                                DMP_FEATURE_SEND_RAW_GYRO ;//| DMP_FEATURE_GYRO_CAL;

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

void MPU9250Device::getAnglesAndAccelerometer(float *angles, float *accelerometer) {
  long unsigned int sensor_data;
  long quat[4];
  short gyro[3], accel[3], sensors;
  unsigned char more;
  dmp_read_fifo(gyro, accel, quat, &sensor_data, &sensors, &more);
  Quaternion q((float)quat[0] / QUAT_SCALEFACTOR, (float)quat[1] / QUAT_SCALEFACTOR,
               (float)quat[2] / QUAT_SCALEFACTOR, (float)quat[3] / QUAT_SCALEFACTOR);
  q.getAngles(&angles[0], &angles[1], &angles[2]);
  accelerometer[0] = (float) accel[0];
  accelerometer[1] = (float) accel[1];
  accelerometer[2] = (float) accel[2];
}

void MPU9250Device::getMagnetometer(float *values) {
  unsigned char magSampled;
  short mag[3];
  magSampled  = mpu_get_compass_reg(mag);
  if(magSampled == 0) {
    values[0] = (float)mag[0] / MAG_SCALEFACTOR;
    values[1] = (float)mag[1] / MAG_SCALEFACTOR;
    values[2] = (float)mag[2] / MAG_SCALEFACTOR;
  }
}

void MPU9250Device::calibrate() {
  
}


