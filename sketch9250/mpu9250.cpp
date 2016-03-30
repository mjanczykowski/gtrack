#include "mpu9250.h"
#include <Arduino.h>

MPU9250Device::MPU9250Device() {
  
}

MPU9250Device::~MPU9250Device() {
  
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

