#include <I2Cdev.h>
#include <helper_3dmath.h>

extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

#ifndef _MPU9250_LIB
#define _MPU9250_LIB

#define _PI              32768.0
#define _TWOPI           65536.0
#define DEFAULT_MPU_HZ   200
#define GYRO_ORIENTATION B10000101

class MPU9250Device {
  MPU9250Device();
  ~MPU9250Device();

  void init();
  void enable();
  void disable();
  void getAngles(float *angles);
  void getMagnetometer(float *angle);
  void calibrate();

  private:
  unsigned char revision;
  short calibrationPhase = 0;
};

#endif

