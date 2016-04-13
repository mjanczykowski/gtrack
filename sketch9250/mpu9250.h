/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */
 
#ifndef _MPU9250_LIB
#define _MPU9250_LIB

#define _PI              32768.0
#define _TWOPI           65536.0
#define DEFAULT_MPU_HZ   200
#define GYRO_ORIENTATION B10000101
#define MAG_SCALEFACTOR  4.f
#define QUAT_SCALEFACTOR 1073741824.0f

class MPU9250Device {
  public:
  void init();
  void enable();
  void disable();
  void getAnglesAndAccelerometer(float *angles, float *accelerometer);
  void getMagnetometer(float *values);
  void calibrate();

  private:
  unsigned char revision;
};

#endif


