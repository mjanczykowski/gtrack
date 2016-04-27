/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */
 
#ifndef _MPU9250_LIB
#define _MPU9250_LIB

#include "quaternion.h"

#define _PI                         32768.0
#define _TWOPI                      65536.0
#define DEFAULT_MPU_HZ              100
#define GYRO_ORIENTATION            B10001000
#define MAG_SCALEFACTOR             4.f
#define QUAT_SCALEFACTOR            1073741824.0f
#define MPU_ADDRESS                 0x68
#define MPU_USERCTRL_REG            0x6A
#define MPU_USERCTRL_FIFO_RESET_BIT 2
#define MPU_OVERFLOW_VALUE          0x10
#define MAX_FIFO_COUNT              1008
#define MPU_INT_STATUS_REG          0x3A
#define MPU_FIFO_COUNTH_REG         0x72
#define MPU_FIFO_COUNT_LEN          2

class MPU9250Device {
  public:
  void init();
  void enable();
  void disable();
  bool getQuaternion(Quaternion *quaternion, volatile bool *hasMoreMeasurements);
  void getMagnetometer(float *values);

  private:
  unsigned char revision;
  uint8_t buffer[2];
  bool checkFIFO();
  void resetFIFO();
  uint8_t getMPUStatus();
  uint16_t getFIFOCount();
};

#endif

