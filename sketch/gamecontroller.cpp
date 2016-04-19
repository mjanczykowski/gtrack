#include "GameController.h"

#if defined(_USING_HID)

#define REPORT_ID 0x04
#define REPORT_STRUCTURE_SIZE_BYTES 8

static const uint8_t _hidReportDescriptor[] PROGMEM = {
  
  // Joystick
  0x05, 0x01,           // USAGE_PAGE (Generic Desktop)
  0x09, 0x04,           // USAGE (Joystick)
  0xa1, 0x01,           // COLLECTION (Application)
  0x85, REPORT_ID,      //   REPORT_ID (4)
  
  // Z Axis
  0x09, 0x01,			      //   USAGE (Pointer)
  0xA1, 0x00,			      //   COLLECTION (Physical)
  0x09, 0x32,		        //     USAGE (z)
  0x15, 0x00,           //   LOGICAL_MINIMUM (0)
  0x26, 0xff, 0xff,     //   LOGICAL_MAXIMUM (65535)
  0x75, 16,             //   REPORT_SIZE (16)
  0x95, 1,		          //     REPORT_COUNT (1)
  0x81, 0x82,		        //     INPUT (Data,Var,Abs)
  0xc0,				          //   END_COLLECTION

  // X, Y, and Z Axis rotation
  0x09, 0x01,           //   USAGE (Pointer)
  0xA1, 0x00,           //   COLLECTION (Physical)
  0x09, 0x33,           //     USAGE (rx)
  0x09, 0x34,           //     USAGE (ry)
  0x09, 0x35,           //     USAGE (rz)
  0x16, 0x00, 0x80,     //   LOGICAL_MINIMUM (-32768)
  0x26, 0xff, 0x7f,     //   LOGICAL_MAXIMUM (32767)
  0x75, 16,             //   REPORT_SIZE (16)
  0x95, 3,              //     REPORT_COUNT (3)
  0x81, 0x82,           //     INPUT (Data,Var,Abs)
  0xc0,                 //   END_COLLECTION
                              
  0xc0				          // END_COLLECTION
};

GameController::GameController()
{
  static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
  HID().AppendDescriptor(&node);
  
  // Initalize State
  zAxis = 0;
  xAxisRotation = 0;
  yAxisRotation = 0;
  zAxisRotation = 0;
}

void GameController::start()
{
  sendReport();
}

void GameController::setZAxis(int16_t value)
{
  zAxis = value;
}

void GameController::setXAxisRotation(int16_t value)
{
  xAxisRotation = value;
}
void GameController::setYAxisRotation(int16_t value)
{
	yAxisRotation = value;
}
void GameController::setZAxisRotation(int16_t value)
{
	zAxisRotation = value;
}

void GameController::sendReport()
{
	int16_t data[4];
	
  data[0] = zAxis + 32767;
  data[1] = xAxisRotation;
  data[2] = yAxisRotation;
  data[3] = zAxisRotation;

	HID().SendReport(REPORT_ID, data, REPORT_STRUCTURE_SIZE_BYTES);
}

#endif
