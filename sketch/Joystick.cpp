/*
  Joystick.cpp

  Copyright (c) 2015, Matthew Heironimus

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Joystick.h"

#if defined(_USING_HID)

#define JOYSTICK_REPORT_ID 0x04
#define JOYSTICK_STATE_SIZE 8

static const uint8_t _hidReportDescriptor[] PROGMEM = {
  
	// Joystick
	0x05, 0x01,			      // USAGE_PAGE (Generic Desktop)
	0x09, 0x04,			      // USAGE (Joystick)
	0xa1, 0x01,			      // COLLECTION (Application)
	0x85, JOYSTICK_REPORT_ID, //   REPORT_ID (3)

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

Joystick_::Joystick_()
{
	// Setup HID report structure
	static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
	HID().AppendDescriptor(&node);
	
	// Initalize State
	zAxis = 0;
	xAxisRotation = 0;
	yAxisRotation = 0;
	zAxisRotation = 0;
}

void Joystick_::begin(bool initAutoSendState)
{
	autoSendState = initAutoSendState;
	sendState();
}

void Joystick_::end()
{
}

void Joystick_::setZAxis(int16_t value)
{
	zAxis = value;
	if (autoSendState) sendState();
}

void Joystick_::setXAxisRotation(int16_t value)
{
	xAxisRotation = value;
	if (autoSendState) sendState();
}
void Joystick_::setYAxisRotation(int16_t value)
{
	yAxisRotation = value;
	if (autoSendState) sendState();
}
void Joystick_::setZAxisRotation(int16_t value)
{
	zAxisRotation = value;
	if (autoSendState) sendState();
}

void Joystick_::sendState()
{
	int16_t data[4];
	
  data[0] = zAxis + 32767;
  data[1] = xAxisRotation;
  data[2] = yAxisRotation;
  data[3] = zAxisRotation;

	// HID().SendReport(Report number, array of values in same order as HID descriptor, length)
	HID().SendReport(JOYSTICK_REPORT_ID, data, JOYSTICK_STATE_SIZE);
}

Joystick_ Joystick;

#endif
