#ifndef GAMECONTROLLER_h
#define GAMECONTROLLER_h

#include "HID.h"

#if !defined(_USING_HID)

#warning "Using legacy HID core (non pluggable)"

#else

class GameController
{
private:
	int16_t	 xAxis;
	int16_t	 yAxis;
	int16_t	 zAxis;
	int16_t	 xAxisRotation;
	int16_t	 yAxisRotation;
	int16_t	 zAxisRotation;

public:
	GameController();

	void start();

	void setXAxisRotation(int16_t value);
	void setYAxisRotation(int16_t value);
	void setZAxisRotation(int16_t value);

  void setZAxis(int16_t value);

	void sendReport();
};

#endif
#endif
