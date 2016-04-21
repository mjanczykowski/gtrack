/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */

#ifndef _MEDIAN
#define _MEDIAN

#define MEDIAN_WINDOW_SIZE 5

class MedianFilter {
  public:
  void addMeasurement(short measurement);
  void getFilteredMeasurement(short *measurement);
  
  private:
  int measurements_present = 0, current_measurement = 0;
  short measurements[MEDIAN_WINDOW_SIZE] = {0}, measurements_copy[MEDIAN_WINDOW_SIZE] = {0};
};

#endif

