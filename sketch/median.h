/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */

#ifndef _MEDIAN
#define _MEDIAN

class MedianFilter {
  public:
    MedianFilter(short medianWindowSize);
    ~MedianFilter();
  
    void addMeasurement(short measurement);
    void getFilteredMeasurement(short *measurement);
  
  private:
    int measurements_present = 0, current_measurement = 0;
    short medianWindowSize;
    short *measurements;
    short *measurements_copy;
};

#endif

