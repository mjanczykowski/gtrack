#ifndef MEDIAN_H
#define MEDIAN_H

struct Measurements{
  float ax, ay, az, gx, gy, gz;
  
  Measurements(float ax, float ay, float az, float gx, float gy, float gz);
  Measurements();
};

class Median
{
  private:
    float *ax;
    float *ay;
    float *az;
    float *gx;
    float *gy;
    float *gz;
    int i, len;
    Measurements getMedian();
    static float computeMedian(float *tab, int n);  
  public:
    Median(int count, Measurements startMeasurements);
    ~Median();
    Measurements getMeasurements(Measurements newMeasurements);
};

#endif
