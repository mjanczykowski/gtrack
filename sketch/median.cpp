#include "median.h"
#include <stdlib.h>

Median::Median(int count, Measurements startMeasurements)
{
  ax = (float*) malloc(count * sizeof(float));
  ay = (float*) malloc(count * sizeof(float));
  az = (float*) malloc(count * sizeof(float));
  gx = (float*) malloc(count * sizeof(float));
  gy = (float*) malloc(count * sizeof(float));
  gz = (float*) malloc(count * sizeof(float));
  i = 0;
  len = count;
  for(;i<count;i++){
    ax[i] = startMeasurements.ax;
    ay[i] = startMeasurements.ay;
    az[i] = startMeasurements.az;
    gx[i] = startMeasurements.gx;
    gy[i] = startMeasurements.gy;
    gz[i] = startMeasurements.gz;
  }
}

Median::~Median()
{
  free(ax);
  free(ay);
  free(az);
  free(gx);
  free(gy);
  free(gz);
}


Measurements Median::getMeasurements(Measurements newMeasurements){
  i = (i + 1) % len;
  
  ax[i] = newMeasurements.ax;
  ay[i] = newMeasurements.ay;
  az[i] = newMeasurements.az;
  gx[i] = newMeasurements.gx;
  gy[i] = newMeasurements.gy;
  gz[i] = newMeasurements.gz;
  
  return getMedian();
}

Measurements Median::getMedian(){
  Measurements result;
  
  result.ax = Median::computeMedian(this -> ax, len);
  result.ay = Median::computeMedian(this -> ay, len);
  result.az = Median::computeMedian(this -> az, len);
  result.gx = Median::computeMedian(this -> gx, len);
  result.gy = Median::computeMedian(this -> gy, len);
  result.gz = Median::computeMedian(this -> gz, len);
  
  return result;
}

float Median::computeMedian(float *tab, int n){
  float *arr = (float*) malloc(n * sizeof(float));
  float res, temp;
  int i = 0, j;
  //Copy
  for(;i<n;i++){
    arr[i] = tab[i];
  }
  //Sort (we accept O(n^2) since n is small enough) - insertion sort
  for(i=1; i<n; i++){
    temp = arr[i];
    for(j=i-1; j>=0 && arr[j] > temp; j--){
      arr[j+1] = arr[j];
    }
    arr[j+1] = temp;
  }  
  //Get median
  if(n % 2 == 0){
    res = (arr[n/2-1] + arr[n/2])/2.0;
  } else {
    res = arr[n/2];
  }
  //Clean up & return
  free(arr);
  return res;
}

Measurements::Measurements(float ax, float ay, float az, float gx, float gy, float gz){
  this -> ax = ax;
  this -> ay = ay;
  this -> az = az;
  this -> gx = gx;
  this -> gy = gy;
  this -> gz = gz;
}

Measurements::Measurements(){
  ax = ay = az = gx = gy = gz = 0.0;
}
