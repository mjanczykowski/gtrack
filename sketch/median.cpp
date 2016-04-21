/* GTRACK v. 0.1
 * 
 * (C) 2016 Michał Ciołczyk, Michał Janczykowski
 */

#include "median.h"

short tmp = 0;

inline int min(int a, int b) {
  if(a < b) return a;
  return b;
}

inline void copy_array(short *from, short *to, int size) {
  for(int i = 0; i < size; i++)
    to[i] = from[i];
}

inline short median(short *arr, int size) {
  for (int i = 1; i < size; i++)
    for (int j = i; j > 0 && arr[j] < arr[j - 1]; j--) {
      tmp = arr[j];
      arr[j] = arr[j - 1];
      arr[j - 1] = tmp;
    }
  return arr[size / 2];
}

void MedianFilter::addMeasurement(short measurement) {
  this -> measurements[this -> current_measurement] = measurement;
  this -> current_measurement = (this -> current_measurement + 1) % MEDIAN_WINDOW_SIZE;
  this -> measurements_present = min(MEDIAN_WINDOW_SIZE, this -> measurements_present + 1);
}

void MedianFilter::getFilteredMeasurement(short *measurement) {
  copy_array(this -> measurements, this -> measurements_copy, this -> measurements_present);
  *measurement = median(this -> measurements_copy, this -> measurements_present);
}

