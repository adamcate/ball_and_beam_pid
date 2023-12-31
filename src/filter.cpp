/*
filter.cpp
created by: Adam Cate
*/

#include "filter.h"
#include <Arduino.h>

int get_median(int *array, int num){
  bool is_sorted = false;

  while(!is_sorted){
    is_sorted = true;
    for(int i = 0; i < num - 1; ++i){
      if(array[i] > array[i+1]){ // if the adjacent elements are not in order, swap them
        is_sorted = false;
        int temp = array[i];
        array[i] = array[i+1];
        array[i+1] = temp; 
      }
    }
  }

  return array[num/2]; // return the median index
}

float get_avg(int *array, int num){
  int sum = 0;
  for(int i = 0; i < num; ++i){
    sum += array[i];
  }
  return((float)sum / num);
}

void add_sample(int sample, int *array, int num){
  for(int i = 0; i < num - 1; ++i){
    array[i] = array[i+1];
  }

  array[num-1] = sample;

}

float std_deviation(int *array, int num){
  float avg = get_avg(array, num);

  float sum = 0.f;
  float curr = 0.f;

  for(int i = 0; i < num; ++i){
    curr = (float)array[num] - avg; // do calculation once for efficiency
    sum += curr * curr; // square it
  }

  sum = sum / (num);

  return sqrtf(sum);

}