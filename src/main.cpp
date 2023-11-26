#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

#include "../include/PID_v1.h"



#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define SERVO_PIN    9

#define MAX_DISTANCE 30 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MEDIAN_SAMPLES 15
#define OUTLIER_THRESHOLD 8

int samples[MEDIAN_SAMPLES]{};
int sorted_samples[MEDIAN_SAMPLES]{};

// Define vars we'll be connecting to
double Setpoint, Input, Output;

double Kp = 1.3, Ki = 1.1, Kd = 0.7;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo myServo;

int last_dist = 0;
int dist = 0; // the distance we want to measure

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

void setup() {
  Serial.begin(9600);
  Input = sonar.ping_median(3);
  dist = Input; last_dist = Input;

  myServo.attach(SERVO_PIN);
  myServo.write(45); delay(3000);
  Setpoint = 15;

  myPID.SetOutputLimits(-45, 45);

  myPID.SetMode(AUTOMATIC);
}

void loop() {
  delay(15);
  last_dist = dist;
  dist = sonar.ping_cm();

  if(dist > 27 && dist < 30){dist = 27;}
  //if(abs(dist-last_dist) >= OUTLIER_THRESHOLD) dist = last_dist + ((float)(dist-last_dist) * 0.2);
  add_sample(dist, samples, MEDIAN_SAMPLES);

  for(int i = 0; i < MEDIAN_SAMPLES; i++){
    sorted_samples[i] = samples[i];
  }

  
  Input = get_median(sorted_samples, MEDIAN_SAMPLES);
  if(abs(Input - Setpoint) <= 2) Input = Setpoint;
  myPID.Compute();

  myServo.write((int)((float)85 - (float)Output));
}