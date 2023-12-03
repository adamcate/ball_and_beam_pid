#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

#include "../include/PID_v1.h"
#include "filter.h"



#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define SERVO_PIN    9

#define PING_INTERVAL 15

#define MAX_DISTANCE 30 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MEDIAN_SAMPLES 10

int samples[MEDIAN_SAMPLES]{};
int diff_samples[MEDIAN_SAMPLES - 1]{};

int sorted_samples[MEDIAN_SAMPLES]{};

// Define vars we'll be connecting to
double Setpoint, Input, Output;

const double Akp = -0.0641, Aki = -0.0013, Akd = -0.6467;
const double Ckp = 0.2, Cki = 1, Ckd = 0.15;

double Kp = 0.03144, Ki = 0.01752, Kd = 0.02;
//double Kp = 0, Ki = 0, Kd = 0.05;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo myServo;

int last_dist = 0;
int dist = 0; // the distance we want to measure

int outlier_ctr = 0;

long last_time = 0;
long curr_time = 0;

void prune_outliers(){

  for(int i = 0; i < MEDIAN_SAMPLES - 1; ++i){
    diff_samples[i] = samples[i+1] - samples[i];
  }

  float stddev = std_deviation(diff_samples, MEDIAN_SAMPLES - 1);
  float avg = get_avg(diff_samples, MEDIAN_SAMPLES - 1);

  for(int i = 1; i < MEDIAN_SAMPLES - 1; ++i){
    if((float)diff_samples[i] >= avg + stddev || (float)diff_samples[i] <= avg - stddev){
      diff_samples[i] = diff_samples[i-1];
    }
    samples[i] = samples[i-1] + diff_samples[i-1];
    samples[i] = constrain(samples[i],0,MAX_DISTANCE);
  }
  samples[MEDIAN_SAMPLES - 1] = samples[MEDIAN_SAMPLES - 2] + diff_samples[MEDIAN_SAMPLES - 2];
  samples[MEDIAN_SAMPLES - 1] = constrain(samples[MEDIAN_SAMPLES - 1],0,MAX_DISTANCE);
}

void setup() {
  Serial.begin(9600);
  Input = sonar.ping_cm();
  dist = Input; last_dist = Input;

  myServo.attach(SERVO_PIN);
  myServo.write(45); delay(3000);
  Setpoint = 9;

  myPID.SetOutputLimits(-3.142, 3.142);
  myPID.SetDerivativeFilterSamples(10);

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(15);
}

void loop() { 
  curr_time = millis();

  last_dist = dist;
  
  if(curr_time - last_time >=  PING_INTERVAL){ 
    last_time = curr_time;  
    dist = sonar.ping_cm();

    //Serial.println(dist);

    if(dist > 27 && dist < 30){dist = 27;}

    add_sample(dist, samples, MEDIAN_SAMPLES);
    outlier_ctr++;

    if(outlier_ctr >= MEDIAN_SAMPLES){
      //prune_outliers();
      outlier_ctr = 0;
    }
    for(int i = 0; i < MEDIAN_SAMPLES; i++){
      sorted_samples[i] = samples[i];
    }
  }
  
  Input = get_median(sorted_samples, MEDIAN_SAMPLES);
  
  /*if(abs(Input - Setpoint) <= 4){
    Kp = Ckp; Ki = Cki; Kd = Ckd;
  }else{
    Kp = Akp; Ki = Aki; Kd = Akd;
  }*/

  if(abs(Input - Setpoint) <= 1){
    Input = Setpoint;
  }
  myPID.Compute();
  myServo.write(92-constrain(degrees(Output),-45,45));
  Serial.print(Input); Serial.print(" "); Serial.println(degrees(Output));
}