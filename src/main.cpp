#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

#include "../include/PID_v1.h"
#include "filter.h"



#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define SERVO_PIN    9

#define PING_INTERVAL 50

#define MAX_DISTANCE 30 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MEDIAN_SAMPLES 15
#define OUTLIER_THRESHOLD 300
#define OUTLIER_REJECT_CYCLES 5 

int samples[MEDIAN_SAMPLES]{};
int sorted_samples[MEDIAN_SAMPLES]{};

// Define vars we'll be connecting to
double Setpoint, Input, Output;

const double Akp = 1.2, Aki = 1, Akd = 1;
const double Ckp = 0.2, Cki = 1, Ckd = 0.15;

double Kp = 1.1, Ki = 1, Kd = 0.85;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo myServo;

int last_dist = 0;
int dist = 0; // the distance we want to measure

int outlier_ctr = 0;

long last_time = 0;
long curr_time = 0;

void setup() {
  Serial.begin(9600);
  Input = sonar.ping_cm();
  dist = Input; last_dist = Input;

  myServo.attach(SERVO_PIN);
  myServo.write(45); delay(3000);
  Setpoint = 15;

  myPID.SetOutputLimits(-45, 45);

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100);
}

void loop() {
  curr_time = millis();

  last_dist = dist;
  
  if(curr_time - last_time >=  PING_INTERVAL){ 
    last_time = curr_time;  
    dist = sonar.ping_cm();

    //Serial.println(dist);

    if(dist > 27 && dist < 30){dist = 27;}
    /*if(abs(dist-last_dist) >= OUTLIER_THRESHOLD || outlier_ctr != 0) 
      {dist = last_dist + ((float)(dist-last_dist) * 0.1);
      outlier_ctr++;
      if(outlier_ctr >= OUTLIER_REJECT_CYCLES){outlier_ctr = 0;}
    }*/
    //if(dist == 0) add_sample(samples[MEDIAN_SAMPLES - 1], samples, MEDIAN_SAMPLES);
    add_sample(dist, samples, MEDIAN_SAMPLES);

    for(int i = 0; i < MEDIAN_SAMPLES; i++){
      sorted_samples[i] = samples[i];
    }

  }
  
  Input = get_median(sorted_samples, MEDIAN_SAMPLES);
  
  if(abs(Input - Setpoint) <= 4){
    Kp = Ckp; Ki = Cki; Kd = Ckd;
  }else{
    Kp = Akp; Ki = Aki; Kd = Akd;
  }

  if(abs(Input - Setpoint) <= 1){
    Input = Setpoint;
  }
  myPID.Compute();

  myServo.write((int)((float)85 - (float)Output));

  Serial.print("Input:\t"); Serial.print(Input); Serial.print(" Output:\t"); Serial.println(Output);
}