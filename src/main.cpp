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

#define POTENTIOMETER_SAMPLES 20

int samples[MEDIAN_SAMPLES]{};
int diff_samples[MEDIAN_SAMPLES - 1]{};

int sorted_samples[MEDIAN_SAMPLES]{};

int pot_samples[POTENTIOMETER_SAMPLES]{};


// Define vars we'll be connecting to
double Setpoint, Input, Output;

double Kp = 0.03144, Ki = 0.0252, Kd = 0.03;
//double Kp = 0, Ki = 0, Kd = 0.05;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo myServo;

int last_dist = 0;
int dist = 0; // the distance we want to measure

long last_time = 0;
long curr_time = 0;

void setup() {
  Serial.begin(9600);
  Input = sonar.ping_cm();
  dist = Input; last_dist = Input;

  myServo.attach(SERVO_PIN);
  myServo.write(70); delay(2000);

  myPID.SetOutputLimits(-3.142, 3.142);
  myPID.SetDerivativeFilterSamples(15);

  myPID.SetMode(AUTOMATIC);

  for(int i = 0; i < POTENTIOMETER_SAMPLES; ++i){
    int pot_sample = (int)(analogRead(A1)/1023.f*18.f + 7.f);

    add_sample(constrain(pot_sample,7,25), pot_samples, POTENTIOMETER_SAMPLES);
    Setpoint = (int)get_avg(pot_samples, POTENTIOMETER_SAMPLES);
    delay(30);
  }

  myPID.SetSampleTime(15);
}

void loop() { 
  curr_time = millis();

  last_dist = dist;
  
  if(curr_time - last_time >=  PING_INTERVAL){
    Serial.print("s");Serial.print(Setpoint);Serial.print("i"); Serial.print(Input);Serial.print("o");Serial.println((int)degrees(Output));
    int pot_sample = (int)(analogRead(A1)/1023.f*18.f + 7.f);

    add_sample(constrain(pot_sample,7,25), pot_samples, POTENTIOMETER_SAMPLES);
    Setpoint = (int)get_avg(pot_samples, POTENTIOMETER_SAMPLES);

    last_time = curr_time;  
    dist = sonar.ping_cm();

    //Serial.println(dist);

    if(dist > 27 && dist < 30){dist = 27;}

    add_sample(dist, samples, MEDIAN_SAMPLES);

    for(int i = 0; i < MEDIAN_SAMPLES; i++){
      sorted_samples[i] = samples[i];
    }
  }
  
  Input = get_median(sorted_samples, MEDIAN_SAMPLES);

  if(abs(Input - Setpoint) <= 1){
    Input = Setpoint;
  }

  myPID.Compute();

  int angle = constrain(degrees(Output),-45,45);
  myServo.write(92-angle);
}