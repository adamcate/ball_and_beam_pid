/*
MECH 3032 Final Project - Ball and Beam control systems

Libraries used:
  * Servo
  * NewPing - better ultrasonic support (https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home)
  * Arduino-PID-Library (https://github.com/br3ttb/Arduino-PID-Library/tree/master)

IDE used: Visual Studio Code w/ PlatformIO extension for Arduino support
*/

#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

#include "../include/PID_v1.h"
#include "filter.h"


#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define SERVO_PIN    9   // Arduino pin tied to the servo motor

#define PING_INTERVAL 15
#define PID_SAMPLE_TIME 15

#define MAX_DISTANCE 30     // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MEDIAN_SAMPLES 10   // The number of samples used for the ultrasonic median filter. More samples means reduced noise, but with greater input delay.

// used for the potentiometer knob filter
#define POTENTIOMETER_SAMPLES 20

// Since noise in the PID derivative calculation causes massive instability, this defines the number of samples
// to be used on the averaging filter on the derivative component.
#define DERIVATIVE_FILTER_SAMPLES 15

// stores a rolling set of ultrasonic samples. Used for median filtering the very noisy ultrasonic input
int samples[MEDIAN_SAMPLES]{};

// the list that get_median writes to
int sorted_samples[MEDIAN_SAMPLES]{};

int pot_samples[POTENTIOMETER_SAMPLES]{}; // store the raw potentiometer samples, which will be filtered for stability


// define the PID variables.
double Setpoint, Input, Output;

// represents the proportional, integral, and derivative gain values
double Kp = 0.03144, Ki = 0.0252, Kd = 0.02;

// since the servo may not be 100% level at 90 degrees, the value that produces a level beam is found experimentally
const int servo_level_angle = 92;

// initialize the PID object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// instantiate the servo object
Servo myServo;

int dist = 0; // the distance we want to measure

long last_time = 0;
long curr_time = 0;

void setup() {
  Serial.begin(9600);

  Input = sonar.ping_cm();              // get an initial ultrasonic reading
  dist = Input;

  myServo.attach(SERVO_PIN);            // initialize the servo
  myServo.write(70); delay(2000);       // tilt the beam to 70 degrees initially, making sure the roller has to be stabilized

  myPID.SetOutputLimits(-3.142, 3.142); // set the output to -pi, pi for the sake of easier analysis
  myPID.SetDerivativeFilterSamples(DERIVATIVE_FILTER_SAMPLES); // set the number of averaging filters for the derivative gain

  myPID.SetMode(AUTOMATIC);             // set the PID controller to automatic mode

  // fill the potentiometer sample array with values to prevent setpoint from starting at zero
  for(int i = 0; i < POTENTIOMETER_SAMPLES; ++i){
    int pot_sample = (int)(analogRead(A1)/1023.f*18.f + 7.f);

    add_sample(constrain(pot_sample,7,25), pot_samples, POTENTIOMETER_SAMPLES);
    
    delay(30);
  }

  Setpoint = (int)get_avg(pot_samples, POTENTIOMETER_SAMPLES); // calculate the filtered setpoint

  myPID.SetSampleTime(PID_SAMPLE_TIME); // set the PID calculation sample rate
}

void loop() { 
  curr_time = millis();

  
  // instead of using delay(), keep track of last and current time and only take sensor readings every PING_INTERVAL
  if(curr_time - last_time >=  PING_INTERVAL){
    
    last_time = curr_time;

    // print some basic info to the serial monitor. This is what the processing sketch reads
    // for its own visualization.
    // seperating by commas makes it easier to parse on the processing end of things.
    Serial.print((int)Setpoint); Serial.print(","); Serial.print((int)Input);Serial.print(",");Serial.println((int)degrees(Output));

    int pot_sample = (int)(analogRead(A1)/1023.f*18.f + 7.f); // convert the analog input to a setpoint value between 7 and 25

    // add the current potentiometer sample to the list of sample, constraining from 7cm - 25cm
    add_sample(constrain(pot_sample,7,25), pot_samples, POTENTIOMETER_SAMPLES);
    Setpoint = (int)get_avg(pot_samples, POTENTIOMETER_SAMPLES); // apply the averaging filter to the potentiometer samples to get a stable setpoint
 
    dist = sonar.ping_cm(); // use NewPing to poll the ultrasonic sensor

    if(dist > 27 && dist < 30){dist = 27;} // clamp the ultrasonic output

    add_sample(dist, samples, MEDIAN_SAMPLES); // add the current distance sample to the working list

    for(int i = 0; i < MEDIAN_SAMPLES; i++){ // copy the sample array to the array to be sorted before running get_median
      sorted_samples[i] = samples[i];
    }

    Input = get_median(sorted_samples, MEDIAN_SAMPLES); // apply the median filter, and set the PID input
  }

  myPID.Compute(); // compute the PID output. will run only every 15 ms, per myPID.setSampleTime() in void setup()

  int angle = constrain(degrees(Output),-45,45); // convert the PID output to degrees, and clamp to -45, 45 degrees
  myServo.write(servo_level_angle - angle);      // write the output value to the servo motor, taking into account the level point
}