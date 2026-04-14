#include <Arduino.h>
#include "AS5600.h"
#include <ESP32Encoder.h>
#include <Pid.h>


#define CLK 25 // CLK ENCODER 
#define DT 26 // DT ENCODER 


AS5600 as5600; // using those libararies to set up instances for my encoders

ESP32Encoder n20;

int signalA = 18; // DC motors have 2 polarities. These basically just refer to what direction to spin
int signalB = 19;
float pendOffset = 74;

int deadzone = 55; // my dc motor doesn't move until a minimum of 55 is applied on the analogWrite
// basically so our motor is always moving, to make microadjustments


// in ORDER: P, I, D, Max angle, min angle
PID arm_pid(0.5f, 0.0f, 0.11f, 340.0f, -340.0f); // setting up class instances with these parameters
PID pend_pid(10.0f, 0.0f, 0.1f, 208.0f, 168.0f);
unsigned long lastLoopTime = 0; // using a variable here to keep the poll rate (hertz), always constant to keep dt stable

void setup() {
    
// basically just setting up some basics for the libararies, also setting the i2c speed for the encoders, plus serial speed 
  n20.attachHalfQuad ( DT, CLK );
  n20.setCount (0);
  pinMode(signalA, OUTPUT);
  pinMode(signalB, OUTPUT);
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(50000); 
  as5600.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

if (micros() - lastLoopTime >= 1000) {
  lastLoopTime = micros(); // regulates our loop rate, better than using delay

  float current_time = micros() / 1000000.0f; // finds current time (VERY important)

  // getting raw sensor data and stuff, also making a variable linked to that class instance with the method in it
  float rawdeg = (as5600.readAngle() * AS5600_RAW_TO_DEGREES);
  float current_pend_deg = fmodf(rawdeg + pendOffset, 360.0f);
  float current_arm_deg = n20.getCount() / 2.0f;
  float arm_output = arm_pid.calculation(0, current_arm_deg, current_time, false);
  float pend_output = pend_pid.calculation(185, current_pend_deg, current_time, true);

  float totalOutput = -arm_output + pend_output;
// total output that goes to the motor, arm output is negative here since these 2 technically counteract each other

  bool safeRange = ((current_pend_deg > 165 && current_pend_deg < 205) && ((current_arm_deg < 340) && current_arm_deg > -340));
  // only writes to the motor when the sensor values are within these parameters, its a hardcoded safety to prevent snapping wires, etc
  if (safeRange){

  if (totalOutput > 0){
    // writes in the appropriate direction
    analogWrite(signalA, constrain(totalOutput + deadzone, 0, 255));
    analogWrite(signalB, 0);
  }else{
    analogWrite(signalA, 0);
    analogWrite(signalB, constrain(fabsf(totalOutput - deadzone), 0, 255));
    // fabsf (absolute val) from arduino libarary to use absolute value, (you cant write a negative pwm val to a motor)
  }
}
  else{
    totalOutput = 0; // if not, sets the output to 0 for safety, the motor doesnt move.

  }
  

}
}
