#pragma once

// Authored and created by: Soham Dave
// Date: April 12, 2026



#define PID_H

#include <Arduino.h>

class PID { // setting private vars within the class
    private:
        float kp;
        float ki;
        float kd;

        float integral;
        float last_error;
        float last_time;

        float maxAngle;
        float minAngle;
    
    public:
        PID(float p, float i, float d, float mxAngle, float mnAngle){ // public function to create an instance of that class
            kp = p;
            ki = i;
            kd = d;

            integral = 0.0f;
            last_error = 0.0f;
            last_time = 0.0f;

            maxAngle = mxAngle;
            minAngle = mnAngle;

        }

        float calculation(float target, float current_deg, float current_time, bool modulus){ // method within it, does the PID Math
            if (last_time == 0.0f){
                last_time = current_time;
            }

            float dt = current_time - last_time;
            float error;

            if (modulus) {
                
                error = fmodf((target - current_deg + 180.0f), 360.0f);
                if (error < 0) error += 360.0f; 
                error -= 180.0f;
            }
            else {
                error = target - current_deg;
            }

            float derivative = 0.0f;
            if (dt > 0.0f){
                derivative = (error - last_error)/dt;
            }



            integral += error*dt;


            

            float output = (kp*error) + (ki*integral) + (kd*derivative);

            last_error = error;
            last_time = current_time;

            return output;



        }

};