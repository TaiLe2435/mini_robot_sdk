#include "motion.h"

#include <Arduino.h>
#include <DRV8835MotorShield.h>

const int LED_PIN {2};
const int M1DIR {27};
const int M1PWM {15};
const int M2DIR {25};
const int M2PWM {26};
const int MD {23};

DRV8835MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);

void init_motors()
{
    // uncomment one or both if motor directions need to be changed
    // motors.flipM1(true);
    // motors.flipM2(true);
    return;
}

void stop()
{
    motors.setM1Speed(0);
    motors.setM1Speed(0);
    return;
}

void move(int v, int theta)
{
    // add code to calculate left and right speeds
    int vL {0};
    int vR {0};

    if( (abs(vL) || abs(vR)) > 400)
    {
        Serial.print("MotionError: Speed commands is  Speeds can only be from 400 to -400");
        Serial.print(vL);
        Serial.print(" and ");
        Serial.print(vR);
        Serial.print(". Speeds can only be from 400 to -400.");
        return;
    }

    static unsigned long pulse = 0;
    if(millis() - pulse > 20) {
        motors.setM1Speed(vL);
        motors.setM2Speed(vR);
        pulse = millis();

    } 
    return;
}

void move_wheels(int vL, int vR)
{
    if( (abs(vL) || abs(vR)) > 400)
    {
        Serial.print("MotionError: Speed commands is  Speeds can only be from 400 to -400");
        Serial.print(vL);
        Serial.print(" and ");
        Serial.print(vR);
        Serial.print(". Speeds can only be from 400 to -400.");
        return;
    }

    static unsigned long pulse = 0;
    if(millis() - pulse > 20) {
        motors.setM1Speed(vL);
        motors.setM2Speed(vR);
        pulse = millis();

    } 
    return;
}