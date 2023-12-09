// #include <Arduino.h>
// #include <DRV8835MotorShield.h>

/*
 * This example uses the DRV8835MotorShield library to drive each motor with the
 * Pololu DRV8835 Dual Motor Driver Shield for Arduino forward, then backward. 
 * The yellow user LED is on when a motor is set to a positive speed and off when
 * a motor is set to a negative speed.
 */
// const int LED_PIN {2};
// const int M1DIR {27};
// const int M1PWM {15};
// const int M2DIR {25};
// const int M2PWM {26};
// const int MD {23};

// DRV8835MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);

// void setup()
// {
//   pinMode(LED_PIN, OUTPUT);
  
//   // uncomment one or both of the following lines if your motors' directions need to be flipped
//   //motors.flipM1(true);
//   //motors.flipM2(true);
// }

// void loop()
// {
//   // run M1 motor with positive speed
  
//   digitalWrite(LED_PIN, HIGH);
  
//   for (int speed = 0; speed <= 400; speed++)
//   {
//     motors.setM1Speed(speed);
//     delay(20);
//   }

//   for (int speed = 400; speed >= 0; speed--)
//   {
//     motors.setM1Speed(speed);
//     delay(20);
//   }
  
//   // run M1 motor with negative speed
  
//   digitalWrite(LED_PIN, LOW);
  
//   for (int speed = 0; speed >= -400; speed--)
//   {
//     motors.setM1Speed(speed);
//     delay(20);
//   }
  
//   for (int speed = -400; speed <= 0; speed++)
//   {
//     motors.setM1Speed(speed);
//     delay(20);
//   }

//   // run M2 motor with positive speed
  
//   digitalWrite(LED_PIN, HIGH);
  
//   for (int speed = 0; speed <= 400; speed++)
//   {
//     motors.setM2Speed(speed);
//     delay(20);
//   }

//   for (int speed = 400; speed >= 0; speed--)
//   {
//     motors.setM2Speed(speed);
//     delay(20);
//   }
  
//   // run M2 motor with negative speed
  
//   digitalWrite(LED_PIN, LOW);
  
//   for (int speed = 0; speed >= -400; speed--)
//   {
//     motors.setM2Speed(speed);
//     delay(20);
//   }
  
//   for (int speed = -400; speed <= 0; speed++)
//   {
//     motors.setM2Speed(speed);
//     delay(20);
//   }
  
//   delay(500);
// }