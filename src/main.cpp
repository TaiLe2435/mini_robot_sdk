#include <Arduino.h>
#include "IMU/IMU.h"
#include "motion/motion.h"
#include "bluetooth/bluetooth.h"


// put function declarations here:
int myFunction(int, int);

// void setup() {
//  Serial.begin(115200);
//   // put your setup code here, to run once:
//   int result = myFunction(2, 3);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   int temp = myFunction(1,1);
// }

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}