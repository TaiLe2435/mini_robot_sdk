#include "bluetooth.h"

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <iostream>
#include <string>
#include <sstream>

BluetoothSerial SerialBT;

void BTinit()
{

  SerialBT.begin("mini_robot");
  delay(1000);
  
}

int getBT()
{
    String input = "0";
    if (SerialBT.available() > 0) 
    {

        input = SerialBT.readStringUntil('\n').toInt();

        int val= input.toInt();
        return val;
    }
    return 9999;
}

