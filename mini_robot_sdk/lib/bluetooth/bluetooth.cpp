#include "bluetooth.h"

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <iostream>
#include <string>
#include <sstream>

int myArray[3];
bool newData = false;
byte* ddata = reinterpret_cast<byte*>(&myArray);
size_t pcDataLen = sizeof(myArray);

BluetoothSerial SerialBT;

void BTinit()
{

  SerialBT.begin("mini_robot");
  delay(1000);
  
}

byte* getBT()
{

    if (SerialBT.available() >= pcDataLen)
    {
      byte inByte;
      for (byte n = 0; n < pcDataLen; n++){
        ddata[n] = SerialBT.read();
      }
      while (SerialBT.available() > 0){
        byte dumpByte = SerialBT.read();
      }
      newData = true;
    }

    return ddata;
}

