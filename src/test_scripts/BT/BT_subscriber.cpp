#include <Arduino.h>
#include <BluetoothSerial.h>
   
BluetoothSerial SerialBT;
    
void setup()
{
  SerialBT.begin("ESP32test");
  delay(1000);
}
    
void loop()
{
  String inputFromOtherSide;
  if (SerialBT.available() > 0) {
    inputFromOtherSide = SerialBT.readStringUntil('\n').toInt() + 100;
    SerialBT.println(inputFromOtherSide);
  }
  delay(50);
}