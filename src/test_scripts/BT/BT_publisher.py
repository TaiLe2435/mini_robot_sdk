#################################################################
# Tommy Le
# Northern Illinois University, Mechanical Engineering Dept
# 6/19/2022
# This is a basic example showing how serial communication works.
# This code is to be ran with readDatafromPython.ino
#################################################################

import serial      #lib for serial communication
import time

arduinoData = serial.Serial('COM7', 9600)  #initializing port
arduinoData.reset_input_buffer()

while True:         #inf loop
    cmd = input('Please Enter a Number: ') #take user input
    cmd = cmd + '\r'    #add \r to indicate end of string
    arduinoData.write(cmd.encode()) #write to serial port
    inVar = arduinoData.read_until().decode()
    # time.sleep(0.1)
    print("IN: " + str(inVar))

# x = 0

# for x in range(50):
#     arduinoData.write(bytes(str(x), 'utf_8'))
#     inVar = arduinoData.read_until().decode()
#     print("OUT: " + str(x) + " IN: " + str(inVar))
#     time.sleep(0.5)
arduinoData.close()