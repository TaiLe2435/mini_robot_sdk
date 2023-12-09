import serial      #lib for serial communication
import time

arduinoData = serial.Serial('COM7', 115200)  #initializing port | or COM4
arduinoData.reset_input_buffer()

while True:         #inf loop

    cmd = input('Please Enter a Number: ') #take user input
    cmd = cmd + '\n'    #add \r to indicate end of string
    arduinoData.write(cmd.encode()) #write to serial port
    
arduinoData.close()