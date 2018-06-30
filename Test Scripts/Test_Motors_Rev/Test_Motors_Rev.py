import serial
import struct
import time

#AUVIC
#Application to sweep the motors through various speeds, forward and reverse

#global defines
comPortName = "COM3"

#start script
print("")
print("**Test Motors Rev Application**")
print("")

try:
	print("Opening serial port: " + comPortName)
	com = serial.Serial(comPortName, 9600)
except serial.SerialException as e:
	print("Could not open serial port. Exiting...")
	quit()
	
for i in range(8):
	
	print("Motor Number: " + str(i + 1))
	for j in range(10):
		command = "M" + str(i + 1) + "F" + str(j * 10) + "\n"
		com.write(command)
		time.sleep(0.3)
	com.write("STP\n")
	time.sleep(1)
	
	for j in range(10):
		command = "M" + str(i + 1) + "R" + str(j * 10) + "\n"
		com.write(command)
		time.sleep(0.3)
		
	com.write("STP\n")
	time.sleep(1)