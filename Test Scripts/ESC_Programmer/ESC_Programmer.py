import serial
import struct
import time

#AUVIC
#Application to program the ESCs used to drive the motors

#global defines
comPortName = "COM3"
motorNumber = "5"
commandHeader = "PW" + motorNumber
maxThrottle = "120"
neturalThrottle = "090"
minThrottle = "060"

#def
def setMaxThrottle():
	com.write(commandHeader + maxThrottle + "\n")
	
def setMinThrottle():
	com.write(commandHeader + minThrottle + "\n")

def setNeutralThrottle():
	com.write(commandHeader + neturalThrottle + "\n")

#end def

#start script
print("")
print("**ESC Programmer Application**")
print("")

try:
	print("Opening serial port: " + comPortName)
	com = serial.Serial(comPortName, 9600)
except serial.SerialException as e:
	print("Could not open serial port. Exiting...")
	quit()

raw_input("Continue")	

com.write("DEBUG\n")
setMaxThrottle()


while 1:

	input = raw_input("Select option: ")
	if input == "m" or input == "M":
		setMaxThrottle()
		print("Setting Max throttle")
		print("")
	elif input == "n" or input == "N":
		setNeutralThrottle()
		print("Setting Neutral throttle")
		print("")
		pass
	elif input == "b" or input == "B":
		setMinThrottle()
		print("Setting Min throttle")
		print("")
	elif input[0] == "!":
		com.write(input[1:] + "\n")
		print("")
print("Closing serial port")
com.close()
print("Exiting...")
