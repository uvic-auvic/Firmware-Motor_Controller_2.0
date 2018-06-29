import serial
import time

com1 = serial.Serial("/dev/ttyUSB0", 9600)

expectedString = "Motor Controller\r\n"

sucessCount = 0
errorCount = 0

while 1:

	com1.write("RID\n")
	time.sleep(0.5)
	
	receivedString = com1.read(com1.inWaiting())

	if receivedString == expectedString:
		
		sucessCount = sucessCount + 1
		sucessPercentage = (sucessCount * 100)/(sucessCount + errorCount)
		print("SUCESS COUNT:" + str(sucessCount) + " SUCESS RATE: " + str(sucessCount) + "/" + str(sucessCount + errorCount) + " " + str(sucessPercentage) + "%")

	else:

		errorCount = errorCount + 1;
		sucessPercentage = (sucessCount * 100)/(sucessCount + errorCount)
		print("ERROR COUNT:" + str(errorCount) + " SUCESS RATE: " + str(sucessCount) + "/" + str(sucessCount + errorCount) + " " + str(sucessPercentage) + "%")
		print(receivedString)
