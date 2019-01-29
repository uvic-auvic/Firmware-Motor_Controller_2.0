import datetime as dt
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import time

comPortName = "/dev/ttyUSB0"

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ys = []
xs = []

try:
	print("Opening serial port: " + comPortName)
	com = serial.Serial(comPortName, 9600)
except serial.SerialException as e:
	print("Could not open serial port. Exiting...")
	quit()

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):
    ys.append(get_rpm())
    # ys = ys[-20:]
    ax.clear()
    ax.plot(ys)

def get_rpm():
    com.write("RV1\n".encode("ASCII"))
    receivedString = com.readline().decode("ASCII").rstrip("\r\n")
    print(receivedString)
    print(type(receivedString))
    return int(receivedString)

def set_rpm(rpm):
    command = "M1" + "F" + str(rpm) + "\n"
    com.write(command.encode("ASCII"))

set_rpm(20)
# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=10)

# for i in range(0, 100):
    # ys.append(get_rpm())

# ax.plot(ys)
plt.show()
com.write("STP\n".encode("ASCII"))
time.sleep(1)