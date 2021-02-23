import serial
import time

arduino = serial.Serial(port='/dev/cu.usbmodem14601', baudrate = 9600, timeout = 1)
time.sleep(3)

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data

while True:
    num = input("Enter rotation speed (multiple of 100 RPM): ") # rotation speed input
    value = write_read(num)
    print(value)