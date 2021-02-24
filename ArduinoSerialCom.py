import serial
import time

arduino = serial.Serial(port='/dev/cu.usbmodem14601', baudrate = 9600, timeout = 1)
time.sleep(3)

def write_read(num):
    arduino.write(bytes(num, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data

while True:
    goal_speed = input("Enter rotation speed (multiple of 100 RPM): ") # rotation speed input
    while int(goal_speed) % 100 != 0:  # speed must be a multiple of 100 RPM, ask again
        goal_speed = input("Enter rotation speed (multiple of 100 RPM): ")
    speed = write_read(goal_speed)
    print(speed)

    goal_time = input("Enter centrifugation duration (minutes): ") # duration input
    duration = write_read(goal_time)
    print(duration)