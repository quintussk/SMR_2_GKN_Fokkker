import serial
import time

# Set up the serial connection
ser = serial.Serial('COM3', 9600)  # Replace 'COM3' with your Arduino's port (e.g., '/dev/ttyUSB0' for Linux)
time.sleep(2)  # Wait for the Arduino to reset

# Read data from Arduino
while True:
    if ser.in_waiting > 0:  # Check if there is data available from Arduino
        data = ser.readline().decode('utf-8').strip()
        print(f"Received from Arduino: {data}")

    # Send data to Arduino
    ser.write(b'Hello Arduino!\n')
    time.sleep(1)
