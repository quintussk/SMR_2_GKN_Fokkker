import serial

# Configure the serial port
ser = serial.Serial(
    port='COM9',  # Replace with your serial port
    baudrate=9600,
    timeout=1
)

try:
    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8').rstrip()
            print(message)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()