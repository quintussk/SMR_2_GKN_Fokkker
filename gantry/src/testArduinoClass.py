from arduino import ArduinoConnection
arduino = ArduinoConnection(port="COM9")

while True:
    feedback = arduino.read_feedback()
    print (feedback)

    
