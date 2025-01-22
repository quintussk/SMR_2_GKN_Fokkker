import serial
import time
import asyncio

class ArduinoConnection:
    def __init__(self, port, baudrate=9600, timeout=1):
        try:
            self.connection = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            time.sleep(2)  # Wacht even tot de verbinding stabiel is
            print(f"Verbonden met Arduino op {port}")
            self.direction1 = 0
            self.speed1 = 0
            self.direction2 = 0
            self.speed2 = 0
            self.request_values()
            self.camera_homed = False
            
        except serial.SerialException as e:
            print(f"Cannot connect to Arduino: {e}")
            self.connection = None

    def send_command(self, command):
        """
        Sends a command to the Arduino via the serial connection.
        """
        if self.connection:
            try:
                self.connection.write(f"{command}\n".encode())
                print(f"Command '{command}' sent to Arduino")
            except Exception as e:
                print(f"Error sending command: {e}")
        else:
            print("No active connection to Arduino")

    def send_steps(self, horizontal_step, vertical_step):
        """
        Sends the step values to the Arduino via the serial connection.
        """
        if self.connection:
            try:
                command = f"H{horizontal_step}V{vertical_step}"
                self.connection.write(f"{command}\n".encode())
                print(f"Steps '{command}' sent to Arduino")
            except Exception as e:
                print(f"Error sending steps: {e}")
        else:
            print("No active connection to Arduino")

    def change_speed(self, speed_value):
        if speed_value == "increase":
            self.speed1 += 50
            self.speed2 += 50
        elif speed_value == "decrease":
            self.speed1 -= 50
            self.speed2 -= 50
        self.speed1 = max(0, min(4000, self.speed1))
        self.speed2 = max(0, min(4000, self.speed2))
        self.send_command(f"SPD1:{self.speed1}")
        self.send_command(f"SPD2:{self.speed2}")

    def read_feedback(self):
        """
        Leest feedback van de Arduino en werkt de interne variabelen bij.
        """
        if self.connection and self.connection.in_waiting > 0:
            try:
                feedback = self.connection.readline().decode().strip()
                print(f"Feedback ontvangen: {feedback}")
                if feedback.startswith("Motor 1"):
                    if "Direction = 1" in feedback:
                        self.direction1 = 1
                    elif "Direction = -1" in feedback:
                        self.direction1 = -1
                    if "Speed =" in feedback:
                        self.speed1 = int(feedback.split("Speed = ")[1])
                elif feedback.startswith("Motor 2"):
                    if "Direction = 1" in feedback:
                        self.direction2 = 1
                    elif "Direction = -1" in feedback:
                        self.direction2 = -1
                    if "Speed =" in feedback:
                        self.speed2 = int(feedback.split("Speed = ")[1])
            except Exception as e:
                print(f"Fout bij het lezen van feedback: {e}")

    async def Wait_For_Location_Reached(self):
        """
        Continuously listens to the Arduino until the 'Steppers reached location' message is received.
        """
        print("Waiting for confirmation from Arduino...")
        while True:
            if self.connection and self.connection.in_waiting > 0:
                try:
                    # Read feedback from the Arduino
                    feedback = self.connection.readline().decode().strip()
                    print(f"Feedback received: {feedback}")
                    # Check if the steppers have reached their location
                    if feedback == "Steppers reached location":
                        print("Confirmation received: Steppers have reached their destination!")
                        return True
                except Exception as e:
                    print(f"Error reading feedback: {e}")
            await asyncio.sleep(0.1)  # Prevent CPU overuse by adding a short delay

    def request_values(self):
        """
        Vraagt de huidige waarden van de Arduino op en leest de feedback.
        """
        self.send_command("values")
        time.sleep(0.1)  # Wacht even op de reactie
        while self.connection.in_waiting > 0:
            self.read_feedback()

    def move_manual(self, direction):
        """
        Verplaatst handmatig de motors in een bepaalde richting.
        """
        if self.connection:
            directions = {
                'up': "DIR1:1\nDIR2:0",
                'down': "DIR1:-1\nDIR2:0",
                'left': "DIR2:1\nDIR1:0",
                'right': "DIR2:-1\nDIR1:0",
                'up-left': "DIR1:1\nDIR2:1",
                'up-right': "DIR1:1\nDIR2:-1",
                'down-left': "DIR1:-1\nDIR2:1",
                'down-right': "DIR1:-1\nDIR2:-1",
                'stop': "DIR1:0\nDIR2:0"
            }
            if direction in directions:
                self.send_command(directions[direction])
            else:
                print("Ongeldige richting")

    async def change_speed_motor(self, motor: str, speed: int):
        """
        Stelt de snelheid in voor een specifieke motor.
        SPD1 = Mold motor
        SPD2 = Camera motor
        """
        if self.connection:
            if motor == "Mold":
                self.speed1 = speed
                self.send_command(f"SPD1:{speed}")
            elif motor == "Camera":
                self.speed2 = speed
                self.send_command(f"SPD2:{speed}")
            else:
                print("Ongeldige motor")


    async def check_camera(self) -> bool:
        """
        Controleert of de camera motor thuis is.
        """
        self.send_command("CAMERA:POS")
        while True:
            if self.connection and self.connection.in_waiting > 0:
                try:
                    # Read feedback from the Arduino
                    feedback = self.connection.readline().decode().strip()
                    print(f"Feedback received: {feedback}")
                    # Check if the steppers have reached their location
                    if feedback == "Camera on position":
                        print("Confirmation received: Camera is on position")
                        return True
                    elif feedback == "Camera not on position":
                        print("Confirmation received: Camera is not on position" )
                        return False
                except Exception as e:
                    print(f"Error reading feedback: {e}")
            await asyncio.sleep(0.1)  # Prevent CPU overuse by adding a short delay

    async def home_camera(self): 
        """
        Zet de camera motor naar de home positie.
        """
        if self.camera_homed:
            return
        self.send_command("H0V30000")
        while True:
            if self.connection and self.connection.in_waiting > 0:
                try:
                    # Read feedback from the Arduino
                    feedback = self.connection.readline().decode().strip()
                    print(f"Feedback received: {feedback}")
                    # Check if the steppers have reached their location
                    if feedback == "Steppers reached location":
                        print("Confirmation received: Steppers have reached their destination!")
                        return True
                except Exception as e:
                    print(f"Error reading feedback: {e}")
            await asyncio.sleep(0.1)  # Prevent CPU overuse by adding a short delay



=======
>>>>>>> Stashed changes
    def close(self):
        """
        Sluit de seriële verbinding.
        """
        if self.connection:
            self.connection.close()
            print("Arduino verbinding gesloten")

    def __del__(self):
        """
        Zorg ervoor dat de verbinding wordt gesloten bij het verwijderen van het object.
        """
        self.close()

if __name__ == "__main__":
    async def main():
        arduino = ArduinoConnection(port="//dev/ttyACM1")
        position_reached = await arduino.check_camera()
        if not position_reached:
            await arduino.change_speed_motor("Camera",100)
            await arduino.home_camera()
        print("Camera is homed")

    asyncio.run(main())
