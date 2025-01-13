import serial
import time

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
        except serial.SerialException as e:
            print(f"Kan geen verbinding maken met Arduino: {e}")
            self.connection = None

    def send_command(self, command):
        """
        Stuurt een commando naar de Arduino via de seriële verbinding.
        """
        if self.connection:
            try:
                self.connection.write(f"{command}\n".encode())
                print(f"Commando '{command}' verzonden naar Arduino")
            except Exception as e:
                print(f"Fout bij het verzenden van commando: {e}")
        else:
            print("Geen actieve verbinding met Arduino")

    def set_speed(self, speed_value):
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