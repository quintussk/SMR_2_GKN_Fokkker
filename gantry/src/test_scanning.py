import cv2
import numpy as np
import time
from pathlib import Path
from arduino import ArduinoConnection
from ultralytics import YOLO  # Import YOLO
import asyncio

class Scanner:
    def __init__(self, arduino):
        # Camera-instellingen
        self.camera_index = 4  # Camera-index, pas aan naar jouw camera
        self.frame_width = 1920  # Breedte van de frame
        self.frame_height = 1080  # Hoogte van de frame

        # Scanning-instellingen
        self.scan_speed = 0.85  # Bewegingstijd tussen frames (in seconden)
        self.overlap = 60  # Pixels overlap tussen frames (verlaagd voor minder overlap)
        self.crop_percentage = 0.3  # Percentage van beeldbreedte dat wordt gebruikt

        path_model = Path(__file__).parent / "best.pt"
        self.yolo_model = YOLO(path_model)  # Pas "best.pt" aan naar jouw YOLO-modelbestand

        # Uitvoerinstellingen
        self.output_filename = "Test_Scan.jpg"  # Naam van het gestitchte scanbestand
        self.save_directory = Path(__file__).parent / "Scans"  # Opslaglocatie

        # Arduino-verbinding
        self.arduinoClass = arduino

        # Camera-initialisatie
        self.camera = cv2.VideoCapture(self.camera_index)
        if not self.camera.isOpened():
            raise ValueError(f"Camera met index {self.camera_index} kon niet worden geopend.")

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        # self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Zet automatische belichting uit
        # self.camera.set(cv2.CAP_PROP_EXPOSURE, -8)  # Lagere belichting
        # self.camera.set(cv2.CAP_PROP_BRIGHTNESS, 100)  # Lagere helderheid
        # self.camera.set(cv2.CAP_PROP_CONTRAST, 70)  # Verhoog contrast
        # self.camera.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 4000)  # Stel witbalans in (indien ondersteund)

        # Camera warming-up
        print("Camera warming-up...")
        for _ in range(20):  # Laat de camera frames stabiliseren (verhoogd voor meer consistentie)
            self.camera.read()
        print("Camera klaar voor gebruik.")

    def crop_frame(self, frame):
        """
        Cropt een frame zodat alleen een smaller deel wordt gebruikt.
        """
        height, width, _ = frame.shape

        # Crop-start en -eind aanpassen voor een smaller deel
        left_crop_percentage = 0.4  # Begin crop (40% van links)
        right_crop_percentage = 0.4  # Eind crop (40% van rechts)

        crop_start = int(width * left_crop_percentage)  # Startpunt croppen
        crop_end = int(width * (1 - right_crop_percentage))  # Eindpunt croppen

        return frame[:, crop_start:crop_end]

    def sharpen_image(self, image):
        """
        Verhoogt de scherpte van een afbeelding met behulp van een krachtigere sharpening-kernel.
        """
        kernel = np.array([[1, -2, 1],
                           [-2, 5, -2],
                           [1, -2, 1]])
        sharpened = cv2.filter2D(image, -1, kernel)
        return sharpened

    def auto_adjust_contrast(self, image):
        """
        Past contrast adaptief aan met CLAHE (Contrast Limited Adaptive Histogram Equalization).
        """
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        lab = cv2.merge((l, a, b))
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def stitch_frames(self, stitched_image, new_frame):
        """
        Voegt een nieuw frame toe aan een gestitchte afbeelding.
        """
        if stitched_image is None:
            return new_frame
        else:
            combined_frame = np.hstack((stitched_image[:, :-self.overlap], new_frame))  # Voeg frames samen
            return combined_frame
        
    def adjust_brightness(self, image, alpha=1.5, beta=50):
        """
        Pas helderheid en contrast van een afbeelding aan.
        """
        return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

    def scan_start(self, duration=10):
        """
        Start een scan en slaat het resultaat op.
        Args:
            duration (int): Duur van de scan in seconden.
        """
        print("Scan gestart...")
        start_time = time.time()
        stitched_image = None
        previous_frame = None

        while time.time() - start_time < duration:
            # Grijp meerdere keren het frame om buffer te legen
            for _ in range(5):  # Pas het aantal aan als nodig
                self.camera.grab()

            # Lees het nieuwste frame
            ret, frame = self.camera.read()
            if not ret:
                print("Kon geen frame ophalen. Controleer de camera.")
                break

            # Controleer of het frame verschilt van het vorige frame
            if previous_frame is not None:
                diff = cv2.absdiff(previous_frame, frame)
                if np.sum(diff) < 5000:  # Pas de drempelwaarde aan
                    print("Frame lijkt op het vorige frame. Overslaan...")
                    continue

            # Update previous_frame
            previous_frame = frame.copy()

            # Verhoog contrast en scherpte
            # frame = self.auto_adjust_contrast(frame)
            # frame = self.sharpen_image(frame)

            # Crop het frame
            cropped_frame = self.crop_frame(frame)

            # Stitch het frame aan de bestaande afbeelding
            stitched_image = self.stitch_frames(stitched_image, cropped_frame)

            # Wacht een beetje voor de volgende beweging
            time.sleep(self.scan_speed)

        # results = self.yolo_model(stitched_image)

        # for result in results:
        #         boxes = result.boxes  # Haal de bounding boxes op
        #         for box in boxes:
        #             # Verkrijg de coördinaten van de bounding box
        #             x1, y1, x2, y2 = map(int, box.xyxy[0])  # Converteer naar integers
        #             # Teken alleen de bounding box op het frame
        #             cv2.rectangle(stitched_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Groen met dikte 2
        
        # Sla het gestitchte beeld op
        if stitched_image is not None:
            self.save_directory.mkdir(parents=True, exist_ok=True)
            output_path = self.save_directory / self.output_filename
            cv2.imwrite(str(output_path), stitched_image)
            print(f"Scan voltooid! Opgeslagen als {output_path}")
        else:
            print("Geen afbeelding om op te slaan.")

    def release(self):
        """
        Sluit de camera en vrijgegeven resources.
        """
        self.camera.release()
        print("Camera vrijgegeven.")

    async def check_if_camera_is_home(self):
        """
        Checks if the camera is at the home position.
        """
        position_reached = await self.arduinoClass.check_camera()
        if not position_reached:
            await self.arduinoClass.change_speed_motor("Camera", 100)
            await self.arduinoClass.home_camera()
        print("Camera is homed")


async def main():
    arduino = ArduinoConnection(port="//dev/ttyACM0")
    scanner = Scanner(arduino)

    steps = -3000

    await arduino.Relay("ON")
    await scanner.check_if_camera_is_home()
    await arduino.change_speed_motor(motor="Mold", speed=500)
    await arduino.change_speed_motor(motor="Camera", speed=300)  # Verhoogde snelheid

    await arduino.send_steps(0, steps)  # Send steps to Arduino
    try:
        scanner.scan_start(duration=10)  # Scan 5 seconden
    finally:
        scanner.release()


if __name__ == "__main__":
    asyncio.run(main())