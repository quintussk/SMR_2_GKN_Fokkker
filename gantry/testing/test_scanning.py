import cv2
import numpy as np
import time
from pathlib import Path

class Scanner:
    def __init__(self):
        # Camera-instellingen
        self.camera_index = 0  # Camera-index, pas aan naar jouw camera
        self.frame_width = 1920  # Breedte van de frame
        self.frame_height = 1080  # Hoogte van de frame

        # Scanning-instellingen
        self.scan_speed = 0.5  # Bewegingstijd tussen frames (in seconden)
        self.overlap = 50  # Pixels overlap tussen frames
        self.crop_percentage = 0.3  # Percentage van beeldbreedte dat wordt gebruikt

        # Uitvoerinstellingen
        self.output_filename = "Test_Scan.jpg"  # Naam van het gestitchte scanbestand
        self.save_directory = Path(__file__).parent / "Scans"  # Opslaglocatie

        # Camera-initialisatie
        self.camera = cv2.VideoCapture(self.camera_index)
        if not self.camera.isOpened():
            raise ValueError(f"Camera met index {self.camera_index} kon niet worden geopend.")

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

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

    def stitch_frames(self, stitched_image, new_frame):
        """
        Voegt een nieuw frame toe aan een gestitchte afbeelding.
        """
        if stitched_image is None:
            return new_frame
        else:
            overlap_region = stitched_image[:, -self.overlap:]  # Laatste deel van de stitched afbeelding
            combined_frame = np.hstack((stitched_image[:, :-self.overlap], new_frame))  # Voeg frames samen
            return combined_frame

    def scan_start(self, duration=10):
        """
        Start een scan en slaat het resultaat op.
        Args:
            duration (int): Duur van de scan in seconden.
        """
        print("Scan gestart...")
        start_time = time.time()
        stitched_image = None

        while time.time() - start_time < duration:
            # Capture een frame
            ret, frame = self.camera.read()
            if not ret:
                print("Kon geen frame ophalen. Controleer de camera.")
                break

            # Crop het frame
            cropped_frame = self.crop_frame(frame)

            # Stitch het frame aan de bestaande afbeelding
            stitched_image = self.stitch_frames(stitched_image, cropped_frame)

            # Wacht een beetje voor de volgende beweging
            time.sleep(self.scan_speed)

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

if __name__ == "__main__":
    scanner = Scanner()
    try:
        scanner.scan_start(duration=10)  # Scan 10 seconden
    finally:
        scanner.release()