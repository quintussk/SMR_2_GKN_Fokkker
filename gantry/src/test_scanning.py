import cv2
import numpy as np
import time
from pathlib import Path
from arduino import ArduinoConnection
from ultralytics import YOLO
import asyncio
import threading
from queue import Queue
import json

class Camera:
    def __init__(self):
        # Camera-instellingen
        self.camera_index = 4
        self.frame_width = 1920
        self.frame_height = 1080

        # Scanning-instellingen
        self.scan_speed = 0.85
        self.overlap = 60
        self.crop_percentage = 0.3

        path_model = Path(__file__).parent / "best.pt"
        self.yolo_model = YOLO(path_model)

        self.main_stitced_image = None
        self.stitched_filepath = Path(__file__).parent / "Pictures" / "stitched_image.jpg"

        self.Stop_Scanning = False
        self.processing = False

        self.length_scan = 0
        
        # Queue voor beeldverwerking
        self.process_queue = Queue()
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self._process_thread, daemon=True)
        self.processing_thread.start()

        # Camera-initialisatie
        self.camera = cv2.VideoCapture(self.camera_index)
        if not self.camera.isOpened():
            raise ValueError(f"Camera met index {self.camera_index} kon niet worden geopend.")

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        # Camera warming-up
        print("Camera warming-up...")
        for _ in range(20):
            self.camera.read()
        print("Camera klaar voor gebruik.")

        self.epoxy_found = False

    def reset(self):
        self.main_stitced_image = None
        self.epoxy_found = False

    def _process_thread(self):
        """
        Aparte thread voor beeldverwerking.
        """
        while True:
            if not self.processing:
                time.sleep(0.1)
                continue

            try:
                # Haal data uit de queue
                data = self.process_queue.get()
                if data is None:
                    continue

                stitched_image, mold, filename = data
                
                if stitched_image is not None:
                    # Verwerk het beeld met YOLO
                    processed_image = self.proces_image(stitched_image)

                    # Voeg toe aan hoofdafbeelding
                    self.add_processed_image_to_main_stitch(processed_image)
                    
                    # Sla de individuele scan op
                    path_temp = Path(__file__).parent / "Pictures" / mold / "scans"
                    path_clear = Path(__file__).parent / "Pictures" / mold / "without_YOLO"
                    path_temp.mkdir(parents=True, exist_ok=True)
                    path_clear.mkdir(parents=True, exist_ok=True)
                    clear_filepath = path_clear / filename
                    temp_filepath = path_temp / filename
                    cv2.imwrite(str(clear_filepath), stitched_image)
                    cv2.imwrite(str(temp_filepath), processed_image)
                    print(f"Scan voltooid! Opgeslagen als {temp_filepath}")

            except Exception as e:
                print(f"Error in processing thread: {e}")

    def crop_frame(self, frame):
        height, width, _ = frame.shape
        left_crop_percentage = 0.4
        right_crop_percentage = 0.4
        crop_start = int(width * left_crop_percentage)
        crop_end = int(width * (1 - right_crop_percentage))
        return frame[:, crop_start:crop_end]

    def generate_frames(self):
        while True:
            success, frame = self.camera.read()
            if not success:
                break
            else:
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    print("Frame encoding failed")
                    break
                frame = buffer.tobytes()

                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def stitch_frames(self, stitched_image, new_frame, direction="left_to_right"):
        if stitched_image is None:
            return new_frame
        else:
            if direction == "left_to_right":
                combined_frame = np.hstack((stitched_image[:, :-self.overlap], new_frame))
            else:
                combined_frame = np.hstack((new_frame, stitched_image[:, self.overlap:]))
            return combined_frame

    def scan_start(self, steps, mold, filename, Y_total):
        """
        Start een scan en slaat het resultaat op.
        """
        print(f"Scan gestart voor {mold}...")
        
        direction = "left_to_right" if steps > 0 else "right_to_left"
        print(f"Scanrichting: {direction}")

        self.length_scan = Y_total

        stitched_image = None
        previous_frame = None
        self.Stop_Scanning = False
        self.processing = True
        frames_captured = 0

        while not self.Stop_Scanning:
            for _ in range(5):
                self.camera.grab()

            ret, frame = self.camera.read()
            if not ret:
                print("Kon geen frame ophalen. Controleer de camera.")
                break

            if previous_frame is not None:
                diff = cv2.absdiff(previous_frame, frame)
                if np.sum(diff) < 5000:
                    print("Frame lijkt op het vorige frame. Overslaan...")
                    continue

            previous_frame = frame.copy()
            frames_captured += 1

            # frame = self.dynamic_brightness_contrast(frame)

            cropped_frame = self.crop_frame(frame)
            stitched_image = self.stitch_frames(stitched_image, cropped_frame, direction)
            time.sleep(self.scan_speed)

        stitched_image = self.normalize_stitched_image(stitched_image)

        # Plaats het laatste stitched image in de queue voor verwerking
        if stitched_image is not None:
            self.process_queue.put((stitched_image, mold, filename))
        
        # Wacht tot de verwerking klaar is
        while not self.process_queue.empty():
            time.sleep(0.1)
        
        self.processing = False

    def dynamic_brightness_contrast(self, image):
        """
        Past dynamisch helderheid en contrast aan over de afbeelding om uniformiteit te bereiken.
        Args:
            image (np.ndarray): De invoerafbeelding.
        Returns:
            np.ndarray: De aangepast afbeelding.
        """
        # Converteer afbeelding naar LAB-kleurruimte
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)

        # Pas CLAHE toe op het lichtheidskanaal
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_channel = clahe.apply(l_channel)

        # Combineer de kanalen opnieuw
        lab = cv2.merge((l_channel, a_channel, b_channel))
        adjusted_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        return adjusted_image
    
    def normalize_stitched_image(self, stitched_image):
        """
        Normaliseert de belichting van een volledig gestitchte afbeelding.
        Args:
            stitched_image (np.ndarray): De gestitchte afbeelding.
        Returns:
            np.ndarray: De genormaliseerde afbeelding.
        """
        # Converteer naar LAB en pas CLAHE toe op het lichtheidskanaal
        lab = cv2.cvtColor(stitched_image, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_channel = clahe.apply(l_channel)

        lab = cv2.merge((l_channel, a_channel, b_channel))
        normalized_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        return normalized_image

    def adjust_brightness(self, image, alpha=1.5, beta=50):
        """
        Pas helderheid en contrast van een afbeelding aan.
        """
        return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

    def update_json(self, scan_image, detections):
        """
        Update the JSON file with real-world coordinates of detected objects in centimeters.

        Args:
            scan_image (PIL.Image or ndarray): The scanned image containing detected objects.
            detections: YOLO detections containing bounding box information.
        """
        epoxy_dir = Path(__file__).parent / "Epoxy"
        json_file_path = epoxy_dir / "epoxy.json"

        try:
            # Zorg dat de map bestaat
            epoxy_dir.mkdir(parents=True, exist_ok=True)

            # Haal afmetingen van de afbeelding op
            if isinstance(scan_image, np.ndarray):
                frame_height, frame_width, _ = scan_image.shape
            else:
                frame_width, frame_height = scan_image.size

            print(f"Afbeeldingsgrootte: Breedte={frame_width}, Hoogte={frame_height}")

            # Laad bestaande JSON of maak een nieuwe
            if json_file_path.exists():
                with open(json_file_path, "r") as file:
                    data = json.load(file)
            else:
                data = {"epoxy_points": []}

            # Bereken verhouding (lengte scan in cm / breedte afbeelding in pixels)
            ratio = self.length_scan / frame_width
            print(f"Pixel-naar-cm-verhouding: {ratio}")

            # Voeg YOLO-detecties toe aan JSON
            for result in detections:
                boxes = result.boxes
                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    # Bereken het midden van de bounding box
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    print(f"Detectie centrum (pixels): ({center_x}, {center_y})")

                    # Converteer naar real-world coördinaten (in centimeters)
                    real_x = center_x * ratio  # X-coördinaat in cm
                    real_y = center_y * ratio  # Y-coördinaat in cm

                    print(f"Omgezet naar real-world coördinaten (cm): ({real_x}, {real_y})")

                    # Voeg punt toe aan data
                    point = {
                        "id": len(data["epoxy_points"]) + 1,
                        "x": float(real_x),
                        "y": float(real_y),
                        "removed": False
                    }
                    data["epoxy_points"].append(point)

            # Sla de geüpdatete JSON op
            with open(json_file_path, "w") as file:
                json.dump(data, file, indent=4)

            print(f"Geüpdatete epoxy-punten opgeslagen in: {json_file_path}")

        except Exception as e:
            print(f"Fout bij het updaten van JSON: {e}")

    def add_processed_image_to_main_stitch(self, processed_image):
        """
        Voegt een nieuwe processed_image toe aan de hoofdafbeelding door deze er bovenop te plaatsen.
        De resulterende afbeelding wordt opgeslagen als stitched_image.jpg
        """
        if self.main_stitced_image is None:
            self.main_stitced_image = processed_image
        else:
            # Zorg ervoor dat beide afbeeldingen dezelfde breedte hebben
            if self.main_stitced_image.shape[1] != processed_image.shape[1]:
                # Resize processed_image om dezelfde breedte te krijgen
                scale_factor = self.main_stitced_image.shape[1] / processed_image.shape[1]
                new_height = int(processed_image.shape[0] * scale_factor)
                processed_image = cv2.resize(processed_image, (self.main_stitced_image.shape[1], new_height))

            # Voeg de nieuwe afbeelding verticaal toe (bovenop de bestaande)
            self.main_stitced_image = np.vstack((processed_image, self.main_stitced_image))

        # Sla de bijgewerkte hoofdafbeelding op
        stitched_filepath = Path(__file__).parent / "Pictures" / "stitched_image.jpg"
        stitched_filepath.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(stitched_filepath), self.main_stitced_image)
        print(f"Updated main stitched image saved at {stitched_filepath}")

    def proces_image(self, image):
        results = self.yolo_model(image)
        for result in results:
                self.epoxy_found = True
                boxes = result.boxes  # Haal de bounding boxes op
                for box in boxes:
                    # Verkrijg de coördinaten van de bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Converteer naar integers
                    # Teken alleen de bounding box op het frame
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Groen met dikte 2

        self.update_json(image,results)
        return image

    def release(self):
        self.Stop_Scanning = True
        self.processing = False
        self.camera.release()
        print("Camera vrijgegeven.")

    def stop_scan(self):
        print("Stopping scan")
        self.Stop_Scanning = True
        
async def scan_and_wait(scanner: Camera,arduino:ArduinoConnection, steps, mold, filename):
        """
        Start een scan en wacht op Arduino's terugkoppeling.
        """
        # Start de scan in een aparte task
        scan_task = asyncio.create_task(
            asyncio.to_thread(
                scanner.scan_start,
                steps,
                mold,
                filename, 200 
            )
        )
        
        # Wacht op Arduino's terugkoppeling
        while not await arduino.Wait_For_Location_Reached():
            await asyncio.sleep(0.1)
        
        # Stop de scan
        scanner.stop_scan()
        
        # Wacht tot de scan volledig is afgerond
        await scan_task

async def main():
    arduino = ArduinoConnection(port="//dev/ttyACM0")
    scanner = Camera()

    steps = -3725

    await arduino.Relay("ON")
    # await scanner.check_if_camera_is_home()
    await arduino.change_speed_motor(motor="Mold", speed=500)
    await arduino.change_speed_motor(motor="Camera", speed=300)

    await arduino.send_steps(0, steps)
    
    try:
        await scan_and_wait(
            scanner=scanner,
            arduino=arduino,
            steps=steps,
            mold="Testing",
            filename="scan4.jpg"
        )
    finally:
        scanner.release()
        await arduino.Relay("OFF")
    await arduino.Relay("OFF")


if __name__ == "__main__":
    asyncio.run(main())