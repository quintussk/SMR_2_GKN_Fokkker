from pathlib import Path
import cv2
import os
import time
import numpy as np
import asyncio
import threading
from ultralytics import YOLO  # Import YOLO
import json
from rich import print as print

class Camera:
    def __init__(self, index=4):
        self.camera = cv2.VideoCapture(index)
        if not self.camera.isOpened():
            raise ValueError(f"Camera with index {index} could not be opened.")
        
                # Stel de resolutie in op 1080p
        # self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)  # Breedte
        # self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)  # Hoogte
        
        self.current_frame = None
        self.running = True
        self.stitched_image = None
        self.last_X = 0
        self.last_Y = 0

        self.frame_width = 1920  # Breedte van de frame
        self.frame_height = 1080  # Hoogte van de frame

        # Scanning-instellingen
        self.scan_speed = 0.85  # Bewegingstijd tussen frames (in seconden)
        self.overlap = 60  # Pixels overlap tussen frames (verlaagd voor minder overlap)
        self.crop_percentage = 0.3  # Percentage van beeldbreedte dat wordt gebruikt

        path_model = Path(__file__).parent / "best.pt"
        self.yolo_model = YOLO(path_model)  # Pas "best.pt" aan naar jouw YOLO-modelbestand

        # Threading voor process_image
        self.process_queue = []  # Lijst om frames te verwerken
        self.processing = False  # Houd bij of er een verwerking bezig is
        self.processing_lock = threading.Lock()  # Lock voor threadveiligheid

        # Start thread voor het verwerken van beelden
        self.processing_thread = threading.Thread(target=self._process_images_thread, daemon=True)
        self.processing_thread.start()

        # Start achtergrond thread om frames te lezen
        # self.capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
        # self.capture_thread.start()

    def _capture_frames(self):
        """
        Continu leest frames van de camera en slaat deze op in `self.current_frame`.
        """
        while self.running:
            success, frame = self.camera.read()
            if success:
                # frame = self.adjust_brightness(frame, alpha=1.5, beta=50)
                self.current_frame = frame
            else:
                print("Failed to read frame from camera")
            time.sleep(0.03)

    def adjust_brightness(self, image, alpha=1.5, beta=50):
        """
        Pas helderheid en contrast van een afbeelding aan.
        """
        return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

    async def take_picture(self, mold_name: str, filename: str):
        """
        Neemt een foto door de camera te laten stabiliseren en het 6e frame vast te leggen.
        """
        # Zorg dat de camera voldoende tijd krijgt om zich aan te passen
        for _ in range(6):  # Pak meerdere frames om de camera te laten stabiliseren
            self.camera.grab()

        # Haal het 6e frame op
        success, frame = self.camera.retrieve()
        if not success:
            raise RuntimeError("Failed to retrieve frame from the camera.")
        
        # Pas helderheid aan
        frame = self.adjust_brightness(frame, alpha=1.5, beta=50)

        # Maak directories aan
        path_temp = Path(__file__).parent / "Pictures" / mold_name / "temporary"
        path_temp.mkdir(parents=True, exist_ok=True)

        # Opslaan van de afbeelding
        temp_filepath = path_temp / filename
        cv2.imwrite(str(temp_filepath), frame)
        print(f"Temporary image saved at: {temp_filepath}")

        # Voeg het frame toe aan de verwerkingswachtrij
        with self.processing_lock:
            self.process_queue.append((mold_name, filename, frame))

    def _process_images_thread(self):
        """
        Thread die continu de wachtrij controleert en frames verwerkt.
        """
        while self.running:
            if self.process_queue:
                with self.processing_lock:
                    mold_name, filename, frame = self.process_queue.pop(0)

                # Verwerk het frame
                self.process_image(mold_name, filename, frame)
            else:
                time.sleep(0.1)  # Voorkomt continu loopen als er niets te verwerken is

    def update_json(self, coords, detections):
        """
        Update the JSON file with real-world coordinates of detected objects in centimeters.

        Args:
            mold_name (str): Name of the mold.
            coords (tuple): The (scan_x, scan_y) coordinates of the camera's position (in cm).
            detections: YOLO detections containing bounding box information.
        """
        epoxy_dir = Path(__file__).parent / "Epoxy"
        json_file_path = epoxy_dir / "epoxy.json"

        try:
            # Extract scan coordinates from filename
            scan_x, scan_y = coords
            print(f"Scan coordinates: ({scan_x}, {scan_y})")

            # Load existing JSON
            if json_file_path.exists():
                with open(json_file_path, "r") as file:
                    data = json.load(file)
            else:
                data = {"epoxy_points": []}  # Initialize JSON structure if file doesn't exist

            # Retrieve frame dimensions
            frame_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
            frame_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

            print(f"frame_width: {frame_width}, frame_height: {frame_height}")

            center_frame_width = frame_width / 2
            center_frame_height = frame_height / 2

            if frame_width == 0 or frame_height == 0:
                raise ValueError("Frame width or height is invalid. Ensure the camera is initialized correctly.")

            # Define pixel-to-cm scale factor (adjust based on your camera's FOV)
            pixel_to_cm_scale = 0.1047  # Example: 30 cm FOV / 1920 pixels

            pixels_per_mm = 1
            pixels_per_cm = pixels_per_mm * 10

            # Convert YOLO detections to real-world coordinates and add to epoxy points
            for result in detections:
                boxes = result.boxes
                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    # Calculate center point of detection in image coordinates
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    print(f"Detection center (image): ({center_x}, {center_y})")

                    # Convert to real-world coordinates in centimeters
                    # different because X and Y are swapped in REAL World
                    real_y = scan_y + ((center_x - center_frame_width) / pixels_per_cm)
                    real_x = abs(scan_x) - ((center_y - center_frame_height) / pixels_per_cm)

                    print(f"Converted real-world coordinates (cm): ({real_x}, {real_y})")

                    # Add new point to JSON data
                    point = {
                        "id": len(data["epoxy_points"]) + 1,
                        "x": float(real_x),
                        "y": float(real_y),
                        "removed": False
                    }
                    data["epoxy_points"].append(point)

            # Save updated JSON
            with open(json_file_path, "w") as file:
                json.dump(data, file, indent=4)

            print(f"Updated epoxy points saved to: {json_file_path}")

        except Exception as e:
            print(f"Error updating JSON: {e}")

    def process_image(self, mold_name: str, filename: str, frame):
        """
        Verwerkt de afbeelding met YOLO-inferentie en slaat de verwerkte afbeelding op in de YOLO-map.
        """
        try:
            print(f"Processing image: {filename} in mold {mold_name}...")

            # Voer YOLO-inferentie uit
            results = self.yolo_model(frame)

            # Update JSON met YOLO-resultaten
            self.update_json(mold_name, self.extract_coordinates(filename), results)

            for result in results:
                boxes = result.boxes  # Haal de bounding boxes op
                for box in boxes:
                    # Verkrijg de coördinaten van de bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Converteer naar integers
                    # Teken alleen de bounding box op het frame
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Groen met dikte 2

            # Stitch de afbeelding
            coords = self.extract_coordinates(filename)
            self.stitch_images(frame, coordinates=coords)

            # Verplaats het originele beeld naar de permanente map
            path_temp = Path(__file__).parent / "Pictures" / mold_name / "temporary"
            path_permanent = Path(__file__).parent / "Pictures" / mold_name / "permanent"
            path_YOLO = Path(__file__).parent / "Pictures" / mold_name / "YOLO"
            path_permanent.mkdir(parents=True, exist_ok=True)
            path_YOLO.mkdir(parents=True, exist_ok=True)

            temp_filepath = path_temp / filename
            permanent_filepath = path_permanent / filename
            yolo_filepath = path_YOLO / filename

            if not temp_filepath.exists():
                print(f"Temp file does not exist: {temp_filepath}")
                return

            # Verplaats naar de permanente map
            temp_filepath.rename(permanent_filepath)
            print(f"Image moved to permanent location: {permanent_filepath}")

            # Sla de verwerkte YOLO-afbeelding op in de YOLO-map
            cv2.imwrite(str(yolo_filepath), frame)
            print(f"YOLO-processed image saved to: {yolo_filepath}")

        except Exception as e:
            print(f"Error processing image {filename}: {e}")

    def extract_coordinates(self, filename: str):
        """
        Extracts the X and Y coordinates from the filename.
        """
        try:
            x_start = filename.index("X") + 1
            x_end = filename.index("_Y")
            y_start = x_end + 2
            y_end = filename.index(".jpg")

            current_X = int(filename[x_start:x_end])
            current_Y = int(filename[y_start:y_end])
            return current_X, current_Y
        except ValueError as e:
            print(f"Error parsing coordinates from filename: {filename}")
            return 0, 0

    def stitch_images(self, new_image, coordinates):
        """
        Stitches a new image into the composite image based on relative placement logic.

        Args:
            new_image: The new image to add.
            coordinates: (current_X, current_Y) coordinates of the new image.
        """
        current_X, current_Y = coordinates
        print(f"Current picture coordinates: X={current_X}, Y={current_Y}")
        height, width, _ = new_image.shape

        # Initialiseer stitched_image canvas als deze nog niet bestaat
        if self.stitched_image is None:
            # Begin met een canvas dat groot genoeg is voor de eerste afbeelding
            self.stitched_image = np.zeros((height, width, 3), dtype=np.uint8)
            self.current_pixel_X = 0  # Startcoördinaat in pixels
            self.current_pixel_Y = 0  # Startcoördinaat in pixels
            self.stitched_image[:height, :width] = new_image
            print(f"Initialized stitched image at (0, 0).")
            return

        # Bereken de nieuwe plaatsing op basis van de relatie met de vorige coördinaten
        new_pixel_X = self.current_pixel_X
        new_pixel_Y = self.current_pixel_Y

        if current_X > self.last_X:  # Verticaal onder de vorige foto
            new_pixel_X = self.current_pixel_X + height
            print(f"Placing vertically below at ({new_pixel_X}, {new_pixel_Y}).")

        elif current_X < self.last_X:  # Verticaal boven de vorige foto
            new_pixel_X = self.current_pixel_X - height
            print(f"Placing vertically above at ({new_pixel_X}, {new_pixel_Y}).")

        elif current_Y > self.last_Y:  # Horizontaal rechts van de vorige foto
            new_pixel_Y = self.current_pixel_Y + width
            print(f"Placing horizontally right at ({new_pixel_X}, {new_pixel_Y}).")

        elif current_Y < self.last_Y:  # Horizontaal links van de vorige foto
            new_pixel_Y = self.current_pixel_Y - width
            print(f"Placing horizontally left at ({new_pixel_X}, {new_pixel_Y}).")

        # Bereken of het canvas vergroot moet worden
        canvas_height = max(self.stitched_image.shape[0], new_pixel_X + height)
        canvas_width = max(self.stitched_image.shape[1], new_pixel_Y + width)  # Vergroot alleen rechts en links
        left_extension = max(0, -new_pixel_Y)  # Bepaal hoeveel ruimte nodig is aan de linkerkant
        top_extension = max(0, -new_pixel_X)  # Bepaal hoeveel ruimte nodig is aan de bovenkant

        if left_extension > 0 or top_extension > 0 or canvas_height > self.stitched_image.shape[0] or canvas_width > self.stitched_image.shape[1]:
            # Maak een nieuw vergroot canvas
            new_canvas = np.zeros((canvas_height + top_extension, canvas_width + left_extension, 3), dtype=np.uint8)

            # Kopieer het oude canvas naar de juiste positie in het nieuwe canvas
            y_offset = left_extension  # Schuif het oude canvas naar rechts als er linksuitbreiding is
            x_offset = top_extension  # Schuif het oude canvas naar beneden als er bovenuitbreiding is
            new_canvas[x_offset:x_offset + self.stitched_image.shape[0], y_offset:y_offset + self.stitched_image.shape[1]] = self.stitched_image
            self.stitched_image = new_canvas
            print(f"Canvas expanded to ({self.stitched_image.shape[1]}, {self.stitched_image.shape[0]}).")

            # Pas de pixelcoördinaten aan
            new_pixel_X += x_offset
            new_pixel_Y += y_offset

        # Plaats de nieuwe afbeelding in het canvas
        self.stitched_image[new_pixel_X:new_pixel_X + height, new_pixel_Y:new_pixel_Y + width] = new_image
        print(f"Image placed at pixel coordinates: ({new_pixel_X}, {new_pixel_Y}).")

        # Werk de laatste X, Y en pixelcoördinaten bij
        self.last_X, self.last_Y = current_X, current_Y
        self.current_pixel_X, self.current_pixel_Y = new_pixel_X, new_pixel_Y

        # Sla de bijgewerkte gestitchte afbeelding op
        stitched_filepath = Path(__file__).parent / "Pictures" / "stitched_image.jpg"
        cv2.imwrite(str(stitched_filepath), self.stitched_image)
        print(f"Updated stitched image saved at: {stitched_filepath}")

    def release(self):
        """
        Stop de threads en release de camera.
        """
        self.running = False
        self.capture_thread.join()
        self.processing_thread.join()
        self.camera.release()
        print("Camera and processing threads released.")

    def generate_frames(self):
        while True:
            success, frame = self.camera.read()

            results = self.yolo_model(frame)

            for result in results:
                boxes = result.boxes  # Haal de bounding boxes op
                for box in boxes:
                    # Verkrijg de coördinaten van de bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Converteer naar integers
                    # Teken alleen de bounding box op het frame
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Groen met dikte 2

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

async def main():
    camera = Camera()
    schuif = 50  # De afstand tussen de foto's
    max_rijen = 4  # Aantal rijen
    max_kolommen = 4  # Aantal kolommen

    try:
        for kolom in range(max_kolommen):  # Itereer door de kolommen
            if kolom % 2 == 0:  # Voor even kolommen (van boven naar beneden)
                for rij in range(max_rijen):
                    current_X = kolom * schuif
                    current_Y = rij * schuif
                    await camera.take_picture(
                        mold_name="test2",
                        filename=f"test2_image_X{current_X}_Y{current_Y}.jpg"
                    )
                    await asyncio.sleep(0.5)
            else:  # Voor oneven kolommen (van beneden naar boven)
                for rij in range(max_rijen - 1, -1, -1):
                    current_X = kolom * schuif
                    current_Y = rij * schuif
                    await camera.take_picture(
                        mold_name="test2",
                        filename=f"test2_image_X{current_X}_Y{current_Y}.jpg"
                    )
                    await asyncio.sleep(0.5)
    finally:
        camera.release()

if __name__ == "__main__":
    asyncio.run(main())