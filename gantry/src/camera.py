from pathlib import Path
import cv2
import os
import time
import numpy as np
import asyncio


class Camera:
    def __init__(self, index=1):  # Default index is 0
        self.camera = cv2.VideoCapture(index)
        if not self.camera.isOpened():
            raise ValueError(f"Camera with index {index} could not be opened.")
        self.stitched_image = None  # Houd de samengestelde afbeelding bij
        self.max_width = 0  # Max canvas breedte
        self.max_height = 0  # Max canvas hoogte
        # self.current_X = 0
        # self.current_Y = 0
        self.last_X = 0
        self.last_Y = 0

    async def take_picture(self, mold_name: str, filename: str):
        """
        Takes a picture and saves it in a temporary folder within the mold-specific directory.

        Args:
            mold_name (str): The name of the mold (used as a directory name).
            filename (str): The filename for the image.
        """
        success, frame = self.camera.read()
        if success:
            # Maak de tijdelijke directory
            path_temp = Path(__file__).parent / "Pictures" / mold_name / "temporary"
            path_temp.mkdir(parents=True, exist_ok=True)

            # Sla de afbeelding tijdelijk op
            temp_filepath = path_temp / filename
            cv2.imwrite(str(temp_filepath), frame)
            print(f"Temporary image saved at: {temp_filepath}")

            # Verwerk de afbeelding direct in de hoofdfunctie
            await self.process_image(mold_name, filename, frame)
        else:
            raise RuntimeError("Failed to take picture")
        
    async def process_image(self, mold_name: str, filename: str, frame):
        """
        Processes the image using image recognition and stitches it into the composite image.

        Args:
            mold_name (str): The name of the mold (used as a directory name).
            filename (str): The filename for the image.
            frame: The image frame to process.
        """
        try:
            print(f"Processing image: {filename} in mold {mold_name}...")

            # Stitch de afbeelding
            coords = self.extract_coordinates(filename)
            await self.stitch_images(frame,coordinates=coords)

            # Verplaats het verwerkte beeld naar de permanente map
            path_temp = Path(__file__).parent / "Pictures" / mold_name / "temporary"
            path_permanent = Path(__file__).parent / "Pictures" / mold_name / "permanent"
            path_permanent.mkdir(parents=True, exist_ok=True)

            temp_filepath = path_temp / filename
            permanent_filepath = path_permanent / filename

            if not temp_filepath.exists():
                print(f"Temp file does not exist: {temp_filepath}")
                return

            temp_filepath.rename(permanent_filepath)
            print(f"Image moved to permanent location: {permanent_filepath}")

        except Exception as e:
            print(f"Error processing image {filename}: {e}")

    def extract_coordinates(self, filename: str):
        """
        Extracts the X and Y coordinates from the filename.
        The filename format is expected to be {mold}_image_X{current_X}_Y{current_Y}.jpg.

        Args:
            filename (str): The filename of the image.

        Returns:
            tuple: (current_X, current_Y)
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
        
    async def stitch_images(self, new_image, coordinates):
        """
        Stitches a new image into the composite image based on relative placement logic.

        Args:
            new_image: The new image to add.
            coordinates: (current_X, current_Y) coordinates of the new image.
        """
        current_X, current_Y = coordinates
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

        elif current_Y > self.last_Y:  # Horizontaal rechts van de vorige foto
            new_pixel_Y = self.current_pixel_Y + width
            print(f"Placing horizontally right at ({new_pixel_X}, {new_pixel_Y}).")

        elif current_Y < self.last_Y:  # Horizontaal links van de vorige foto
            new_pixel_Y = self.current_pixel_Y - width
            print(f"Placing horizontally left at ({new_pixel_X}, {new_pixel_Y}).")

        # Bereken of het canvas vergroot moet worden
        canvas_height = max(self.stitched_image.shape[0], new_pixel_X + height)
        canvas_width = max(self.stitched_image.shape[1], new_pixel_Y + width, -new_pixel_Y)

        # Vergroot het canvas indien nodig
        if canvas_height > self.stitched_image.shape[0] or canvas_width > self.stitched_image.shape[1]:
            new_canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

            # Verschuif het oude canvas naar de juiste plek in het nieuwe canvas
            y_offset = max(0, -new_pixel_Y)  # Verplaats oude canvas als er linksuitbreiding is
            x_offset = 0  # Geen verplaatsing nodig voor verticale uitbreiding
            new_canvas[x_offset:x_offset + self.stitched_image.shape[0], y_offset:y_offset + self.stitched_image.shape[1]] = self.stitched_image
            self.stitched_image = new_canvas
            print(f"Canvas expanded to ({canvas_width}, {canvas_height}).")

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
        Releases the camera resource.
        """
        self.camera.release()

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