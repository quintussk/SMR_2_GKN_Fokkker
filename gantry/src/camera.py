from pathlib import Path
import cv2
import os
import time
from threading import Thread

class Camera:
    def __init__(self, index=1):  # Default index is 0
        self.camera = cv2.VideoCapture(index)
        if not self.camera.isOpened():
            raise ValueError(f"Camera with index {index} could not be opened.")

    async def take_picture(self, mold_name: str, filename: str):
        """
        Takes a picture and saves it in a temporary folder within the mold-specific directory.

        Args:
            mold_name (str): The name of the mold (used as a directory name).
            filename (str): The filename for the image.
        """
        success, frame = self.camera.read()
        if success:
            # Create directory structure for the mold and temporary folder using Pathlib
            path_temp = Path(__file__).parent / "Pictures" / mold_name / "temporary"
            path_temp.mkdir(parents=True, exist_ok=True)

            # Save the image temporarily
            temp_filepath = path_temp / filename
            cv2.imwrite(str(temp_filepath), frame)
            print(f"Temporary image saved at: {temp_filepath}")

            # Start a separate thread for image recognition
            Thread(target=self.process_image, args=(mold_name, filename), daemon=True).start()
        else:
            raise RuntimeError("Failed to take picture")

    def process_image(self, mold_name: str, filename: str):
        """
        Processes the image using image recognition and moves it to a permanent location.

        Args:
            mold_name (str): The name of the mold (used as a directory name).
            filename (str): The filename for the image.
        """
        try:
            # Simulate image recognition processing (replace with actual logic)
            print(f"Processing image: {filename} in mold {mold_name}...")
            time.sleep(2)  # Simulate processing delay

            # Move the processed image to the permanent folder
            path_temp = Path(__file__).parent / "Pictures" / mold_name / "temporary"
            path_permanent = Path(__file__).parent / "Pictures" / mold_name / "permanent"
            path_permanent.mkdir(parents=True, exist_ok=True)

            temp_filepath = path_temp / filename
            permanent_filepath = path_permanent / filename

            # Move the file to the permanent directory
            temp_filepath.rename(permanent_filepath)
            print(f"Image moved to permanent location: {permanent_filepath}")
        except Exception as e:
            print(f"Error processing image {filename}: {e}")

    def release(self):
        """
        Releases the camera resource.
        """
        self.camera.release()

    def generate_frames(self):
        while True:
            # Capture frame-by-frame
            success, frame = self.camera.read()
            if not success:
                break
            else:

                # Encode frame to JPEG format
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    print("Frame encoding failed")
                    break
                frame = buffer.tobytes()

                # Yield the frame as part of an HTTP response
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


if __name__ == "__main__":
    print(Path(__file__).parent)