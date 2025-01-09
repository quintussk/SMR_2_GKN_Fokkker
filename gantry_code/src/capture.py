import cv2
import numpy as np
import os

class ImageCaptureAndStitch:
    def __init__(self, image_dir='./images', camera_index=0):
        self.image_dir = image_dir
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise Exception("Error: Unable to access the camera.")
        
        self.captured_frames = []
        self.stitched_counter = 1
        os.makedirs(self.image_dir, exist_ok=True)
        self.is_running = False

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Error: Unable to read frame from camera.")
        self.captured_frames.append(frame.copy())
        print(f"Captured {len(self.captured_frames)} frame(s).")
        if len(self.captured_frames) == 3:
            self.stitch_images()

    def stitch_images(self):
        try:
            stitched_image = np.hstack(self.captured_frames)
            while os.path.exists(os.path.join(self.image_dir, f'stitched{self.stitched_counter}.jpg')):
                self.stitched_counter += 1
            output_path = os.path.join(self.image_dir, f'stitched{self.stitched_counter}.jpg')
            cv2.imwrite(output_path, stitched_image)
            print(f"Stitched image saved at {output_path}")
            self.captured_frames = []
        except Exception as e:
            print(f"Error stitching images: {e}")

    def start_capture(self):
        self.is_running = True
        while self.is_running:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Unable to read frame from camera.")
                break
            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.release_resources()

    def stop_capture(self):
        self.is_running = False

    def release_resources(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    image_capture_and_stitch = ImageCaptureAndStitch()
    image_capture_and_stitch.start_capture()