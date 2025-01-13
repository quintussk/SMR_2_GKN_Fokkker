import cv2
import numpy as np
import os

horizontal_frames = 3
vertical_frames = 3

class ImageCaptureAndStitch:
    def __init__(self, image_dir='./images', camera_index=0):
        self.image_dir = image_dir
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise Exception("Error: Unable to access the camera.")
        
        self.captured_frames = []
        self.horizontal_stitched_images = []
        self.stitched_counter = 1
        os.makedirs(self.image_dir, exist_ok=True)
        self.is_running = False

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Error: Unable to read frame from camera.")
        self.captured_frames.append(frame.copy())
        print(f"Captured {len(self.captured_frames)} frame(s).")
        if len(self.captured_frames) == horizontal_frames:
            self.stitch_horizontally()

    def stitch_horizontally(self):
        try:
            stitched_image = np.hstack(self.captured_frames)
            self.horizontal_stitched_images.append(stitched_image)
            self.captured_frames = []
            print(f"Stitched horizontally {len(self.horizontal_stitched_images)} image(s).")
            if len(self.horizontal_stitched_images) == vertical_frames:
                self.stitch_vertically()
        except Exception as e:
            print(f"Error stitching images horizontally: {e}")

    def stitch_vertically(self):
        # Function to detect and label red blobs
        def detect_red(frame):
            # Convert the frame to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the HSV range for red color
            lower_red1 = np.array([0, 120, 70])  # Lower red range (hue near 0)
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])  # Upper red range (hue near 180)
            upper_red2 = np.array([180, 255, 255])

            # Threshold the HSV frame to get only red colors
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)

            # Find contours in the mask
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for i, contour in enumerate(contours):
                # Ignore small blobs by filtering based on contour area
                if cv2.contourArea(contour) > 10:  
                    # Draw a bounding box around the contour
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Label the blob
                    cv2.putText(frame, f'Red Blob {i+1}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

            return frame, red_mask
        try:
            final_stitched_image = np.vstack(self.horizontal_stitched_images)
            print("Final stitched image shape:", final_stitched_image.shape)  # Debug print
            while os.path.exists(os.path.join(self.image_dir, f'stitched{self.stitched_counter}.jpg')):
                self.stitched_counter += 1

            labeled_frame, red_mask = detect_red(final_stitched_image)
            print("Labeled frame shape:", labeled_frame.shape)  # Debug print

            output_path = os.path.join(self.image_dir, f'stitched{self.stitched_counter}.jpg')
            cv2.imwrite(output_path, labeled_frame)
            print(f"Final stitched image saved at {output_path}")
            self.horizontal_stitched_images = []
        except Exception as e:
            print(f"Error stitching images vertically: {e}")

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