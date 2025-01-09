import cv2
import numpy as np
import os

# Directory containing the images
image_dir = './images'

# Initialize the webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()

# List to store captured frames
captured_frames = []

# Counter for stitched image naming
stitched_counter = 1

while True:
    # Read frame from the camera
    ret, frame = cap.read()

    if not ret:
        print("Error: Unable to read frame from camera.")
        break

    # Display the current frame
    cv2.imshow("Camera", frame)

    # Wait for a key press
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        # Capture the current frame
        captured_frames.append(frame.copy())
        print(f"Captured {len(captured_frames)} frame(s).")

        # Automatically stitch every 3 captured images
        if len(captured_frames) == 3:
            try:
                # Stitch images horizontally
                stitched_image = np.hstack(captured_frames)

                # Generate a unique name for the stitched image
                while os.path.exists(os.path.join(image_dir, f'stitched{stitched_counter}.jpg')):
                    stitched_counter += 1
                output_path = os.path.join(image_dir, f'stitched{stitched_counter}.jpg')

                # Save the stitched image
                cv2.imwrite(output_path, stitched_image)
                print(f"Stitched image saved at {output_path}")

                # Clear captured frames
                captured_frames = []

            except Exception as e:
                print(f"Error stitching images: {e}")

    elif key == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
