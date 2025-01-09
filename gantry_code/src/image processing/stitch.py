import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(0)

# List to store captured frames
captured_frames = []

if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()

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

    elif key == ord('s'):
        if len(captured_frames) < 2:
            print("Need at least two images to create a collage.")
        else:
            # Stitch images horizontally
            try:
                collage = np.vstack(captured_frames)
                cv2.imshow("Collage", collage)
    
            except Exception as e:
                print(f"Error creating")

    elif key == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
