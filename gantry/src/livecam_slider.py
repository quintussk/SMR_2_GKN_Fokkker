import cv2
import numpy as np
import os

def nothing(x):
    pass

# Create a window
cv2.namedWindow('Trackbars')

# Create trackbars for color change
cv2.createTrackbar('HMin', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('SMin', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('VMin', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('HMax', 'Trackbars', 179, 179, nothing)
cv2.createTrackbar('SMax', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('VMax', 'Trackbars', 255, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMax', 'Trackbars', 179)
cv2.setTrackbarPos('SMax', 'Trackbars', 255)
cv2.setTrackbarPos('VMax', 'Trackbars', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0

# Use camera index 0 instead of URL
camera_index = 2
cap = cv2.VideoCapture(camera_index)

# Directory to save captured images
save_path = "./vision/image"
os.makedirs(save_path, exist_ok=True)

# Function to detect and label red blobs
def detect_red(frame):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin', 'Trackbars')
    sMin = cv2.getTrackbarPos('SMin', 'Trackbars')
    vMin = cv2.getTrackbarPos('VMin', 'Trackbars')
    hMax = cv2.getTrackbarPos('HMax', 'Trackbars')
    sMax = cv2.getTrackbarPos('SMax', 'Trackbars')
    vMax = cv2.getTrackbarPos('VMax', 'Trackbars')

    # Define the HSV range for red color using trackbar values
    lower_red1 = np.array([hMin, sMin, vMin])
    upper_red1 = np.array([hMax, sMax, vMax])
    lower_red2 = np.array([160, sMin, vMin])
    upper_red2 = np.array([179, sMax, vMax])

    # Threshold the HSV frame to get only red colors
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.add(red_mask1, red_mask2)

    # Apply morphological operations to reduce noise
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for i, contour in enumerate(contours):
        # Ignore small blobs by filtering based on contour area
        if cv2.contourArea(contour) > 50:  # Adjust the area threshold as needed
            # Draw a bounding box around the contour
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Label the blob
            cv2.putText(frame, f'Red Blob {i+1}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    return frame, red_mask

# Add a loop to capture frames from the camera
image_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read frame from camera.")
        break

    # Detect and label red blobs
    labeled_frame, mask = detect_red(frame)

    # Display the results
    cv2.imshow("Live Feed", labeled_frame)
    cv2.imshow("Red Mask", mask)

    key = cv2.waitKey(1) & 0xFF

    # Press 'q' to exit the loop
    if key == ord('q'):
        break

    # Press 'c' to capture and save the frame
    if key == ord('c'):
        image_path = os.path.join(save_path, f"capture_{image_count}.jpg")
        cv2.imwrite(image_path, frame)
        print(f"Image saved to {image_path}")
        image_count += 1

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
 