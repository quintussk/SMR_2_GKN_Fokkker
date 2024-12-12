import cv2
import numpy as np
url = "http://192.168.0.100:8080/video"

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
            cv2.putText(frame, f'Red Blob {i+1}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame, red_mask

# Start live camera feed
cap = cv2.VideoCapture(url)  # 0 for the default camera; use 1, 2, etc., for other cameras

if not cap.isOpened():
    print("Error: Could not access the camera.")
    exit()

while True:
    ret, frame = cap.read()  # Capture frame-by-frame
    if not ret:
        print("Error: Unable to read frame.")
        break

    # Detect and label red blobs
    labeled_frame, mask = detect_red(frame)

    # Display the results
    cv2.imshow("Live Feed", labeled_frame)
    cv2.imshow("Red Mask", mask)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
