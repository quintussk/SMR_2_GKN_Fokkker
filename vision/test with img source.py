import cv2
import numpy as np
import os

# Directory to load images from
source_path = "./vision/source"
if not os.path.exists(source_path):
    print("Error: Source directory does not exist.")
    exit()

# Directory to save processed images
save_path = "./vision/image"
os.makedirs(save_path, exist_ok=True)

# Function to detect and label red blobs
def detect_red(frame):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the HSV range for red color
    lower_red1 = np.array([0, 10, 70])  # Lower red range (hue near 0)
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

# Process images from the source directory
image_count = 0
for file_name in sorted(os.listdir(source_path)):
    file_path = os.path.join(source_path, file_name)
    
    # Ensure the file is an image
    if not (file_name.endswith(".jpg") or file_name.endswith(".png")):
        continue

    # Read the image
    frame = cv2.imread(file_path)
    if frame is None:
        print(f"Error: Unable to read {file_name}")
        continue

    # Detect and label red blobs
    labeled_frame, mask = detect_red(frame)

    # Display the results
    cv2.imshow("Image", labeled_frame)
    cv2.imshow("Red Mask", mask)

    key = cv2.waitKey(0) & 0xFF  # Wait for a key press

    # Press 'q' to exit the loop
    if key == ord('q'):
        break

    # Press 'c' to save the processed frame
    if key == ord('c'):
        image_path = os.path.join(save_path, f"processed_{image_count}.jpg")
        cv2.imwrite(image_path, labeled_frame)
        print(f"Processed image saved to {image_path}")
        image_count += 1

# Close all windows
cv2.destroyAllWindows()
