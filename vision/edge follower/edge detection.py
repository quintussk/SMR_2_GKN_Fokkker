import cv2
import numpy as np
import serial  # Import pyserial
import time

# Replace with the correct URL for the video stream
video_source = "http://192.168.0.100:8080/video"  # Change based on your actual source

# Configure the serial connection to the Arduino
arduino_port = "COM3"  # Replace with your Arduino's port (e.g., COM3, /dev/ttyUSB0)
baud_rate = 9600  # Ensure it matches the Arduino's serial configuration
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  # Allow the connection to initialize

# Load saved settings from a text file
def load_settings(file_name="settings.txt"):
    try:
        with open(file_name, "r") as file:
            settings = [int(line.strip()) for line in file.readlines()]
            return settings if len(settings) == 3 else [5, 100, 200]
    except FileNotFoundError:
        return [5, 100, 200]

# Save settings to a text file
def save_settings(settings, file_name="settings.txt"):
    with open(file_name, "w") as file:
        file.writelines(f"{val}\n" for val in settings)

# Initialize settings
kernel_size, low_threshold, high_threshold = load_settings()

# Open video capture
cap = cv2.VideoCapture(video_source)

if not cap.isOpened():
    print("Error: Couldn't open video stream.")
    exit()

# Callback function for trackbars
def update_settings(val):
    global kernel_size, low_threshold, high_threshold
    kernel_size = cv2.getTrackbarPos("Blur Kernel", "Controls") * 2 + 1
    low_threshold = cv2.getTrackbarPos("Low Threshold", "Controls")
    high_threshold = cv2.getTrackbarPos("High Threshold", "Controls")

# Create a control window with sliders
cv2.namedWindow("Controls")
cv2.createTrackbar("Blur Kernel", "Controls", kernel_size // 2, 10, update_settings)
cv2.createTrackbar("Low Threshold", "Controls", low_threshold, 255, update_settings)
cv2.createTrackbar("High Threshold", "Controls", high_threshold, 255, update_settings)

while True:
    # Read a frame from the stream
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur
    blurred = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, low_threshold, high_threshold)

    # Calculate the middle y-coordinate
    half_y = edges.shape[0] // 2

    # Extract the row of pixels at the middle y-coordinate
    edge_row = edges[half_y, :]

    # Find x-coordinates where edges exist
    edge_x_coords = np.where(edge_row > 0)[0]

    if edge_x_coords.size > 0:
        # Calculate the average of the x-coordinates
        average_x = np.sum(edge_x_coords) / edge_x_coords.size

        # Send average_x to Arduino
        arduino.write(f"{int(average_x)}\n".encode('utf-8'))
        print(f"Sent to Arduino: {int(average_x)}")

    # Display the original frame and edges side by side
    cv2.imshow("Original Frame", frame)
    cv2.imshow("Edge Detection", edges)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Save the settings before exiting
save_settings([kernel_size, low_threshold, high_threshold])

# Release the capture object, close windows, and Arduino connection
cap.release()
cv2.destroyAllWindows()
arduino.close()
