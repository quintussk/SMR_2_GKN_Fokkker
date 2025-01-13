import cv2

def find_available_cameras(max_index=10):
    available_cameras = []
    for index in range(max_index):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            available_cameras.append(index)
            cap.release()
    return available_cameras

# Scan for cameras
max_camera_index = 10  # Adjust this number as needed
available_cameras = find_available_cameras(max_camera_index)

if available_cameras:
    print(f"Available camera indices: {available_cameras}")
else:
    print("No cameras found.")
