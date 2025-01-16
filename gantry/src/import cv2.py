import cv2
from ultralytics import YOLO

# Load a model
model = YOLO("best.pt")  # pretrained YOLO11n model

# Open a connection to the webcam
cap = cv2.VideoCapture(2)  # Change the index if you have multiple cameras

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Run inference on the frame
    results = model(frame)  # return a Results object

    # Process results
    for result in results:
        boxes = result.boxes  # Boxes object for bounding box outputs
        masks = result.masks  # Masks object for segmentation masks outputs
        keypoints = result.keypoints  # Keypoints object for pose outputs
        probs = result.probs  # Probs object for classification outputs
        obb = result.obb  # Oriented boxes object for OBB outputs

        # Display the results
        result.show()  # display to screen

    # Display the frame
    cv2.imshow("Webcam Feed", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close windows
cap.release()
cv2.destroyAllWindows()