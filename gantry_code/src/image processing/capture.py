import cv2
import os

def capture_image(save_directory="images"):
    # Create the directory if it doesn't exist
    if not os.path.exists(save_directory):
        os.makedirs(save_directory)
    
    # Find the next incremental filename
    existing_files = [f for f in os.listdir(save_directory) if f.startswith("image") and f.endswith(".jpg")]
    if existing_files:
        # Extract the numbers from filenames and find the maximum
        numbers = [int(f[5:-4]) for f in existing_files if f[5:-4].isdigit()]
        next_number = max(numbers) + 1
    else:
        next_number = 1
    
    filename = f"image{next_number}.jpg"
    
    # Initialize the webcam
    cap = cv2.VideoCapture(0)
    
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        print("Error: Could not open webcam")
        return
    
    # Capture a single frame
    ret, frame = cap.read()
    
    if ret:
        # Save the captured image to the specified directory
        image_path = os.path.join(save_directory, filename)
        cv2.imwrite(image_path, frame)
        print(f"Image saved at {image_path}")
    else:
        print("Error: Could not capture image")
    
    # Release the webcam and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Example usage
capture_image()
