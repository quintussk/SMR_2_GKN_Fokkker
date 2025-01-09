from flask import Flask, Response, render_template, request, jsonify, send_from_directory
import cv2
import os
from camera import Camera
from capture import ImageCaptureAndStitch
# from arduino import ArduinoConnection

app = Flask(__name__)

# Initialize the camera (0 is the default camera index)
camera_feed = Camera() 
image_capture_and_stitch = ImageCaptureAndStitch()
# arduino = ArduinoConnection(port="/dev/ttyACM0")

# Directory to save captured images
image_dir = 'c:/Users/ihsan/Documents/SMR_2_GKN_Fokkker/images'
os.makedirs(image_dir, exist_ok=True)

@app.route('/video_feed')
def video_feed():
    # Return the video stream response
    return Response(camera_feed.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/move', methods=['POST'])
def move():
    try:
        data = request.get_json()
        direction = data.get('direction')
        if direction:
            # Send the command to Arduino
            print(direction)
            # arduino.move_manual(direction)
            return jsonify({"status": "success", "message": f"Moved {direction}"})
        else:
            return jsonify({"status": "error", "message": "No direction provided"}), 400
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/capture_image', methods=['POST'])
def capture_image():
    try:
        # Capture the current frame
        image_capture_and_stitch.capture_frame()
        return jsonify({"status": "success", "message": "Image captured successfully"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/start_capture', methods=['POST'])
def start_capture():
    try:
        image_capture_and_stitch.start_capture()
        return jsonify({"status": "success", "message": "Capture started"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/stop_capture', methods=['POST'])
def stop_capture():
    try:
        image_capture_and_stitch.stop_capture()
        return jsonify({"status": "success", "message": "Capture stopped"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

# @app.route('/latest_image')
# def latest_image():
#     try:
#         # Get the latest captured image
#         image_files = sorted(os.listdir(image_dir), key=lambda x: os.path.getctime(os.path.join(image_dir, x)))
#         if not image_files:
#             return jsonify({"status": "error", "message": "No images found"}), 404

#         latest_image_path = image_files[-1]
#         return send_from_directory(image_dir, latest_image_path)
#     except Exception as e:
#         return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/stitched_images')
def stitched_images():
    try:
        # Get the list of stitched images
        image_files = [f for f in os.listdir(image_dir) if f.startswith('stitched')]
        return jsonify({"status": "success", "images": image_files})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/images/<filename>')
def get_image(filename):
    try:
        return send_from_directory(image_dir, filename)
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/adjust_speed', methods=['POST'])
def adjust_speed():
    try:
        data = request.get_json()
        print(f"Received data: {data}")  # Debugging
        speed = data.get('speed')
        if speed:
            print(f"Adjusting speed: {speed}")
            # arduino.speed1
            # arduino.speed2
            # arduino.set_speed(speed)
            return jsonify({"status": "success", "message": f"Speed adjusted to {speed}"})
        else:
            return jsonify({"status": "error", "message": "No speed provided"}), 400
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/get_speeds', methods=['GET'])
def get_speeds():
    try:
        # speed1 = arduino.speed1  # Verkrijg snelheid 1
        # speed2 = arduino.speed2  # Verkrijg snelheid 2
        speed1 = 100  # Placeholder value
        speed2 = 200  # Placeholder value
        return jsonify({"speed1": speed1, "speed2": speed2})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)