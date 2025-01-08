from flask import Flask, Response, render_template, request, jsonify
import cv2
from camera import Camera
from arduino import ArduinoConnection

app = Flask(__name__)

# Initialize the camera (0 is the default camera index)
camera_feed = Camera() 
arduino = ArduinoConnection(port="/dev/ttyACM0")

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
            arduino.move_manual(direction)
            return jsonify({"status": "success", "message": f"Moved {direction}"})
        else:
            return jsonify({"status": "error", "message": "No direction provided"}), 400
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
            arduino.speed1
            arduino.speed2
            arduino.set_speed(speed)
            return jsonify({"status": "success", "message": f"Speed adjusted to {speed}"})
        else:
            return jsonify({"status": "error", "message": "No speed provided"}), 400
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500
    
@app.route('/get_speeds', methods=['GET'])
def get_speeds():
    try:
        speed1 = arduino.speed1  # Verkrijg snelheid 1
        speed2 = arduino.speed2  # Verkrijg snelheid 2
        return jsonify({"speed1": speed1, "speed2": speed2})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)