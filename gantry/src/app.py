from flask import Flask, Response, render_template, request, jsonify, send_from_directory
import cv2
import os
from capture import ImageCaptureAndStitch
from arduino import ArduinoConnection
from datetime import datetime
from scan import Scanning
import threading
import asyncio
from pathlib import Path
import json
from rich import print as print
from test_scanning import Camera

app = Flask(__name__)

# Initialize the camera (0 is the default camera index)
# image_capture_and_stitch = ImageCaptureAndStitch()

# camera_feed = Camera() 

scanner = Camera()

arduino = ArduinoConnection(port="//dev/ttyACM0")
Gantry_Scan = Scanning(arduinoClass=arduino, Camera=scanner)

# Directory to save captured images
image_dir = 'c:/Users/ihsan/Documents/SMR_2_GKN_Fokkker/images'
os.makedirs(image_dir, exist_ok=True)

@app.route('/video_feed')
def video_feed_rgb():
    return Response(scanner.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    # Dynamisch pad naar de afbeelding
    image_path = Path(__file__).parent / "Pictures" / "stitched_image.jpg"
    print(image_path)
    # Controleer of het bestand bestaat
    print(f"Checking image path: {image_path}")
    print(f"Does the file exist? {image_path.exists()}")
    if image_path.exists():
        image_url = f"/images/{image_path.name}"
    else:
        image_url = "/images/default.jpg"  # Fallback als het bestand niet bestaat
    return render_template('index.html', image_url=image_url)

@app.route('/move', methods=['POST'])
def move():
    try:
        direction = data.get('direction')
        data = request.get_json()
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
# def capture_image():
#     try:
#         # Capture the current frame
#         image_capture_and_stitch.capture_frame()
#         return jsonify({"status": "success", "message": "Image captured successfully"})
#     except Exception as e:
#         return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/start_capture', methods=['POST'])
# def start_capture():
#     try:
#         image_capture_and_stitch.start_capture()
#         return jsonify({"status": "success", "message": "Capture started"})
#     except Exception as e:
#         return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/stop_capture', methods=['POST'])
# def stop_capture():
#     try:
#         image_capture_and_stitch.stop_capture()
#         return jsonify({"status": "success", "message": "Capture stopped"})
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
        # Gebruik de map waarin de afbeelding zich bevindt
        images_dir = Path(__file__).parent / "Pictures"
        return send_from_directory(images_dir, filename)
    except Exception as e:
        print(f"Error serving image: {e}")  # Log foutmelding
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

@app.route('/latest_image', methods=['GET'])
def latest_image():
    try:
        images_dir = Path(__file__).parent / "Pictures"
        image_path = images_dir / "stitched_image.jpg"
        if image_path.exists():
            return jsonify({"status": "success", "image_url": f"/images/{image_path.name}"})
        else:
            return jsonify({"status": "error", "message": "Image not found"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

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

@app.route('/start_new_scan', methods=['POST'])
def start_new_scan():
    data = request.json
    x_distance = data.get('x_distance')
    y_distance = data.get('y_distance')
    now = datetime.now()
    formatted_datetime = now.strftime("%d-%m-%Y_%H:%M")
    print(f"Scan started at X: {x_distance} cm and Y: {y_distance} cm, at {formatted_datetime}")
    # Voeg logica toe om de scan te starten

    def start_scan_thread(x_distance, y_distance, formatted_datetime):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(Gantry_Scan.Start_Scanning(X_Total=x_distance, Y_Total=y_distance, mold=formatted_datetime))
        loop.close()

    global scan_thread
    scan_thread = threading.Thread(target=start_scan_thread, args=(x_distance, y_distance, formatted_datetime))
    scan_thread.start()
    return jsonify({"status": "success", "message": "Scan gestart"})

@app.route('/status_scan', methods=['GET'])
def status_scan():
    try:
        # Zorg ervoor dat je de scanstatus en voortgang opslaat in je Scanning-klasse
        status = Gantry_Scan.status_scan()
        return jsonify({
            "progress": status.get("progress", 0),
            "scan_status": status.get("scan_status", "idle"),  # Voeg "Epoxy found" of "Epoxy not found" hier toe
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/stop_scan', methods=['POST'])
def stop_scan():
    global scan_thread
    try:
        # Stop de scan actief in de Scanning-klasse
        asyncio.run(Gantry_Scan.stop_scanning())

        # Controleer of de scan thread bestaat en actief is
        if 'scan_thread' in globals() and scan_thread.is_alive():
            print("Waiting for the scan thread to finish...")
            scan_thread.join()  # Wacht tot de thread is gestopt
            del globals()['scan_thread']  # Verwijder de referentie naar de thread
            print("Scan thread stopped.")

        return jsonify({"status": "success", "message": "Scan stopped successfully"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500
    
@app.route('/epoxy_data', methods=['GET'])
def epoxy_data():
    epoxy_dir = Path(__file__).parent / "Epoxy"
    json_file_path = epoxy_dir / "epoxy.json"

    if not json_file_path.exists():
        print(f"File not found: {json_file_path}")
        return jsonify(error="File not found", data={"mold_dimensions": {"x": 0, "y": 0}, "epoxy_points": []})

    try:
        with open(json_file_path, 'r') as file:
            data = json.load(file)
        return jsonify(data=data)
    except Exception as e:
        print(f"Error reading JSON file: {e}")
        return jsonify(error=str(e), data={"mold_dimensions": {"x": 0, "y": 0}, "epoxy_points": []})
    
@app.route('/update_mold_dimensions', methods=['POST'])
def update_mold_dimensions():
    epoxy_dir = Path(__file__).parent / "Epoxy"
    json_file_path = epoxy_dir / "epoxy.json"

    try:
        data = request.get_json()
        x_distance = data.get('x_distance')
        y_distance = data.get('y_distance')

        epoxy_data = {
            'mold_dimensions': {
                'x': float(x_distance),
                'y': float(y_distance)
            },
            'epoxy_points': []
        }

        with open(json_file_path, 'w') as file:
            json.dump(epoxy_data, file, indent=4)

        return jsonify({"status": "success"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/update_epoxy_point', methods=['POST'])
def update_epoxy_point():
    epoxy_dir = Path(__file__).parent / "Epoxy"
    json_file_path = epoxy_dir / "epoxy.json"

    try:
        data = request.get_json()
        point_id = data.get('id')
        removed = data.get('removed')

        with open(json_file_path, 'r') as file:
            epoxy_data = json.load(file)

        for point in epoxy_data['epoxy_points']:
            if point['id'] == point_id:
                point['removed'] = removed

        with open(json_file_path, 'w') as file:
            json.dump(epoxy_data, file, indent=4)

        return jsonify({"status": "success"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/move_steps', methods=['POST'])
def move_steps():
    try:
        data = request.get_json()
        horizontal_steps = data.get('horizontal_steps')
        vertical_steps = data.get('vertical_steps')
        if horizontal_steps is not None and vertical_steps is not None:
            # Use asyncio to run the async function
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(arduino.send_steps(horizontal_steps, vertical_steps))
            return jsonify({"status": "success", "message": f"Moved {horizontal_steps} horizontal steps and {vertical_steps} vertical steps"})
        else:
            return jsonify({"status": "error", "message": "Steps not provided"}), 400
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)