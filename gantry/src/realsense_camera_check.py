import pyrealsense2 as rs
import numpy as np
from flask import Flask, Response, render_template_string

app = Flask(__name__)

# Initialize RealSense
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 15)
cfg.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 15)

try:
    pipeline.start(cfg)
    print("RealSense pipeline started successfully!")
except Exception as e:
    print(f"Failed to start RealSense pipeline: {e}")
    exit(1)

# HTML template for Flask app
HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>RealSense Stream</title>
</head>
<body>
    <h1>RealSense Camera Feed</h1>
    <img src="{{ url_for('video_feed') }}" />
</body>
</html>
"""

def get_frame():
    while True:
        try:
            frames = pipeline.poll_for_frames()
            if not frames:
                continue  # Probeer opnieuw als er geen frames beschikbaar zijn
            
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            _, jpeg = cv2.imencode('.jpg', color_image)
            frame = jpeg.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        except Exception as e:
            print(f"Error in get_frame: {e}")
            break

@app.route('/')
def index():
    """
    Render the main page with the video feed.
    """
    return render_template_string(HTML)

@app.route('/video_feed')
def video_feed():
    """
    Provide the video feed for the Flask app.
    """
    return Response(get_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Run the Flask app
        app.run(host='0.0.0.0', port=5000)
    finally:
        # Stop the RealSense pipeline when the app is stopped
        pipeline.stop()
        print("RealSense pipeline stopped.")