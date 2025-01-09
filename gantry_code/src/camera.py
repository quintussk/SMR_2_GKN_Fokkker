import cv2

class Camera:
    def __init__(self):

        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            raise ValueError(f"Camera met index {0} kon niet worden geopend.")

    def generate_frames(self):
        while True:
            # Capture frame-by-frame
            success, frame = self.camera.read()
            if not success:
                break
            else:

                # Encode frame to JPEG format
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    print("Frame encoding failed")
                    break
                frame = buffer.tobytes()

                # Yield the frame as part of an HTTP response
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def release(self):
        # Release the camera resource
        self.camera.release()