import cv2

class Camera:
    def __init__(self):
        pass

    def connect(self):
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            raise ValueError(f"Camera met index {0} kon niet worden geopend.")
        
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 100  # Minimum area of a blob
        params.maxArea = 5000  # Maximum area of a blob
        params.filterByCircularity = True
        params.minCircularity = 0.1
        params.filterByConvexity = True
        params.minConvexity = 0.5
        params.filterByInertia = True
        params.minInertiaRatio = 0.01
                # Create a blob detector with the specified parameters
        self.detector = cv2.SimpleBlobDetector_create(params)

    def generate_frames(self):
        while True:
            # Capture frame-by-frame
            success, frame = self.camera.read()
            if not success:
                break
            else:
                # Convert frame to grayscale for blob detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Detect blobs
                keypoints = self.detector.detect(gray)

                # Draw detected blobs as red circles
                frame_with_blobs = cv2.drawKeypoints(frame, keypoints, None, (0, 0, 255),
                                                     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                # Encode frame to JPEG format
                ret, buffer = cv2.imencode('.jpg', frame_with_blobs)
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