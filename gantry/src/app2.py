import cv2

def open_camera():
    # Open de standaardcamera (camera index 0)
    cap = cv2.VideoCapture(4)

    if not cap.isOpened():
        print("Error: Kan de camera niet openen")
        return

    print("Druk op 'q' om het venster te sluiten")

    while True:
        # Lees een frame van de camera
        ret, frame = cap.read()

        if not ret:
            print("Error: Kan geen frame lezen van de camera")
            break

        # Toon de camerafeed in een venster
        cv2.imshow("Camera Feed", frame)

        # Wacht op de 'q'-toets om te stoppen
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Sluit de camera en sluit alle vensters
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    open_camera()