# Import necessary libraries
from ultralytics import YOLO
import cv2
import time
import serial

def initialize_camera():
    # Initialize the video capture
    cap = cv2.VideoCapture("rtsp://admin:abcd1234@172.22.57.25")

    # Warm-up the camera
    for _ in range(5):
        cap.read()

    return cap


def initialize_com(com_no=None):
    # Since we are not using a COM port anymore, return None
    # In the original code, we were initializing a serial connection, but now we don't need that.
    print("Serial communication is disabled on this device.")
    return None


def capture_frame(cap):
    # Clear the buffer
    cap.grab()

    # Read a frame from the video
    ret, frame = cap.read()
    if not ret:
        print("Error: Could Not Capture Image")

    return frame


def pred_person(yolo_model, frame):
    # Predict detection
    results = yolo_model(frame, conf=0.4, save=False, show=False, verbose=False)

    # Check number of people detected
    num_person = results[0].boxes.shape

    # Return detection information
    if num_person[0] == 0:
        total_persons_detected = 0
        detection_occurred = False
    else:
        total_persons_detected = num_person[0]
        detection_occurred = True

    return detection_occurred, total_persons_detected


def valid_pred(yolo_model, num_valid, cap):
    # Variable to count number of True detection
    d_count = 0

    # Loop to check multiple frames
    for _ in range(num_valid):
        # Capture a single frame
        frame = capture_frame(cap)
        detection, total_persons_detected = pred_person(yolo_model, frame)

        # Increment variable if detection is True
        if detection is True:
            d_count += 1

    # Make prediction based on majority
    if d_count > (num_valid / 2):
        detection_occurred = True
    else:
        detection_occurred = False

    return detection_occurred, total_persons_detected


def send_command(ser, command):
    """
    Send a command over serial.
    :param command: Command to send (as a list of hex values).
    :return: None
    """
    if ser is not None:
        # Convert the command list to bytes and send if serial communication is active
        ser.write(str(command).encode())
    else:
        # Print command to CLI (since we aren't using a COM port)
        print(f"Command to be sent (in CLI instead of serial): {command}")


def setup(yolo_model, num_valid, cap, ser):
    # Give Relay OFF Signal
    print("Relay OFF (Before Prediction)")
    off_command = 0  # Example command
    send_command(ser, off_command)
    relay_on = False
    
    while True:
        # Prediction
        detection_occurred, total_persons_detected = valid_pred(yolo_model, num_valid, cap)

        if detection_occurred == True and relay_on == False:
            # Give Relay ON Signal
            print("Relay ON (After Prediction)")
            on_command = 1  # Example command
            send_command(ser, on_command)
            relay_on = True
        elif detection_occurred == False and relay_on == True:
            # Give Relay OFF Signal
            print("Relay OFF (After Prediction)")
            off_command = 0  # Example command
            send_command(ser, off_command)
            relay_on = False


def main():
    yolo_model = YOLO("model2_111e.pt")  # YOLO model
    num_valid = 3  # Keep this a small odd number
    com_no = None  # We no longer use COM port
    cap = initialize_camera()
    ser = initialize_com(com_no)  # Now ser will be None

    # Main Loop
    setup(yolo_model, num_valid, cap, ser)

    # Cleanup
    cap.release()
    # If serial communication was used, we would close it here
    if ser:
        ser.close()
    # cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

