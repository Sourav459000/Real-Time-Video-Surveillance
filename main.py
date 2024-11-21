# run cmd before execution in a virtual env 'pip install -r requirements.txt'
# 'pip freeze > requirements.txt'

# Import library
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

     # Show frame
    # cv2.imshow('YOLOv8 Detection', frame)
    # if cv2.waitKey(1) == ord('q'):
    #     break

    return cap


def initialize_com(com_no):
    # Setup serial connection
    # ser = serial.Serial(port=com_no, baudrate=9600, parity=serial.PARITY_NONE,
    #                     stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)

    ser = serial.Serial(port=com_no, baudrate=9600)
    return ser


def capture_frame(cap):
    # Clear the buffer
    cap.grab()

    # Read a frame from the video
    ret, frame = cap.read()
    if not ret:
        print("Error: Could Not Capture Image")

    return frame


def pred_person(yolo_model, frame):
    # Load the YOLO model
    # model = YOLO(yolo_model)

    # Perdict detection
    results = yolo_model(frame, conf=0.4, save=False, show=False, verbose=False)

    # Person's Detected
    num_person = results[0].boxes.shape

    # Set Return Variables
    if num_person[0] == 0:
        total_persons_detected = 0
        detection_occurred = False
        # print("Number of Person :",num_person[0])
    else:
        total_persons_detected = num_person[0]
        detection_occurred = True
        # print("Number of Person :",num_person[0])

    return detection_occurred, total_persons_detected


def valid_pred(yolo_model, num_valid, cap):
    # Variable to count number of True detection
    d_count = 0

    # Loop to check mutiple frames
    for _ in range(num_valid):
        # Capture a single frame
        frame = capture_frame(cap)
        detection, total_persons_detected = pred_person(yolo_model, frame)

        # Increment variable
        if detection is True:
            d_count += 1

    # Make prediction
    if d_count > (num_valid/2):
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
    # Convert the command list to bytes
    # command_bytes = bytes(command)

    # Write the command to the serial port
    # ser.write(command_bytes)
    ser.write(str(command).encode())


def setup(yolo_model, num_valid, cap, ser):
    # Give Relay OFF Signal
    print("Relay OFF (Before Prediction)")
    # off_command = [0xFF, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0x00, 0x70, 0x5D]
    off_command = 0
    send_command(ser, off_command)
    relay_on = False
    
    while True:

        # Prediction
        detection_occurred, total_persons_detected = valid_pred(yolo_model, num_valid, cap)

        if detection_occurred == True and relay_on == False:
            # Give Relay ON Signal
            print("Relay ON (After Prediction)")
            # on_command = [0xFF, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0xFF, 0x30, 0x1D]
            on_command = 1
            send_command(ser, on_command)
            relay_on = True
        elif detection_occurred == False and relay_on == True:
            # Give Relay OFF Signal
            print("Relay OFF (After Prediction)")
            # off_command = [0xFF, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0x00, 0x70, 0x5D]
            off_command = 0
            send_command(ser, off_command)
            relay_on = False


def main():
    yolo_model = YOLO("model2_111e.pt") # YoLo model
    num_valid = 3 # Keep this a small odd number
    com_no = 'COM13' # Check COM port number
    cap = initialize_camera()
    ser = initialize_com(com_no)

    # Main Loop
    setup(yolo_model, num_valid, cap, ser) 

    # Cleanup
    cap.release()
    ser.close()
    # cv2.destroyAllWindows()


if __name__ == "__main__" :
    main()
