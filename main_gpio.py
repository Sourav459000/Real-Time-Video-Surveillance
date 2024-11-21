# Import necessary libraries
from ultralytics import YOLO
import cv2
import time
import Jetson.GPIO as GPIO

# Initialize camera function
def initialize_camera():
    # Initialize the video capture
    cap = cv2.VideoCapture("rtsp://admin:abcd1234@172.22.57.25")

    # Warm-up the camera
    for _ in range(5):
        cap.read()

    return cap


def initialize_gpio(pin_number):
    # Set the GPIO mode to BCM
    GPIO.setmode(GPIO.BOARD)
    
    # Set the specified GPIO pin as an output pin
    GPIO.setup(pin_number, GPIO.OUT)

    # Ensure the pin is initially LOW (LED OFF)
    GPIO.output(pin_number, GPIO.LOW)


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


def send_command(pin_number, command):
    """
    Send a command via GPIO to control the LED (or other device).
    :param pin_number: GPIO pin number to control.
    :param command: Command to send (either 0 for OFF or 1 for ON).
    :return: None
    """
    if command == 1:
        GPIO.output(pin_number, GPIO.HIGH)  # Turn ON the LED (or device)
        print("LED ON (GPIO HIGH)")
    else:
        GPIO.output(pin_number, GPIO.LOW)  # Turn OFF the LED (or device)
        print("LED OFF (GPIO LOW)")


def setup(yolo_model, num_valid, cap, pin_number):
    # Give Relay OFF Signal
    print("Relay OFF (Before Prediction)")
    send_command(pin_number, 0)  # LED OFF initially
    relay_on = False
    
    while True:
        # Prediction
        detection_occurred, total_persons_detected = valid_pred(yolo_model, num_valid, cap)

        if detection_occurred == True and relay_on == False:
            # Give Relay ON Signal
            print("Relay ON (After Prediction)")
            send_command(pin_number, 1)  # LED ON
            relay_on = True
        elif detection_occurred == False and relay_on == True:
            # Give Relay OFF Signal
            print("Relay OFF (After Prediction)")
            send_command(pin_number, 0)  # LED OFF
            relay_on = False


def main():
    yolo_model = YOLO("model2_111e.pt")  # YOLO model
    num_valid = 3  # Keep this a small odd number
    pin_number = 15  # GPIO pin number (e.g., GPIO17 for controlling an LED)
    
    cap = initialize_camera()
    
    # Initialize GPIO for controlling the LED
    initialize_gpio(pin_number)

    # Main Loop
    setup(yolo_model, num_valid, cap, pin_number)

    # Cleanup
    cap.release()
    GPIO.cleanup()  # Clean up GPIO settings

if __name__ == "__main__":
    main()

