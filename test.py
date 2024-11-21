from ultralytics import YOLO
import cv2

try:
    model = YOLO("model2_111e.pt")
    # Uncomment any of the following lines for different RTSP sources
    # results = model.predict(source='rtsp://root:root@169.254.100.140/axis-media/media.amp', show=True) #default_Axis
    # results = model.predict(source='rtsp://root:root@172.27.120.251/axis-media/media.amp', show=True) #dhcp_Axis
    # results = model.predict(source='rtsp://admin:abcd1234@192.168.1.64', show=True, conf=0.6) #default_Hik
    # results = model.predict(source='rtsp://admin:abcd1234@192.168.0.64', show=True, conf=0.6)
    # results = model.predict(source='rtsp://admin:abcd1234@169.254.124.87', show=True) #dhcp_Hik
    # results = model.predict(source='rtsp://root:root@192.168.0.187/axis-media/media.amp', show=True) #dhcp_Axis_2
    # results = model.predict(source='list3.streams', show=True, conf=0.65) #multistreaming
    # results = model.predict(source=0, show=True, conf=0.6)
    results = model.predict(source='rtsp://admin:abcd1234@172.22.57.25', show=True, conf=0.6)  # Hik_college_network
    print(results)

except KeyboardInterrupt:
    print("\nProcess interrupted by user. Exiting...")
except Exception as e:
    print(f"An error occurred: {e}")

