# Smart Glasses for Blind Assistance

## Project Overview
This project is a smart glasses prototype for blind and visually impaired users.  
It uses an ESP32-CAM for image capture, an ESP8266 for sensor communication, and onboard sensors such as a Time-of-Flight (ToF) sensor and MPU6050 IMU.  
The system is designed to detect nearby obstacles and relevant objects, then provide feedback through audio or vibration.

## Current Development Phase
We are currently testing the system locally on a laptop before moving to AWS/cloud.  
At this stage, only the ESP32-CAM frames are sent to the local machine, where a local ML model performs inference.  
Face detection is not part of the model. The model should focus only on objects, obstacles, and currency recognition if needed.

## Hardware Components
### Microcontrollers
- ESP32-CAM: captures and streams camera frames.
- ESP8266: handles sensor communication and telemetry.

### Sensors
- ToF sensor: used for obstacle distance measurement.
- MPU6050: used for motion and tilt detection.

### Output Devices
- Audio output via earphone, bone-conduction headset, or phone TTS.
- Optional vibration motor for silent alerts.

## Data Flow
1. ESP32-CAM captures frames from the front view.
2. ESP32-CAM exposes a `/jpg` endpoint for snapshots and `/stream` endpoint for live MJPEG video.
3. A laptop receives frames over Wi-Fi.
4. A local Python ML pipeline processes each frame.
5. The ML model detects objects and returns labels/bounding boxes.
6. The system can later be expanded to include sensor fusion from ToF and MPU6050 data.

## Software Stack
### ESP32-CAM Side
- Arduino IDE or PlatformIO
- `esp_camera.h`
- `WiFi.h`
- `WebServer.h`

### Laptop Side
- Python 3
- OpenCV
- NumPy
- Requests
- Ultralytics YOLO
- Optional TTS library such as pyttsx3

## Local Testing Goal
The goal of the local test is to verify:
- ESP32-CAM frame capture works.
- The `/jpg` snapshot endpoint works.
- The `/stream` endpoint works for live video.
- The laptop can receive frames and run ML inference successfully.
- The model produces useful object labels for blind assistance.

## Recommended Model Scope
The ML model should detect:
- person
- chair
- table
- wall
- door
- stair
- vehicle
- bicycle
- currency notes

The model should not include face detection.

## File Structure
```text
esp32_cam_local_ai/
├── esp32cam_firmware/
│   └── esp32cam_webserver.ino
├── python_client/
│   ├── detect.py
│   ├── requirements.txt
│   └── models/
│       └── best.pt
└── README.txt
```

## Run Workflow
1. Flash the ESP32-CAM firmware.
2. Connect the ESP32-CAM to Wi-Fi.
3. Read the ESP32-CAM IP address from Serial Monitor.
4. Open `/jpg` or `/stream` in a browser to verify camera output.
5. Run the Python client on the laptop.
6. Confirm that YOLO inference works on the received frames.

## Notes
- Start with snapshot mode if stream handling is unstable.
- Use better lighting for camera testing.
- Keep obstacle sensing local in future versions for safety.
- AWS cloud integration may be added later after local testing is stable.
