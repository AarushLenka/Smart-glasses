# Smart Glasses for Blind Assistance

Prototype for blind and visually impaired users. ESP32-CAM streams video to a local laptop running YOLO object detection. Feedback via TTS audio, optional vibration.

## Current Setup

- ESP32-CAM streams frames over Wi-Fi using `/jpg` and `/stream`.
- Local Python client fetches snapshots and runs YOLO inference.
- Detected objects: person, chair, table, wall, door, stair, vehicle, bicycle.
- Face detection intentionally excluded from scope.

## Hardware

- ESP32-CAM: camera + Wi-Fi stream server.
- ESP8266: reserved for sensor telemetry (not yet integrated).
- ToF sensor: reserved for obstacle distance (not yet integrated).
- MPU6050 IMU: reserved for motion/tilt (not yet integrated).

## Project Structure

```text
esp32_cam_local_ai/
├── esp32cam_firmware/
│   └── esp32cam_webserver.ino
├── python_client/
│   ├── detect.py
│   ├── requirements.txt
│   └── models/
└── README.txt
```

## What Works Now

- ESP32-CAM web server with `/jpg` snapshot endpoint.
- Python snapshot-mode detector using OpenCV, YOLO, and pyttsx3.
- Placeholder model directory ready for weights.

## Next Steps

1. Add or train YOLO weights and copy to `python_client/models/best.pt`.
2. Flash firmware and update Wi-Fi credentials if needed.
3. Install Python dependencies:
   ```bash
   pip install -r esp32_cam_local_ai/python_client/requirements.txt
   ```
4. Run local inference:
   ```bash
   python esp32_cam_local_ai/python_client/detect.py --ip <esp32-cam-ip>
   ```
5. Verify `/jpg` and `/stream` in a browser first.
6. Integrate ESP8266 + ToF distance + MPU6050 motion data.
7. Move obstacle sensing fully local for safety.
8. Add AWS/cloud backend only after local testing is stable.
