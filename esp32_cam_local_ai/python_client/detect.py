import argparse
import os
import time

import cv2
import numpy as np
import pyttsx3
import requests
from ultralytics import YOLO

DEFAULT_IP = "192.168.1.100"
MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "best.pt")
LABELS_OF_INTEREST = {
    "person",
    "chair",
    "table",
    "wall",
    "door",
    "stair",
    "vehicle",
    "bicycle",
}


def get_jpg_frame(base_url: str):
    """Fetch a single JPEG frame from the ESP32-CAM /jpg endpoint."""
    response = requests.get(f"{base_url}/jpg", timeout=5)
    response.raise_for_status()
    image_array = np.frombuffer(response.content, dtype=np.uint8)
    return cv2.imdecode(image_array, cv2.IMREAD_COLOR)


def speak(text: str):
    """Announce detection using local TTS."""
    try:
        engine = pyttsx3.init()
        engine.say(text)
        engine.runAndWait()
    except Exception as exc:
        print(f"TTS failed: {exc}")


def run_snapshot_mode(base_url: str, model: YOLO, interval: float = 2.0):
    """Periodically fetch snapshots and run inference."""
    print(f"Snapshot mode: fetching from {base_url}/jpg every {interval}s")
    print("Press Ctrl+C to stop.")

    last_announcement = ""

    while True:
        try:
            frame = get_jpg_frame(base_url)
        except Exception as exc:
            print(f"Frame fetch failed: {exc}")
            time.sleep(interval)
            continue

        results = model(frame, verbose=False)
        detected = []

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                label = result.names[cls_id]
                conf = float(box.conf[0])
                if label in LABELS_OF_INTEREST and conf > 0.5:
                    detected.append((label, conf))

        if detected:
            detected.sort(key=lambda x: x[1], reverse=True)
            names = [d[0] for d in detected]
            announcement = ", ".join(names)
            print(f"Detected: {announcement}")

            if announcement != last_announcement:
                speak(announcement)
                last_announcement = announcement
        else:
            print("No relevant objects detected.")

        time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description="ESP32-CAM local YOLO client.")
    parser.add_argument(
        "--ip",
        default=os.environ.get("ESP32_IP", DEFAULT_IP),
        help="ESP32-CAM IP address (or set ESP32_IP env var)",
    )
    parser.add_argument(
        "--model",
        default=MODEL_PATH,
        help="Path to YOLO model weights",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="Seconds between snapshot fetches",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        print(f"Model not found: {args.model}")
        print("Place trained weights at python_client/models/best.pt")
        return

    base_url = f"http://{args.ip}"
    print(f"Loading model from {args.model}...")
    model = YOLO(args.model)

    try:
        run_snapshot_mode(base_url, model, args.interval)
    except KeyboardInterrupt:
        print("\nStopping.")


if __name__ == "__main__":
    main()
