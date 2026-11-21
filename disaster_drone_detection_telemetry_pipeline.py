"""
AI-Based Disaster Response Drone (Jetson Nano 4GB)
Captures frames from the camera, runs YOLOv8 human detection (class 0),
fuses GPS coordinates from Pixhawk MAVLink GPS_RAW_INT over UART, and
transmits survivor geolocation packets via LoRa SX1278 (433 MHz).
"""

from datetime import datetime
import time
import cv2
from pymavlink import mavutil
from ultralytics import YOLO


CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MODEL_PATH = "yolov8n.pt"
CONFIDENCE_THRESHOLD = 0.50

MAVLINK_PORT = "/dev/ttyTHS1"
MAVLINK_BAUD = 921600


def init_mavlink(port: str = MAVLINK_PORT, baud: int = MAVLINK_BAUD):
    """Connect to Pixhawk TELEM1 MAVLink stream."""
    try:
        conn = mavutil.mavlink_connection(port, baud=baud)
        conn.wait_heartbeat(timeout=5)
        print(f"[MAVLink] Connected on {port} @ {baud} baud.")
        return conn
    except Exception as exc:
        print(f"[MAVLink WARN] Could not connect on {port}: {exc}")
        return None


def get_gps(mav_conn) -> tuple[float, float, float, int]:
    """Extract latitude, longitude, altitude (m), and visible satellites from GPS_RAW_INT."""
    if mav_conn is None:
        return 0.0, 0.0, 0.0, 0

    msg = mav_conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
    if msg is None:
        return 0.0, 0.0, 0.0, 0

    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1000.0
    satellites = int(msg.satellites_visible)
    return lat, lon, alt, satellites


def lora_send(payload: str) -> None:
    """
    Transmit formatted packet over SX1278 LoRa (433 MHz, SF12, BW 125 kHz, CR 4/5).
    Replace or bind this function to your SPI SX1278 driver module on the Jetson Nano.
    """
    print(f"[LoRa TX @ 433MHz] {payload}")


def format_packet(lat: float, lon: float, conf: float) -> str:
    """Format survivor geolocation packet matching ground station parser."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return f"HUMAN {lat:.6f} {lon:.6f}|{int(round(conf * 100))}|{timestamp}"


def main() -> None:
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        raise RuntimeError("Failed to open camera stream.")

    model = YOLO(MODEL_PATH)
    mav_conn = init_mavlink()

    last_tx_time = 0.0
    tx_cooldown_sec = 2.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            results = model(frame, classes=[0], verbose=False)
            now = time.time()

            for result in results:
                for box in result.boxes:
                    conf = float(box.conf[0])
                    if conf > CONFIDENCE_THRESHOLD:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        if now - last_tx_time >= tx_cooldown_sec:
                            lat, lon, alt, sat = get_gps(mav_conn)
                            packet = format_packet(lat, lon, conf)
                            lora_send(packet)
                            print(
                                f"[{datetime.now().strftime('%H:%M:%S')}] SURVIVOR DETECTED! "
                                f"Lat: {lat:.6f}, Lon: {lon:.6f}, Conf: {conf:.0%}, "
                                f"Alt: {alt:.1f}m, Sats: {sat}"
                            )
                            last_tx_time = now

    except KeyboardInterrupt:
        print("\nShutting down survivor tracker...")
    finally:
        cap.release()


if __name__ == "__main__":
    main()