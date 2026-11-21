# AI-Based Disaster Response Drone (Autonomous Payload Drone with Target Detection)

An autonomous F450 quadcopter system designed for waypoint payload delivery (up to 500 g) and real-time aerial survivor detection using a **Pixhawk 2.4.8** flight controller (ArduPilot + Neo-M8N GPS) paired with a **Seeed Studio Jetson Nano 4GB** (CUDA-accelerated **YOLOv8** + **Raspberry Pi Camera Module 2**). Survivor coordinates are fused from MAVLink telemetry (`GPS_RAW_INT` over `TELEM1` at 921600 baud) and transmitted to a ground station via an **SX1278 LoRa** module at 433 MHz.

Developed at **SRM Institute of Science & Technology**, Dept. of Electronics and Instrumentation Engineering, under the guidance of **Dr. G.Y. Rajaa Vikhram**.

## System Architecture

- **Flight Control & Navigation (Pixhawk 2.4.8 + Neo-M8N GPS):** Runs ArduPilot firmware for autonomous waypoint navigation, attitude stabilization, PWM motor control via 4x 40A ESCs and 920kV BLDC motors, and MG90S servo payload release (AUX OUT 5, triggered via FlySky i6).
- **AI Perception & Telemetry Fusion (Jetson Nano 4GB + Pi Camera Module 2):** Runs YOLOv8 (`classes=[0]`, confidence > 0.5 at 640x480) with CUDA acceleration, reads MAVLink `GPS_RAW_INT` messages over UART (`/dev/ttyTHS1` at 921600 baud), and formats survivor geolocation packets.
- **Beyond-Line-of-Sight Telemetry (LoRa SX1278 @ 433 MHz):** Transmits formatted survivor packets (`HUMAN|<lat>|<lon>|<conf>%` or `HUMAN <lat> <lon>|<conf>|<timestamp>`) over SPI to the ground station.
- **Power Distribution:** Powered by a 5200mAh 3S LiPo battery (11.1V nominal) with an APM power module (5V / 3A BEC) for the Pixhawk and a separate 12V-to-5V (3A) buck converter for the Jetson Nano.

## Key Performance Benchmarks (From Testing)

- **Human Detection:** ~87% average detection rate (95% single person in open field) at 15-20 FPS across 10-30 m altitude.
- **Geolocation Accuracy:** ~2.1 m mean error while hovering and ~3.2 m during slow forward flight (~2.5 m CEP GPS baseline).
- **LoRa Link:** Up to ~2 km range in open terrain with ~95% packet success rate.
- **Payload Release:** Up to 500 g capacity using a 3D-printed PLA mechanism and MG90S servo (~96% success rate over 50 tests).
- **Flight Endurance:** ~10-12 minutes per 5200mAh 3S LiPo charge.