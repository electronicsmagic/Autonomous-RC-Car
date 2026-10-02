# ============================================================
# UNO Q ARUCO ROBOTIC CAR
# MAIN APPLICATION
# ============================================================

from arduino.app_bricks.web_ui import WebUI
from arduino.app_utils import App

from config import (
    AREA_HEIGHT,
    AREA_WIDTH,
    CAMERA_DEVICE,
    ESP32_IP,
    ESP32_TIMEOUT_SEC,
    FRAME_HEIGHT,
    FRAME_WIDTH,
    INITIAL_EXPOSURE,
    JPEG_QUALITY,
    STREAM_DELAY_SEC,
    TARGET_MARKER_ID,
)

from camera import CameraController
from calibration import CalibrationManager
from aruco_tracker import ArucoTracker
from car_control import CarController
from navigation import NavigationController


# ============================================================
# WebUI
# ============================================================

ui = WebUI()


# ============================================================
# Application Components
# ============================================================

camera = CameraController(
    ui=ui,
    camera_device=CAMERA_DEVICE,
    initial_exposure=INITIAL_EXPOSURE,
)


calibration = CalibrationManager(
    ui=ui,
    area_width=AREA_WIDTH,
    area_height=AREA_HEIGHT,
)


car = CarController(
    ui=ui,
    esp32_ip=ESP32_IP,
    timeout_sec=ESP32_TIMEOUT_SEC,
)


# ============================================================
# STEP 3B
# AUTOMATIC TURNING ONLY
# ============================================================

navigation = NavigationController(
    ui=ui,
    car=car,
)


# ============================================================
# ArUco Tracker
# ============================================================

tracker = ArucoTracker(
    ui=ui,
    calibration=calibration,
    camera=camera,
    navigation=navigation,
    frame_width=FRAME_WIDTH,
    frame_height=FRAME_HEIGHT,
    target_marker_id=TARGET_MARKER_ID,
    jpeg_quality=JPEG_QUALITY,
    stream_delay_sec=STREAM_DELAY_SEC,
)


# ============================================================
# WebUI Connection
# ============================================================

def on_connect(sid):

    print(
        f"[WebUI] Browser connected: {sid}"
    )

    # --------------------------------------------------------
    # Send current camera state
    # --------------------------------------------------------

    camera.send_status()

    # --------------------------------------------------------
    # Send current calibration state
    # --------------------------------------------------------

    calibration.send_status()

    # --------------------------------------------------------
    # Send current navigation state
    # --------------------------------------------------------

    navigation.send_status()


def on_disconnect(sid):

    print(
        f"[WebUI] Browser disconnected: {sid}"
    )


ui.on_connect(
    on_connect
)

ui.on_disconnect(
    on_disconnect
)


# ============================================================
# Register WebUI Messages
# ============================================================

camera.register_handlers()

calibration.register_handlers()

car.register_handlers()

navigation.register_handlers()


# ============================================================
# Start Camera / ArUco Tracker
# ============================================================

tracker.start()


# ============================================================
# Application Information
# ============================================================

print(
    "=========================================="
)

print(
    "UNO Q ARUCO ROBOTIC CAR"
)

print(
    "=========================================="
)

print(
    f"Target Marker : D{TARGET_MARKER_ID}"
)

print(
    f"Tracking Area : "
    f"{AREA_WIDTH} ft x {AREA_HEIGHT} ft"
)

print(
    f"Camera        : "
    f"{FRAME_WIDTH} x {FRAME_HEIGHT}"
)

print(
    f"ESP32         : {ESP32_IP}"
)

print(
    "=========================================="
)

print(
    "STEP 3B:"
)

print(
    "Automatic Turning Only"
)

print(
    "Automatic Speed: 15%"
)

print(
    "Allowed Commands: LEFT / RIGHT / STOP"
)

print(
    "FORWARD: DISABLED"
)

print(
    "BACKWARD: DISABLED"
)

print(
    "=========================================="
)

print(
    "Starting WebUI App..."
)


# ============================================================
# Run App
# ============================================================

App.run()