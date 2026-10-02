# ============================================================
# UNO Q ARUCO ROBOTIC CAR - CONFIGURATION
# ============================================================

# Camera
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
INITIAL_EXPOSURE = 15

CAMERA_DEVICE = (
    "/dev/v4l/by-id/"
    "usb-EMEET_EMEET_SmartCam_C960_4K_A251227000112585"
    "-video-index0"
)

# ArUco
TARGET_MARKER_ID = 17
ARUCO_DICTIONARY = "DICT_4X4_50"

# Tracking area - feet
AREA_WIDTH = 5.0 
AREA_HEIGHT = 7.0

# ESP32 RC car
ESP32_IP = "192.168.1.149"
ESP32_TIMEOUT_SEC = 2.0

# Video stream
JPEG_QUALITY = 40
STREAM_DELAY_SEC = 0.04
