# ============================================================
# UNO Q ARUCO ROBOTIC CAR - CAMERA MODULE
# ============================================================

import fcntl
import os
import struct
import threading


# ============================================================
# V4L2 Constants
# ============================================================

V4L2_CID_BRIGHTNESS = 0x00980900
V4L2_CID_CONTRAST = 0x00980901

V4L2_CID_EXPOSURE_AUTO = 0x009A0901
V4L2_CID_EXPOSURE_ABSOLUTE = 0x009A0902

VIDIOC_G_CTRL = 0xC008561B
VIDIOC_S_CTRL = 0xC008561C


class CameraController:
    """V4L2 controls for the EMEET camera."""

    def __init__(
        self,
        ui,
        camera_device,
        initial_exposure=10,
    ):
        self.ui = ui
        self.camera_device = camera_device
        self.initial_exposure = int(initial_exposure)

        self.control_lock = threading.Lock()
        self.control_fd = None

        self.brightness = None
        self.contrast = None
        self.exposure = None
        self.auto_exposure = None

    # ========================================================
    # V4L2 Device
    # ========================================================

    def open_control(self):
        try:
            fd = os.open(
                self.camera_device,
                os.O_RDWR,
            )

            with self.control_lock:
                if self.control_fd is not None:
                    os.close(self.control_fd)
                self.control_fd = fd

            print(
                f"[V4L2] Control interface opened: "
                f"{self.camera_device}"
            )
            return True

        except Exception as exc:
            print(
                f"[V4L2 ERROR] Could not open "
                f"{self.camera_device}: {exc}"
            )
            return False

    def close_control(self):
        with self.control_lock:
            fd = self.control_fd
            self.control_fd = None

        if fd is not None:
            try:
                os.close(fd)
                print("[V4L2] Control interface closed")
            except Exception as exc:
                print(
                    f"[V4L2 ERROR] Closing control interface: {exc}"
                )

    # ========================================================
    # V4L2 Get/Set
    # ========================================================

    def get_control(self, control_id):
        with self.control_lock:
            fd = self.control_fd

        if fd is None:
            print(
                "[V4L2 ERROR] Camera control interface is not open."
            )
            return None

        try:
            data = struct.pack("Ii", control_id, 0)
            result = fcntl.ioctl(
                fd,
                VIDIOC_G_CTRL,
                data,
            )
            _, value = struct.unpack("Ii", result)
            return value

        except Exception as exc:
            print(
                f"[V4L2 READ ERROR] control=0x{control_id:08x}: {exc}"
            )
            return None

    def set_control(self, control_id, value):
        with self.control_lock:
            fd = self.control_fd

        if fd is None:
            print(
                "[V4L2 ERROR] Camera control interface is not open."
            )
            return False

        try:
            data = struct.pack(
                "Ii",
                control_id,
                int(value),
            )
            fcntl.ioctl(
                fd,
                VIDIOC_S_CTRL,
                data,
            )
            return True

        except Exception as exc:
            print(
                f"[V4L2 SET ERROR] control=0x{control_id:08x} "
                f"value={value}: {exc}"
            )
            return False

    # ========================================================
    # Exposure
    # ========================================================

    def set_manual_exposure(self, exposure):
        try:
            exposure = int(float(exposure))
        except (TypeError, ValueError):
            print(
                f"[CAMERA ERROR] Invalid exposure value: {exposure}"
            )
            return False

        exposure = max(1, min(5000, exposure))

        print("------------------------------------------")
        print("[CAMERA] Setting Manual Exposure")
        print("[CAMERA] Requested exposure:", exposure)

        if not self.set_control(V4L2_CID_EXPOSURE_AUTO, 1):
            print(
                "[CAMERA ERROR] Could not switch to Manual Exposure."
            )
            return False

        self.auto_exposure = self.get_control(
            V4L2_CID_EXPOSURE_AUTO
        )

        print(
            "[CAMERA] Auto Exposure:",
            self.auto_exposure,
        )

        if not self.set_control(
            V4L2_CID_EXPOSURE_ABSOLUTE,
            exposure,
        ):
            print("[CAMERA ERROR] Could not set exposure.")
            return False

        actual_exposure = self.get_control(
            V4L2_CID_EXPOSURE_ABSOLUTE
        )

        self.exposure = actual_exposure

        print(
            "[CAMERA] Exposure after setting:",
            actual_exposure,
        )
        print("------------------------------------------")

        return actual_exposure == exposure

    # ========================================================
    # Brightness / Contrast
    # ========================================================

    def set_brightness(self, value):
        try:
            value = int(float(value))
        except (TypeError, ValueError):
            return False

        value = max(-64, min(64, value))

        success = self.set_control(
            V4L2_CID_BRIGHTNESS,
            value,
        )

        if success:
            self.brightness = self.get_control(
                V4L2_CID_BRIGHTNESS
            )
            print("[CAMERA] Brightness:", self.brightness)

        return success

    def set_contrast(self, value):
        try:
            value = int(float(value))
        except (TypeError, ValueError):
            return False

        value = max(0, min(100, value))

        success = self.set_control(
            V4L2_CID_CONTRAST,
            value,
        )

        if success:
            self.contrast = self.get_control(
                V4L2_CID_CONTRAST
            )
            print("[CAMERA] Contrast:", self.contrast)

        return success

    # ========================================================
    # Browser Camera Control
    # ========================================================

    def handle_control(self, sid, data):
        print("------------------------------------------")
        print("[CAMERA CONTROL] Received:", data)
        print("------------------------------------------")

        if not isinstance(data, dict):
            print("[CAMERA CONTROL ERROR] Data is not an object")
            return

        try:
            if "brightness" in data:
                self.set_brightness(data["brightness"])

            if "contrast" in data:
                self.set_contrast(data["contrast"])

            if "exposure" in data:
                self.set_manual_exposure(data["exposure"])

            if "auto_exposure" in data:
                value = int(float(data["auto_exposure"]))
                if self.set_control(
                    V4L2_CID_EXPOSURE_AUTO,
                    value,
                ):
                    self.auto_exposure = self.get_control(
                        V4L2_CID_EXPOSURE_AUTO
                    )
                    print(
                        "[CAMERA] Auto Exposure:",
                        self.auto_exposure,
                    )

        except Exception as exc:
            print(f"[CAMERA CONTROL ERROR] {exc}")

    # ========================================================
    # Status
    # ========================================================

    def send_status(self):
        self.ui.send_message(
            "camera_status",
            message={
                "brightness": self.brightness,
                "contrast": self.contrast,
                "exposure": self.exposure,
                "auto_exposure": self.auto_exposure,
            },
        )

    # ========================================================
    # Initialize Camera Controls
    # ========================================================

    def initialize(self):
        if not self.open_control():
            return False

        print("==========================================")
        print("CAMERA CONTROL VALUES")
        print("==========================================")

        self.brightness = self.get_control(
            V4L2_CID_BRIGHTNESS
        )
        self.contrast = self.get_control(
            V4L2_CID_CONTRAST
        )
        self.exposure = self.get_control(
            V4L2_CID_EXPOSURE_ABSOLUTE
        )
        self.auto_exposure = self.get_control(
            V4L2_CID_EXPOSURE_AUTO
        )

        print("Brightness:", self.brightness)
        print("Contrast:", self.contrast)
        print("Exposure:", self.exposure)
        print("Auto Exposure:", self.auto_exposure)
        print("==========================================")

        print("==========================================")
        print("INITIAL MANUAL EXPOSURE SETUP")
        print("==========================================")

        if self.set_manual_exposure(self.initial_exposure):
            print(
                "[CAMERA] Initial exposure successfully set."
            )
        else:
            print(
                "[CAMERA WARNING] Initial exposure setting failed."
            )

        print("==========================================")
        self.send_status()
        return True

    def register_handlers(self):
        self.ui.on_message(
            "camera_control",
            self.handle_control,
        )
