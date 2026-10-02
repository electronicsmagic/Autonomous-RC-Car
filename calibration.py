# ============================================================
# UNO Q ARUCO ROBOTIC CAR - CALIBRATION MODULE
# ============================================================

import threading

import cv2
import numpy as np


class CalibrationManager:
    """Four-point camera-to-floor homography calibration."""

    def __init__(self, ui, area_width, area_height):
        self.ui = ui

        self.area_width = float(area_width)
        self.area_height = float(area_height)

        self.calibration_points = {}

        self.current_step = 1

        self.transform_matrix = None

        self.latest_marker_center = None

        self.state_lock = threading.Lock()

    # ========================================================
    # CALIBRATION MARKER IDs
    # ========================================================
    #
    # D0 = (0,0)
    # D1 = (5,0)
    # D2 = (5,7)
    # D3 = (0,7)
    #

    @property
    def calibration_marker_ids(self):
        return {
            1: 0,
            2: 1,
            3: 2,
            4: 3,
        }

    # ========================================================
    # CALIBRATION PHYSICAL POSITIONS
    # ========================================================

    @property
    def calibration_positions(self):
        return {
            1: (0.0, 0.0),
            2: (self.area_width, 0.0),
            3: (
                self.area_width,
                self.area_height,
            ),
            4: (
                0.0,
                self.area_height,
            ),
        }

    # ========================================================
    # CURRENT CALIBRATION MARKER
    # ========================================================

    def get_current_marker_id(self):
        with self.state_lock:
            step = self.current_step

        return self.calibration_marker_ids.get(step)

    # ========================================================
    # MARKER STATE
    # ========================================================

    def set_latest_marker_center(self, center):
        with self.state_lock:
            self.latest_marker_center = center

    def clear_latest_marker_center(self):
        self.set_latest_marker_center(None)

    # ========================================================
    # STATUS
    # ========================================================

    def send_status(self):
        with self.state_lock:
            captured = [
                step in self.calibration_points
                for step in range(1, 5)
            ]

            step = self.current_step

            calibrated = (
                self.transform_matrix is not None
            )

        marker_id = None

        if not calibrated:
            marker_id = self.calibration_marker_ids.get(
                step
            )

        self.ui.send_message(
            "calibration_status",
            message={
                "step": step,
                "captured": captured,
                "calibrated": calibrated,
                "area_width": self.area_width,
                "area_height": self.area_height,

                # New information for the WebUI
                "marker_id": marker_id,
            },
        )

    # ========================================================
    # HOMOGRAPHY
    # ========================================================

    def calculate_transform(self):

        if len(self.calibration_points) != 4:
            print(
                "[CALIBRATION] "
                "Not enough points."
            )

            return False

        src_pts = np.array(
            [
                self.calibration_points[1],
                self.calibration_points[2],
                self.calibration_points[3],
                self.calibration_points[4],
            ],
            dtype=np.float32,
        )

        dst_pts = np.array(
            [
                [0.0, 0.0],

                [
                    self.area_width,
                    0.0,
                ],

                [
                    self.area_width,
                    self.area_height,
                ],

                [
                    0.0,
                    self.area_height,
                ],
            ],
            dtype=np.float32,
        )

        matrix = cv2.getPerspectiveTransform(
            src_pts,
            dst_pts,
        )

        with self.state_lock:
            self.transform_matrix = matrix

        print(
            "=========================================="
        )

        print(
            "CALIBRATION COMPLETE"
        )

        print(
            "=========================================="
        )

        print(
            "Tracking area:"
        )

        print(
            f"Width  = "
            f"{self.area_width} ft"
        )

        print(
            f"Height = "
            f"{self.area_height} ft"
        )

        print(
            "Calibration markers:"
        )

        for step in range(1, 5):

            marker_id = (
                self.calibration_marker_ids[step]
            )

            physical = (
                self.calibration_positions[step]
            )

            pixel = (
                self.calibration_points[step]
            )

            print(
                f"Step {step}: "
                f"D{marker_id} "
                f"pixel={pixel} "
                f"-> "
                f"physical={physical}"
            )

        print(
            "------------------------------------------"
        )

        print(
            "Perspective transformation matrix:"
        )

        print(matrix)

        print(
            "=========================================="
        )

        self.ui.send_message(
            "calibration_complete",
            message={
                "success": True,
            },
        )

        return True

    # ========================================================
    # PIXEL -> REAL POSITION
    # ========================================================

    def calculate_real_position(
        self,
        pixel_x,
        pixel_y,
    ):

        with self.state_lock:
            matrix = self.transform_matrix

        if matrix is None:
            return None

        point = np.array(
            [
                [
                    [
                        float(pixel_x),
                        float(pixel_y),
                    ]
                ]
            ],
            dtype=np.float32,
        )

        transformed = cv2.perspectiveTransform(
            point,
            matrix,
        )

        x = float(
            transformed[0][0][0]
        )

        y = float(
            transformed[0][0][1]
        )

        return (
            round(x, 3),
            round(y, 3),
        )

    # ========================================================
    # CAPTURE
    # ========================================================

    def handle_capture(
        self,
        sid,
        data,
    ):

        print(
            "------------------------------------------"
        )

        print(
            "[CALIBRATION] Capture requested"
        )

        print(
            "------------------------------------------"
        )

        with self.state_lock:

            marker = (
                self.latest_marker_center
            )

            step = self.current_step

        if marker is None:

            expected_id = (
                self.calibration_marker_ids[
                    step
                ]
            )

            print(
                "[CALIBRATION ERROR] "
                f"D{expected_id} is not "
                "currently visible."
            )

            self.ui.send_message(
                "calibration_error",
                message={
                    "error":
                        f"D{expected_id} is not visible. "
                        f"Place D{expected_id} at the "
                        f"required calibration position."
                },
            )

            return

        pixel_x, pixel_y = marker

        with self.state_lock:

            self.calibration_points[
                step
            ] = (
                float(pixel_x),
                float(pixel_y),
            )

        marker_id = (
            self.calibration_marker_ids[
                step
            ]
        )

        physical_position = (
            self.calibration_positions[
                step
            ]
        )

        print(
            f"[CALIBRATION] "
            f"Step {step} captured"
        )

        print(
            f"Marker = D{marker_id}"
        )

        print(
            f"Pixel position = "
            f"({pixel_x}, {pixel_y})"
        )

        print(
            f"Physical position = "
            f"({physical_position[0]}, "
            f"{physical_position[1]}) feet"
        )

        # ----------------------------------------------------
        # More calibration steps remain
        # ----------------------------------------------------

        if step < 4:

            with self.state_lock:
                self.current_step += 1

            self.ui.send_message(
                "calibration_captured",
                message={
                    "step": step,

                    "marker_id":
                        marker_id,

                    "pixel_x":
                        pixel_x,

                    "pixel_y":
                        pixel_y,

                    "next_step":
                        step + 1,

                    "next_marker_id":
                        self.calibration_marker_ids[
                            step + 1
                        ],
                },
            )

            self.send_status()

            return

        # ----------------------------------------------------
        # Last calibration point
        # ----------------------------------------------------

        self.ui.send_message(
            "calibration_captured",
            message={
                "step": step,

                "marker_id":
                    marker_id,

                "pixel_x":
                    pixel_x,

                "pixel_y":
                    pixel_y,

                "next_step":
                    None,

                "next_marker_id":
                    None,
            },
        )

        self.calculate_transform()

        self.send_status()

    # ========================================================
    # RESET
    # ========================================================

    def handle_reset(
        self,
        sid,
        data,
    ):

        with self.state_lock:

            self.calibration_points = {}

            self.current_step = 1

            self.transform_matrix = None

            self.latest_marker_center = None

        print(
            "=========================================="
        )

        print(
            "[CALIBRATION] RESET"
        )

        print(
            "=========================================="
        )

        self.ui.send_message(
            "calibration_reset",
            message={
                "success": True,
            },
        )

        self.send_status()

    # ========================================================
    # WEBUI REGISTRATION
    # ========================================================

    def register_handlers(self):

        self.ui.on_message(
            "capture_calibration",
            self.handle_capture,
        )

        self.ui.on_message(
            "reset_calibration",
            self.handle_reset,
        )