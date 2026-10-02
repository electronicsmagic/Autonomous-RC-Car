# ============================================================
# UNO Q ARUCO ROBOTIC CAR
# ARUCO TRACKER MODULE
#
# STEP 3B CONNECTION
#
# D17 tracking provides:
#     X
#     Y
#     Heading
#
# to:
#
#     1. WebUI
#     2. NavigationController
#
# IMPORTANT:
#     NavigationController is responsible for
#     automatic LEFT / RIGHT / STOP commands.
#
#     ArucoTracker itself does NOT command the car.
# ============================================================

import base64
import math
import threading
import time

import cv2


class ArucoTracker:
    """
    Camera capture, ArUco detection, calibration,
    robotic-car position and orientation tracking,
    WebUI video streaming, and navigation input.

    Heading convention:

        0 degrees   = +X direction
        90 degrees  = +Y direction
        180 degrees = -X direction
        270 degrees = -Y direction

    D17 orientation:

        The TOP EDGE of D17 is treated as the
        FRONT direction of the robotic car.

    ArUco corner order:

        corner[0] = top-left
        corner[1] = top-right
        corner[2] = bottom-right
        corner[3] = bottom-left

    Therefore:

        front direction =
        midpoint(top-left, top-right)
        minus marker center
    """

    def __init__(
        self,
        ui,
        calibration,
        camera,
        navigation,
        frame_width,
        frame_height,
        target_marker_id,
        jpeg_quality=40,
        stream_delay_sec=0.04,
    ):

        self.ui = ui

        self.calibration = calibration

        self.camera = camera

        # ====================================================
        # STEP 3B NAVIGATION
        # ====================================================

        self.navigation = navigation

        self.frame_width = int(
            frame_width
        )

        self.frame_height = int(
            frame_height
        )

        self.target_marker_id = int(
            target_marker_id
        )

        self.jpeg_quality = int(
            jpeg_quality
        )

        self.stream_delay_sec = float(
            stream_delay_sec
        )

        self.thread = None

        self.running = False

        # ----------------------------------------------------
        # Last known heading
        # ----------------------------------------------------

        self.latest_heading_deg = None

    # ========================================================
    # MARKER CENTER
    # ========================================================

    def _get_marker_center(
        self,
        marker_corners,
    ):

        c = marker_corners[0]

        pixel_x = int(
            (
                c[0][0]
                + c[1][0]
                + c[2][0]
                + c[3][0]
            ) / 4
        )

        pixel_y = int(
            (
                c[0][1]
                + c[1][1]
                + c[2][1]
                + c[3][1]
            ) / 4
        )

        return pixel_x, pixel_y

    # ========================================================
    # MARKER FRONT POINT
    # ========================================================

    def _get_marker_front_point(
        self,
        marker_corners,
    ):
        """
        Determine the pixel point representing
        the FRONT of D17.

        The TOP EDGE is the car front.

            corner 0 -------- corner 1
               |                |
               |      D17       |
               |                |
            corner 3 -------- corner 2
        """

        c = marker_corners[0]

        front_x = (
            float(c[0][0])
            + float(c[1][0])
        ) / 2.0

        front_y = (
            float(c[0][1])
            + float(c[1][1])
        ) / 2.0

        return front_x, front_y

    # ========================================================
    # CALCULATE HEADING
    # ========================================================

    def _calculate_heading(
        self,
        marker_corners,
    ):
        """
        Calculate car heading in real-world coordinates.

        Heading convention:

            0°   = +X
            90°  = +Y
            180° = -X
            270° = -Y

        Returned range:

            0 <= heading < 360
        """

        center_x, center_y = (
            self._get_marker_center(
                marker_corners
            )
        )

        front_x, front_y = (
            self._get_marker_front_point(
                marker_corners
            )
        )

        center_real = (
            self.calibration
            .calculate_real_position(
                center_x,
                center_y,
            )
        )

        front_real = (
            self.calibration
            .calculate_real_position(
                front_x,
                front_y,
            )
        )

        if (
            center_real is None
            or front_real is None
        ):

            return None

        center_world_x, center_world_y = (
            center_real
        )

        front_world_x, front_world_y = (
            front_real
        )

        dx = (
            front_world_x
            - center_world_x
        )

        dy = (
            front_world_y
            - center_world_y
        )

        magnitude = math.sqrt(
            (dx * dx)
            + (dy * dy)
        )

        if magnitude < 0.0001:

            return None

        # ----------------------------------------------------
        # Calculate heading
        # ----------------------------------------------------

        heading_deg = math.degrees(
            math.atan2(
                dy,
                dx,
            )
        )

        # ----------------------------------------------------
        # Normalize to 0-360
        # ----------------------------------------------------

        if heading_deg < 0:

            heading_deg += 360.0

        heading_deg %= 360.0

        return round(
            heading_deg,
            1,
        )

    # ========================================================
    # DRAW POSITION
    # ========================================================

    def _draw_position(
        self,
        frame,
        marker_id,
        pixel_x,
        pixel_y,
        real_position,
        heading_deg,
    ):

        # ----------------------------------------------------
        # Marker center
        # ----------------------------------------------------

        cv2.circle(
            frame,
            (
                pixel_x,
                pixel_y,
            ),
            6,
            (0, 255, 255),
            -1,
        )

        # ----------------------------------------------------
        # Marker ID
        # ----------------------------------------------------

        cv2.putText(
            frame,
            f"D{marker_id}",
            (
                pixel_x + 10,
                pixel_y - 15,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        # ----------------------------------------------------
        # X / Y
        # ----------------------------------------------------

        if real_position is not None:

            room_x, room_y = (
                real_position
            )

            text = (
                f"X={room_x:.2f}ft "
                f"Y={room_y:.2f}ft"
            )

            cv2.putText(
                frame,
                text,
                (
                    max(
                        5,
                        pixel_x - 100
                    ),
                    pixel_y + 35,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )

        # ----------------------------------------------------
        # Heading
        # ----------------------------------------------------

        if heading_deg is not None:

            heading_text = (
                f"Heading={heading_deg:.1f} deg"
            )

            cv2.putText(
                frame,
                heading_text,
                (
                    max(
                        5,
                        pixel_x - 100
                    ),
                    pixel_y + 60,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

    # ========================================================
    # DRAW FRONT ARROW
    # ========================================================

    def _draw_front_arrow(
        self,
        frame,
        marker_corners,
        heading_deg,
    ):
        """
        Draw an arrow from the marker center toward
        the detected FRONT direction.
        """

        c = marker_corners[0]

        # ----------------------------------------------------
        # Center
        # ----------------------------------------------------

        center_x = int(
            (
                c[0][0]
                + c[1][0]
                + c[2][0]
                + c[3][0]
            ) / 4
        )

        center_y = int(
            (
                c[0][1]
                + c[1][1]
                + c[2][1]
                + c[3][1]
            ) / 4
        )

        # ----------------------------------------------------
        # Front point
        # ----------------------------------------------------

        front_x = (
            float(c[0][0])
            + float(c[1][0])
        ) / 2.0

        front_y = (
            float(c[0][1])
            + float(c[1][1])
        ) / 2.0

        # ----------------------------------------------------
        # Direction vector
        # ----------------------------------------------------

        dx = (
            front_x
            - float(center_x)
        )

        dy = (
            front_y
            - float(center_y)
        )

        length = math.sqrt(
            dx * dx
            + dy * dy
        )

        if length < 1.0:

            return

        # ----------------------------------------------------
        # Make arrow longer
        # ----------------------------------------------------

        arrow_length = max(
            45.0,
            length * 3.0,
        )

        scale = (
            arrow_length
            / length
        )

        end_x = int(
            center_x
            + dx * scale
        )

        end_y = int(
            center_y
            + dy * scale
        )

        # ----------------------------------------------------
        # Draw arrow
        # ----------------------------------------------------

        cv2.arrowedLine(
            frame,
            (
                center_x,
                center_y,
            ),
            (
                end_x,
                end_y,
            ),
            (0, 255, 0),
            4,
            tipLength=0.25,
        )

        # ----------------------------------------------------
        # Heading near arrow
        # ----------------------------------------------------

        if heading_deg is not None:

            text_x = end_x + 8

            text_y = end_y

            if text_x < 5:

                text_x = 5

            if text_y < 25:

                text_y = 25

            if text_y > frame.shape[0] - 5:

                text_y = (
                    frame.shape[0] - 5
                )

            cv2.putText(
                frame,
                f"{heading_deg:.1f} deg",
                (
                    text_x,
                    text_y,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

    # ========================================================
    # SEND MARKER POSITION
    # ========================================================

    def _send_marker_position(
        self,
        marker_id,
        pixel_x,
        pixel_y,
        real_position,
        heading_deg,
    ):

        if real_position is not None:

            room_x, room_y = (
                real_position
            )

            print(
                f"[D{marker_id}] "
                f"Pixel=({pixel_x},{pixel_y}) "
                f"Position="
                f"({room_x}ft,{room_y}ft) "
                f"Heading={heading_deg} deg"
            )

        else:

            room_x = None

            room_y = None

            print(
                f"[D{marker_id}] "
                f"Pixel=({pixel_x},{pixel_y}) "
                f"Position=NOT CALIBRATED "
                f"Heading={heading_deg} deg"
            )

        # ====================================================
        # WEBPAGE
        # ====================================================

        self.ui.send_message(
            "marker_position",
            message={
                "id":
                    marker_id,

                "pixel_x":
                    pixel_x,

                "pixel_y":
                    pixel_y,

                "room_x":
                    room_x,

                "room_y":
                    room_y,

                "heading_deg":
                    heading_deg,
            },
        )

        # ====================================================
        # STEP 3B NAVIGATION
        #
        # Pass exactly the same position and heading
        # used by the webpage.
        # ====================================================

        if self.navigation is not None:

            try:

                self.navigation.update(
                    room_x,
                    room_y,
                    heading_deg,
                )

            except Exception as exc:

                print(
                    "[NAVIGATION ERROR] "
                    f"{exc}"
                )

    # ========================================================
    # CAMERA / TRACKER
    # ========================================================

    def run(self):

        print(
            "[CAM] Starting camera initialization..."
        )

        print(
            f"[CAM] Opening camera: "
            f"{self.camera.camera_device}"
        )

        # ----------------------------------------------------
        # Open V4L2 camera
        # ----------------------------------------------------

        cap = cv2.VideoCapture(
            self.camera.camera_device,
            cv2.CAP_V4L2,
        )

        if not cap.isOpened():

            print(
                "[CAM ERROR] Could not open "
                f"{self.camera.camera_device}"
            )

            return

        # ----------------------------------------------------
        # MJPEG
        # ----------------------------------------------------

        cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(
                *"MJPG"
            ),
        )

        # ----------------------------------------------------
        # Resolution
        # ----------------------------------------------------

        cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.frame_width,
        )

        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.frame_height,
        )

        # ----------------------------------------------------
        # Reduce buffering
        # ----------------------------------------------------

        cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1,
        )

        # ----------------------------------------------------
        # Test frame
        # ----------------------------------------------------

        ok, frame = cap.read()

        if (
            not ok
            or frame is None
        ):

            print(
                "[CAM ERROR] Camera opened but "
                "could not read frame."
            )

            cap.release()

            return

        # ----------------------------------------------------
        # Actual resolution
        # ----------------------------------------------------

        actual_width = int(
            cap.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )

        actual_height = int(
            cap.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )

        print(
            "=========================================="
        )

        print(
            "[CAM SUCCESS] Camera running on "
            f"{self.camera.camera_device}"
        )

        print(
            "[CAM] Requested resolution: "
            f"{self.frame_width}x"
            f"{self.frame_height}"
        )

        print(
            "[CAM] Actual resolution: "
            f"{actual_width}x"
            f"{actual_height}"
        )

        print(
            "[CAM] Format: MJPG"
        )

        print(
            "=========================================="
        )

        # ====================================================
        # CAMERA CONTROLS
        # ====================================================

        if not self.camera.initialize():

            cap.release()

            return

        # ====================================================
        # ARUCO DICTIONARY
        # ====================================================

        dictionary = (
            cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50
            )
        )

        parameters = (
            cv2.aruco.DetectorParameters()
        )

        detector = cv2.aruco.ArucoDetector(
            dictionary,
            parameters,
        )

        print(
            "=========================================="
        )

        print(
            "D0-D3 CALIBRATION + D17 TRACKING"
        )

        print(
            "=========================================="
        )

        print(
            "D0 = (0,0)"
        )

        print(
            "D1 = (AREA_WIDTH,0)"
        )

        print(
            "D2 = (AREA_WIDTH,AREA_HEIGHT)"
        )

        print(
            "D3 = (0,AREA_HEIGHT)"
        )

        print(
            "D17 = MOVING ROBOTIC CAR"
        )

        print(
            "D17 FRONT = TOP EDGE OF MARKER"
        )

        print(
            "Heading:"
        )

        print(
            "  0   deg = +X"
        )

        print(
            "  90  deg = +Y"
        )

        print(
            "  180 deg = -X"
        )

        print(
            "  270 deg = -Y"
        )

        print(
            "STEP 3B:"
        )

        print(
            "Automatic navigation connected"
        )

        print(
            "Automatic turning speed = 15%"
        )

        print(
            "FORWARD = DISABLED"
        )

        print(
            "BACKWARD = DISABLED"
        )

        print(
            "=========================================="
        )

        self.calibration.send_status()

        frame_count = 0

        self.running = True

        while self.running:

            try:

                # =================================================
                # READ FRAME
                # =================================================

                ok, frame = cap.read()

                if (
                    not ok
                    or frame is None
                ):

                    print(
                        "[CAM WARNING] "
                        "Failed to read frame"
                    )

                    time.sleep(
                        0.05
                    )

                    continue

                frame_count += 1

                # =================================================
                # GRAYSCALE
                # =================================================

                gray = cv2.cvtColor(
                    frame,
                    cv2.COLOR_BGR2GRAY,
                )

                # =================================================
                # ARUCO DETECTION
                # =================================================

                corners, ids, _ = (
                    detector.detectMarkers(
                        gray
                    )
                )

                # =================================================
                # GET CURRENT CALIBRATION STATE
                # =================================================

                with self.calibration.state_lock:

                    calibrated = (
                        self.calibration
                        .transform_matrix
                        is not None
                    )

                    current_step = (
                        self.calibration
                        .current_step
                    )

                # =================================================
                # CALIBRATION MODE
                # =================================================

                if not calibrated:

                    expected_marker_id = (
                        self.calibration
                        .calibration_marker_ids
                        .get(
                            current_step
                        )
                    )

                    calibration_marker_found = (
                        False
                    )

                    if ids is not None:

                        ids_flat = (
                            ids.flatten()
                        )

                        cv2.aruco.drawDetectedMarkers(
                            frame,
                            corners,
                            ids,
                        )

                        for i, marker_id in enumerate(
                            ids_flat
                        ):

                            marker_id = int(
                                marker_id
                            )

                            if (
                                marker_id
                                != expected_marker_id
                            ):

                                continue

                            pixel_x, pixel_y = (
                                self._get_marker_center(
                                    corners[i]
                                )
                            )

                            calibration_marker_found = (
                                True
                            )

                            self.calibration.set_latest_marker_center(
                                (
                                    pixel_x,
                                    pixel_y,
                                )
                            )

                            cv2.circle(
                                frame,
                                (
                                    pixel_x,
                                    pixel_y,
                                ),
                                7,
                                (0, 255, 255),
                                -1,
                            )

                            cv2.putText(
                                frame,
                                (
                                    f"D{marker_id} "
                                    f"CALIBRATION"
                                ),
                                (
                                    pixel_x + 10,
                                    pixel_y - 10,
                                ),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 255, 255),
                                2,
                            )

                            physical = (
                                self.calibration
                                .calibration_positions[
                                    current_step
                                ]
                            )

                            cv2.putText(
                                frame,
                                (
                                    f"Step "
                                    f"{current_step}/4 "
                                    f"-> "
                                    f"("
                                    f"{physical[0]:.1f},"
                                    f"{physical[1]:.1f}"
                                    f") ft"
                                ),
                                (
                                    20,
                                    65,
                                ),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.65,
                                (0, 255, 255),
                                2,
                            )

                            break

                    if not calibration_marker_found:

                        self.calibration.clear_latest_marker_center()

                    physical = (
                        self.calibration
                        .calibration_positions[
                            current_step
                        ]
                    )

                    cv2.putText(
                        frame,
                        (
                            f"CALIBRATION "
                            f"STEP {current_step}/4"
                        ),
                        (
                            20,
                            30,
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2,
                    )

                    cv2.putText(
                        frame,
                        (
                            f"PLACE D"
                            f"{expected_marker_id} "
                            f"AT "
                            f"("
                            f"{physical[0]:.1f},"
                            f"{physical[1]:.1f}"
                            f") ft"
                        ),
                        (
                            20,
                            95,
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )

                # =================================================
                # TRACKING MODE
                # =================================================

                else:

                    target_marker_found = (
                        False
                    )

                    if ids is not None:

                        ids_flat = (
                            ids.flatten()
                        )

                        cv2.aruco.drawDetectedMarkers(
                            frame,
                            corners,
                            ids,
                        )

                        for i, marker_id in enumerate(
                            ids_flat
                        ):

                            marker_id = int(
                                marker_id
                            )

                            # -------------------------------------
                            # ONLY D17
                            # -------------------------------------

                            if (
                                marker_id
                                != self.target_marker_id
                            ):

                                continue

                            target_marker_found = (
                                True
                            )

                            # -------------------------------------
                            # D17 CENTER
                            # -------------------------------------

                            pixel_x, pixel_y = (
                                self._get_marker_center(
                                    corners[i]
                                )
                            )

                            # -------------------------------------
                            # PIXEL -> REAL X/Y
                            # -------------------------------------

                            real_position = (
                                self.calibration
                                .calculate_real_position(
                                    pixel_x,
                                    pixel_y,
                                )
                            )

                            # -------------------------------------
                            # HEADING
                            # -------------------------------------

                            heading_deg = (
                                self._calculate_heading(
                                    corners[i]
                                )
                            )

                            self.latest_heading_deg = (
                                heading_deg
                            )

                            # -------------------------------------
                            # DRAW POSITION
                            # -------------------------------------

                            self._draw_position(
                                frame,
                                self.target_marker_id,
                                pixel_x,
                                pixel_y,
                                real_position,
                                heading_deg,
                            )

                            # -------------------------------------
                            # DRAW FRONT ARROW
                            # -------------------------------------

                            self._draw_front_arrow(
                                frame,
                                corners[i],
                                heading_deg,
                            )

                            # -------------------------------------
                            # SEND POSITION TO WEBPAGE
                            # AND NAVIGATION
                            # -------------------------------------

                            self._send_marker_position(
                                self.target_marker_id,
                                pixel_x,
                                pixel_y,
                                real_position,
                                heading_deg,
                            )

                            break

                    # =================================================
                    # D17 NOT FOUND
                    # =================================================

                    if not target_marker_found:

                        # ------------------------------------------------
                        # Tell webpage marker is temporarily unavailable.
                        # ------------------------------------------------

                        self.ui.send_message(
                            "marker_position",
                            message={
                                "id":
                                    self.target_marker_id,

                                "pixel_x":
                                    None,

                                "pixel_y":
                                    None,

                                "room_x":
                                    None,

                                "room_y":
                                    None,

                                "heading_deg":
                                    None,
                            },
                        )

                        # ------------------------------------------------
                        # IMPORTANT:
                        #
                        # Do NOT send STOP to navigation here.
                        #
                        # NavigationController must decide what to
                        # do when tracking is temporarily lost.
                        #
                        # This avoids repeatedly overwriting its
                        # command every frame.
                        # ------------------------------------------------

                    # ------------------------------------------------
                    # CALIBRATED STATUS
                    # ------------------------------------------------

                    cv2.putText(
                        frame,
                        "CALIBRATED - D17 TRACKING",
                        (
                            20,
                            30,
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )

                # =================================================
                # ENCODE FRAME
                # =================================================

                success, buffer = (
                    cv2.imencode(
                        ".jpg",
                        frame,
                        [
                            cv2.IMWRITE_JPEG_QUALITY,
                            self.jpeg_quality,
                        ],
                    )
                )

                if success:

                    image_b64 = (
                        base64.b64encode(
                            buffer
                        ).decode(
                            "utf-8"
                        )
                    )

                    self.ui.send_message(
                        "video_feed",
                        message={
                            "image":
                                "data:image/jpeg;base64,"
                                + image_b64
                        },
                    )

                # =================================================
                # DEBUG
                # =================================================

                if frame_count % 30 == 0:

                    print(
                        f"[STREAMING] "
                        f"Sent frame "
                        f"#{frame_count}"
                    )

                time.sleep(
                    self.stream_delay_sec
                )

            except Exception as exc:

                print(
                    "[ERROR IN TRACKER THREAD] "
                    f"{exc}"
                )

                time.sleep(
                    0.1
                )

        # ========================================================
        # CLEANUP
        # ========================================================

        cap.release()

        self.camera.close_control()

        print(
            "[CAM] Tracker stopped."
        )

    # ========================================================
    # START
    # ========================================================

    def start(self):

        if (
            self.thread is not None
            and self.thread.is_alive()
        ):

            print(
                "[CAM] Tracker already running"
            )

            return

        self.thread = threading.Thread(
            target=self.run,
            daemon=True,
            name="ArUcoTracker",
        )

        self.thread.start()

    # ========================================================
    # STOP
    # ========================================================

    def stop(self):

        self.running = False