# ============================================================
# UNO Q ARUCO ROBOTIC CAR
# NAVIGATION CONTROLLER
#
# STEP 3C
# AUTOMATIC TURNING + FORWARD MOVEMENT
#
# IMPORTANT:
#   - Automatic turn speed    = 15%
#   - Automatic forward speed = 15%
#   - LEFT / RIGHT / FORWARD / STOP
#   - BACKWARD is NEVER used
#
# STEP 3C PURPOSE:
#
#   1. Calculate direction to target.
#   2. Turn toward target using short pulses.
#   3. When heading is correct, move FORWARD.
#   4. Continuously monitor heading.
#   5. If heading becomes incorrect:
#          STOP
#          correct heading
#          FORWARD again
#
# NOTE:
#   This version intentionally does not finish the complete
#   target-arrival behavior of Step 3D.
# ============================================================

import math
import threading
import time


class NavigationController:
    """
    Step 3C automatic navigation controller.

    Receives from ArUcoTracker:

        current X
        current Y
        current heading

    Receives from webpage:

        target X
        target Y

    Calculates:

        distance
        required heading
        heading error

    Controls:

        LEFT
        RIGHT
        FORWARD
        STOP

    BACKWARD is explicitly forbidden.
    """

    # ========================================================
    # AUTOMATIC SPEED
    # ========================================================

    AUTO_TURN_SPEED = 10 # old 15

    AUTO_FORWARD_SPEED = 10  # old 15

    # ========================================================
    # HEADING TOLERANCE
    # ========================================================

    HEADING_TOLERANCE_DEG = 5.0

    # ========================================================
    # MINIMUM DISTANCE
    # ========================================================

    MIN_DISTANCE_FT = 0.05

    # ========================================================
    # TURN PULSE SETTINGS
    # ========================================================

    TURN_PULSE_LONG_SEC = 0.30

    TURN_PULSE_MEDIUM_SEC = 0.18

    TURN_PULSE_SHORT_SEC = 0.10

    TURN_PULSE_FINE_SEC = 0.05

    # ========================================================
    # TURN SETTLE TIME
    # ========================================================

    TURN_SETTLE_TIME_SEC = 0.08

    # ========================================================
    # FORWARD CONTROL
    # ========================================================

    # Small delay between forward control updates.
    #
    # The actual movement is continuous because the ESP32
    # remains in FORWARD state. We simply reevaluate the pose.
    # ========================================================

    FORWARD_UPDATE_INTERVAL_SEC = 0.05

    # ========================================================
    # INITIALIZATION
    # ========================================================

    def __init__(
        self,
        ui,
        car,
    ):

        self.ui = ui

        self.car = car

        # ----------------------------------------------------
        # State locks
        # ----------------------------------------------------

        self.state_lock = threading.Lock()

        self.pulse_lock = threading.Lock()

        # ----------------------------------------------------
        # Navigation state
        # ----------------------------------------------------

        self.active = False

        self.target_x = None

        self.target_y = None

        # ----------------------------------------------------
        # Current pose
        # ----------------------------------------------------

        self.current_x = None

        self.current_y = None

        self.current_heading = None

        # ----------------------------------------------------
        # Calculated values
        # ----------------------------------------------------

        self.distance = None

        self.required_heading = None

        self.heading_error = None

        self.command = "STOP"

        # ----------------------------------------------------
        # Last command sent
        # ----------------------------------------------------

        self.last_sent_command = None

        # ----------------------------------------------------
        # Last speed
        # ----------------------------------------------------

        self.last_sent_speed = None

        # ----------------------------------------------------
        # Turning pulse state
        # ----------------------------------------------------

        self.pulse_thread = None

        self.turn_in_progress = False

        # ----------------------------------------------------
        # Prevent multiple forward workers.
        # ----------------------------------------------------

        self.forward_worker = None

        self.forward_worker_running = False

    # ========================================================
    # NORMALIZE HEADING
    # ========================================================

    @staticmethod
    def normalize_heading(
        heading,
    ):
        """
        Normalize heading:

            0 <= heading < 360
        """

        if heading is None:

            return None

        try:

            heading = float(
                heading
            )

        except (
            TypeError,
            ValueError,
        ):

            return None

        heading %= 360.0

        if heading < 0:

            heading += 360.0

        return heading

    # ========================================================
    # HEADING ERROR
    # ========================================================

    @staticmethod
    def calculate_heading_error(
        current_heading,
        required_heading,
    ):
        """
        Calculate shortest signed heading error.

        Positive:
            RIGHT

        Negative:
            LEFT
        """

        if (
            current_heading is None
            or required_heading is None
        ):

            return None

        current_heading = (
            NavigationController
            .normalize_heading(
                current_heading
            )
        )

        required_heading = (
            NavigationController
            .normalize_heading(
                required_heading
            )
        )

        if (
            current_heading is None
            or required_heading is None
        ):

            return None

        error = (
            required_heading
            - current_heading
        )

        while error > 180.0:

            error -= 360.0

        while error < -180.0:

            error += 360.0

        return error

    # ========================================================
    # CALCULATE NAVIGATION
    # ========================================================

    @staticmethod
    def calculate_navigation(
        current_x,
        current_y,
        current_heading,
        target_x,
        target_y,
    ):
        """
        Calculate:

            distance
            required heading
            heading error
        """

        if (
            current_x is None
            or current_y is None
            or current_heading is None
        ):

            return None

        if (
            target_x is None
            or target_y is None
        ):

            return None

        try:

            current_x = float(
                current_x
            )

            current_y = float(
                current_y
            )

            current_heading = float(
                current_heading
            )

            target_x = float(
                target_x
            )

            target_y = float(
                target_y
            )

        except (
            TypeError,
            ValueError,
        ):

            return None

        # ----------------------------------------------------
        # Vector to target
        # ----------------------------------------------------

        dx = (
            target_x
            - current_x
        )

        dy = (
            target_y
            - current_y
        )

        # ----------------------------------------------------
        # Distance
        # ----------------------------------------------------

        distance = math.sqrt(
            (dx * dx)
            + (dy * dy)
        )

        # ----------------------------------------------------
        # Required heading
        #
        # 0°   = +X
        # 90°  = +Y
        # 180° = -X
        # 270° = -Y
        # ----------------------------------------------------

        if distance < 0.000001:

            required_heading = (
                NavigationController
                .normalize_heading(
                    current_heading
                )
            )

        else:

            required_heading = (
                math.degrees(
                    math.atan2(
                        dy,
                        dx,
                    )
                )
            )

            required_heading = (
                NavigationController
                .normalize_heading(
                    required_heading
                )
            )

        # ----------------------------------------------------
        # Heading error
        # ----------------------------------------------------

        heading_error = (
            NavigationController
            .calculate_heading_error(
                current_heading,
                required_heading,
            )
        )

        return {
            "distance":
                distance,

            "required_heading":
                required_heading,

            "heading_error":
                heading_error,
        }

    # ========================================================
    # SET TARGET
    # ========================================================

    def set_target(
        self,
        target_x,
        target_y,
    ):
        """
        Start automatic navigation.
        """

        try:

            target_x = float(
                target_x
            )

            target_y = float(
                target_y
            )

        except (
            TypeError,
            ValueError,
        ):

            print(
                "[NAV] Invalid target: "
                f"X={target_x} "
                f"Y={target_y}"
            )

            self._send_error(
                "Invalid navigation target."
            )

            return False

        # ----------------------------------------------------
        # Stop old activity.
        # ----------------------------------------------------

        self._cancel_turn_pulse()

        self._stop_forward_worker()

        self._send_car_command(
            "STOP"
        )

        # ----------------------------------------------------
        # Store target.
        # ----------------------------------------------------

        with self.state_lock:

            self.target_x = target_x

            self.target_y = target_y

            self.active = True

            self.distance = None

            self.required_heading = None

            self.heading_error = None

            self.command = "STOP"

            self.last_sent_command = None

            self.last_sent_speed = None

        print(
            "=========================================="
        )

        print(
            "[NAVIGATION 3C]"
        )

        print(
            f"Target X = {target_x:.2f} ft"
        )

        print(
            f"Target Y = {target_y:.2f} ft"
        )

        print(
            f"Turn speed = "
            f"{self.AUTO_TURN_SPEED}%"
        )

        print(
            f"Forward speed = "
            f"{self.AUTO_FORWARD_SPEED}%"
        )

        print(
            f"Heading tolerance = "
            f"±{self.HEADING_TOLERANCE_DEG:.1f}°"
        )

        print(
            "BACKWARD = DISABLED"
        )

        print(
            "=========================================="
        )

        # ----------------------------------------------------
        # Set automatic speed.
        # ----------------------------------------------------

        self._set_speed(
            self.AUTO_TURN_SPEED
        )

        # ----------------------------------------------------
        # Evaluate latest pose.
        # ----------------------------------------------------

        with self.state_lock:

            current_x = self.current_x

            current_y = self.current_y

            current_heading = (
                self.current_heading
            )

        self.update(
            current_x,
            current_y,
            current_heading,
        )

        return True

    # ========================================================
    # RESET
    # ========================================================

    def reset(self):
        """
        Stop all automatic navigation.
        """

        print(
            "[NAV] Reset navigation"
        )

        self._stop_forward_worker()

        self._cancel_turn_pulse()

        with self.state_lock:

            self.active = False

            self.target_x = None

            self.target_y = None

            self.distance = None

            self.required_heading = None

            self.heading_error = None

            self.command = "STOP"

            self.last_sent_command = None

            self.last_sent_speed = None

        self._send_car_command(
            "STOP"
        )

        self._send_status()

    # ========================================================
    # UPDATE POSE
    # ========================================================

    def update(
        self,
        current_x,
        current_y,
        current_heading,
    ):
        """
        Called by ArUcoTracker for every valid D17 pose.
        """

        # ----------------------------------------------------
        # Store pose.
        # ----------------------------------------------------

        with self.state_lock:

            self.current_x = current_x

            self.current_y = current_y

            self.current_heading = (
                current_heading
            )

            active = self.active

            target_x = self.target_x

            target_y = self.target_y

        # ----------------------------------------------------
        # No autonomous target.
        # ----------------------------------------------------

        if not active:

            return

        # ----------------------------------------------------
        # Calculate navigation.
        # ----------------------------------------------------

        result = (
            self.calculate_navigation(
                current_x,
                current_y,
                current_heading,
                target_x,
                target_y,
            )
        )

        # ----------------------------------------------------
        # Invalid pose.
        # ----------------------------------------------------

        if result is None:

            with self.state_lock:

                self.distance = None

                self.required_heading = None

                self.heading_error = None

                self.command = "STOP"

            self._stop_forward_worker()

            self._cancel_turn_pulse()

            self._send_car_command(
                "STOP"
            )

            self._send_status()

            return

        # ----------------------------------------------------
        # Store calculated values.
        # ----------------------------------------------------

        distance = result[
            "distance"
        ]

        required_heading = result[
            "required_heading"
        ]

        heading_error = result[
            "heading_error"
        ]

        with self.state_lock:

            self.distance = distance

            self.required_heading = (
                required_heading
            )

            self.heading_error = (
                heading_error
            )

        # ====================================================
        # DECIDE WHETHER TO TURN OR DRIVE
        # ====================================================

        if (
            distance
            <= self.MIN_DISTANCE_FT
        ):

            # ------------------------------------------------
            # Step 3C does not yet implement final
            # target-stop behavior, but physically we MUST
            # stop if we are already at the target.
            # ------------------------------------------------

            print(
                "[NAV 3C] "
                "Target position reached. STOP."
            )

            self._stop_forward_worker()

            self._cancel_turn_pulse()

            with self.state_lock:

                self.command = "STOP"

                self.active = False

            self._send_car_command(
                "STOP"
            )

            self._send_status()

            return

        # ----------------------------------------------------
        # Missing heading.
        # ----------------------------------------------------

        if (
            heading_error is None
        ):

            self._stop_forward_worker()

            self._cancel_turn_pulse()

            with self.state_lock:

                self.command = "STOP"

            self._send_car_command(
                "STOP"
            )

            self._send_status()

            return

        # ----------------------------------------------------
        # Need heading correction.
        # ----------------------------------------------------

        if (
            abs(
                heading_error
            )
            > self.HEADING_TOLERANCE_DEG
        ):

            # -----------------------------------------------
            # Stop forward motion before turning.
            # -----------------------------------------------

            self._stop_forward_worker()

            self._send_car_command(
                "STOP"
            )

            # -----------------------------------------------
            # Positive error -> RIGHT
            # Negative error -> LEFT
            # -----------------------------------------------

            if heading_error > 0:

                command = "RIGHT" # old "RIGHT" 

            else:

                command = "LEFT" # old "LEFT"

            with self.state_lock:

                self.command = command

            self._start_turn_pulse(
                command,
                heading_error,
            )

        # ----------------------------------------------------
        # Heading good -> FORWARD
        # ----------------------------------------------------

        else:

            # -----------------------------------------------
            # Stop any remaining turn pulse.
            # -----------------------------------------------

            self._cancel_turn_pulse()

            with self.state_lock:

                self.command = "FORWARD"

            self._start_forward_motion()

        # ----------------------------------------------------
        # Send webpage status.
        # ----------------------------------------------------

        self._send_status()

    # ========================================================
    # TURN PULSE DURATION
    # ========================================================

    def _get_pulse_duration(
        self,
        heading_error,
    ):
        """
        Shorter pulse as heading becomes more accurate.
        """

        if heading_error is None:

            return self.TURN_PULSE_FINE_SEC

        error = abs(
            float(
                heading_error
            )
        )

        if error > 30.0:

            return (
                self.TURN_PULSE_LONG_SEC
            )

        if error > 15.0:

            return (
                self.TURN_PULSE_MEDIUM_SEC
            )

        if error > 8.0:

            return (
                self.TURN_PULSE_SHORT_SEC
            )

        return (
            self.TURN_PULSE_FINE_SEC
        )

    # ========================================================
    # START TURN PULSE
    # ========================================================

    def _start_turn_pulse(
        self,
        direction,
        heading_error,
    ):
        """
        Start one controlled LEFT/RIGHT pulse.
        """

        direction = (
            str(direction)
            .upper()
            .strip()
        )

        if direction not in {
            "LEFT",
            "RIGHT",
        }:

            return

        with self.pulse_lock:

            if self.turn_in_progress:

                return

            self.turn_in_progress = True

        duration = (
            self._get_pulse_duration(
                heading_error
            )
        )

        print(
            "[NAV 3C] "
            f"{direction} pulse "
            f"{duration:.2f}s "
            f"error={heading_error:.1f}°"
        )

        self.pulse_thread = threading.Thread(
            target=self._run_turn_pulse,
            args=(
                direction,
                duration,
            ),
            daemon=True,
            name="NavigationTurnPulse",
        )

        self.pulse_thread.start()

    # ========================================================
    # RUN TURN PULSE
    # ========================================================

    def _run_turn_pulse(
        self,
        direction,
        duration,
    ):
        """
        Execute:

            turn
            stop
            settle

        The tracker will then reevaluate the heading.
        """

        try:

            with self.state_lock:

                active = self.active

            if not active:

                return

            # ------------------------------------------------
            # Set turning speed.
            # ------------------------------------------------

            self._set_speed(
                self.AUTO_TURN_SPEED
            )

            # ------------------------------------------------
            # Turn.
            # ------------------------------------------------

            self._send_car_command(
                direction
            )

            end_time = (
                time.monotonic()
                + duration
            )

            while (
                time.monotonic()
                < end_time
            ):

                with self.state_lock:

                    active = self.active

                if not active:

                    break

                time.sleep(
                    0.01
                )

            # ------------------------------------------------
            # Stop after pulse.
            # ------------------------------------------------

            self._send_car_command(
                "STOP"
            )

            # ------------------------------------------------
            # Allow the camera to update.
            # ------------------------------------------------

            time.sleep(
                self.TURN_SETTLE_TIME_SEC
            )

        except Exception as exc:

            print(
                "[NAV ERROR] "
                f"Turn pulse failed: {exc}"
            )

            self._send_car_command(
                "STOP"
            )

        finally:

            with self.pulse_lock:

                self.turn_in_progress = False

    # ========================================================
    # CANCEL TURN PULSE
    # ========================================================

    def _cancel_turn_pulse(self):
        """
        Stop turning.
        """

        with self.pulse_lock:

            self.turn_in_progress = False

        self._send_car_command(
            "STOP"
        )

    # ========================================================
    # START FORWARD MOTION
    # ========================================================

    def _start_forward_motion(self):
        """
        Start continuous FORWARD motion at 15%.

        The camera/tracker remains responsible for continuously
        calling update(), so heading errors can interrupt
        forward motion.
        """

        with self.state_lock:

            active = self.active

        if not active:

            return

        # ----------------------------------------------------
        # Do not start another worker.
        # ----------------------------------------------------

        if (
            self.forward_worker is not None
            and self.forward_worker.is_alive()
        ):

            return

        self.forward_worker_running = True

        self.forward_worker = threading.Thread(
            target=self._forward_worker_loop,
            daemon=True,
            name="NavigationForward",
        )

        self.forward_worker.start()

    # ========================================================
    # FORWARD WORKER
    # ========================================================

    def _forward_worker_loop(self):
        """
        Keep the car moving forward until:

            1. navigation stops
            2. a heading correction is needed
            3. target is reached
        """

        try:

            while self.forward_worker_running:

                with self.state_lock:

                    active = self.active

                    heading_error = (
                        self.heading_error
                    )

                    distance = (
                        self.distance
                    )

                if not active:

                    break

                # ------------------------------------------------
                # Target reached.
                # ------------------------------------------------

                if (
                    distance is not None
                    and
                    distance
                    <= self.MIN_DISTANCE_FT
                ):

                    break

                # ------------------------------------------------
                # Heading no longer acceptable.
                #
                # The regular update() will handle correction.
                # ------------------------------------------------

                if (
                    heading_error is None
                    or
                    abs(
                        heading_error
                    )
                    > self.HEADING_TOLERANCE_DEG
                ):

                    break

                # ------------------------------------------------
                # Make sure automatic speed is 15%.
                # ------------------------------------------------

                self._set_speed(
                    self.AUTO_FORWARD_SPEED
                )

                # ------------------------------------------------
                # Send FORWARD if required.
                # ------------------------------------------------

                self._send_car_command(
                    "FORWARD"
                )

                time.sleep(
                    self.FORWARD_UPDATE_INTERVAL_SEC
                )

        except Exception as exc:

            print(
                "[NAV ERROR] "
                f"Forward worker failed: {exc}"
            )

        finally:

            self._send_car_command(
                "STOP"
            )

            self.forward_worker_running = False

    # ========================================================
    # STOP FORWARD WORKER
    # ========================================================

    def _stop_forward_worker(self):
        """
        Stop forward motion worker.
        """

        self.forward_worker_running = False

        self._send_car_command(
            "STOP"
        )

    # ========================================================
    # SET SPEED
    # ========================================================

    def _set_speed(
        self,
        speed,
    ):
        """
        Set ESP32 speed.

        For Step 3C automatic operation:

            speed = 15%
        """

        try:

            speed = int(
                float(speed)
            )

        except (
            TypeError,
            ValueError,
        ):

            print(
                "[NAV ERROR] "
                f"Invalid speed: {speed}"
            )

            return False

        speed = max(
            0,
            min(
                100,
                speed,
            ),
        )

        if (
            speed
            == self.last_sent_speed
        ):

            return True

        try:

            print(
                "[NAV 3C] "
                f"Setting speed = "
                f"{speed}%"
            )

            result = (
                self.car.send_speed(
                    speed
                )
            )

            if result.get(
                "success",
                False,
            ):

                self.last_sent_speed = (
                    speed
                )

                return True

            print(
                "[NAV ERROR] "
                f"Speed command failed: "
                f"{result}"
            )

            return False

        except Exception as exc:

            print(
                "[NAV ERROR] "
                f"Speed request failed: "
                f"{exc}"
            )

            return False

    # ========================================================
    # SEND CAR COMMAND
    # ========================================================

    def _send_car_command(
        self,
        command,
    ):
        """
        Safety-controlled automatic command.

        Allowed:

            LEFT
            RIGHT
            FORWARD
            STOP

        Never allowed:

            BACKWARD
        """

        command = (
            str(command)
            .upper()
            .strip()
        )

        # ----------------------------------------------------
        # HARD SAFETY LIMIT
        # ----------------------------------------------------

        allowed_commands = {
            "LEFT",
            "RIGHT",
            "FORWARD",
            "STOP",
        }

        if command not in allowed_commands:

            print(
                "[NAV SAFETY] "
                f"BLOCKED command: "
                f"{command}"
            )

            command = "STOP"

        # ----------------------------------------------------
        # Suppress duplicates.
        # ----------------------------------------------------

        if (
            command
            == self.last_sent_command
        ):

            return

        self.last_sent_command = (
            command
        )

        print(
            "[NAV 3C] CAR COMMAND:",
            command,
        )

        try:

            result = (
                self.car.send_command(
                    command.lower()
                )
            )

            print(
                "[NAV 3C] "
                "Car command result:",
                result,
            )

        except Exception as exc:

            print(
                "[NAV ERROR] "
                f"Car command failed: "
                f"{exc}"
            )

    # ========================================================
    # SEND NAVIGATION STATUS
    # ========================================================

    def _send_status(self):
        """
        Send navigation state to webpage.
        """

        with self.state_lock:

            active = self.active

            current_x = self.current_x

            current_y = self.current_y

            current_heading = (
                self.current_heading
            )

            target_x = self.target_x

            target_y = self.target_y

            distance = self.distance

            required_heading = (
                self.required_heading
            )

            heading_error = (
                self.heading_error
            )

            command = self.command

        try:

            self.ui.send_message(
                "navigation_status",
                message={
                    "active":
                        active,

                    "current_x":
                        current_x,

                    "current_y":
                        current_y,

                    "current_heading":
                        current_heading,

                    "target_x":
                        target_x,

                    "target_y":
                        target_y,

                    "distance":
                        distance,

                    "required_heading":
                        required_heading,

                    "heading_error":
                        heading_error,

                    "command":
                        command,

                    "auto_drive":
                        active,

                    "mode":
                        "STEP_3C_TURN_AND_FORWARD",

                    "auto_turn_speed":
                        self.AUTO_TURN_SPEED,

                    "auto_forward_speed":
                        self.AUTO_FORWARD_SPEED,
                },
            )

        except Exception as exc:

            print(
                "[NAV ERROR] "
                f"Could not send navigation "
                f"status: {exc}"
            )

    # ========================================================
    # SEND ERROR
    # ========================================================

    def _send_error(
        self,
        error,
    ):

        try:

            self.ui.send_message(
                "navigation_error",
                message={
                    "error":
                        str(error)
                },
            )

        except Exception as exc:

            print(
                "[NAV ERROR] "
                f"Could not send error: {exc}"
            )

    # ========================================================
    # REGISTER WEBUI HANDLERS
    # ========================================================

    def register_handlers(self):
        """
        Register WebUI callbacks.

        IMPORTANT:

            WebUI.on_message(
                message_name,
                callback
            )
        """

        # ----------------------------------------------------
        # Navigation target
        # ----------------------------------------------------

        def handle_navigation_target(
            sid,
            data,
        ):

            print(
                "[NAV WEB] navigation_target:",
                data,
            )

            if not isinstance(
                data,
                dict,
            ):

                self._send_error(
                    "Invalid navigation target."
                )

                return

            target_x = data.get(
                "target_x"
            )

            target_y = data.get(
                "target_y"
            )

            self.set_target(
                target_x,
                target_y,
            )

        self.ui.on_message(
            "navigation_target",
            handle_navigation_target,
        )

        # ----------------------------------------------------
        # Navigation reset
        # ----------------------------------------------------

        def handle_navigation_reset(
            sid,
            data,
        ):

            print(
                "[NAV WEB] navigation_reset"
            )

            self.reset()

        self.ui.on_message(
            "navigation_reset",
            handle_navigation_reset,
        )

        print(
            "[NAV] WebUI handlers registered."
        )

    # ========================================================
    # PUBLIC STATUS
    # ========================================================

    def send_status(self):

        self._send_status()