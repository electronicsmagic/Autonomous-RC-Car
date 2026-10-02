# ============================================================
# UNO Q ARUCO ROBOTIC CAR - CAR CONTROL MODULE
# ============================================================

import urllib.request


class CarController:
    """UNO Q -> ESP32 HTTP bridge for the RC car."""

    VALID_COMMANDS = {
        "forward",
        "backward",
        "left",
        "right",
        "stop",
    }

    def __init__(self, ui, esp32_ip, timeout_sec=2.0):
        self.ui = ui
        self.esp32_ip = esp32_ip
        self.timeout_sec = float(timeout_sec)
        self.current_speed = 50

    # ========================================================
    # HTTP helper
    # ========================================================

    def _get(self, url):
        request = urllib.request.Request(
            url,
            method="GET",
        )

        with urllib.request.urlopen(
            request,
            timeout=self.timeout_sec,
        ) as response:
            return response.read().decode(
                "utf-8",
                errors="replace",
            )

    # ========================================================
    # Car command
    # ========================================================

    def send_command(self, command):
        command = str(command).lower().strip()

        if command not in self.VALID_COMMANDS:
            print(f"[CAR ERROR] Invalid command: {command}")
            return {
                "success": False,
                "command": command,
                "error": "Invalid command",
            }

        url = f"http://{self.esp32_ip}/{command}"

        try:
            print(
                f"[CAR] Sending {command.upper()} -> {url}"
            )

            response_text = self._get(url)

            print(
                f"[CAR] ESP32 response: {response_text}"
            )

            return {
                "success": True,
                "command": command.upper(),
                "response": response_text,
            }

        except Exception as exc:
            print(
                f"[CAR ERROR] ESP32 request failed: {exc}"
            )
            return {
                "success": False,
                "command": command.upper(),
                "error": str(exc),
            }

    def handle_command(self, sid, data):
        command = ""

        if isinstance(data, dict):
            command = data.get("command", "")

        result = self.send_command(command)

        self.ui.send_message(
            "car_command_result",
            message=result,
        )

    # ========================================================
    # Speed
    # ========================================================

    def send_speed(self, speed):
        try:
            speed = int(float(speed))
        except (TypeError, ValueError):
            print(
                f"[CAR SPEED ERROR] Invalid speed: {speed}"
            )
            return {
                "success": False,
                "speed": speed,
                "error": "Invalid speed value",
            }

        speed = max(0, min(100, speed))
        self.current_speed = speed

        url = (
            f"http://{self.esp32_ip}"
            f"/speed?value={speed}"
        )

        try:
            print(
                f"[CAR] Setting speed: {speed}% -> {url}"
            )

            response_text = self._get(url)

            print(
                f"[CAR] ESP32 speed response: "
                f"{response_text}"
            )

            return {
                "success": True,
                "speed": speed,
                "response": response_text,
            }

        except Exception as exc:
            print(
                f"[CAR SPEED ERROR] "
                f"ESP32 request failed: {exc}"
            )
            return {
                "success": False,
                "speed": speed,
                "error": str(exc),
            }

    def handle_speed(self, sid, data):
        speed = None

        if isinstance(data, dict):
            speed = data.get("speed")

        result = self.send_speed(speed)

        self.ui.send_message(
            "car_speed_result",
            message=result,
        )

    # ========================================================
    # WebUI Registration
    # ========================================================

    def register_handlers(self):
        self.ui.on_message(
            "car_control",
            self.handle_command,
        )
        self.ui.on_message(
            "car_speed",
            self.handle_speed,
        )