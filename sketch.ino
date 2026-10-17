/*
  ============================================================
  ESP32 ROBOTIC CAR
  WITH BLINKERS + SPEED CONTROL
  ============================================================

  Hardware:
      4 x Continuous-Rotation Servos
      4 x Direction LEDs

  ESP32 = Wi-Fi Web Server

  Commands:
      /forward
      /backward
      /left
      /right
      /stop
      /speed?value=0-100

  Speed:
      0   = stop
      100 = maximum
*/


#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>


// ============================================================
// Wi-Fi
// ============================================================

const char* WIFI_SSID = "You WiFi log in name "; // inter your WiFi log in name 

const char* WIFI_PASSWORD = "WiFi Passowrd ";   // inter WiFi password 


// ============================================================
// Web Server
// ============================================================

WebServer server(80);


// ============================================================
// Servo GPIO Pins
// ============================================================

const int SERVO_FRONT_RIGHT = 21;
const int SERVO_BACK_RIGHT  = 4;

const int SERVO_FRONT_LEFT  = 32;
const int SERVO_BACK_LEFT   = 14;


// ============================================================
// LED GPIO Pins
// ============================================================

const int LED_FRONT_RIGHT = 23;
const int LED_BACK_RIGHT  = 19;

const int LED_FRONT_LEFT  = 13;
const int LED_BACK_LEFT   = 15;


// ============================================================
// Servo Objects
// ============================================================

Servo servoFrontRight;
Servo servoBackRight;

Servo servoFrontLeft;
Servo servoBackLeft;


// ============================================================
// Continuous-Rotation Servo Values
// ============================================================

// Maximum forward and backward values.
// These are kept from your original sketch.

const int SERVO_FORWARD  = 180;
const int SERVO_BACKWARD = 0;


// ============================================================
// Individual Servo Stop Values
// ============================================================
//
// These are calibrated from your /calibration page.
//

int SERVO_STOP_FRONT_RIGHT = 90;
int SERVO_STOP_BACK_RIGHT  = 90;

int SERVO_STOP_FRONT_LEFT  = 90;
int SERVO_STOP_BACK_LEFT   = 90;


// ============================================================
// CAR SPEED
// ============================================================
//
// 0   = stop
// 100 = maximum speed
//

int CAR_SPEED = 50;


// ============================================================
// Current Car State
// ============================================================

String currentCommand = "STOP";


// ============================================================
// Blinker Timing
// ============================================================

unsigned long previousBlinkMillis = 0;

const unsigned long blinkInterval = 400;

bool blinkState = LOW;


// ============================================================
// Forward Speed Conversion
// ============================================================

int getForwardServoValue(
    int stopValue
)
{
    return map(
        CAR_SPEED,
        0,
        100,
        stopValue,
        SERVO_FORWARD
    );
}


// ============================================================
// Backward Speed Conversion
// ============================================================

int getBackwardServoValue(
    int stopValue
)
{
    return map(
        CAR_SPEED,
        0,
        100,
        stopValue,
        SERVO_BACKWARD
    );
}


// ============================================================
// Stop All LEDs
// ============================================================

void ledsOff()
{
    digitalWrite(
        LED_FRONT_RIGHT,
        LOW
    );

    digitalWrite(
        LED_BACK_RIGHT,
        LOW
    );

    digitalWrite(
        LED_FRONT_LEFT,
        LOW
    );

    digitalWrite(
        LED_BACK_LEFT,
        LOW
    );
}


// ============================================================
// Attach All Motors
// ============================================================

void attachMotors()
{
    if (!servoFrontRight.attached())
    {
        servoFrontRight.attach(
            SERVO_FRONT_RIGHT,
            500,
            2400
        );
    }

    if (!servoBackRight.attached())
    {
        servoBackRight.attach(
            SERVO_BACK_RIGHT,
            500,
            2400
        );
    }

    if (!servoFrontLeft.attached())
    {
        servoFrontLeft.attach(
            SERVO_FRONT_LEFT,
            500,
            2400
        );
    }

    if (!servoBackLeft.attached())
    {
        servoBackLeft.attach(
            SERVO_BACK_LEFT,
            500,
            2400
        );
    }
}


// ============================================================
// Stop All Motors
// ============================================================
//
// Detach completely.
// This was part of your original working approach.
//

void stopMotors()
{
    if (servoFrontRight.attached())
    {
        servoFrontRight.detach();
    }

    if (servoBackRight.attached())
    {
        servoBackRight.detach();
    }

    if (servoFrontLeft.attached())
    {
        servoFrontLeft.detach();
    }

    if (servoBackLeft.attached())
    {
        servoBackLeft.detach();
    }
}


// ============================================================
// STOP CAR
// ============================================================

void stopCar()
{
    stopMotors();

    ledsOff();

    currentCommand = "STOP";

    blinkState = LOW;

    Serial.println(
        "[CAR] STOP"
    );
}


// ============================================================
// FORWARD
// ============================================================

void moveForward()
{
    attachMotors();

    // Physical FORWARD is currently the same motor pattern
    // as the original RIGHT turn.

    int leftForwardValue =
        getForwardServoValue(
            SERVO_STOP_FRONT_LEFT
        );

    int leftBackForwardValue =
        getForwardServoValue(
            SERVO_STOP_BACK_LEFT
        );

    int rightBackwardValue =
        getBackwardServoValue(
            SERVO_STOP_FRONT_RIGHT
        );

    int rightBackBackwardValue =
        getBackwardServoValue(
            SERVO_STOP_BACK_RIGHT
        );


    servoFrontLeft.write(
        leftForwardValue
    );

    servoBackLeft.write(
        leftBackForwardValue
    );

    servoFrontRight.write(
        rightBackwardValue
    );

    servoBackRight.write(
        rightBackBackwardValue
    );


    ledsOff();


    // Headlights
    digitalWrite(
        LED_FRONT_LEFT,
        HIGH
    );

    digitalWrite(
        LED_FRONT_RIGHT,
        HIGH
    );


    currentCommand = "FORWARD";


    Serial.print(
        "[CAR] FORWARD SPEED="
    );

    Serial.print(
        CAR_SPEED
    );

    Serial.println(
        "%"
    );
}

// ============================================================
// BACKWARD
// ============================================================

void moveBackward()
{
    attachMotors();

    // Physical BACKWARD is currently the same motor pattern
    // as the original LEFT turn.

    int leftBackwardValue =
        getBackwardServoValue(
            SERVO_STOP_FRONT_LEFT
        );

    int leftBackBackwardValue =
        getBackwardServoValue(
            SERVO_STOP_BACK_LEFT
        );

    int rightForwardValue =
        getForwardServoValue(
            SERVO_STOP_FRONT_RIGHT
        );

    int rightBackForwardValue =
        getForwardServoValue(
            SERVO_STOP_BACK_RIGHT
        );


    servoFrontLeft.write(
        leftBackwardValue
    );

    servoBackLeft.write(
        leftBackBackwardValue
    );

    servoFrontRight.write(
        rightForwardValue
    );

    servoBackRight.write(
        rightBackForwardValue
    );


    ledsOff();


    // Taillights
    digitalWrite(
        LED_BACK_LEFT,
        HIGH
    );

    digitalWrite(
        LED_BACK_RIGHT,
        HIGH
    );


    currentCommand = "BACKWARD";


    Serial.print(
        "[CAR] BACKWARD SPEED="
    );

    Serial.print(
        CAR_SPEED
    );

    Serial.println(
        "%"
    );
}

// ============================================================
// TURN LEFT
// ============================================================

void turnLeft()
{
    attachMotors();

    // Physical LEFT is currently the same motor pattern
    // as the original BACKWARD movement.

    int leftBackwardValue =
        getBackwardServoValue(
            SERVO_STOP_FRONT_LEFT
        );

    int leftBackBackwardValue =
        getBackwardServoValue(
            SERVO_STOP_BACK_LEFT
        );

    int rightBackwardValue =
        getBackwardServoValue(
            SERVO_STOP_FRONT_RIGHT
        );

    int rightBackBackwardValue =
        getBackwardServoValue(
            SERVO_STOP_BACK_RIGHT
        );


    servoFrontLeft.write(
        leftBackwardValue
    );

    servoBackLeft.write(
        leftBackBackwardValue
    );

    servoFrontRight.write(
        rightBackwardValue
    );

    servoBackRight.write(
        rightBackBackwardValue
    );


    ledsOff();


    currentCommand = "LEFT";


    // Left turn signal
    blinkState = HIGH;

    previousBlinkMillis =
        millis();


    digitalWrite(
        LED_FRONT_LEFT,
        HIGH
    );

    digitalWrite(
        LED_BACK_LEFT,
        HIGH
    );


    Serial.print(
        "[CAR] LEFT SPEED="
    );

    Serial.print(
        CAR_SPEED
    );

    Serial.println(
        "%"
    );
}

// ============================================================
// TURN RIGHT
// ============================================================

void turnRight()
{
    attachMotors();

    // Physical RIGHT is currently the same motor pattern
    // as the original FORWARD movement.

    int leftForwardValue =
        getForwardServoValue(
            SERVO_STOP_FRONT_LEFT
        );

    int leftBackForwardValue =
        getForwardServoValue(
            SERVO_STOP_BACK_LEFT
        );

    int rightForwardValue =
        getForwardServoValue(
            SERVO_STOP_FRONT_RIGHT
        );

    int rightBackForwardValue =
        getForwardServoValue(
            SERVO_STOP_BACK_RIGHT
        );


    servoFrontLeft.write(
        leftForwardValue
    );

    servoBackLeft.write(
        leftBackForwardValue
    );

    servoFrontRight.write(
        rightForwardValue
    );

    servoBackRight.write(
        rightBackForwardValue
    );


    ledsOff();


    currentCommand = "RIGHT";


    // Right turn signal
    blinkState = HIGH;

    previousBlinkMillis =
        millis();


    digitalWrite(
        LED_FRONT_RIGHT,
        HIGH
    );

    digitalWrite(
        LED_BACK_RIGHT,
        HIGH
    );


    Serial.print(
        "[CAR] RIGHT SPEED="
    );

    Serial.print(
        CAR_SPEED
    );

    Serial.println(
        "%"
    );
}

// ============================================================
// SET SPEED
// ============================================================
//
// Browser:
//     /speed?value=50
//
// Range:
//     0 - 100
//

void handleSpeed()
{
    if (!server.hasArg("value"))
    {
        server.send(
            400,
            "text/plain",
            "Missing speed value"
        );

        return;
    }


    int newSpeed =
        server.arg("value").toInt();


    newSpeed =
        constrain(
            newSpeed,
            0,
            100
        );


    CAR_SPEED =
        newSpeed;


    Serial.print(
        "[CAR] SPEED = "
    );

    Serial.print(
        CAR_SPEED
    );

    Serial.println(
        "%"
    );


    // Speed 0 means STOP
    if (CAR_SPEED == 0)
    {
        stopCar();

        server.send(
            200,
            "text/plain",
            "SPEED 0% - STOP"
        );

        return;
    }


    // Re-apply current movement at new speed

    if (currentCommand == "FORWARD")
    {
        moveForward();
    }
    else if (currentCommand == "BACKWARD")
    {
        moveBackward();
    }
    else if (currentCommand == "LEFT")
    {
        turnLeft();
    }
    else if (currentCommand == "RIGHT")
    {
        turnRight();
    }


    server.send(
        200,
        "text/plain",
        "SPEED "
        + String(CAR_SPEED)
        + "%"
    );
}


// ============================================================
// ROOT WEB PAGE
// ============================================================

void handleRoot()
{
    String html = R"rawliteral(
<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta
    name="viewport"
    content="width=device-width,
             initial-scale=1.0"
>

<title>
ESP32 Robotic Car
</title>

<style>

body {
    font-family: Arial, sans-serif;
    text-align: center;
    margin: 0;
    padding: 20px;
    background: #222;
    color: white;
}

h1 {
    margin-bottom: 10px;
}

#status {
    margin: 20px;
    padding: 12px;
    background: #333;
    border-radius: 8px;
    font-size: 20px;
}

#speedBox {
    max-width: 400px;
    margin: 20px auto;
    padding: 15px;
    background: #333;
    border-radius: 10px;
}

#speedValue {
    font-size: 28px;
    font-weight: bold;
    color: #66ccff;
}

#speedSlider {
    width: 90%;
}

.controls {
    max-width: 450px;
    margin: auto;
}

button {
    width: 120px;
    height: 70px;
    margin: 8px;
    font-size: 18px;
    font-weight: bold;
    border-radius: 12px;
    border: none;
    cursor: pointer;
}

button:active {
    transform: scale(0.95);
}

.forward {
    display: block;
    margin-left: auto;
    margin-right: auto;
}

.backward {
    display: block;
    margin-left: auto;
    margin-right: auto;
}

.stop {
    background: #cc0000;
    color: white;
}

</style>

</head>


<body>

<h1>
ESP32 Robotic Car
</h1>


<div id="status">
STOP
</div>


<!-- ================================================== -->
<!-- SPEED -->
<!-- ================================================== -->

<div id="speedBox">

    <div>
        CAR SPEED
    </div>

    <div id="speedValue">
        50%
    </div>

    <input
        id="speedSlider"
        type="range"
        min="0"
        max="100"
        value="50"
        oninput="updateSpeed(this.value)"
    >

</div>


<!-- ================================================== -->
<!-- CAR CONTROLS -->
<!-- ================================================== -->

<div class="controls">

<button
    class="forward"
    onclick="sendCommand('forward')"
>
    ▲<br>
    FORWARD
</button>


<div>

<button
    onclick="sendCommand('left')"
>
    ◀ LEFT
</button>


<button
    class="stop"
    onclick="sendCommand('stop')"
>
    ■ STOP
</button>


<button
    onclick="sendCommand('right')"
>
    RIGHT ▶
</button>

</div>


<button
    class="backward"
    onclick="sendCommand('backward')"
>
    ▼<br>
    BACKWARD
</button>

</div>


<script>

function sendCommand(command)
{
    fetch("/" + command)

        .then(
            response =>
                response.text()
        )

        .then(
            data =>
            {
                document
                    .getElementById(
                        "status"
                    )
                    .innerText = data;
            }
        )

        .catch(
            error =>
            {
                console.error(error);

                document
                    .getElementById(
                        "status"
                    )
                    .innerText =
                        "Connection Error";
            }
        );
}


function updateSpeed(value)
{
    document
        .getElementById(
            "speedValue"
        )
        .innerText =
            value + "%";


    fetch(
        "/speed?value=" + value
    )

        .then(
            response =>
                response.text()
        )

        .then(
            data =>
            {
                document
                    .getElementById(
                        "status"
                    )
                    .innerText =
                        data;
            }
        )

        .catch(
            error =>
            {
                console.error(error);

                document
                    .getElementById(
                        "status"
                    )
                    .innerText =
                        "Speed Connection Error";
            }
        );
}

</script>

</body>

</html>
)rawliteral";


    server.send(
        200,
        "text/html",
        html
    );
}


// ============================================================
// HTTP COMMAND HANDLERS
// ============================================================

void handleForward()
{
    moveForward();

    server.send(
        200,
        "text/plain",
        "FORWARD"
    );
}


void handleBackward()
{
    moveBackward();

    server.send(
        200,
        "text/plain",
        "BACKWARD"
    );
}


void handleLeft()
{
    turnLeft();

    server.send(
        200,
        "text/plain",
        "LEFT"
    );
}


void handleRight()
{
    turnRight();

    server.send(
        200,
        "text/plain",
        "RIGHT"
    );
}


void handleStop()
{
    stopCar();

    server.send(
        200,
        "text/plain",
        "STOP"
    );
}


// ============================================================
// STATUS
// ============================================================

void handleStatus()
{
    String json =
        "{"
        "\"command\":\""
        + currentCommand
        + "\","
        "\"speed\":"
        + String(CAR_SPEED)
        + "}";

    server.send(
        200,
        "application/json",
        json
    );
}


// ============================================================
// SERVO CALIBRATION PAGE
// ============================================================

void handleCalibration()
{
    String html = R"rawliteral(
<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta
    name="viewport"
    content="width=device-width,
             initial-scale=1.0"
>

<title>
ESP32 Servo Calibration
</title>

<style>

body {
    font-family: Arial, sans-serif;
    text-align: center;
    margin: 0;
    padding: 20px;
    background: #222;
    color: white;
}

h1 {
    margin-bottom: 8px;
}

.info {
    max-width: 500px;
    margin: 10px auto 20px;
    padding: 12px;
    background: #333;
    border-radius: 8px;
}

.servo {
    max-width: 500px;
    margin: 15px auto;
    padding: 15px;
    background: #333;
    border-radius: 10px;
}

.servo h2 {
    margin: 0 0 10px;
}

.value {
    font-size: 28px;
    font-weight: bold;
    margin: 8px;
}

input[type=range] {
    width: 90%;
}

button {
    padding: 12px 18px;
    margin: 6px;
    font-size: 16px;
    border: none;
    border-radius: 8px;
    cursor: pointer;
}

.stop {
    background: #cc0000;
    color: white;
}

.save {
    background: #228822;
    color: white;
}

.test {
    background: #555;
    color: white;
}

a {
    color: #66ccff;
}

</style>

</head>


<body>


<h1>
Servo Stop Calibration
</h1>


<div class="info">

    Adjust each servo until
    its wheel completely stops.

    <br>

    The value is normally
    close to 90.

    <br><br>

    <button
        class="stop"
        onclick="send('/stop')"
    >
        STOP ALL MOTORS
    </button>

</div>


<!-- ================================================== -->
<!-- FRONT RIGHT -->
<!-- ================================================== -->

<div class="servo">

    <h2>
        Front Right
    </h2>

    <div
        class="value"
        id="frValue"
    >
        90
    </div>

    <input
        id="fr"
        type="range"
        min="70"
        max="110"
        value="90"
        oninput="setServo('fr', this.value)"
    >

    <br>

    <button
        class="test"
        onclick="testServo('fr')"
    >
        TEST
    </button>

    <button
        class="save"
        onclick="saveServo('fr')"
    >
        SAVE
    </button>

</div>


<!-- ================================================== -->
<!-- BACK RIGHT -->
<!-- ================================================== -->

<div class="servo">

    <h2>
        Back Right
    </h2>

    <div
        class="value"
        id="brValue"
    >
        90
    </div>

    <input
        id="br"
        type="range"
        min="70"
        max="110"
        value="90"
        oninput="setServo('br', this.value)"
    >

    <br>

    <button
        class="test"
        onclick="testServo('br')"
    >
        TEST
    </button>

    <button
        class="save"
        onclick="saveServo('br')"
    >
        SAVE
    </button>

</div>


<!-- ================================================== -->
<!-- FRONT LEFT -->
<!-- ================================================== -->

<div class="servo">

    <h2>
        Front Left
    </h2>

    <div
        class="value"
        id="flValue"
    >
        90
    </div>

    <input
        id="fl"
        type="range"
        min="70"
        max="110"
        value="90"
        oninput="setServo('fl', this.value)"
    >

    <br>

    <button
        class="test"
        onclick="testServo('fl')"
    >
        TEST
    </button>

    <button
        class="save"
        onclick="saveServo('fl')"
    >
        SAVE
    </button>

</div>


<!-- ================================================== -->
<!-- BACK LEFT -->
<!-- ================================================== -->

<div class="servo">

    <h2>
        Back Left
    </h2>

    <div
        class="value"
        id="blValue"
    >
        90
    </div>

    <input
        id="bl"
        type="range"
        min="70"
        max="110"
        value="90"
        oninput="setServo('bl', this.value)"
    >

    <br>

    <button
        class="test"
        onclick="testServo('bl')"
    >
        TEST
    </button>

    <button
        class="save"
        onclick="saveServo('bl')"
    >
        SAVE
    </button>

</div>


<p>

<a href="/">
    ← Back to Robotic Car
</a>

</p>


<script>

function send(url)
{
    fetch(url)

        .then(
            response =>
                response.text()
        )

        .then(
            data =>
                console.log(data)
        )

        .catch(
            error =>
                console.error(error)
        );
}


function updateDisplay(
    id,
    value
)
{
    document
        .getElementById(
            id + "Value"
        )
        .innerText =
            value;
}


function setServo(
    id,
    value
)
{
    updateDisplay(
        id,
        value
    );

    fetch(
        "/servo?name="
        + id
        + "&value="
        + value
    )

        .then(
            response =>
                response.text()
        )

        .then(
            data =>
                console.log(data)
        )

        .catch(
            error =>
                console.error(error)
        );
}


function testServo(id)
{
    const value =
        document
            .getElementById(id)
            .value;

    setServo(
        id,
        value
    );
}


function saveServo(id)
{
    const value =
        document
            .getElementById(id)
            .value;

    fetch(
        "/servo/save?name="
        + id
        + "&value="
        + value
    )

        .then(
            response =>
                response.text()
        )

        .then(
            data =>
                alert(data)
        )

        .catch(
            error =>
                alert(
                    "Connection Error"
                )
        );
}

</script>


</body>

</html>
)rawliteral";


    server.send(
        200,
        "text/html",
        html
    );
}


// ============================================================
// SERVO CALIBRATION
// ============================================================

void handleServo()
{
    if (
        !server.hasArg("name") ||
        !server.hasArg("value")
    )
    {
        server.send(
            400,
            "text/plain",
            "Missing servo name or value"
        );

        return;
    }


    String name =
        server.arg("name");

    int value =
        server.arg("value").toInt();


    value =
        constrain(
            value,
            70,
            110
        );


    attachMotors();


    if (name == "fr")
    {
        SERVO_STOP_FRONT_RIGHT =
            value;

        servoFrontRight.write(
            value
        );
    }

    else if (name == "br")
    {
        SERVO_STOP_BACK_RIGHT =
            value;

        servoBackRight.write(
            value
        );
    }

    else if (name == "fl")
    {
        SERVO_STOP_FRONT_LEFT =
            value;

        servoFrontLeft.write(
            value
        );
    }

    else if (name == "bl")
    {
        SERVO_STOP_BACK_LEFT =
            value;

        servoBackLeft.write(
            value
        );
    }

    else
    {
        server.send(
            400,
            "text/plain",
            "Unknown servo"
        );

        return;
    }


    server.send(
        200,
        "text/plain",
        "Servo "
        + name
        + " = "
        + String(value)
    );
}


// ============================================================
// SERVO SAVE
// ============================================================

void handleServoSave()
{
    if (
        !server.hasArg("name") ||
        !server.hasArg("value")
    )
    {
        server.send(
            400,
            "text/plain",
            "Missing servo name or value"
        );

        return;
    }


    String name =
        server.arg("name");

    int value =
        constrain(
            server.arg("value").toInt(),
            70,
            110
        );


    attachMotors();


    if (name == "fr")
    {
        SERVO_STOP_FRONT_RIGHT =
            value;

        servoFrontRight.write(
            value
        );
    }

    else if (name == "br")
    {
        SERVO_STOP_BACK_RIGHT =
            value;

        servoBackRight.write(
            value
        );
    }

    else if (name == "fl")
    {
        SERVO_STOP_FRONT_LEFT =
            value;

        servoFrontLeft.write(
            value
        );
    }

    else if (name == "bl")
    {
        SERVO_STOP_BACK_LEFT =
            value;

        servoBackLeft.write(
            value
        );
    }

    else
    {
        server.send(
            400,
            "text/plain",
            "Unknown servo"
        );

        return;
    }


    server.send(
        200,
        "text/plain",
        "Running stop value saved: "
        + name
        + " = "
        + String(value)
    );
}


// ============================================================
// NOT FOUND
// ============================================================

void handleNotFound()
{
    server.send(
        404,
        "text/plain",
        "Not Found"
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );

    delay(
        1000
    );


    Serial.println();

    Serial.println(
        "=========================================="
    );

    Serial.println(
        "ESP32 ROBOTIC CAR"
    );

    Serial.println(
        "WITH SPEED CONTROL"
    );

    Serial.println(
        "=========================================="
    );


    // ========================================================
    // LED Setup
    // ========================================================

    pinMode(
        LED_FRONT_RIGHT,
        OUTPUT
    );

    pinMode(
        LED_BACK_RIGHT,
        OUTPUT
    );

    pinMode(
        LED_FRONT_LEFT,
        OUTPUT
    );

    pinMode(
        LED_BACK_LEFT,
        OUTPUT
    );


    ledsOff();


    // ========================================================
    // Servo Setup
    // ========================================================

    servoFrontRight.setPeriodHertz(
        50
    );

    servoBackRight.setPeriodHertz(
        50
    );

    servoFrontLeft.setPeriodHertz(
        50
    );

    servoBackLeft.setPeriodHertz(
        50
    );


    attachMotors();


    delay(
        500
    );


    stopCar();


    // ========================================================
    // Wi-Fi
    // ========================================================

    Serial.println(
        "[WiFi] Connecting..."
    );


    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );


    while (
        WiFi.status() != WL_CONNECTED
    )
    {
        delay(
            500
        );

        Serial.print(
            "."
        );
    }


    Serial.println();

    Serial.println(
        "[WiFi] Connected!"
    );


    Serial.print(
        "[WiFi] ESP32 IP address: "
    );

    Serial.println(
        WiFi.localIP()
    );


    // ========================================================
    // Web Server Routes
    // ========================================================

    server.on(
        "/",
        handleRoot
    );


    server.on(
        "/forward",
        handleForward
    );


    server.on(
        "/backward",
        handleBackward
    );


    server.on(
        "/left",
        handleLeft
    );


    server.on(
        "/right",
        handleRight
    );


    server.on(
        "/stop",
        handleStop
    );


    server.on(
        "/speed",
        handleSpeed
    );


    server.on(
        "/status",
        handleStatus
    );


    server.on(
        "/calibration",
        handleCalibration
    );


    server.on(
        "/servo",
        handleServo
    );


    server.on(
        "/servo/save",
        handleServoSave
    );


    server.onNotFound(
        handleNotFound
    );


    server.begin();


    Serial.println(
        "[HTTP] Server started"
    );


    Serial.println(
        "=========================================="
    );


    Serial.print(
        "Open in browser: http://"
    );

    Serial.print(
        WiFi.localIP()
    );

    Serial.println(
        "/"
    );


    Serial.print(
        "Servo calibration: http://"
    );

    Serial.print(
        WiFi.localIP()
    );

    Serial.println(
        "/calibration"
    );


    Serial.println(
        "Speed control: 0-100%"
    );


    Serial.println(
        "=========================================="
    );
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Handle Web Requests
    // --------------------------------------------------------

    server.handleClient();


    // --------------------------------------------------------
    // Blinkers
    // --------------------------------------------------------

    if (
        currentCommand == "LEFT" ||
        currentCommand == "RIGHT"
    )
    {
        unsigned long currentMillis =
            millis();


        if (
            currentMillis -
            previousBlinkMillis >=
            blinkInterval
        )
        {
            previousBlinkMillis =
                currentMillis;


            blinkState =
                !blinkState;


            if (
                currentCommand == "LEFT"
            )
            {
                digitalWrite(
                    LED_FRONT_LEFT,
                    blinkState
                );

                digitalWrite(
                    LED_BACK_LEFT,
                    blinkState
                );
            }

            else if (
                currentCommand == "RIGHT"
            )
            {
                digitalWrite(
                    LED_FRONT_RIGHT,
                    blinkState
                );

                digitalWrite(
                    LED_BACK_RIGHT,
                    blinkState
                );
            }
        }
    }
}
