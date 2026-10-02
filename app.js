console.log(
    "========== UNO Q ArUco Robotic Car app.js START =========="
);


// ======================================================
// HTML ELEMENTS
// ======================================================

const status =
    document.getElementById("status");

const errorContainer =
    document.getElementById("error-container");

const camera =
    document.getElementById("camera");

const cameraPlaceholder =
    document.getElementById("cameraPlaceholder");

const markerID =
    document.getElementById("marker-id");

const posX =
    document.getElementById("pos-x");

const posY =
    document.getElementById("pos-y");

const roomX =
    document.getElementById("room-x");

const roomY =
    document.getElementById("room-y");

const headingDeg =
    document.getElementById("heading-deg");

const captureButton =
    document.getElementById("captureButton");

const resetButton =
    document.getElementById("resetButton");

const calibrationInstruction =
    document.getElementById("calibration-instruction");

const calibrationResult =
    document.getElementById("calibration-result");

const exposureSlider =
    document.getElementById("exposureSlider");

const exposureValue =
    document.getElementById("exposureValue");


// ======================================================
// RC CAR SPEED ELEMENTS
// ======================================================

const speedSlider =
    document.getElementById("speedSlider");

const speedValue =
    document.getElementById("speedValue");


// ======================================================
// CALIBRATION STATUS ELEMENTS
// ======================================================

const stepElements = [

    document.getElementById("step1"),

    document.getElementById("step2"),

    document.getElementById("step3"),

    document.getElementById("step4")

];


// ======================================================
// CALIBRATION POSITION LABEL ELEMENTS
// ======================================================

const stepPositionElements = [

    document.getElementById(
        "step1-position"
    ),

    document.getElementById(
        "step2-position"
    ),

    document.getElementById(
        "step3-position"
    ),

    document.getElementById(
        "step4-position"
    )

];


// ======================================================
// ESP32 / RC CAR CONTROL ELEMENTS
// ======================================================

const carConnectionStatus =
    document.getElementById(
        "car-connection-status"
    );

const carCommandStatus =
    document.getElementById(
        "car-command-status"
    );

const carForward =
    document.getElementById(
        "btn-forward"
    );

const carBackward =
    document.getElementById(
        "btn-reverse"
    );

const carLeft =
    document.getElementById(
        "btn-left"
    );

const carRight =
    document.getElementById(
        "btn-right"
    );

const carStop =
    document.getElementById(
        "btn-stop"
    );


// ======================================================
// AUTOMATIC NAVIGATION - STEP 3A
// ======================================================

const targetXInput =
    document.getElementById(
        "target-x"
    );

const targetYInput =
    document.getElementById(
        "target-y"
    );

const goToTargetButton =
    document.getElementById(
        "go-to-target"
    );

const navigationResetButton =
    document.getElementById(
        "navigation-reset"
    );

const navCurrentX =
    document.getElementById(
        "nav-current-x"
    );

const navCurrentY =
    document.getElementById(
        "nav-current-y"
    );

const navCurrentHeading =
    document.getElementById(
        "nav-current-heading"
    );

const navTargetX =
    document.getElementById(
        "nav-target-x"
    );

const navTargetY =
    document.getElementById(
        "nav-target-y"
    );

const navDistance =
    document.getElementById(
        "nav-distance"
    );

const navRequiredHeading =
    document.getElementById(
        "nav-required-heading"
    );

const navHeadingError =
    document.getElementById(
        "nav-heading-error"
    );

const navCommand =
    document.getElementById(
        "nav-command"
    );

const autoDriveStatus =
    document.getElementById(
        "auto-drive-status"
    );


// ======================================================
// SOCKET.IO
// ======================================================

const socket = io();


// ======================================================
// CALIBRATION STATE
// ======================================================

let currentStep = 1;

let calibrated = false;

let targetMarkerVisible = false;


// ======================================================
// TRACKING AREA DIMENSIONS
// ======================================================

let areaWidth = 0;

let areaHeight = 0;


// ======================================================
// CAR STATE
// ======================================================

let currentCarCommand = "STOP";

let currentCarSpeed = 50;


// ======================================================
// NAVIGATION STATE
// ======================================================

let navigationActive = false;

let navigationTargetX = null;

let navigationTargetY = null;


// ======================================================
// NAVIGATION SETTINGS
// ======================================================

/*
 * Heading tolerance used by TEST MODE.
 *
 * If the heading error is larger than this value,
 * the proposed command will be LEFT or RIGHT.
 *
 * If the heading is within this tolerance,
 * the proposed command will be FORWARD.
 */

const HEADING_TOLERANCE_DEG = 10.0;


/*
 * Distance tolerance.
 *
 * When the car is this close to the target,
 * the proposed command becomes STOP.
 */

const TARGET_DISTANCE_TOLERANCE_FT = 0.10;


// ======================================================
// HELPERS
// ======================================================

function elementExists(element) {

    return (
        element !== null &&
        element !== undefined
    );

}


// ======================================================
// FORMAT COORDINATE
// ======================================================

function formatCoordinate(value) {

    const number =
        Number(value);


    if (
        !Number.isFinite(number)
    ) {

        return "--";

    }


    if (
        Number.isInteger(number)
    ) {

        return String(number);

    }


    return number.toFixed(3);

}


// ======================================================
// FORMAT HEADING
// ======================================================

function formatHeading(value) {

    const number =
        Number(value);


    if (
        !Number.isFinite(number)
    ) {

        return "--";

    }


    /*
     * Normalize heading to 0-360 degrees.
     *
     * Example:
     *
     * -90  -> 270
     *  360 -> 0
     *  450 -> 90
     */

    let heading =
        number % 360;


    if (
        heading < 0
    ) {

        heading += 360;

    }


    return heading.toFixed(1) + "°";

}


// ======================================================
// NORMALIZE ANGLE
// ======================================================

function normalizeAngle(angle) {

    let result =
        Number(angle);


    if (
        !Number.isFinite(result)
    ) {

        return null;

    }


    result =
        ((result + 180) % 360) - 180;


    return result;

}


// ======================================================
// CALIBRATION POSITIONS
// ======================================================

function getCalibrationPositions() {

    return [

        {
            x: 0,
            y: 0
        },

        {
            x: areaWidth,
            y: 0
        },

        {
            x: areaWidth,
            y: areaHeight
        },

        {
            x: 0,
            y: areaHeight
        }

    ];

}


// ======================================================
// UPDATE CALIBRATION POSITION LABELS
// ======================================================

function updateCalibrationPositionLabels() {

    const positions =
        getCalibrationPositions();


    for (
        let i = 0;
        i < 4;
        i++
    ) {

        if (
            !elementExists(
                stepPositionElements[i]
            )
        ) {

            continue;

        }


        const position =
            positions[i];


        stepPositionElements[i].innerHTML =
            "("
            + formatCoordinate(position.x)
            + ","
            + formatCoordinate(position.y)
            + ")";

    }

}


// ======================================================
// CAR CONNECTION STATUS
// ======================================================

function setCarConnectionStatus(
    text,
    good
) {

    if (
        !elementExists(
            carConnectionStatus
        )
    ) {

        return;

    }


    carConnectionStatus.innerHTML =
        text;


    if (
        good === true
    ) {

        carConnectionStatus.style.color =
            "#00ff99";

    }
    else if (
        good === false
    ) {

        carConnectionStatus.style.color =
            "#ff5555";

    }
    else {

        carConnectionStatus.style.color =
            "#ffaa00";

    }

}


// ======================================================
// CAR COMMAND DISPLAY
// ======================================================

function setCarCommandDisplay(
    command
) {

    currentCarCommand =
        String(
            command
        ).toUpperCase();


    if (
        elementExists(
            carCommandStatus
        )
    ) {

        carCommandStatus.innerHTML =
            "Car: "
            + currentCarCommand;

    }

}


// ======================================================
// SEND CAR COMMAND
// ======================================================

function sendCarCommand(
    command
) {

    const normalizedCommand =
        String(command)
            .toLowerCase()
            .trim();


    const validCommands = [

        "forward",

        "backward",

        "left",

        "right",

        "stop"

    ];


    if (
        !validCommands.includes(
            normalizedCommand
        )
    ) {

        console.error(
            "[CAR] Invalid command:",
            normalizedCommand
        );

        return;

    }


    console.log(
        "[CAR] Manual command:",
        normalizedCommand
    );


    setCarCommandDisplay(
        normalizedCommand
    );


    setCarConnectionStatus(
        "RC Car: Sending command...",
        null
    );


    socket.emit(
        "car_control",
        {
            command:
                normalizedCommand
        }
    );

}


// ======================================================
// SEND CAR SPEED
// ======================================================

function sendCarSpeed(
    speed
) {

    let normalizedSpeed =
        Number(speed);


    if (
        !Number.isFinite(
            normalizedSpeed
        )
    ) {

        console.error(
            "[CAR SPEED] Invalid speed:",
            speed
        );

        return;

    }


    normalizedSpeed =
        Math.round(
            normalizedSpeed
        );


    normalizedSpeed =
        Math.max(
            0,
            Math.min(
                100,
                normalizedSpeed
            )
        );


    currentCarSpeed =
        normalizedSpeed;


    if (
        elementExists(
            speedValue
        )
    ) {

        speedValue.innerHTML =
            normalizedSpeed
            + "%";

    }


    if (
        elementExists(
            speedSlider
        )
    ) {

        speedSlider.value =
            normalizedSpeed;

    }


    console.log(
        "[CAR SPEED] Setting speed:",
        normalizedSpeed
        + "%"
    );


    setCarConnectionStatus(
        "RC Car: Setting speed...",
        null
    );


    socket.emit(
        "car_speed",
        {
            speed:
                normalizedSpeed
        }
    );

}


// ======================================================
// STEP 3A - CALCULATE NAVIGATION
// ======================================================

function calculateNavigation(
    currentX,
    currentY,
    currentHeading
) {

    if (
        !navigationActive
    ) {

        return;

    }


    if (
        !Number.isFinite(
            Number(currentX)
        ) ||
        !Number.isFinite(
            Number(currentY)
        ) ||
        !Number.isFinite(
            Number(currentHeading)
        )
    ) {

        return;

    }


    if (
        !Number.isFinite(
            Number(navigationTargetX)
        ) ||
        !Number.isFinite(
            Number(navigationTargetY)
        )
    ) {

        return;

    }


    currentX =
        Number(currentX);

    currentY =
        Number(currentY);

    currentHeading =
        Number(currentHeading);

    const targetX =
        Number(navigationTargetX);

    const targetY =
        Number(navigationTargetY);


    // --------------------------------------------------
    // Difference between current position and target
    // --------------------------------------------------

    const dx =
        targetX - currentX;

    const dy =
        targetY - currentY;


    // --------------------------------------------------
    // Distance
    // --------------------------------------------------

    const distance =
        Math.sqrt(
            (dx * dx)
            +
            (dy * dy)
        );


    // --------------------------------------------------
    // Required heading
    //
    // Same coordinate convention as aruco_tracker.py:
    //
    // 0   = +X
    // 90  = +Y
    // 180 = -X
    // 270 = -Y
    // --------------------------------------------------

    let requiredHeading =
        Math.atan2(
            dy,
            dx
        )
        *
        180
        /
        Math.PI;


    if (
        requiredHeading < 0
    ) {

        requiredHeading += 360;

    }


    requiredHeading %=
        360;


    // --------------------------------------------------
    // Heading error
    //
    // Result is -180 to +180.
    //
    // Positive:
    //     turn RIGHT / clockwise
    //
    // Negative:
    //     turn LEFT / counter-clockwise
    // --------------------------------------------------

    const headingError =
        normalizeAngle(
            requiredHeading
            -
            currentHeading
        );


    // --------------------------------------------------
    // Proposed command
    //
    // TEST MODE ONLY.
    //
    // NOTHING is sent to ESP32.
    // --------------------------------------------------

    let proposedCommand =
        "STOP";


    if (
        distance
        <=
        TARGET_DISTANCE_TOLERANCE_FT
    ) {

        proposedCommand =
            "STOP";

    }
    else if (
        headingError === null
    ) {

        proposedCommand =
            "STOP";

    }
    else if (
        Math.abs(
            headingError
        )
        <=
        HEADING_TOLERANCE_DEG
    ) {

        proposedCommand =
            "FORWARD";

    }
    else if (
        headingError > 0
    ) {

        proposedCommand =
            "RIGHT";

    }
    else {

        proposedCommand =
            "LEFT";

    }


    // --------------------------------------------------
    // Display current position
    // --------------------------------------------------

    if (
        elementExists(
            navCurrentX
        )
    ) {

        navCurrentX.innerHTML =
            currentX.toFixed(2)
            + " ft";

    }


    if (
        elementExists(
            navCurrentY
        )
    ) {

        navCurrentY.innerHTML =
            currentY.toFixed(2)
            + " ft";

    }


    if (
        elementExists(
            navCurrentHeading
        )
    ) {

        navCurrentHeading.innerHTML =
            formatHeading(
                currentHeading
            );

    }


    // --------------------------------------------------
    // Display target
    // --------------------------------------------------

    if (
        elementExists(
            navTargetX
        )
    ) {

        navTargetX.innerHTML =
            targetX.toFixed(2)
            + " ft";

    }


    if (
        elementExists(
            navTargetY
        )
    ) {

        navTargetY.innerHTML =
            targetY.toFixed(2)
            + " ft";

    }


    // --------------------------------------------------
    // Display distance
    // --------------------------------------------------

    if (
        elementExists(
            navDistance
        )
    ) {

        navDistance.innerHTML =
            distance.toFixed(2)
            + " ft";

    }


    // --------------------------------------------------
    // Display required heading
    // --------------------------------------------------

    if (
        elementExists(
            navRequiredHeading
        )
    ) {

        navRequiredHeading.innerHTML =
            formatHeading(
                requiredHeading
            );

    }


    // --------------------------------------------------
    // Display heading error
    // --------------------------------------------------

    if (
        elementExists(
            navHeadingError
        )
    ) {

        navHeadingError.innerHTML =
            headingError.toFixed(1)
            + "°";

    }


    // --------------------------------------------------
    // Display proposed command
    // --------------------------------------------------

    if (
        elementExists(
            navCommand
        )
    ) {

        navCommand.innerHTML =
            proposedCommand;

    }


    // --------------------------------------------------
    // TEST MODE STATUS
    // --------------------------------------------------

    if (
        elementExists(
            autoDriveStatus
        )
    ) {

        autoDriveStatus.innerHTML =
            "AUTO DRIVE: DISABLED — TEST MODE";

    }


    // --------------------------------------------------
    // Console diagnostics
    // --------------------------------------------------

    console.log(
        "[NAV 3A]",
        "Current=("
        + currentX.toFixed(2)
        + ","
        + currentY.toFixed(2)
        + ")",
        "Target=("
        + targetX.toFixed(2)
        + ","
        + targetY.toFixed(2)
        + ")",
        "Distance="
        + distance.toFixed(2)
        + "ft",
        "CurrentHeading="
        + currentHeading.toFixed(1),
        "RequiredHeading="
        + requiredHeading.toFixed(1),
        "HeadingError="
        + headingError.toFixed(1),
        "WouldCommand="
        + proposedCommand
    );

}


// ======================================================
// SOCKET CONNECTED
// ======================================================

socket.on(
    "connect",
    function () {

        console.log(
            "[SOCKET] Connected to UNO Q"
        );


        if (
            elementExists(status)
        ) {

            status.innerHTML =
                "Connected to UNO Q";

        }


        if (
            elementExists(errorContainer)
        ) {

            errorContainer.innerHTML =
                "";

        }


        setCarConnectionStatus(
            "RC Car: Ready",
            null
        );


        setCarCommandDisplay(
            "STOP"
        );


        if (
            elementExists(
                autoDriveStatus
            )
        ) {

            autoDriveStatus.innerHTML =
                "AUTO DRIVE: DISABLED — TEST MODE";

        }

    }
);


// ======================================================
// SOCKET DISCONNECTED
// ======================================================

socket.on(
    "disconnect",
    function () {

        console.log(
            "[SOCKET] Disconnected"
        );


        if (
            elementExists(status)
        ) {

            status.innerHTML =
                "Disconnected";

        }


        if (
            elementExists(
                errorContainer
            )
        ) {

            errorContainer.innerHTML =
                "Lost connection to UNO Q";

        }


        setCarConnectionStatus(
            "RC Car: Unknown",
            false
        );

    }
);


// ======================================================
// LIVE VIDEO
// ======================================================

socket.on(
    "video_feed",
    function (message) {

        if (
            !message ||
            !message.image
        ) {

            return;

        }


        if (
            elementExists(camera)
        ) {

            camera.src =
                message.image;


            camera.style.display =
                "block";

        }


        if (
            elementExists(
                cameraPlaceholder
            )
        ) {

            cameraPlaceholder.style.display =
                "none";

        }

    }
);


// ======================================================
// CAMERA EXPOSURE
// ======================================================

if (
    elementExists(
        exposureSlider
    )
) {

    exposureSlider.addEventListener(
        "input",
        function () {

            const exposure =
                Number(
                    exposureSlider.value
                );


            if (
                elementExists(
                    exposureValue
                )
            ) {

                exposureValue.innerHTML =
                    exposure;

            }


            console.log(
                "[CAMERA] Exposure:",
                exposure
            );


            socket.emit(
                "camera_control",
                {
                    exposure:
                        exposure
                }
            );

        }
    );

}


// ======================================================
// RC CAR SPEED SLIDER
// ======================================================

if (
    elementExists(
        speedSlider
    )
) {

    speedSlider.addEventListener(
        "input",
        function () {

            const speed =
                Number(
                    speedSlider.value
                );


            if (
                elementExists(
                    speedValue
                )
            ) {

                speedValue.innerHTML =
                    speed
                    + "%";

            }


            sendCarSpeed(
                speed
            );

        }
    );

}


// ======================================================
// CAR SPEED RESULT
// ======================================================

socket.on(
    "car_speed_result",
    function (message) {

        console.log(
            "[CAR SPEED RESULT]:",
            message
        );


        if (
            !message
        ) {

            return;

        }


        if (
            message.speed !== undefined &&
            message.speed !== null
        ) {

            currentCarSpeed =
                Number(
                    message.speed
                );


            if (
                elementExists(
                    speedValue
                )
            ) {

                speedValue.innerHTML =
                    currentCarSpeed
                    + "%";

            }


            if (
                elementExists(
                    speedSlider
                )
            ) {

                speedSlider.value =
                    currentCarSpeed;

            }

        }


        if (
            message.success
        ) {

            setCarConnectionStatus(
                "RC Car: Connected",
                true
            );

        }
        else {

            setCarConnectionStatus(
                "RC Car: Connection Error",
                false
            );

        }

    }
);


// ======================================================
// MARKER POSITION + HEADING
// ======================================================

socket.on(
    "marker_position",
    function (message) {

        console.log(
            "[MARKER] Position:",
            message
        );


        if (
            !message
        ) {

            targetMarkerVisible =
                false;

            return;

        }


        // ==================================================
        // MARKER ID
        // ==================================================

        if (
            elementExists(markerID) &&
            message.id !== undefined &&
            message.id !== null
        ) {

            markerID.innerHTML =
                message.id;

        }


        // ==================================================
        // PIXEL X
        // ==================================================

        if (
            elementExists(posX)
        ) {

            if (
                message.pixel_x !== undefined &&
                message.pixel_x !== null
            ) {

                posX.innerHTML =
                    message.pixel_x
                    + " px";

            }
            else {

                posX.innerHTML =
                    "--";

            }

        }


        // ==================================================
        // PIXEL Y
        // ==================================================

        if (
            elementExists(posY)
        ) {

            if (
                message.pixel_y !== undefined &&
                message.pixel_y !== null
            ) {

                posY.innerHTML =
                    message.pixel_y
                    + " px";

            }
            else {

                posY.innerHTML =
                    "--";

            }

        }


        // ==================================================
        // MARKER VISIBILITY
        // ==================================================

        targetMarkerVisible =
            (
                message.room_x !== undefined &&
                message.room_x !== null &&
                message.room_y !== undefined &&
                message.room_y !== null
            );


        // ==================================================
        // REAL X / Y
        // ==================================================

        if (
            message.room_x === null ||
            message.room_x === undefined ||
            message.room_y === null ||
            message.room_y === undefined
        ) {

            if (
                elementExists(roomX)
            ) {

                roomX.innerHTML =
                    "--";

            }


            if (
                elementExists(roomY)
            ) {

                roomY.innerHTML =
                    "--";

            }

        }
        else {

            if (
                elementExists(roomX)
            ) {

                roomX.innerHTML =
                    Number(
                        message.room_x
                    ).toFixed(2)
                    + " ft";

            }


            if (
                elementExists(roomY)
            ) {

                roomY.innerHTML =
                    Number(
                        message.room_y
                    ).toFixed(2)
                    + " ft";

            }

        }


        // ==================================================
        // HEADING
        // ==================================================

        if (
            elementExists(headingDeg)
        ) {

            if (
                message.heading_deg !== undefined &&
                message.heading_deg !== null
            ) {

                headingDeg.innerHTML =
                    formatHeading(
                        message.heading_deg
                    );

            }
            else {

                headingDeg.innerHTML =
                    "--";

            }

        }


        // ==================================================
        // STATUS
        // ==================================================

        if (
            elementExists(status)
        ) {

            if (
                message.room_x === null ||
                message.room_x === undefined ||
                message.room_y === null ||
                message.room_y === undefined
            ) {

                status.innerHTML =
                    "Target marker detected — calibration required";

            }
            else {

                status.innerHTML =
                    "Tracking target marker";

            }

        }


        // ==================================================
        // STEP 3A NAVIGATION CALCULATION
        // ==================================================

        if (
            navigationActive &&
            message.room_x !== undefined &&
            message.room_x !== null &&
            message.room_y !== undefined &&
            message.room_y !== null &&
            message.heading_deg !== undefined &&
            message.heading_deg !== null
        ) {

            calculateNavigation(
                Number(
                    message.room_x
                ),
                Number(
                    message.room_y
                ),
                Number(
                    message.heading_deg
                )
            );

        }

    }
);


// ======================================================
// CAMERA STATUS
// ======================================================

socket.on(
    "camera_status",
    function (message) {

        console.log(
            "[CAMERA STATUS]:",
            message
        );


        if (
            !message
        ) {

            return;

        }


        if (
            message.exposure !== null &&
            message.exposure !== undefined
        ) {

            if (
                elementExists(
                    exposureSlider
                )
            ) {

                exposureSlider.value =
                    message.exposure;

            }


            if (
                elementExists(
                    exposureValue
                )
            ) {

                exposureValue.innerHTML =
                    message.exposure;

            }

        }

    }
);


// ======================================================
// CALIBRATION STATUS
// ======================================================

socket.on(
    "calibration_status",
    function (message) {

        console.log(
            "[CALIBRATION STATUS]:",
            message
        );


        if (
            !message
        ) {

            return;

        }


        if (
            message.step !== undefined &&
            message.step !== null
        ) {

            currentStep =
                Number(
                    message.step
                );

        }


        calibrated =
            Boolean(
                message.calibrated
            );


        if (
            message.area_width !== undefined &&
            message.area_height !== undefined
        ) {

            areaWidth =
                Number(
                    message.area_width
                );


            areaHeight =
                Number(
                    message.area_height
                );


            updateCalibrationPositionLabels();

        }


        if (
            Array.isArray(
                message.captured
            )
        ) {

            for (
                let i = 0;
                i < 4;
                i++
            ) {

                if (
                    !elementExists(
                        stepElements[i]
                    )
                ) {

                    continue;

                }


                stepElements[i].innerHTML =
                    message.captured[i]
                        ? "Captured"
                        : "Not captured";

            }

        }


        if (
            calibrated
        ) {

            if (
                elementExists(
                    calibrationInstruction
                )
            ) {

                calibrationInstruction.innerHTML =
                    "Calibration complete. Move the target marker anywhere inside the calibrated area.";

            }


            if (
                elementExists(
                    calibrationResult
                )
            ) {

                calibrationResult.innerHTML =
                    "✓ Calibration complete — target marker X/Y is now measured in feet.";

            }


            if (
                elementExists(
                    captureButton
                )
            ) {

                captureButton.disabled =
                    true;


                captureButton.innerHTML =
                    "Calibration Complete";

            }


            return;

        }


        const positions =
            getCalibrationPositions();


        if (
            currentStep >= 1 &&
            currentStep <= 4
        ) {

            const position =
                positions[
                    currentStep - 1
                ];


            if (
                elementExists(
                    calibrationInstruction
                )
            ) {

                calibrationInstruction.innerHTML =
                    "Step "
                    + currentStep
                    + " of 4: Place target marker at ("
                    + formatCoordinate(
                        position.x
                    )
                    + ","
                    + formatCoordinate(
                        position.y
                    )
                    + ") ft";

            }

        }


        if (
            elementExists(
                calibrationResult
            )
        ) {

            calibrationResult.innerHTML =
                "Calibration in progress.";

        }

    }
);


// ======================================================
// CAR COMMAND RESULT
// ======================================================

socket.on(
    "car_command_result",
    function (message) {

        console.log(
            "[CAR RESULT]:",
            message
        );


        if (
            !message
        ) {

            return;

        }


        if (
            message.success
        ) {

            setCarConnectionStatus(
                "RC Car: Connected",
                true
            );


            if (
                message.command !== undefined
            ) {

                setCarCommandDisplay(
                    message.command
                );

            }

        }
        else {

            setCarConnectionStatus(
                "RC Car: Connection Error",
                false
            );

        }

    }
);


// ======================================================
// CALIBRATION CAPTURED
// ======================================================

socket.on(
    "calibration_captured",
    function (message) {

        console.log(
            "[CALIBRATION CAPTURED]:",
            message
        );


        if (
            !message
        ) {

            return;

        }


        const step =
            Number(
                message.step
            );


        if (
            step >= 1 &&
            step <= 4
        ) {

            if (
                elementExists(
                    stepElements[
                        step - 1
                    ]
                )
            ) {

                stepElements[
                    step - 1
                ].innerHTML =
                    "Captured";

            }

        }

    }
);


// ======================================================
// CALIBRATION COMPLETE
// ======================================================

socket.on(
    "calibration_complete",
    function (message) {

        console.log(
            "[CALIBRATION COMPLETE]:",
            message
        );


        calibrated =
            true;


        if (
            elementExists(
                calibrationResult
            )
        ) {

            calibrationResult.innerHTML =
                "✓ Calibration complete.";

        }


        if (
            elementExists(
                calibrationInstruction
            )
        ) {

            calibrationInstruction.innerHTML =
                "Move the target marker anywhere inside the calibrated area.";

        }


        if (
            elementExists(
                captureButton
            )
        ) {

            captureButton.disabled =
                true;


            captureButton.innerHTML =
                "Calibration Complete";

        }

    }
);


// ======================================================
// CALIBRATION ERROR
// ======================================================

socket.on(
    "calibration_error",
    function (message) {

        console.error(
            "[CALIBRATION ERROR]:",
            message
        );


        if (
            message &&
            message.error
        ) {

            alert(
                message.error
            );

        }
        else {

            alert(
                "Calibration error."
            );

        }

    }
);


// ======================================================
// CAPTURE BUTTON
// ======================================================

if (
    elementExists(
        captureButton
    )
) {

    captureButton.addEventListener(
        "click",
        function () {

            console.log(
                "[CALIBRATION] Capture button clicked"
            );


            if (
                calibrated
            ) {

                return;

            }


            captureButton.disabled =
                true;


            captureButton.innerHTML =
                "Capturing...";


            socket.emit(
                "capture_calibration",
                {}
            );


            setTimeout(
                function () {

                    if (
                        !calibrated
                    ) {

                        captureButton.disabled =
                            false;

                        captureButton.innerHTML =
                            "Capture Position";

                    }

                },
                1500
            );

        }
    );

}


// ======================================================
// RESET BUTTON
// ======================================================

if (
    elementExists(
        resetButton
    )
) {

    resetButton.addEventListener(
        "click",
        function () {

            console.log(
                "[CALIBRATION] Reset"
            );


            currentStep =
                1;


            calibrated =
                false;


            for (
                let i = 0;
                i < 4;
                i++
            ) {

                if (
                    elementExists(
                        stepElements[i]
                    )
                ) {

                    stepElements[i].innerHTML =
                        "Not captured";

                }

            }


            socket.emit(
                "reset_calibration",
                {}
            );


            if (
                elementExists(
                    captureButton
                )
            ) {

                captureButton.disabled =
                    false;

                captureButton.innerHTML =
                    "Capture Position";

            }


            if (
                elementExists(
                    calibrationResult
                )
            ) {

                calibrationResult.innerHTML =
                    "Calibration reset.";

            }


            const positions =
                getCalibrationPositions();


            if (
                elementExists(
                    calibrationInstruction
                )
            ) {

                calibrationInstruction.innerHTML =
                    "Step 1 of 4: Place target marker at ("
                    + formatCoordinate(
                        positions[0].x
                    )
                    + ","
                    + formatCoordinate(
                        positions[0].y
                    )
                    + ") ft";

            }

        }
    );

}


// ======================================================
// RC CAR BUTTONS
// ======================================================

function bindCarButton(
    element,
    command
) {

    if (
        !elementExists(element)
    ) {

        console.warn(
            "[CAR] Button not found:",
            command
        );

        return;

    }


    element.addEventListener(
        "click",
        function (e) {

            e.preventDefault();

            sendCarCommand(
                command
            );

        }
    );

}


bindCarButton(
    carForward,
    "forward"
);


bindCarButton(
    carBackward,
    "backward"
);


bindCarButton(
    carLeft,
    "left"
);


bindCarButton(
    carRight,
    "right"
);


bindCarButton(
    carStop,
    "stop"
);


// ======================================================
// AUTOMATIC NAVIGATION - GO TO TARGET
// ======================================================

if (
    elementExists(
        goToTargetButton
    )
) {

    goToTargetButton.addEventListener(
        "click",
        function () {

            const x =
                Number(
                    targetXInput.value
                );

            const y =
                Number(
                    targetYInput.value
                );


            // --------------------------------------------------
            // Validate target
            // --------------------------------------------------

            if (
                !Number.isFinite(x) ||
                !Number.isFinite(y)
            ) {

                alert(
                    "Please enter valid Target X and Target Y."
                );

                return;

            }


            // --------------------------------------------------
            // Validate calibrated area
            // --------------------------------------------------

            if (
                calibrated &&
                areaWidth > 0 &&
                areaHeight > 0
            ) {

                if (
                    x < 0 ||
                    x > areaWidth ||
                    y < 0 ||
                    y > areaHeight
                ) {

                    alert(
                        "Target is outside the calibrated area."
                    );

                    return;

                }

            }


            navigationTargetX =
                x;

            navigationTargetY =
                y;

            navigationActive =
                true;


            console.log(
                "=========================================="
            );

            console.log(
                "[NAV 3A] GO TO TARGET"
            );

            console.log(
                "[NAV 3A] Target X:",
                x
            );

            console.log(
                "[NAV 3A] Target Y:",
                y
            );

            console.log(
                "[NAV 3A] Automatic driving: DISABLED"
            );

            console.log(
                "=========================================="
            );


            // --------------------------------------------------
            // Display target immediately
            // --------------------------------------------------

            if (
                elementExists(
                    navTargetX
                )
            ) {

                navTargetX.innerHTML =
                    x.toFixed(2)
                    + " ft";

            }


            if (
                elementExists(
                    navTargetY
                )
            ) {

                navTargetY.innerHTML =
                    y.toFixed(2)
                    + " ft";

            }


            if (
                elementExists(
                    autoDriveStatus
                )
            ) {

                autoDriveStatus.innerHTML =
                    "AUTO DRIVE: DISABLED — TEST MODE";

            }


            if (
                elementExists(
                    navCommand
                )
            ) {

                navCommand.innerHTML =
                    "WAITING FOR POSITION";

            }


            /*
             * IMPORTANT:
             *
             * We intentionally DO NOT send:
             *
             * socket.emit("car_control", ...)
             *
             * here.
             *
             * Step 3A is calculation/testing only.
             */

            // --------------------------------------------------
            // Tell UNO Q target exists.
            //
            // This is informational only.
            // If no UNO Q navigation handler exists,
            // nothing depends on this message.
            // --------------------------------------------------

            socket.emit(
                "navigation_target",
                {
                    target_x:
                        x,

                    target_y:
                        y
                }
            );

        }
    );

}


// ======================================================
// AUTOMATIC NAVIGATION - RESET
// ======================================================

if (
    elementExists(
        navigationResetButton
    )
) {

    navigationResetButton.addEventListener(
        "click",
        function () {

            console.log(
                "[NAV 3A] Navigation reset"
            );


            navigationActive =
                false;


            navigationTargetX =
                null;


            navigationTargetY =
                null;


            if (
                elementExists(
                    targetXInput
                )
            ) {

                targetXInput.value =
                    "";

            }


            if (
                elementExists(
                    targetYInput
                )
            ) {

                targetYInput.value =
                    "";

            }


            if (
                elementExists(
                    navCurrentX
                )
            ) {

                navCurrentX.innerHTML =
                    "--";

            }


            if (
                elementExists(
                    navCurrentY
                )
            ) {

                navCurrentY.innerHTML =
                    "--";

            }


            if (
                elementExists(
                    navCurrentHeading
                )
            ) {

                navCurrentHeading.innerHTML =
                    "--";

            }


            if (
                elementExists(
                    navTargetX
                )
            ) {

                navTargetX.innerHTML =
                    "--";

            }


            if (
                elementExists(
                    navTargetY
                )
            ) {

                navTargetY.innerHTML =
                    "--";

            }


            if (
                elementExists(
                    navDistance
                )
            ) {

                navDistance.innerHTML =
                    "--";

            }


            if (
                elementExists(
                    navRequiredHeading
                )
            ) {

                navRequiredHeading.innerHTML =
                    "--";

            }


            if (
                elementExists(
                    navHeadingError
                )
            ) {

                navHeadingError.innerHTML =
                    "--";

            }


            if (
                elementExists(
                    navCommand
                )
            ) {

                navCommand.innerHTML =
                    "STOP";

            }


            if (
                elementExists(
                    autoDriveStatus
                )
            ) {

                autoDriveStatus.innerHTML =
                    "AUTO DRIVE: DISABLED";

            }


            socket.emit(
                "navigation_reset",
                {}
            );

        }
    );

}


// ======================================================
// NAVIGATION STATUS FROM UNO Q
//
// This handler is included for compatibility if we later
// add a navigation controller on the UNO Q.
//
// Step 3A does NOT depend on this event because the browser
// calculates the navigation from marker_position.
// ======================================================

socket.on(
    "navigation_status",
    function (message) {

        console.log(
            "[NAVIGATION STATUS]:",
            message
        );


        if (
            !message
        ) {

            return;

        }


        if (
            elementExists(
                navCurrentX
            )
        ) {

            if (
                message.current_x !== undefined &&
                message.current_x !== null
            ) {

                navCurrentX.innerHTML =
                    Number(
                        message.current_x
                    ).toFixed(2)
                    + " ft";

            }

        }


        if (
            elementExists(
                navCurrentY
            )
        ) {

            if (
                message.current_y !== undefined &&
                message.current_y !== null
            ) {

                navCurrentY.innerHTML =
                    Number(
                        message.current_y
                    ).toFixed(2)
                    + " ft";

            }

        }


        if (
            elementExists(
                navCurrentHeading
            )
        ) {

            if (
                message.current_heading !== undefined &&
                message.current_heading !== null
            ) {

                navCurrentHeading.innerHTML =
                    formatHeading(
                        message.current_heading
                    );

            }

        }


        if (
            elementExists(
                navTargetX
            )
        ) {

            if (
                message.target_x !== undefined &&
                message.target_x !== null
            ) {

                navTargetX.innerHTML =
                    Number(
                        message.target_x
                    ).toFixed(2)
                    + " ft";

            }

        }


        if (
            elementExists(
                navTargetY
            )
        ) {

            if (
                message.target_y !== undefined &&
                message.target_y !== null
            ) {

                navTargetY.innerHTML =
                    Number(
                        message.target_y
                    ).toFixed(2)
                    + " ft";

            }

        }


        if (
            elementExists(
                navDistance
            )
        ) {

            if (
                message.distance !== undefined &&
                message.distance !== null
            ) {

                navDistance.innerHTML =
                    Number(
                        message.distance
                    ).toFixed(2)
                    + " ft";

            }

        }


        if (
            elementExists(
                navRequiredHeading
            )
        ) {

            if (
                message.required_heading !== undefined &&
                message.required_heading !== null
            ) {

                navRequiredHeading.innerHTML =
                    formatHeading(
                        message.required_heading
                    );

            }

        }


        if (
            elementExists(
                navHeadingError
            )
        ) {

            if (
                message.heading_error !== undefined &&
                message.heading_error !== null
            ) {

                navHeadingError.innerHTML =
                    Number(
                        message.heading_error
                    ).toFixed(1)
                    + "°";

            }

        }


        if (
            elementExists(
                navCommand
            )
        ) {

            if (
                message.command !== undefined &&
                message.command !== null
            ) {

                navCommand.innerHTML =
                    String(
                        message.command
                    ).toUpperCase();

            }

        }


        if (
            elementExists(
                autoDriveStatus
            )
        ) {

            autoDriveStatus.innerHTML =
                "AUTO DRIVE: DISABLED — TEST MODE";

        }

    }
);


// ======================================================
// INITIALIZATION
// ======================================================

updateCalibrationPositionLabels();


if (
    elementExists(
        speedSlider
    )
) {

    speedSlider.value =
        currentCarSpeed;

}


if (
    elementExists(
        speedValue
    )
) {

    speedValue.innerHTML =
        currentCarSpeed
        + "%";

}


setCarConnectionStatus(
    "RC Car: Waiting...",
    null
);


setCarCommandDisplay(
    "STOP"
);


if (
    elementExists(
        autoDriveStatus
    )
) {

    autoDriveStatus.innerHTML =
        "AUTO DRIVE: DISABLED";

}


if (
    elementExists(
        navCommand
    )
) {

    navCommand.innerHTML =
        "STOP";

}


console.log(
    "========== UNO Q ArUco Robotic Car app.js LOADED =========="
);