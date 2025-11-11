/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * OpMode to test the encoder functionality and direction of a single motor.
 *
 * Instructions:
 * 1. Configure four motors in the hardware map as: "frontLeft", "frontRight", "backLeft", "backRight".
 * 2. Use Gamepad 1 D-Pad to select the motor to test.
 * 3. The selected motor will run slowly. Check telemetry for tick updates and direction status.
 * 🧠 Analysis of the Integration
 *     Reset and Start Time: When a new motor is selected, the encoder is reset, the runtime timer is reset, and
 *     the motor is commanded to a positive power (TEST_POWER).
 *     Direction Check: The loop() monitors the runtime. After waiting a short time (CHECK_DELAY_SECONDS = 0.5),
 *     it compares the currentTicks to the initialTicks (which should be 0, but we use it for robustness).
 *         Correct: If currentTicks is greater than initialTicks, the positive power resulted in increasing ticks,
 *         which is the correct behavior.
 *         Mismatch: If currentTicks is less than initialTicks, the positive power resulted in decreasing ticks,
 *         indicating the motor/encoder is REVERSED. This requires changing the motor's Direction in the configuration or code.
 *         Original Issue: If the runtime is long enough but the ticks haven't moved much at all
 *         (e.g., less than 50 ticks), it suggests a physical failure
 *         (encoder sensor, faulty port, or original weak signal issue).
 */
@TeleOp(name="Mecanum Encoder Test", group="Test")
public class MecanumEncoderTest extends OpMode {
    private final String TAG = this.getClass().getSimpleName();

    // Motor Declarations
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;

    // Array to hold all motors
    private DcMotorEx[] allMotors;

    // Variable to track the currently selected motor for testing
    private DcMotorEx selectedMotor = null;
    private String selectedMotorName = "NONE";

    // Constant power and encoder mode settings
    private static final double TEST_POWER = 0.25;

    // Variables for direction checking
    private int initialTicks = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private String directionStatus = "Awaiting movement...";

    // Variable to debounce the D-pad presses
    private long lastDpadTime = 0;
    private static final long DPAD_DEBOUNCE_MS = 250;
    private static final double CHECK_DELAY_SECONDS = 0.5; // Time to wait before checking direction

    // --- Initialization Phase ---
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Motors...");

        // 1. Get motors from the hardware map
        try {
            motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
            motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
            motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
            motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

            allMotors = new DcMotorEx[]{motorLeftFront, motorRightFront, motorLeftBack, motorRightBack};

        } catch (Exception e) {
            telemetry.addData("ERROR", "One or more motors not configured correctly.");
            telemetry.addData("Missing Motor", e.getMessage());
            telemetry.update();
            return;
        }

        // 2. Configure all motors
        for (int i = 0; i < allMotors.length; i++) {
            DcMotorEx motor = allMotors[i];
            // Set all motors to run forward direction (adjust based on your robot's setup)
            // Note: If you know which sides should be reversed for your drivetrain, set them here.
            // Check the index to set the direction
            if (i == 0 || i == 1) {
                // Set REVERSE direction for the first two motors (index 0 and 1)
                motor.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                // Set FORWARD direction for all other motors
                motor.setDirection(DcMotorEx.Direction.FORWARD);
            }

            // Set all to STOP_AND_RESET_ENCODER and then back to RUN_WITHOUT_ENCODER
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            // Set power to 0
            motor.setPower(0);
        }

        telemetry.addData("Status", "Initialization Complete.");
        telemetry.addData("Instructions", "Use D-Pad to select motor.");
        telemetry.update();
    }

    // --- Loop Phase (Runs repeatedly while OpMode is active) ---
    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();
        boolean newMotorSelected = false;

        // --- Motor Selection Logic ---
        if (currentTime - lastDpadTime > DPAD_DEBOUNCE_MS) {
            DcMotorEx newSelection = null;
            String newSelectionName = "NONE";

            // Check D-pad input and select motor
            if (gamepad1.dpad_up) {
                newSelection = motorLeftFront;
                newSelectionName = "Left-Front";
            } else if (gamepad1.dpad_right) {
                newSelection = motorRightFront;
                newSelectionName = "Right-Front";
            } else if (gamepad1.dpad_left) {
                newSelection = motorLeftBack;
                newSelectionName = "Left-Back";
            } else if (gamepad1.dpad_down) {
                newSelection = motorRightBack;
                newSelectionName = "Right-Back";
            }

            // If a new motor is selected, perform the switch logic
            if (newSelection != null && newSelection != selectedMotor) {
                // Stop the previously selected motor (if any)
                if (selectedMotor != null) {
                    selectedMotor.setPower(0);
                }

                // Set the new motor as selected
                selectedMotor = newSelection;
                selectedMotorName = newSelectionName;

                // Reset state variables for the new motor
                selectedMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                selectedMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                initialTicks = selectedMotor.getCurrentPosition();
                directionStatus = "Checking direction...";
                runtime.reset(); // Start timer for direction check

                // Start the newly selected motor (positive power)
                selectedMotor.setPower(TEST_POWER);

                newMotorSelected = true;
                lastDpadTime = currentTime;
            }
        }

        // --- Direction Check Logic ---
        if (selectedMotor != null && runtime.seconds() > CHECK_DELAY_SECONDS && !newMotorSelected) {
            int currentTicks = selectedMotor.getCurrentPosition();

            if (currentTicks > initialTicks + 50) { // Check for significant increase (50 ticks is arbitrary but good)
                directionStatus = "✅ **Ticks are INCREASING (Correct)**";
            } else if (currentTicks < initialTicks - 50) {
                directionStatus = "❌ **Ticks are DECREASING (REVERSED)** - Motor/Encoder Direction Mismatch!";
            } else if (runtime.seconds() > CHECK_DELAY_SECONDS * 4) {
                // Check if enough time has passed and ticks are still small (original problem)
                directionStatus = "⚠️ **Ticks not moving** - Possible Encoder/Wiring Failure";
            }
        }

        // --- Telemetry Reporting ---
        int currentTicks = (selectedMotor != null) ? selectedMotor.getCurrentPosition() : 0;

        telemetry.addData("--- Motor Selection ---", "");
        telemetry.addData("Current Motor", "**" + selectedMotorName + "**");
        telemetry.addData("Motor Power", (selectedMotor != null) ? TEST_POWER : 0.0);

        telemetry.addData("--- Encoder Data & Status ---", "");
        telemetry.addData("Encoder Ticks", "**" + currentTicks + "**");
        telemetry.addData("Initial Ticks", initialTicks);
        telemetry.addData("Direction Status", directionStatus);

        if (selectedMotorName.equals("NONE")) {
            telemetry.addData("Action", "Press D-Pad to select a motor.");
        }

        telemetry.update();
    }

    // --- Stop Phase ---
    @Override
    public void stop() {
        // Ensure all motors are stopped when OpMode is stopped
        for (DcMotorEx motor : allMotors) {
            if (motor != null) {
                motor.setPower(0);
            }
        }
        telemetry.addData("Status", "All Motors Stopped.");
        telemetry.update();
    }
}
