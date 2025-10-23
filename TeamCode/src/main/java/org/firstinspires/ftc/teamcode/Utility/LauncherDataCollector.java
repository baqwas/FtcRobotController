/*
 * MIT License
 *
 * Copyright (c) 2024 ParkCircus Productions; All Rights Reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime; // Import ElapsedTime for accurate delay

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;

import java.util.List;

@TeleOp(name = "Localization & Launcher Logger", group = "DataEngineering")
public class LauncherDataCollector extends LinearOpMode {

    // --- Vision Components ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- Hardware Components ---
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;
    private DcMotorEx launcherMotor = null; // New motor for the launcher mechanism
    private VoltageSensor batterySensor = null; // Component to read battery voltage

    // --- Data Logging Components ---
    private Datalogger datalogger;
    private Datalogger.GenericField fTime;
    private Datalogger.GenericField fID;
    private Datalogger.GenericField fRange;
    private Datalogger.GenericField fBearing;
    private Datalogger.GenericField fElevation;
    private Datalogger.GenericField fDriveMotorPower;
    private Datalogger.GenericField fDriveMotorVelocity;
    private Datalogger.GenericField fLauncherCurrentVel;
    private Datalogger.GenericField fLauncherTargetVel;
    private Datalogger.GenericField fSuccessStatus; // 0=Idle, 1=Success, -1=Failure
    private Datalogger.GenericField fBatteryVoltage;
    private Datalogger.GenericField fIsLaunching; // State of the Y button

    // --- State Variables ---
    private double currentTargetVelocity = 1000.0; // Start with a reasonable default velocity
    private boolean prevAState = false; // For rising edge detection on SUCCESS
    private boolean prevBState = false; // For rising edge detection on FAILURE
    private boolean prevYState = false; // For rising edge detection on LAUNCH (Y)
    private ElapsedTime launchTimer = new ElapsedTime();
    private boolean isVelocityStable = false;

    // Constants
    private final double DRIVE_SPEED = 0.5;
    private final int VELOCITY_INCREMENT = 50; // Ticks/sec increment for dpad
    private final double VELOCITY_SETTLE_TIME_MS = 1500; // Time in ms to wait for PID to stabilize

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        try {
            // Mecanum Drive Motors
            motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
            motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
            motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
            motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

            // Mecanum Motor Directions
            motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
            motorRightFront.setDirection(DcMotor.Direction.FORWARD);
            motorRightBack.setDirection(DcMotor.Direction.FORWARD);

            // Set drive motor run modes
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Launcher motor setup for velocity control
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Find a primary voltage sensor
            batterySensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            telemetry.addData("Error", "Hardware initialization failed. Check config names: " + e.getMessage());
            telemetry.update();
            sleep(3000);
            return;
        }

        // 2. Initialize AprilTag Vision
        initAprilTag();

        // 3. Initialize DataLogger
        initDataLogger();

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData("TARGET VELOCITY", "%.0f Ticks/Sec", currentTargetVelocity);
        telemetry.addData("--- CONTROL ---", "D-pad Up/Down: Change Velocity");
        telemetry.addData("--- CONTROL ---", "Y: Activate Launcher Motor");
        telemetry.addData("--- LOGGING ---", "A: Success Log (Status=1) | B: Failure Log (Status=-1)");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Start the launch timer immediately when the OpMode starts
            launchTimer.reset();

            while (opModeIsActive()) {

                handleDriveControl();

                // Handles velocity setting (D-pad) and motor activation (Y)
                handleLauncherControl();

                // Check if enough time has passed since the motor was activated (Y pressed)
                if (launcherMotor.getPower() != 0 && launchTimer.milliseconds() >= VELOCITY_SETTLE_TIME_MS) {
                    isVelocityStable = true;
                    telemetry.addData("LAUNCHER STATUS", "Velocity STABLE. Ready for A/B log.");
                } else if (launcherMotor.getPower() != 0) {
                    isVelocityStable = false;
                    telemetry.addData("LAUNCHER STATUS", "Warming up... %.0f / %.0f ms", launchTimer.milliseconds(), VELOCITY_SETTLE_TIME_MS);
                } else {
                    isVelocityStable = false;
                    telemetry.addData("LAUNCHER STATUS", "IDLE. Press Y to activate motor.");
                }


                // Get and set all data fields (AprilTag, Motor, System)
                logAprilTagTelemetry();
                logMotorVelocity();
                logSystemVoltage();

                // --- CRITICAL LOGGING LOGIC ---
                // Data is logged ONLY if A or B is pressed AND the velocity has stabilized.

                fSuccessStatus.set(0); // Default status is IDLE/NOT LOGGED

                boolean currentAState = gamepad1.a; // Success button
                boolean currentBState = gamepad1.b; // Failure button

                // Log only on rising edge of A or B
                boolean loggingTriggered = false;

                if (isVelocityStable) {
                    // Rising edge detection for Success button (A)
                    if (currentAState && !prevAState) {
                        fSuccessStatus.set(1); // Set status to SUCCESS
                        datalogger.writeLine();
                        telemetry.addData("Data Log Status", ">> SUCCESS LOGGED << (Status=1)");
                        loggingTriggered = true;
                    }

                    // Rising edge detection for Failure button (B)
                    else if (currentBState && !prevBState) {
                        fSuccessStatus.set(-1); // Set status to FAILURE
                        datalogger.writeLine();
                        telemetry.addData("Data Log Status", ">> FAILURE LOGGED << (Status=-1)");
                        loggingTriggered = true;
                    }
                } else if (currentAState || currentBState) {
                    // Prevent logging if not stable
                    telemetry.addData("Data Log Status", "ERROR: Launch outcome not logged. Velocity must be stable (Y active for %.0f ms).", VELOCITY_SETTLE_TIME_MS);
                }

                // If a log event happened, we can consider resetting the motor if needed,
                // but usually, you want to manually turn it off with Y again.

                // Update state for next loop iteration
                prevAState = currentAState;
                prevBState = currentBState;
                // -----------------------------

                // Update telemetry on the Driver Station
                telemetry.update();

                idle();
            }
        }

        datalogger.close();
    }

    // --- Initializers ---

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void initDataLogger() {
        // Define all combined fields to be logged
        fTime = new Datalogger.GenericField("Timestamp_ms");
        fID = new Datalogger.GenericField("AprilTag_ID");
        fRange = new Datalogger.GenericField("Range_in");
        fBearing = new Datalogger.GenericField("Bearing_deg");
        fElevation = new Datalogger.GenericField("Elevation_deg");
        fDriveMotorPower = new Datalogger.GenericField("Avg_Drive_Motor_Power");
        fDriveMotorVelocity = new Datalogger.GenericField("Avg_Drive_Motor_Velocity_ticks");

        fLauncherCurrentVel = new Datalogger.GenericField("Launcher_Current_Velocity_ticks");
        fLauncherTargetVel = new Datalogger.GenericField("Launcher_Target_Velocity_ticks");
        fSuccessStatus = new Datalogger.GenericField("Success_Status"); // NEW LOGIC: 0, 1, or -1

        fBatteryVoltage = new Datalogger.GenericField("BatteryVoltage_V");
        fIsLaunching = new Datalogger.GenericField("Launcher_Motor_Active"); // Logs state of Y button

        // Build the Datalogger
        datalogger = new Datalogger.Builder()
                .setFilename("Localization_Launcher_Combined_Data") // New, descriptive filename
                .setFields(fTime, fID, fRange, fBearing, fElevation,
                        fDriveMotorPower, fDriveMotorVelocity, fBatteryVoltage,
                        fLauncherCurrentVel, fLauncherTargetVel, fSuccessStatus, fIsLaunching)
                .setAutoTimestamp(Datalogger.AutoTimestamp.NONE)
                .build();
    }

    // --- Control Handlers ---

    private void handleDriveControl() {
        // Mecanum Kinematics
        double drive = -gamepad1.left_stick_y; // Forward/Backward
        double strafe = gamepad1.left_stick_x;  // Strafe Left/Right
        double turn  = gamepad1.right_stick_x; // Rotation

        double leftFrontPower = drive + strafe + turn;
        double leftBackPower = drive - strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double rightBackPower = drive + strafe - turn;

        // Find the maximum power to ensure no motor power exceeds 1.0
        double maxPower = Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(leftBackPower),
                        Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));

        // Scale powers if necessary to keep them within the [-1.0, 1.0] range
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Apply power, multiplied by the overall DRIVE_SPEED constant
        motorLeftFront.setPower(leftFrontPower * DRIVE_SPEED);
        motorLeftBack.setPower(leftBackPower * DRIVE_SPEED);
        motorRightFront.setPower(rightFrontPower * DRIVE_SPEED);
        motorRightBack.setPower(rightBackPower * DRIVE_SPEED);

        // Log average motor power (average of the four absolute applied powers)
        double avgAppliedPower = (Math.abs(leftFrontPower) + Math.abs(leftBackPower) +
                Math.abs(rightFrontPower) + Math.abs(rightBackPower)) / 4.0;

        fDriveMotorPower.set("%.3f", avgAppliedPower * DRIVE_SPEED);
    }

    private void handleLauncherControl() {
        // --- 1. Adjust Target Velocity (Incremental D-pad) ---
        if (gamepad1.dpad_up) {
            currentTargetVelocity += VELOCITY_INCREMENT;
            sleep(100); // Debounce
        }
        if (gamepad1.dpad_down) {
            currentTargetVelocity -= VELOCITY_INCREMENT;
            sleep(100); // Debounce
        }
        currentTargetVelocity = Math.max(0, currentTargetVelocity); // Velocity cannot be negative

        // --- 2. Apply Target Velocity to Motor (The Launch Button: Y) ---
        boolean currentYState = gamepad1.y;

        if (currentYState && !prevYState) {
            // Rising edge: Start the motor and reset the timer
            launcherMotor.setVelocity(currentTargetVelocity);
            launchTimer.reset();
        } else if (!currentYState && prevYState) {
            // Falling edge: Stop the motor
            launcherMotor.setPower(0);
            isVelocityStable = false; // Reset stability flag when motor stops
        }

        // Hold Y to keep motor running at set velocity
        if (currentYState) {
            launcherMotor.setVelocity(currentTargetVelocity);
            fIsLaunching.set(1); // Launcher motor is actively spinning
        } else {
            launcherMotor.setPower(0);
            fIsLaunching.set(0); // Launcher motor is off
        }

        prevYState = currentYState;

        // --- 3. Update Launcher Fields ---
        fLauncherTargetVel.set("%.0f", currentTargetVelocity);
        fLauncherCurrentVel.set("%.0f", launcherMotor.getVelocity());

        telemetry.addData("TARGET VELOCITY", "%.0f Ticks/Sec (D-pad)", currentTargetVelocity);
        telemetry.addData("ACTUAL VELOCITY", "%.0f Ticks/Sec", launcherMotor.getVelocity());
    }


    // --- Data Logging Methods ---

    private void logAprilTagTelemetry() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Initialize fields to default/safe values
        fTime.set("%d", System.currentTimeMillis());
        fID.set("NONE");
        fRange.set(0.0);
        fBearing.set(0.0);
        fElevation.set(0.0);

        if (!currentDetections.isEmpty()) {
            AprilTagDetection bestDetection = null;
            double minRange = Double.MAX_VALUE;

            // Find the closest tag
            for (AprilTagDetection detection : currentDetections) {
                if (detection.ftcPose.range < minRange) {
                    minRange = detection.ftcPose.range;
                    bestDetection = detection;
                }
            }

            if (bestDetection != null) {
                // Log the key localization data
                fID.set(bestDetection.id);
                fRange.set("%.3f", bestDetection.ftcPose.range);
                fBearing.set("%.3f", bestDetection.ftcPose.bearing);
                fElevation.set("%.3f", bestDetection.ftcPose.elevation);

                telemetry.addData("Range to Tag", "%.2f in", bestDetection.ftcPose.range); // Telemetry for distance
            }
        } else {
            telemetry.addData("Range to Tag", "N/A (No Tag Detected)");
        }
    }

    private void logMotorVelocity() {
        // Log the average absolute velocity of the four drive motors
        double leftFrontVel = motorLeftFront.getVelocity();
        double leftBackVel = motorLeftBack.getVelocity();
        double rightFrontVel = motorRightFront.getVelocity();
        double rightBackVel = motorRightBack.getVelocity();

        double avgVelocity = (Math.abs(leftFrontVel) + Math.abs(leftBackVel) +
                Math.abs(rightFrontVel) + Math.abs(rightBackVel)) / 4.0;

        fDriveMotorVelocity.set("%.0f", avgVelocity);
    }

    private void logSystemVoltage() {
        double voltage = batterySensor.getVoltage();
        fBatteryVoltage.set("%.2f", voltage); // Log voltage to two decimal places
        telemetry.addData("Battery", "Voltage: %.2f V", voltage);
    }
}
