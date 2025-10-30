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
/// precise three-part process:
///  Velocity Control: gamepad1.dpad_up and gamepad1.dpad_down adjust the target velocity incrementally.
///  Motor Activation (setVelocity): gamepad1.y starts the launcher motor at the current target velocity.
///  Launch/Outcome Logging: gamepad1.a (Success) or gamepad1.b (Failure) logs the data, an resets the state.

package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * OpMode for collecting launcher data during tuning and competition.
 * It tracks launcher velocity, battery voltage, and AprilTag localization data
 * triggered by specific gamepad inputs.
 */
@TeleOp(name = "Launcher Data Collector", group = "Utility")
public class LauncherDataCollector extends LinearOpMode {

    private static final String TAG = LauncherDataCollector.class.getSimpleName();
    private final Datalog datalog = new Datalog(TAG);
    private ElapsedTime runtime = new ElapsedTime();

    // Launcher Constants
    private static final double VELOCITY_INCREMENT = 50.0; // Ticks/second change
    private static final double MAX_VELOCITY = 2000.0;
    private static final double MIN_VELOCITY = 0.0;

    // Hardware components
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;
    private DcMotorEx motorLauncher = null;

    private VoltageSensor batterySensor;

    // State Variables
    private double targetLauncherVelocity = 0.0;
    private boolean isLauncherActive = false;
    private boolean yPressed = false;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private int loopCounter = 0;

    // Vision
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private static final int APRIL_TAG_ID = 2; // Target AprilTag ID

    // =========================================================================================
    // Datalog Fields (Local variables now hold data)
    // =========================================================================================

    // Variables to hold the data for the current log line
    private String opModeStatus = "INIT";
    private String launchOutcome = "N/A";
    private double actualLauncherVelocity = 0.0;
    private double driveMotorVelocityAvg = 0.0;
    private double batteryVoltage = 0.0;
    private int aprilTagId = -1;
    private double aprilTagRange = 0.0;
    private double aprilTagBearing = 0.0;
    private double aprilTagElevation = 0.0;

    // =========================================================================================
    // OpMode Control
    // =========================================================================================

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        telemetry.addData("Status", "Hardware Initialized & Vision Ready");
        telemetry.addData("Target Velocity", "%.0f Ticks/s (Use D-Pad Up/Down)", targetLauncherVelocity);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            handleInput();
            mainLoop();
        }

        stopAllMotors();
        datalog.close();
    }

    // =========================================================================================
    // Initialization Methods
    // =========================================================================================

    private void initHardware() {
        try {
            motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
            motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
            motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
            motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");
            motorLauncher = hardwareMap.get(DcMotorEx.class, "motorLauncher");

            // Set motor directions (assuming a standard setup where left motors are reversed)
            motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
            motorRightFront.setDirection(DcMotor.Direction.FORWARD);
            motorRightBack.setDirection(DcMotor.Direction.FORWARD);

            // Set drive motor modes
            DcMotorEx[] driveMotors = {motorLeftFront, motorLeftBack, motorRightFront, motorRightBack};
            for (DcMotorEx motor : driveMotors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Set launcher motor mode
            motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Initialize the battery sensor using the standard FTC VoltageSensor interface
            batterySensor = hardwareMap.voltageSensor.iterator().next();

        } catch (Exception e) {
            telemetry.addData("Error", "Hardware initialization failed: " + e.getMessage());
            // If any critical motor fails, stop the OpMode.
            throw new RuntimeException("Hardware initialization failed", e);
        }
    }

    private void initVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                // REMOVED: setDecimation(2) which is no longer a valid method.
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
    }

    // =========================================================================================
    // Main Loop Logic
    // =========================================================================================

    private void handleInput() {
        // Velocity Adjustment (DPAD Up/Down)
        boolean currentDpadUp = gamepad1.dpad_up;
        if (currentDpadUp && !dpadUpPressed) {
            targetLauncherVelocity += VELOCITY_INCREMENT;
            if (targetLauncherVelocity > MAX_VELOCITY) targetLauncherVelocity = MAX_VELOCITY;
        }
        dpadUpPressed = currentDpadUp;

        boolean currentDpadDown = gamepad1.dpad_down;
        if (currentDpadDown && !dpadDownPressed) {
            targetLauncherVelocity -= VELOCITY_INCREMENT;
            if (targetLauncherVelocity < MIN_VELOCITY) targetLauncherVelocity = MIN_VELOCITY;
        }
        dpadDownPressed = currentDpadDown;

        // Launcher Activation (Y Button)
        boolean currentY = gamepad1.y;
        if (currentY && !yPressed) {
            isLauncherActive = !isLauncherActive;
            if (isLauncherActive) {
                motorLauncher.setVelocity(targetLauncherVelocity);
            } else {
                motorLauncher.setPower(0);
            }
        }
        yPressed = currentY;

        // Log Launch Outcome (A for Success, B for Failure)
        boolean currentA = gamepad1.a;
        if (currentA && !aPressed) {
            logLaunchOutcome("SUCCESS");
        }
        aPressed = currentA;

        boolean currentB = gamepad1.b;
        if (currentB && !bPressed) {
            logLaunchOutcome("FAILURE");
        }
        bPressed = currentB;

        // Telemetry Update
        telemetry.addData("Target Velocity", "%.0f Ticks/s", targetLauncherVelocity);
        telemetry.addData("Launcher Status", isLauncherActive ? "ACTIVE" : "IDLE");
        telemetry.addData("Actual Velocity", "%.0f Ticks/s", actualLauncherVelocity);
        telemetry.addData("Loop Count", loopCounter);
    }

    private void mainLoop() {
        // Set motor power based on gamepad sticks (for driving while logging)
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        motorLeftFront.setPower(frontLeftPower);
        motorRightFront.setPower(frontRightPower);
        motorLeftBack.setPower(backLeftPower);
        motorRightBack.setPower(backRightPower);

        // Update all logging variables
        opModeStatus = "RUNNING";
        logLauncherState();
        logApriltagLocalization();
        logMotorVelocity();
        logSystemVoltage();

        // Write to datalog
        datalog.log(
                opModeStatus,
                loopCounter,
                targetLauncherVelocity,
                actualLauncherVelocity,
                isLauncherActive ? "TRUE" : "FALSE",
                driveMotorVelocityAvg,
                batteryVoltage,
                launchOutcome,
                aprilTagId,
                aprilTagRange,
                aprilTagBearing,
                aprilTagElevation
        );
        loopCounter++; // Increment counter after logging

        telemetry.update();
    }

    private void logLaunchOutcome(String outcome) {
        // Set logging variables for final log entry
        opModeStatus = "LAUNCH_OUTCOME";
        launchOutcome = outcome;
        logLauncherState();
        logApriltagLocalization();
        logMotorVelocity();
        logSystemVoltage();

        // Write to datalog
        datalog.log(
                opModeStatus,
                loopCounter,
                targetLauncherVelocity,
                actualLauncherVelocity,
                isLauncherActive ? "TRUE" : "FALSE",
                driveMotorVelocityAvg,
                batteryVoltage,
                launchOutcome,
                aprilTagId,
                aprilTagRange,
                aprilTagBearing,
                aprilTagElevation
        );
        loopCounter++; // Increment counter after logging

        // Reset state after logging the outcome
        isLauncherActive = false;
        motorLauncher.setPower(0);
        launchOutcome = "N/A"; // Clear outcome field for subsequent logs
        telemetry.addData("Launch Logged", outcome);
        telemetry.update();
    }

    private void stopAllMotors() {
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLauncher.setPower(0);
    }

    // =========================================================================================
    // Logging Helper Methods - now only update local variables
    // =========================================================================================

    private void logLauncherState() {
        actualLauncherVelocity = motorLauncher.getVelocity();
    }

    private void logApriltagLocalization() {
        // Reset localization fields
        aprilTagId = -1;
        aprilTagRange = 0.0;
        aprilTagBearing = 0.0;
        aprilTagElevation = 0.0;

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        if (currentDetections.size() > 0) {
            AprilTagDetection bestDetection = null;
            double minRange = Double.MAX_VALUE;

            // Find the best detection (e.g., closest detection of the target ID)
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == APRIL_TAG_ID && detection.ftcPose.range < minRange) {
                    minRange = detection.ftcPose.range;
                    bestDetection = detection;
                }
            }

            if (bestDetection != null) {
                // Store the key localization data
                aprilTagId = bestDetection.id;
                aprilTagRange = bestDetection.ftcPose.range;
                aprilTagBearing = bestDetection.ftcPose.bearing;
                aprilTagElevation = bestDetection.ftcPose.elevation;

                telemetry.addData("Range to Tag", "%.2f in", bestDetection.ftcPose.range);
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

        driveMotorVelocityAvg = (Math.abs(leftFrontVel) + Math.abs(leftBackVel) +
                Math.abs(rightFrontVel) + Math.abs(rightBackVel)) / 4.0;
    }

    private void logSystemVoltage() {
        batteryVoltage = batterySensor.getVoltage();
        telemetry.addData("Battery", "Voltage: %.2f V", batteryVoltage);
    }

    // =========================================================================================
    // DATALOG Class Definition
    // =========================================================================================

    /**
     * This class encapsulates all the fields that will go into the datalog.
     * It uses the new, simplified Datalogger that requires manual String array formatting.
     */
    public static class Datalog implements AutoCloseable {
        private final Datalogger datalogger;

        // The headers, strictly maintaining the order for the log file
        private static final String[] HEADERS = new String[]{
                "OpModeStatus", "LoopCounter", "TargetVelocity", "ActualVelocity",
                "IsLauncherActive", "DriveVelocityAvg", "BatteryVoltage", "LaunchOutcome",
                "AprilTagID", "Range", "Bearing", "Elevation"
        };

        /**
         * @param name filename for output log
         */
        public Datalog(String name)
        {
            this.datalogger = new Datalogger(name, HEADERS);
        }

        /**
         * Logs a single line of data by manually formatting the values.
         */
        public void log(String opModeStatus, int loopCounter, double targetVel, double actualVel,
                        String isActive, double driveVel, double voltage, String outcome,
                        int id, double range, double bearing, double elevation) {

            // Manually construct the string array for the Datalogger.log() method, applying formatting here.
            datalogger.log(
                    opModeStatus,
                    String.valueOf(loopCounter),
                    String.format("%.0f", targetVel),
                    String.format("%.0f", actualVel),
                    isActive,
                    String.format("%.0f", driveVel),
                    String.format("%.2f", voltage),
                    outcome,
                    String.valueOf(id),
                    id != -1 ? String.format("%.3f", range) : "N/A",
                    id != -1 ? String.format("%.3f", bearing) : "N/A",
                    id != -1 ? String.format("%.3f", elevation) : "N/A"
            );
        }

        /**
         * Closes the datalogger.
         */
        @Override
        public void close() {
            datalogger.close();
        }
    }
}