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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.os.Environment; // <--- NEW IMPORT for external storage

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * OpMode for collecting empirical data on launcher velocity vs. distance (range).
 * Logs successful (1) or unsuccessful (0) launches to a CSV file.
 * Includes PIDF control for accurate velocity setting and logs battery voltage.
 *
 * Controls:
 * - Gamepad 1 D-Pad Up/Down: Adjust Target Velocity (Fine)
 * - Gamepad 1 Left/Right Bumper: Adjust Target Velocity (Coarse)
 * - Gamepad 1 B: LAUNCH sequence start (Spin Up, Fire, Reload)
 * - Gamepad 1 A: Mark Launch SUCCESS (1) and Log Data
 * - Gamepad 1 X: Mark Launch FAILURE (0) and Log Data
 * - Gamepad 1 Y: EXIT OpMode and Close Data Log
 */
@TeleOp(name = "Launcher Data Collector", group = "Test")
public class LauncherDataCollector extends LinearOpMode {

    // --- HARDWARE DECLARATION ---
    private DcMotorEx motorLauncherEx = null;
    private Servo servoGate = null;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private VoltageSensor primaryVoltageSensor = null;

    // --- LAUNCH PARAMETERS ---
    private static final int TARGET_TAG_ID = 20; // Specific AprilTag ID for the Goal
    private static final double DEFAULT_RANGE_INCHES = 36.0; // Range if Tag is lost
    private double targetVelocity = 1500.0; // Initial target velocity in ticks/sec
    private static final double FIRE_TIME = 0.5; // Time the gate servo is open
    private static final double RELOAD_TIME = 1.0; // Time to close the gate

    // PIDF Coefficients (PLACEHOLDER: These should be tuned for your specific motor)
    private static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(
            20.0,   // P (Proportional)
            2.0,    // I (Integral)
            0.0,    // D (Derivative)
            14.0    // F (Feedforward - usually proportional to max velocity)
    );

    // --- DATA LOGGING ---
    private DataLogger dataLogger;
    private int runId = 1;
    private int launchSuccessStatus = -1; // -1: Not logged, 0: Failure, 1: Success

    // --- STATE MANAGEMENT ---
    private enum LaunchState {
        IDLE,           // Waiting for velocity adjustment or launch command
        SPINNING_UP,    // Motor accelerating to targetVelocity
        FIRING,         // Gate is open
        RELOADING,      // Gate is closing
        WAITING_FOR_LOG // Waiting for A (Success) or X (Failure) input
    }
    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime launchTimer = new ElapsedTime();
    private ElapsedTime gamepadTimer = new ElapsedTime();
    private static final double INPUT_DEBOUNCE_TIME = 0.25; // Seconds to prevent double-press

    @Override
    public void runOpMode() {
        // The Telemetry object is resolved because of the new import
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addData("Status", "Initializing Data Collector");
        telemetry.update();

        // 1. Initialize Logger (Must be first to ensure file setup)
        dataLogger = new DataLogger("LauncherDataLog");

        // 2. Initialize Hardware
        initializeHardware();
        initializeVision();

        telemetry.addData("Status", "Initialization Complete. Ready to Start.");
        telemetry.addData("Target Tag", "ID " + TARGET_TAG_ID);
        telemetry.update();

        waitForStart();
        gamepadTimer.reset();

        if (opModeIsActive()) {
            dataLogger.writeHeader(); // Write CSV header after OpMode starts

            while (opModeIsActive()) {
                // --- TeleOp Loop ---
                handleVelocityAdjustment();
                handleLaunchSubsystem();
                handleResultLogging();
                sendTelemetryUpdates();

                // Check for graceful exit
                if (gamepad1.y && gamepadTimer.seconds() > INPUT_DEBOUNCE_TIME) {
                    telemetry.addData("Exiting", "Closing Data Log and OpMode...");
                    telemetry.update();
                    break;
                }

                // Small sleep to yield processing time
                sleep(20);
            }
        }

        // 3. Graceful Termination (Runs when loop breaks or opModeIsActive() becomes false)
        dataLogger.close();
        telemetry.addData("Status", "OpMode Ended. Log File Closed.");
        telemetry.update();
    }

    /**
     * Initializes hardware components, including setting up the motor's PIDF coefficients
     * for accurate velocity control.
     */
    private void initializeHardware() {
        try {
            motorLauncherEx = hardwareMap.get(DcMotorEx.class, "launcher_motor");
            servoGate = hardwareMap.get(Servo.class, "launcher_gate_servo");

            // Get the primary voltage sensor (usually the Control Hub's default)
            primaryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            motorLauncherEx.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLauncherEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLauncherEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // APPLY PIDF COEFFICIENTS for accurate velocity control
            motorLauncherEx.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

            servoGate.setPosition(0.0); // Closed position
            launchState = LaunchState.IDLE; // Set initial state only after init
        } catch (Exception e) {
            telemetry.addData("Error", "Hardware not found or incorrect configuration: " + e.getMessage());
            telemetry.update();
            launchState = LaunchState.IDLE; // Ensure we can still close log
        }
    }

    /**
     * Retrieves the current voltage from the primary voltage sensor.
     * @return Current battery voltage in volts.
     */
    private double getBatteryVoltage() {
        if (primaryVoltageSensor != null) {
            return primaryVoltageSensor.getVoltage();
        }
        return 0.0; // Return 0 if sensor failed to initialize
    }

    /**
     * Sets up the VisionPortal and the AprilTag Processor for distance tracking.
     */
    private void initializeVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        // Ensure streaming is resumed for continuous distance data
        visionPortal.resumeStreaming();
    }

    /**
     * Finds the target AprilTag (ID 20) and returns its range.
     * @return The range in inches, or DEFAULT_RANGE_INCHES if not found.
     */
    private double getTargetRange() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == TARGET_TAG_ID) {
                // Return the distance (range) to the tag from the camera
                return detection.ftcPose.range;
            }
        }
        return DEFAULT_RANGE_INCHES;
    }

    /**
     * Allows the operator to adjust the target velocity for the current test run.
     */
    private void handleVelocityAdjustment() {
        if (launchState != LaunchState.IDLE && launchState != LaunchState.SPINNING_UP) {
            return; // Only allow changes in IDLE or SPINNING_UP states
        }

        // Debounce all velocity controls
        if (gamepadTimer.seconds() < INPUT_DEBOUNCE_TIME) {
            return;
        }

        double change = 0.0;

        // Fine adjustments
        if (gamepad1.dpad_up) {
            change = 5.0;
        } else if (gamepad1.dpad_down) {
            change = -5.0;
        }

        // Coarse adjustments
        else if (gamepad1.right_bumper) {
            change = 50.0;
        } else if (gamepad1.left_bumper) {
            change = -50.0;
        }

        if (change != 0.0) {
            targetVelocity += change;
            targetVelocity = Math.max(0.0, targetVelocity); // Ensure velocity is non-negative

            // If currently spinning up, update target immediately
            if (launchState == LaunchState.SPINNING_UP) {
                motorLauncherEx.setVelocity(targetVelocity);
            }
            gamepadTimer.reset(); // Reset debounce timer
        }
    }


    /**
     * Manages the launch sequence state machine: SPIN -> FIRE -> RELOAD -> WAIT_FOR_LOG.
     */
    private void handleLaunchSubsystem() {
        switch (launchState) {
            case IDLE:
                motorLauncherEx.setVelocity(0.0);
                servoGate.setPosition(0.0);
                launchSuccessStatus = -1; // Reset status

                if (gamepad1.b) { // B to start spin-up
                    launchState = LaunchState.SPINNING_UP;
                    launchTimer.reset();
                    gamepadTimer.reset();
                }
                break;

            case SPINNING_UP:
                motorLauncherEx.setVelocity(targetVelocity);

                if (gamepad1.b && launchTimer.seconds() > 1.0) { // B to fire (with minimum spin time)
                    launchState = LaunchState.FIRING;
                    launchTimer.reset();
                }
                break;

            case FIRING:
                // Ensure velocity is maintained during fire
                motorLauncherEx.setVelocity(targetVelocity);
                servoGate.setPosition(1.0); // Open the gate
                if (launchTimer.seconds() >= FIRE_TIME) {
                    launchState = LaunchState.RELOADING;
                    launchTimer.reset();
                }
                break;

            case RELOADING:
                motorLauncherEx.setVelocity(targetVelocity); // Maintain velocity
                servoGate.setPosition(0.0); // Close the gate
                if (launchTimer.seconds() >= RELOAD_TIME) {
                    // Launch sequence complete, move to logging state
                    launchState = LaunchState.WAITING_FOR_LOG;
                    gamepadTimer.reset(); // Prepare for result logging
                }
                break;

            case WAITING_FOR_LOG:
                // Motor continues spinning here, ready for the next test if desired
                break;
        }
    }

    /**
     * Waits for operator input (A or X) to score the launch and log the result.
     */
    private void handleResultLogging() {
        if (launchState != LaunchState.WAITING_FOR_LOG) {
            return;
        }

        // Debounce input
        if (gamepadTimer.seconds() < INPUT_DEBOUNCE_TIME) {
            return;
        }

        int score = -1;

        if (gamepad1.a) {
            score = 1; // Success
            launchSuccessStatus = 1;
        } else if (gamepad1.x) {
            score = 0; // Failure
            launchSuccessStatus = 0;
        }

        if (score != -1) {
            double distance = getTargetRange();
            double voltage = getBatteryVoltage();

            dataLogger.writeLine(
                    runId,
                    targetVelocity,
                    distance,
                    voltage,
                    score,
                    System.currentTimeMillis()
            );

            // Reset state for the next run
            runId++;
            launchState = LaunchState.SPINNING_UP; // Immediately ready for next shot
            gamepadTimer.reset();
        }
    }

    /**
     * Updates the Driver Station telemetry with real-time data.
     */
    private void sendTelemetryUpdates() {
        double currentRange = getTargetRange();
        double currentVelocity = motorLauncherEx.getVelocity();
        double voltage = getBatteryVoltage();

        telemetry.addData("Run ID", runId);
        telemetry.addData("State", launchState);

        telemetry.addLine()
                .addData("Target Vel (ticks/s)", "%.0f", targetVelocity)
                .addData("Current Vel", "%.0f", currentVelocity);

        telemetry.addLine()
                .addData("Tag Range", "%.2f in", currentRange)
                .addData("Battery Voltage", "%.2f V", voltage);

        String statusMessage = "";
        if (launchState == LaunchState.WAITING_FOR_LOG) {
            statusMessage = "PRESS A (SUCCESS: 1) or X (FAILURE: 0)";
        } else if (launchSuccessStatus != -1) {
            statusMessage = "Last Score: " + (launchSuccessStatus == 1 ? "SUCCESS" : "FAILURE");
        } else if (launchState == LaunchState.SPINNING_UP) {
            statusMessage = "Press B to FIRE!";
        } else if (launchState == LaunchState.IDLE) {
            statusMessage = "Press B to Begin Spin-Up.";
        }

        telemetry.addLine().addData("SCORE INPUT", statusMessage);
        telemetry.addData("LOG FILE", dataLogger.getFilePath());
        telemetry.addData("EXIT", "Press Y to close log and exit.");
        telemetry.update();
    }

    /**
     * DataLogger utility class for simple CSV writing.
     * Log files are saved to the Robot Controller's internal storage in the FIRST/data/ folder.
     */
    private static class DataLogger {
        private FileWriter fileWriter;
        private final String logFileName;
        private static final String DATA_DIR = "FIRST/data"; // Standard FTC log folder

        public DataLogger(String baseName) {
            // Create a unique file name with timestamp
            String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            logFileName = String.format("%s_%s.csv", baseName, timeStamp);

            try {
                // Construct the directory path using the standard Android external storage root
                File externalStorageDir = Environment.getExternalStorageDirectory();
                File directory = new File(externalStorageDir, DATA_DIR);

                // Ensure the directory exists
                if (!directory.exists()) {
                    directory.mkdirs();
                }

                // Create the file within the new directory
                File logFile = new File(directory, logFileName);
                fileWriter = new FileWriter(logFile);

            } catch (IOException e) {
                System.err.println("ERROR: Failed to open data log file: " + logFileName + ". " + e.getMessage());
                fileWriter = null;
            } catch (SecurityException e) {
                // Catch exception if file permissions are incorrect (less common in FTC)
                System.err.println("ERROR: Insufficient permissions to access external storage: " + e.getMessage());
                fileWriter = null;
            }
        }

        public String getFilePath() {
            // Return the relative path for easy user lookup on the phone
            return "/" + DATA_DIR + "/" + logFileName;
        }

        public void writeHeader() {
            if (fileWriter != null) {
                try {
                    // UPDATED HEADER to include BatteryVoltage_V
                    String header = "Timestamp_ms,RunID,TargetVelocity_ticks_per_s,Range_inches,BatteryVoltage_V,Success_Status\n";
                    fileWriter.write(header);
                    fileWriter.flush(); // Ensure header is written immediately
                } catch (IOException e) {
                    System.err.println("ERROR: Failed to write header to log file.");
                }
            }
        }

        // UPDATED METHOD SIGNATURE to include voltage
        public void writeLine(int id, double velocity, double range, double voltage, int status, long timestamp) {
            if (fileWriter != null) {
                try {
                    // UPDATED FORMATTING to include %.2f for voltage
                    String line = String.format(Locale.US, "%d,%d,%.2f,%.2f,%.2f,%d\n",
                            timestamp, id, velocity, range, voltage, status);
                    fileWriter.write(line);
                    // No flush() here to improve performance; rely on close()
                } catch (IOException e) {
                    System.err.println("ERROR: Failed to write data line to log file.");
                }
            }
        }

        public void close() {
            if (fileWriter != null) {
                try {
                    fileWriter.close();
                } catch (IOException e) {
                    System.err.println("ERROR: Failed to close data log file.");
                }
            }
        }
    }
}
