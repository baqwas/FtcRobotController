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

// Package name updated to match your error path
package org.firstinspires.ftc.teamcode.Match.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;

// --- START: NEW IMPORTS FOR DATE/TIME ---
import java.text.SimpleDateFormat;
import java.util.Date;
// --- END: NEW IMPORTS FOR DATE/TIME ---
import java.util.List;

/**
 * OpMode for driving a Mecanum robot towards a specific AprilTag using P-Control,
 * while logging all movement and localization data.
 */
// Class name updated to match your error path
@Autonomous(name = "Log Drive to AprilTag", group = "Match")
@Disabled
public class LogDriveToAprilTag extends LinearOpMode {

    private static final String TAG = "AprilTagDriveTest";

    // --- AprilTag Control Constants ---
    private static final int APRIL_TAG_ID = 20;              // Target AprilTag ID (change as needed)
    private static final double DESIRED_DISTANCE = 18.0;      // Target distance (inches) from tag
    private static final double SPEED_GAIN = 0.02;           // Proportional gain for Forward/Backward
    private static final double STRAFE_GAIN = 0.015;         // Proportional gain for Strafe Left/Right (Bearing)
    private static final double TURN_GAIN = 0.015;           // Proportional gain for Turning (Yaw/Bearing)
    private static final double MAX_POWER = 0.6;             // Max absolute power limit for motors
    private static final double HEADING_TOLERANCE = 1.0;     // Angular tolerance (degrees)
    private static final double RANGE_TOLERANCE = 0.5;       // Distance tolerance (inches)

    // Hardware components
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;
    private VoltageSensor batterySensor;

    // Vision components
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Logging
    // --- START: MODIFIED DATALOG DECLARATION ---
    // Change to initialize in runOpMode() instead of here
    private Datalog datalog = null;
    // --- END: MODIFIED DATALOG DECLARATION ---
    private int loopCounter = 0;
    private String opModeStatus = "INIT";
    private String fsmState = "INIT";

    // =========================================================================================
    // OpMode Control
    // =========================================================================================

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        // --- START: NEW DATE/TIMESTAMP LOGIC ---
        // 1. Define the desired date and time format for the filename.
        // 'yyyyMMdd_HHmmss' formats to (e.g.) 20251030_171500 (YearMonthDay_HourMinuteSecond)
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");

        // 2. Get the current system date and time.
        String timestamp = dateFormat.format(new Date());

        // 3. Define the base filename and append the timestamp.
        String baseFileName = TAG;
        String datalogFilename = baseFileName + "_" + timestamp;

        // 4. Instantiate the Datalog object with the unique filename
        datalog = new Datalog(datalogFilename);
        // --- END: NEW DATE/TIMESTAMP LOGIC ---

        telemetry.addData("Status", "Hardware Initialized & Vision Ready");
        telemetry.addData("Log File", datalogFilename + ".csv"); // Show the generated filename
        telemetry.addData("Target Tag", APRIL_TAG_ID);
        telemetry.addData("Target Range", "%.1f in", DESIRED_DISTANCE);
        telemetry.update();

        // Log the initial state
        logData("INIT", "WAIT_FOR_START", 0, 0, 0);

        waitForStart();

        opModeStatus = "RUNNING";
        fsmState = "SEARCHING";

        // Main logic loop: Drive to the specified tag
        driveToTag(APRIL_TAG_ID);

        // Final state log
        opModeStatus = "STOPPED";
        fsmState = "COMPLETE";
        logData(opModeStatus, fsmState, 0, 0, 0);

        stopAllMotors();
        datalog.close(); // CRITICAL: Close the datalogger to save data
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

            // Set motor directions
            motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
            motorRightFront.setDirection(DcMotor.Direction.FORWARD);
            motorRightBack.setDirection(DcMotor.Direction.FORWARD);

            DcMotorEx[] allMotors = {motorLeftFront, motorLeftBack, motorRightFront, motorRightBack};
            for (DcMotorEx motor : allMotors) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use speed-based driving
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            batterySensor = hardwareMap.voltageSensor.iterator().next();

        } catch (Exception e) {
            telemetry.addData("Error", "Hardware initialization failed: " + e.getMessage());
            throw new RuntimeException("Hardware initialization failed", e);
        }
    }

    private void initVision() {
        // 1. Create the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // 2. Create the Vision Portal, specifying the camera and passing the processor
        visionPortal = new VisionPortal.Builder()
                // Must explicitly set the camera using the standard configuration name
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    // =========================================================================================
    // AprilTag Navigation Logic
    // =========================================================================================

    private void driveToTag(int targetTagId) {
        AprilTagDetection targetDetection = null;

        fsmState = "DRIVING_TO_TAG";

        while (opModeIsActive()) {
            targetDetection = getTargetDetection(targetTagId);

            if (targetDetection != null) {
                // Determine errors
                double rangeError = (targetDetection.ftcPose.range - DESIRED_DISTANCE);
                double bearingError = targetDetection.ftcPose.bearing; // Lateral error
                double turnError = targetDetection.ftcPose.yaw; // Angular error

                // --- P-Control Calculations ---

                // 1. Forward/Backward power (from range)
                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_POWER, MAX_POWER);

                // 2. Strafe power (from bearing)
                double strafe = Range.clip(bearingError * STRAFE_GAIN, -MAX_POWER, MAX_POWER);

                // 3. Turn power (P-control on yaw/bearing)
                double turn = Range.clip(turnError * TURN_GAIN, -MAX_POWER, MAX_POWER);

                // Use bearing for rotation if yaw is near zero or unset
                if (Math.abs(turnError) < 0.001) {
                    turn = Range.clip(bearingError * TURN_GAIN, -MAX_POWER, MAX_POWER);
                }

                // --- Calculate Mecanum Motor Powers ---
                double leftFrontPower = drive + strafe + turn;
                double rightFrontPower = drive - strafe - turn;
                double leftBackPower = drive - strafe + turn;
                double rightBackPower = drive + strafe - turn;

                // Scale powers to prevent exceeding MAX_POWER
                double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > MAX_POWER) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // Apply powers
                motorLeftFront.setPower(leftFrontPower);
                motorRightFront.setPower(rightFrontPower);
                motorLeftBack.setPower(leftBackPower);
                motorRightBack.setPower(rightBackPower);

                // Log data and telemetry
                logData(opModeStatus, fsmState, drive, strafe, turn);

                // Check for completion
                if (Math.abs(rangeError) < RANGE_TOLERANCE && Math.abs(bearingError) < HEADING_TOLERANCE) {
                    telemetry.addData("FSM State", "TARGET_REACHED");
                    stopAllMotors();
                    return; // Exit the loop and finish the OpMode
                }

            } else {
                // Tag not detected, stop motion or perform search routine
                motorLeftFront.setPower(0);
                motorRightFront.setPower(0);
                motorLeftBack.setPower(0);
                motorRightBack.setPower(0);

                fsmState = "SEARCHING";
                logData(opModeStatus, fsmState, 0, 0, 0);
            }

            telemetry.update();
            idle(); // Yield time to other FTC system threads
        }
    }

    private AprilTagDetection getTargetDetection(int targetTagId) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetTagId) {
                return detection;
            }
        }
        return null; // Tag not found
    }

    private void stopAllMotors() {
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
    }

    // =========================================================================================
    // Logging Method
    // =========================================================================================

    private void logData(String status, String state, double drivePwr, double strafePwr, double turnPwr) {
        // Get the current OpMode runtime
        double time = getRuntime(); // <-- MODIFIED: Get runtime here

        // Collect AprilTag Data
        int aprilTagId = -1;
        double aprilTagRange = 0.0;
        double aprilTagBearing = 0.0;
        double aprilTagElevation = 0.0;
        double aprilTagYaw = 0.0;

        AprilTagDetection targetDetection = getTargetDetection(APRIL_TAG_ID);

        if (targetDetection != null) {
            aprilTagId = targetDetection.id;
            aprilTagRange = targetDetection.ftcPose.range;
            aprilTagBearing = targetDetection.ftcPose.bearing;
            aprilTagElevation = targetDetection.ftcPose.elevation;
            aprilTagYaw = targetDetection.ftcPose.yaw;
            telemetry.addData("Tag Range/Bearing/Yaw", "%.1f / %.1f / %.1f", aprilTagRange, aprilTagBearing, aprilTagYaw);
        } else {
            // If tag is not found, these variables remain 0.0, which is logged to the CSV
            telemetry.addData("Tag Status", "NOT FOUND");
        }

        // Log to file - Pass 'time' as the first argument
        datalog.log(
                time, // <-- MODIFIED: This is Field 1: Time(s)
                status,
                state,
                (int)APRIL_TAG_ID,
                aprilTagId,
                aprilTagRange,
                aprilTagBearing,
                aprilTagElevation,
                aprilTagYaw,
                motorLeftFront.getCurrentPosition(),
                motorRightFront.getCurrentPosition(),
                motorLeftBack.getCurrentPosition(),
                motorRightBack.getCurrentPosition(),
                drivePwr,
                strafePwr,
                turnPwr,
                batterySensor.getVoltage()
        );
        loopCounter++;
    }

    // =========================================================================================
    // DATALOG Class Definition (Assuming Datalogger.java exists in org.firstinspires.ftc.teamcode.Utility)
    // =========================================================================================

    /**
     * This class encapsulates all the fields that will go into the datalog.
     * It uses the simplified Datalogger that requires manual String array formatting.
     */
    public static class Datalog implements AutoCloseable {
        private final Datalogger datalogger;

        // The headers, strictly maintaining the order for the log file
        private static final String[] HEADERS = new String[]{
                "Time(s)", // <-- MODIFIED: Added timestamp header
                "OpModeStatus",
                "FSMState",
                "TargetTagID", "DetectedTagID",
                "Range", "Bearing", "Elevation", "Yaw",
                "LFPos", "RFPos", "LBPos", "RBPos",
                "DrivePwr", "StrafePwr", "TurnPwr", "BatteryVoltage"
        };

        /**
         * @param name filename for output log
         */
        public Datalog(String name)
        {
            // The Datalogger constructor handles file creation and header writing.
            this.datalogger = new Datalogger(name, HEADERS);
        }

        /**
         * Logs a single line of data by manually formatting the values.
         */
        public void log(
                double time, // <-- MODIFIED: Added time argument (1st field)
                String opModeStatus, // 2nd field
                String fsmState, // 3rd field
                int targetId, // 4th field
                int detectedId, // 5th field
                double range, double bearing, double elevation, double yaw, // 6th - 9th fields
                int lfPos, int rfPos, int lbPos, int rbPos, // 10th - 13th fields
                double drivePwr, double strafePwr, double turnPwr, // 14th - 16th fields
                double voltage) { // 17th field

            String detectedIdStr = (detectedId != -1) ? String.valueOf(detectedId) : "N/A";

            // Manually construct the string array for the Datalogger.log() method, applying formatting here.
            // **CRITICAL: The order below MUST match the order in the HEADERS array.**
            datalogger.log(
                    String.format("%.3f", time), // <-- MODIFIED: Log formatted time
                    opModeStatus,
                    fsmState,
                    String.valueOf(targetId),
                    detectedIdStr,
                    (detectedId != -1) ? String.format("%.3f", range) : "N/A",
                    (detectedId != -1) ? String.format("%.3f", bearing) : "N/A",
                    (detectedId != -1) ? String.format("%.3f", elevation) : "N/A",
                    (detectedId != -1) ? String.format("%.3f", yaw) : "N/A",
                    String.valueOf(lfPos),
                    String.valueOf(rfPos),
                    String.valueOf(lbPos),
                    String.valueOf(rbPos),
                    String.format("%.3f", drivePwr),
                    String.format("%.3f", strafePwr),
                    String.format("%.3f", turnPwr),
                    String.format("%.2f", voltage)
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