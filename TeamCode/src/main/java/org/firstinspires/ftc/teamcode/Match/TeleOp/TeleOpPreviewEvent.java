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

package org.firstinspires.ftc.teamcode.Match.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// IMU specific imports
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

// AprilTag specific imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

/**
 * {@link TeleOpPreviewEvent} is a basic TeleOp OpMode used for initial robot testing.
 * This class serves as a template and a guide for how to properly document code.
 */
@TeleOp(name = "TeleOp Preview Event", group = "Match")
public class TeleOpPreviewEvent extends LinearOpMode {

    // --- HARDWARE DECLARATIONS ---
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;
    private IMU imu = null;

    // --- VISION DECLARATIONS ---
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private boolean isAutoNavActive = false;
    private final int TARGET_TAG_ID = 20; // Example target tag

    // --- CONTROLS AND STATE ---
    private ElapsedTime runtime = new ElapsedTime();
    private String selectedAlliance = "RED"; // Default to RED
    private boolean aAlreadyPressed = false;

    // --- DRIVE CONSTANTS ---
    private final double DRIVE_SPEED = 0.5;
    private final double TURN_SPEED = 0.3;

    @Override
    public void runOpMode() {
        // --- 1. INITIALIZATION ---
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();
        initializeIMU();
        initializeVision();

        telemetry.addData("Status", "Initialization Complete");
        telemetry.addData("Alliance", "Current: RED (Press A to switch)");
        telemetry.update();

        // --- 2. WAITING FOR START ---
        while (opModeInInit()) {
            handleAllianceSelection();
            idle(); // Yield time to other system tasks
        }

        // --- 3. RUNTIME ---
        runtime.reset();

        // Loop runs until OpMode is stopped
        while (opModeIsActive()) {
            // Check if Auto-Navigation is running
            if (isAutoNavActive) {
                runAutoNavigation();
            } else {
                // If not in AutoNav, run manual Mecanum drive
                handleMecanumDrive();
                handleAutoNavActivation();
            }

            // Other subsystems (e.g., intake, arm) could be handled here
            handleAuxiliarySystems();

            // Final update for Driver Station
            sendTelemetryUpdates();
        }
    }

    /**
     * Initializes all motors and sets their directions and modes.
     */
    private void initializeHardware() {
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "right_back");

        // Set direction: FTC uses a convention where left motors are typically reversed
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set modes
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initializes the IMU (Inertial Measurement Unit) with the robot's orientation.
     */
    private void initializeIMU() {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        // --- RESOLUTION FOR ERROR 1: RevHubOrientationOnRobot Constructor ---
        // Configuration based on user input:
        // RobotController is mounted with the **logo facing forward** and the **USB port facing upward**.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Build the orientation configuration
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU with the specified orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Reset the Yaw to 0 degrees at the start of the OpMode
        imu.resetYaw();
    }


    /**
     * Initializes the VisionPortal and the AprilTag Processor.
     */
    private void initializeVision() {
        // Create the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // Standard FTC family
                .build();

        // --- RESOLUTION FOR ERROR 2: get10dofLibrary() ---
        // The method 'get10dofLibrary' is not part of the standard AprilTagProcessor.Builder() flow.
        // The processor automatically uses the necessary libraries internally.
        // We ensure no attempt is made to call this non-existent method by simply removing it.

        // Create the Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                // Disable telemetry from the VisionPortal itself to clean up the DS screen
                .setTelemetry(telemetry)
                .build();
    }

    /**
     * Allows the user to select the alliance color before the start.
     */
    private void handleAllianceSelection() {
        if (gamepad1.a && !aAlreadyPressed) {
            if (selectedAlliance.equals("RED")) {
                selectedAlliance = "BLUE";
            } else {
                selectedAlliance = "RED";
            }
            aAlreadyPressed = true;
            telemetry.addData("Alliance", "Selected: " + selectedAlliance + " (Press A to switch)");
            telemetry.update();
        } else if (!gamepad1.a) {
            aAlreadyPressed = false;
        }
    }

    /**
     * Implements standard field-centric Mecanum drive control.
     */
    private void handleMecanumDrive() {
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED; // Forward/Backward
        double strafe = gamepad1.left_stick_x * DRIVE_SPEED;  // Strafe Left/Right
        double turn = gamepad1.right_stick_x * TURN_SPEED;   // Turn Left/Right

        // Get robot's current yaw (heading)
        double botYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Field-centric conversion (rotate the movement vector by the negative angle of the bot's yaw)
        double cosYaw = Math.cos(Math.toRadians(-botYaw));
        double sinYaw = Math.sin(Math.toRadians(-botYaw));

        double tempDrive = drive * cosYaw - strafe * sinYaw;
        double tempStrafe = drive * sinYaw + strafe * cosYaw;

        drive = tempDrive;
        strafe = tempStrafe;

        // Calculate motor powers
        motorLeftFront.setPower(drive + strafe + turn);
        motorLeftBack.setPower(drive - strafe + turn);
        motorRightFront.setPower(drive - strafe - turn);
        motorRightBack.setPower(drive + strafe - turn);
    }

    /**
     * Checks if the operator wants to switch to Auto Navigation mode (AprilTag tracking).
     */
    private void handleAutoNavActivation() {
        // Example: Press Y to initiate AprilTag tracking toward the target tag
        if (gamepad1.y) {
            isAutoNavActive = true;
        }
    }

    /**
     * Executes the AprilTag Auto Navigation (simplified Proportional control).
     * The robot drives toward the target tag (ID 20) until it is close.
     */
    private void runAutoNavigation() {
        AprilTagDetection detection = null;
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        // 1. Find the target tag
        for (AprilTagDetection d : currentDetections) {
            if (d.id == TARGET_TAG_ID) {
                detection = d;
                break;
            }
        }

        if (detection == null) {
            telemetry.addData("AutoNav Status", "Target Tag ID " + TARGET_TAG_ID + " Not Found!");
            stopDriveMotors();
            return;
        }

        // 2. Proportional Control based on detection
        double Kp_RANGE = 0.05; // Drive forward/backward
        double Kp_BEARING = 0.02; // Strafe left/right
        double Kp_YAW = 0.015; // Turn

        double targetRange = 10.0; // Target distance in inches from the tag

        // Errors
        double rangeError = detection.ftcPose.range - targetRange; // Distance error
        double bearingError = detection.ftcPose.bearing; // Strafe error (sideways offset)
        double yawError = detection.ftcPose.yaw; // Angle error (turning error)

        // Drive Powers (P-Control only)
        double rangeDrive = rangeError * Kp_RANGE;
        double strafeDrive = -bearingError * Kp_BEARING; // Negative bearing to center
        double turnDrive = -yawError * Kp_YAW; // Negative yaw to straighten

        // Limit drive powers
        rangeDrive = Math.max(-DRIVE_SPEED, Math.min(rangeDrive, DRIVE_SPEED));
        strafeDrive = Math.max(-DRIVE_SPEED, Math.min(strafeDrive, DRIVE_SPEED));
        turnDrive = Math.max(-TURN_SPEED, Math.min(turnDrive, TURN_SPEED));

        // 3. Apply power if outside tolerance
        if (Math.abs(rangeError) > 0.5) { // Stop when within 0.5 inches
            motorLeftFront.setPower(rangeDrive + strafeDrive + turnDrive);
            motorLeftBack.setPower(rangeDrive - strafeDrive + turnDrive);
            motorRightFront.setPower(rangeDrive + straafeDrive - turnDrive);
            motorRightBack.setPower(rangeDrive - strafeDrive - turnDrive);

            telemetry.addData("AutoNav Status", "Tracking Goal Tag...");
            telemetry.addData("Range/Bearing/Yaw Err", "%.1f, %.1f, %.1f", rangeError, bearingError, yawError);
            telemetry.addData("Range/Bearing/Elev", "%.1f, %.1f, %.1f", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);
        } else {
            stopDriveMotors();
            isAutoNavActive = false;
            telemetry.addData("AutoNav Status", "Reached Target Position!");
        }
    }

    /**
     * Placeholder for any auxiliary systems like intake, arm, or drone launcher.
     */
    private void handleAuxiliarySystems() {
        // Example: If gamepad2.right_bumper, run intake motor
        // Example: If gamepad2.left_bumper, raise arm
    }

    /**
     * Helper method to stop all drive motors.
     */
    private void stopDriveMotors() {
        setMotorPower(0);
    }

    /**
     * Helper method to set all drive motor powers simultaneously.
     */
    private void setMotorPower(double power) {
        motorLeftFront.setPower(power);
        motorLeftBack.setPower(power);
        motorRightFront.setPower(power);
        motorRightBack.setPower(power);
    }

    /**
     * Helper method to set all drive motor run modes simultaneously.
     */
    private void setMotorRunMode(DcMotor.RunMode mode) {
        motorLeftFront.setMode(mode);
        motorLeftBack.setMode(mode);
        motorRightFront.setMode(mode);
        motorRightBack.setMode(mode);
    }

    /**
     * Helper method to set all drive motor zero power behaviors simultaneously.
     */
    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motorLeftFront.setZeroPowerBehavior(behavior);
        motorLeftBack.setZeroPowerBehavior(behavior);
        motorRightFront.setZeroPowerBehavior(behavior);
        motorRightBack.setZeroPowerBehavior(behavior);
    }


    /**
     * Final subsystem for all telemetry updates.
     */
    private void sendTelemetryUpdates() {
        telemetry.addData("Runtime", runtime.toString());
        telemetry.addData("Yaw (Field)", "%.1f deg", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("AutoNav Mode", isAutoNavActive ? "ACTIVE (Tracking Tag " + TARGET_TAG_ID + ")" : "MANUAL");
        telemetry.update();
    }
}
