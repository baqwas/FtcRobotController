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
import com.qualcomm.robotcore.hardware.Servo;
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
 * {@link TeleOpPreviewEvent} is a basic TeleOp OpMode for the DECODE 2025-2026 season.
 * This OpMode implements Field-Centric Mecanum drive and auxiliary controls.
 * Hardware: Artifact Intake (Servo), Launcher (DcMotorEx), End-game Slides (DcMotorEx).
 */
@TeleOp(name = "Preview Event TeleOp", group = "Match")
public class TeleOpPreviewEvent extends LinearOpMode {

    // --- HARDWARE DECLARATIONS ---
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;
    private IMU imu = null;

    // --- DECODE 2025-2026 AUXILIARY HARDWARE DECLARATIONS ---
    // Artifact Intake is now a Servo
    private Servo servoIntake = null;
    // Launcher is now a DcMotorEx
    private DcMotorEx motorLauncher = null;
    // End-game Slides/Elevation is now a DcMotorEx
    private DcMotorEx motorSlides = null;

    // --- VISION DECLARATIONS ---
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private boolean isAutoNavActive = false;
    private final int TARGET_TAG_ID = 20; // Example target tag

    // --- CONTROLS AND STATE ---
    private ElapsedTime runtime = new ElapsedTime();
    private String selectedAlliance = "RED"; // Default to RED
    private boolean aAlreadyPressed = false;
    private double launcherPower = 0.0; // State variable for the launcher motor

    // --- DRIVE CONSTANTS ---
    private final double DRIVE_SPEED = 0.5;
    private final double TURN_SPEED = 0.3;

    // --- AUXILIARY CONSTANTS ---
    // Servo positions for the Artifact Intake
    private final double INTAKE_OPEN_POS = 0.8;
    private final double INTAKE_CLOSED_POS = 0.2;
    // Launcher Motor power (for a fast, short launch)
    private final double LAUNCHER_FIRE_POWER = 1.0;
    private final double SLIDES_MAX_POWER = 0.75; // Limit slides speed for control

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
        telemetry.addData("Alliance", "Current: RED (Press A or Cross (X) to switch)");
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

            // Handle the new auxiliary systems for DECODE
            handleAuxiliarySystems();

            // Final update for Driver Station
            sendTelemetryUpdates();
        }
    }

    /**
     * Initializes all motors and sets their directions and modes.
     */
    private void initializeHardware() {
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

        // Set direction: FTC uses a convention where left motors are typically reversed
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set modes
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- AUXILIARY HARDWARE INITIALIZATION (DECODE 2025-2026) ---
        try {
            // Artifact Intake (Servo)
            servoIntake = hardwareMap.get(Servo.class, "servoIntake");
            servoIntake.setPosition(INTAKE_CLOSED_POS); // Start in the closed/safe position

            // Launcher (DcMotorEx)
            motorLauncher = hardwareMap.get(DcMotorEx.class, "motorlauncher");
            motorLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use RUN_WITHOUT_ENCODER for launching

            // End-game Slides/Elevation (DcMotorEx)
            motorSlides = hardwareMap.get(DcMotorEx.class, "motorSlides");
            motorSlides.setDirection(DcMotorSimple.Direction.FORWARD);
            motorSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Simple power control for TeleOp

        } catch (Exception e) {
            telemetry.addData("WARN", "Auxiliary hardware not found. Check config for artifact_intake_servo, launcher_motor, slides_motor.");
        }
    }

    /**
     * Initializes the IMU (Inertial Measurement Unit) with the robot's orientation.
     */
    private void initializeIMU() {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

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

        // Create the Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
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
            telemetry.addData("Alliance", "Selected: " + selectedAlliance + " (Press A or Cross (X) to switch)");
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
            telemetry.addData("AutoNav", "Target Tag ID " + TARGET_TAG_ID + " Not Found!");
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
            motorRightFront.setPower(rangeDrive + strafeDrive - turnDrive);
            motorRightBack.setPower(rangeDrive - strafeDrive - turnDrive);

            telemetry.addData("AutoNav", "Tracking Goal Tag...");
            telemetry.addData("Range/Bearing/Yaw Err", "%.1f, %.1f, %.1f", rangeError, bearingError, yawError);
            telemetry.addData("Range/Bearing/Elev", "%.1f, %.1f, %.1f", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);
        } else {
            stopDriveMotors();
            isAutoNavActive = false;
            telemetry.addData("AutoNav", "Reached Target Position!");
        }
    }

    /**
     * Handles control logic for auxiliary systems (Intake, Launcher, Slides) using Gamepad 2.
     * Uses the requested hardware types and control mappings.
     */
    private void handleAuxiliarySystems() {
        if (servoIntake == null || motorLauncher == null || motorSlides == null) {
            // Safety check in case hardware mapping failed
            return;
        }

        // --- ARTIFACT INTAKE CONTROL (GAMEPAD 2 BUMPERS) ---
        // Right Bumper (R1/RB): Open Intake Position (to accept Artifacts)
        // Left Bumper (L1/LB): Closed Intake Position (to hold/secure Artifacts)
        if (gamepad2.right_bumper) {
            servoIntake.setPosition(INTAKE_OPEN_POS);
        } else if (gamepad2.left_bumper) {
            servoIntake.setPosition(INTAKE_CLOSED_POS);
        }

        // --- END-GAME SLIDES CONTROL (GAMEPAD 2 LEFT STICK Y-AXIS) ---
        // Left Stick Y controls elevation proportionally.
        // Y-axis is negated: stick UP (-1.0) should correspond to slides UP (positive power)
        double slidesPower = -gamepad2.left_stick_y;

        // Apply max power limit
        slidesPower = Math.max(-SLIDES_MAX_POWER, Math.min(slidesPower, SLIDES_MAX_POWER));

        motorSlides.setPower(slidesPower);


        // --- LAUNCHER CONTROL (GAMEPAD 2 Y BUTTON) ---
        // Y/Triangle button fires the motor launcher at max speed, only while held.
        if (gamepad2.y) {
            launcherPower = LAUNCHER_FIRE_POWER; // Set launcher motor to high power
        } else {
            launcherPower = 0.0; // Stop motor when button is released
        }

        motorLauncher.setPower(launcherPower);
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
        telemetry.addData("AutoNav", isAutoNavActive ? "ACTIVE (Tracking Tag " + TARGET_TAG_ID + ")" : "MANUAL");

        // DECODE AUXILIARY SYSTEM TELEMETRY
        // Fixed: Use getPosition() for Servo, not getPower()
        telemetry.addData("Intake Servo Pos", servoIntake != null ? servoIntake.getPosition() : "N/A");

        // Fixed: Ensure the correct motor objects are checked/used for power
        telemetry.addData("Launcher Power", motorLauncher != null ? motorLauncher.getPower() : "N/A");
        telemetry.addData("Slides Power", motorSlides != null ? motorSlides.getPower() : "N/A");

        telemetry.update();
    }
}
