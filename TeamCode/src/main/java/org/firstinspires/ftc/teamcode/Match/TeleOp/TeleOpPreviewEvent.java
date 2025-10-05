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
 * This OpMode implements Field-Centric Mecanum drive and auxiliary controls, including a
 * quadratic function for fly-by launch velocity calculation.
 */
@TeleOp(name = "Preview Event TeleOp", group = "Match")
//@Disabled
public class TeleOpPreviewEvent extends LinearOpMode {

    // --- HARDWARE DECLARATIONS ---
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;
    private IMU imu = null;

    // --- DECODE 2025-2026 AUXILIARY HARDWARE DECLARATIONS ---
    private Servo servoIntake = null;
    private DcMotorEx motorLauncher = null;
    private DcMotorEx motorSlides = null;

    // --- VISION DECLARATIONS ---
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private boolean isAutoNavActive = false;
    private final int TARGET_TAG_ID = 20; // Example target tag

    // --- CONTROLS AND STATE ---
    private ElapsedTime runtime = new ElapsedTime();
    private String selectedAlliance = "RED";
    private boolean aAlreadyPressed = false;

    // --- DRIVE CONSTANTS ---
    private final double DRIVE_SPEED = 0.5;
    private final double TURN_SPEED = 0.3;

    // --- AUXILIARY CONSTANTS ---
    private final double INTAKE_OPEN_POS = 0.8;
    private final double INTAKE_CLOSED_POS = 0.2;
    private final double SLIDES_MAX_POWER = 0.75;

    // --- 🎯 QUADRATIC REGRESSION COEFFICIENTS (FLY-BY LAUNCH) ---
    // NOTE: REPLACE THESE PLACEHOLDER VALUES WITH THE OUTPUT FROM YOUR PYTHON SCRIPT.
    // Target Velocity (y) = A * Range^2 + B * Range + C
    private final double COEFF_A = 0.005;   // Coefficient for Range^2 (x^2)
    private final double COEFF_B = 5.312;   // Coefficient for Range (x)
    private final double COEFF_C = 1050.0;  // Coefficient for Intercept (c)

    // State for the launcher
    private boolean isLauncherSpinning = false;
    private double desiredVelocity = 0.0;

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
            idle();
        }

        // --- 3. RUNTIME ---
        runtime.reset();

        while (opModeIsActive()) {
            if (isAutoNavActive) {
                runAutoNavigation();
            } else {
                handleMecanumDrive();
                handleAutoNavActivation();
            }

            handleAuxiliarySystems();

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

        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- AUXILIARY HARDWARE INITIALIZATION (DECODE 2025-2026) ---
        try {
            servoIntake = hardwareMap.get(Servo.class, "servoIntake");
            servoIntake.setPosition(INTAKE_CLOSED_POS);

            motorLauncher = hardwareMap.get(DcMotorEx.class, "motorlauncher");
            motorLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Launcher mode is initialized to RUN_WITHOUT_ENCODER, but changes in handleAuxiliarySystems
            motorLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motorSlides = hardwareMap.get(DcMotorEx.class, "motorSlides");
            motorSlides.setDirection(DcMotorSimple.Direction.FORWARD);
            motorSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } catch (Exception e) {
            telemetry.addData("WARN", "Auxiliary hardware not found. Check config for artifact_intake_servo, launcher_motor, slides_motor.");
        }
    }

    /**
     * Initializes the IMU (Inertial Measurement Unit) with the robot's orientation.
     */
    private void initializeIMU() {
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }


    /**
     * Initializes the VisionPortal and the AprilTag Processor.
     */
    private void initializeVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

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
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = gamepad1.left_stick_x * DRIVE_SPEED;
        double turn = gamepad1.right_stick_x * TURN_SPEED;

        double botYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double cosYaw = Math.cos(Math.toRadians(-botYaw));
        double sinYaw = Math.sin(Math.toRadians(-botYaw));

        double tempDrive = drive * cosYaw - strafe * sinYaw;
        double tempStrafe = drive * sinYaw + strafe * cosYaw;

        drive = tempDrive;
        strafe = tempStrafe;

        motorLeftFront.setPower(drive + strafe + turn);
        motorLeftBack.setPower(drive - strafe + turn);
        motorRightFront.setPower(drive - strafe - turn);
        motorRightBack.setPower(drive + strafe - turn);
    }

    /**
     * Checks if the operator wants to switch to Auto Navigation mode (AprilTag tracking).
     */
    private void handleAutoNavActivation() {
        if (gamepad1.y) {
            isAutoNavActive = true;
        }
    }

    /**
     * Executes the AprilTag Auto Navigation (simplified Proportional control).
     */
    private void runAutoNavigation() {
        AprilTagDetection detection = null;
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

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

        double Kp_RANGE = 0.05;
        double Kp_BEARING = 0.02;
        double Kp_YAW = 0.015;

        double targetRange = 10.0;

        double rangeError = detection.ftcPose.range - targetRange;
        double bearingError = detection.ftcPose.bearing;
        double yawError = detection.ftcPose.yaw;

        double rangeDrive = rangeError * Kp_RANGE;
        double strafeDrive = -bearingError * Kp_BEARING;
        double turnDrive = -yawError * Kp_YAW;

        rangeDrive = Math.max(-DRIVE_SPEED, Math.min(rangeDrive, DRIVE_SPEED));
        strafeDrive = Math.max(-DRIVE_SPEED, Math.min(strafeDrive, DRIVE_SPEED));
        turnDrive = Math.max(-TURN_SPEED, Math.min(turnDrive, TURN_SPEED));

        if (Math.abs(rangeError) > 0.5) {
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
     * Finds the target AprilTag (ID 20) and returns its range.
     * @return The range in inches, or -1.0 if not found (to indicate no valid range).
     */
    private double getTargetRange() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == TARGET_TAG_ID) {
                // Return the distance (range) to the tag from the camera
                return detection.ftcPose.range;
            }
        }
        return -1.0; // Sentinel value indicating no tag found
    }

    /**
     * Handles control logic for auxiliary systems (Intake, Launcher, Slides) using Gamepad 2.
     * Integrates the quadratic equation for launcher velocity.
     */
    private void handleAuxiliarySystems() {
        if (servoIntake == null || motorLauncher == null || motorSlides == null) {
            return;
        }

        // --- ARTIFACT INTAKE CONTROL (GAMEPAD 2 BUMPERS) ---
        if (gamepad2.right_bumper) {
            servoIntake.setPosition(INTAKE_OPEN_POS);
        } else if (gamepad2.left_bumper) {
            servoIntake.setPosition(INTAKE_CLOSED_POS);
        }

        // --- END-GAME SLIDES CONTROL (GAMEPAD 2 LEFT STICK Y-AXIS) ---
        double slidesPower = -gamepad2.left_stick_y;
        slidesPower = Math.max(-SLIDES_MAX_POWER, Math.min(slidesPower, SLIDES_MAX_POWER));
        motorSlides.setPower(slidesPower);


        // 🚀 --- LAUNCHER CONTROL (FLY-BY VELOCITY) ---
        double currentRange = getTargetRange();

        // --- Gamepad 2 A: Activate Auto-Launch Mode (Spin Up) ---
        if (gamepad2.a) {
            if (currentRange > 0.0) {
                // 1. CALCULATE DESIRED VELOCITY USING QUADRATIC EQUATION: V = A*R^2 + B*R + C
                // R = currentRange, V = desiredVelocity
                desiredVelocity =
                        (COEFF_A * currentRange * currentRange) +
                                (COEFF_B * currentRange) +
                                COEFF_C;

                // 2. Set motor to RUN_USING_ENCODER for velocity control
                if (motorLauncher.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                // 3. Set the calculated velocity
                motorLauncher.setVelocity(desiredVelocity);
                isLauncherSpinning = true;
            } else {
                // Tag is not found: stop the motor and wait
                motorLauncher.setVelocity(0.0);
                isLauncherSpinning = false;
            }
        }
        // --- Gamepad 2 B: Stop Launcher ---
        else if (gamepad2.b) {
            motorLauncher.setVelocity(0.0);
            isLauncherSpinning = false;
            // Optionally, switch back to power mode if needed elsewhere,
            // but keeping it in ENCODER mode is safer for flywheels.
            // motorLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Gamepad 2 Y/Triangle could be used to FIRE the artifact once target velocity is reached
        // This requires additional logic (like a gate servo or timing) not specified here.
        // For now, only the velocity setting logic is implemented.
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
        telemetry.addData("Intake Servo Pos", servoIntake != null ? servoIntake.getPosition() : "N/A");

        // Update Launcher Telemetry for Velocity Control
        if (motorLauncher != null) {
            if (isLauncherSpinning) {
                telemetry.addData("Launcher Status", "Spinning @ %.0f (Actual: %.0f)", desiredVelocity, motorLauncher.getVelocity());
                telemetry.addData("Range Used", "%.2f in", getTargetRange() > 0 ? getTargetRange() : 0.0);
            } else {
                telemetry.addData("Launcher Status", "Stopped");
            }
        }

        telemetry.addData("Slides Power", motorSlides != null ? motorSlides.getPower() : "N/A");
        telemetry.addData("Controls", "A=Auto-Vel | B=Stop Vel");

        telemetry.update();
    }
}