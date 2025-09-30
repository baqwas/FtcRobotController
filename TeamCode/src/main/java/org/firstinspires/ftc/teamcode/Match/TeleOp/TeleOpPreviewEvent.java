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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * {@link TeleOpPreviewEvent} is a modular TeleOp OpMode with dedicated subsystems
 * for Drivetrain, Intake, Launcher (Velocity-Controlled), Elevator, and AprilTag-based Auto-Navigation.
 *
 * This OpMode uses a cooperative multitasking approach where each subsystem
 * runs a non-blocking method inside the main loop.
 */
@TeleOp(name = "Preview Event TeleOp", group = "Match")
public class TeleOpPreviewEvent extends LinearOpMode {

    // --- 1. HARDWARE DECLARATION & CONFIGURATION ---

    // Drivetrain Motors (Mecanum)
    private DcMotor motorLeftFront = null;
    private DcMotor motorLeftBack = null;
    private DcMotor motorRightFront = null;
    private DcMotor motorRightBack = null;

    // Subsystem Motors/Servos
    private DcMotor motorIntake = null;
    private DcMotorEx motorLauncherEx = null; // Changed to DcMotorEx for velocity control
    private DcMotor motorElevator = null;
    private Servo servoGate = null;
    private IMU imu = null;

    // Vision and Navigation
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Alliance Configuration (User Selectable)
    private enum AllianceColor { RED, BLUE }
    private AllianceColor selectedAlliance = AllianceColor.BLUE;
    private int ALLIANCE_GOAL_TAG_ID;

    // AprilTag IDs for the Alliance GOAL (Example IDs)
    private static final int RED_GOAL_TAG_ID = 3;
    private static final int BLUE_GOAL_TAG_ID = 5;

    // Auto-Navigation State
    private boolean isAutoNavActive = false;
    private ElapsedTime runtime = new ElapsedTime();


    // --- 2. INITIALIZATION AND SETUP ---

    @Override
    public void runOpMode() {
        // Wait for the user to select the alliance color before initializing
        selectAllianceColor();

        // Initialize all hardware components
        initializeHardware();

        // Initialize Vision for AprilTags
        initializeVision();

        telemetry.addData("Status", "Hardware and Vision Initialized");
        telemetry.addData("Selected Alliance", selectedAlliance);
        telemetry.addData("Goal Tag ID", ALLIANCE_GOAL_TAG_ID);
        telemetry.addData("Controls", "G1: Drive | G2: Mechanisms");
        telemetry.update();

        // Wait for the game to start (Driver presses PLAY)
        waitForStart();
        runtime.reset();
        isAutoNavActive = false;

        // --- 3. MAIN CONTROL LOOP ---

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Run all modular subsystems non-blockingly in every loop iteration
                handleFieldCentricDrive();
                handleIntakeSubsystem();
                handleLauncherSubsystem(); // Now uses velocity control
                handleElevatorSubsystem();
                handleAutoNavigation();

                // Telemetry Update (always last in the loop)
                sendTelemetryUpdates();

                // Yield processor time for smooth operation
                idle();
            }
        }
    }


    /**
     * Allows the user to select the alliance color before OpMode start.
     */
    private void selectAllianceColor() {
        selectedAlliance = AllianceColor.BLUE;
        ALLIANCE_GOAL_TAG_ID = BLUE_GOAL_TAG_ID;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                selectedAlliance = AllianceColor.BLUE;
                ALLIANCE_GOAL_TAG_ID = BLUE_GOAL_TAG_ID;
            } else if (gamepad1.b) {
                selectedAlliance = AllianceColor.RED;
                ALLIANCE_GOAL_TAG_ID = RED_GOAL_TAG_ID;
            }

            telemetry.addData("Status", "SELECT ALLIANCE COLOR");
            telemetry.addData("Current", selectedAlliance);
            telemetry.addData("Press B", "-> Red Alliance (Tag ID: " + RED_GOAL_TAG_ID + ")");
            telemetry.addData("Press X", "-> Blue Alliance (Tag ID: " + BLUE_GOAL_TAG_ID + ")");
            telemetry.update();
        }
    }

    /**
     * Initializes all hardware components, including the DcMotorEx for the launcher.
     */
    private void initializeHardware() {
        // Mecanum Drive Motors (DcMotor)
        motorLeftFront = hardwareMap.get(DcMotor.class, "left_front_motor");
        motorLeftBack = hardwareMap.get(DcMotor.class, "left_back_motor");
        motorRightFront = hardwareMap.get(DcMotor.class, "right_front_motor");
        motorRightBack = hardwareMap.get(DcMotor.class, "right_back_motor");

        // Mechanism Motors/Servos
        motorIntake = hardwareMap.get(DcMotor.class, "intake_motor");
        motorElevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        servoGate = hardwareMap.get(Servo.class, "launcher_gate_servo");

        // LAUNCHER MOTOR SETUP (DcMotorEx is required for setVelocity())
        motorLauncherEx = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        motorLauncherEx.setDirection(DcMotorSimple.Direction.FORWARD);
        // Set to RUN_USING_ENCODER to enable velocity PID control
        motorLauncherEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLauncherEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Standard Drive Motor Configuration
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Other Mechanisms
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorElevator.setDirection(DcMotorSimple.Direction.FORWARD);
        motorElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new com.qualcomm.hardware.rev.RevHubOrientationOnRobot()));
        imu.resetYaw();

        // Set initial servo position
        servoGate.setPosition(0.0); // Closed position
    }

    /**
     * Sets up the VisionPortal and the AprilTag Processor.
     */
    private void initializeVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagProcessor.get10dofLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        visionPortal.resumeStreaming();
    }


    // --- 4. MODULAR SUBSYSTEMS ---

    // Subsystem 1: Field-Centric Drivetrain
    private void handleFieldCentricDrive() {
        if (isAutoNavActive) { return; }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.a) { imu.resetYaw(); }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        motorLeftFront.setPower((rotY + rotX + rx) / denominator);
        motorLeftBack.setPower((rotY - rotX + rx) / denominator);
        motorRightFront.setPower((rotY - rotX - rx) / denominator);
        motorRightBack.setPower((rotY + rotX - rx) / denominator);
    }

    // Subsystem 2: Intake Operations
    private void handleIntakeSubsystem() {
        double intakePower = gamepad2.left_trigger;
        double outtakePower = gamepad2.right_trigger;

        if (intakePower > 0.1) {
            motorIntake.setPower(intakePower);
        } else if (outtakePower > 0.1) {
            motorIntake.setPower(-outtakePower);
        } else {
            motorIntake.setPower(0.0);
        }
    }

    /**
     * Estimates the target launcher velocity (in encoder ticks/sec) based on the range (distance) to the goal.
     * This uses a placeholder quadratic formula: V = A*R^2 + B*R + C.
     * @param range The distance to the target in inches (ftcPose.range).
     * @return The required velocity in motor ticks per second.
     */
    private double calculateTargetVelocity(double range) {
        // --- Placeholder Quadratic Equation ---
        // These coefficients (A, B, C) must be determined empirically via statistical regression.
        // This example assumes a max velocity of around 2800 ticks/sec at the highest range.
        final double A = 2.0;
        final double B = 50.0;
        final double C = 1000.0;
        final double MAX_VELOCITY = 2800.0;

        double targetVelocity = (A * range * range) + (B * range) + C;

        // Ensure velocity does not exceed the motor's capability
        return Math.min(targetVelocity, MAX_VELOCITY);
    }


    // Subsystem 3: Velocity-Controlled Launcher Operations
    private enum LaunchState {
        IDLE,
        SPINNING_UP,
        FIRING,
        RELOADING
    }

    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime launchTimer = new ElapsedTime();
    private static final double FIRE_TIME = 0.5;
    private static final double RELOAD_TIME = 1.0;
    private static final double DEFAULT_RANGE_INCHES = 36.0; // Default range if no tag is seen

    private void handleLauncherSubsystem() {

        // 1. Localization/Range Check (Passive)
        AprilTagDetection targetDetection = findGoalTag();
        double currentRange = targetDetection != null ? targetDetection.ftcPose.range : DEFAULT_RANGE_INCHES;
        double targetVelocity = calculateTargetVelocity(currentRange);

        switch (launchState) {
            case IDLE:
                motorLauncherEx.setVelocity(0.0); // Stop the motor using velocity=0
                servoGate.setPosition(0.0); // Closed
                // Transition to SPINNING_UP when Gamepad 2 X is pressed
                if (gamepad2.x) {
                    launchState = LaunchState.SPINNING_UP;
                    launchTimer.reset();
                }
                break;

            case SPINNING_UP:
                // Set velocity based on live range data using the calculated PID target
                motorLauncherEx.setVelocity(targetVelocity);
                telemetry.addData("Launch Target Velocity", "%.0f", targetVelocity);

                // Transition to FIRING when Gamepad 2 B is pressed
                if (gamepad2.b) {
                    launchState = LaunchState.FIRING;
                    launchTimer.reset();
                }
                break;

            case FIRING:
                motorLauncherEx.setVelocity(targetVelocity); // Maintain velocity
                servoGate.setPosition(1.0); // Open the gate (FIRE!)
                if (launchTimer.seconds() >= FIRE_TIME) {
                    launchState = LaunchState.RELOADING;
                    launchTimer.reset();
                }
                break;

            case RELOADING:
                motorLauncherEx.setVelocity(targetVelocity); // Maintain velocity
                servoGate.setPosition(0.0); // Close the gate
                if (launchTimer.seconds() >= RELOAD_TIME) {
                    launchState = LaunchState.SPINNING_UP; // Ready for next shot
                }
                // Transition back to IDLE if Gamepad 2 X is pressed again (shutdown)
                if (gamepad2.x) {
                    launchState = LaunchState.IDLE;
                }
                break;
        }

        // Emergency Stop/Shutdown on Gamepad 2 Y
        if (gamepad2.y) {
            launchState = LaunchState.IDLE;
        }

        telemetry.addData("Launch State", launchState);
        telemetry.addData("Launcher Velocity (Current)", "%.0f", motorLauncherEx.getVelocity());
    }


    // Subsystem 4: Elevator/Linear Slide Operations
    private void handleElevatorSubsystem() {
        double elevatorPower = 0.0;

        if (gamepad2.dpad_up) {
            // Future: Use motorElevator.setTargetPosition() and motorElevator.setMode(RUN_TO_POSITION)
            elevatorPower = 0.8;
            telemetry.addData("Elevator", "RAISING (Future Encoder Control)");
        } else if (gamepad2.dpad_down) {
            // Future: Use motorElevator.setTargetPosition() and motorElevator.setMode(RUN_TO_POSITION)
            elevatorPower = -0.5;
            telemetry.addData("Elevator", "LOWERING (Future Encoder Control)");
        } else {
            // This ensures the elevator holds position when not actively controlled
            elevatorPower = 0.0;
        }

        motorElevator.setPower(elevatorPower);
    }


    // Subsystem 5: AprilTag-based Auto-Navigation
    private void handleAutoNavigation() {
        if (gamepad1.b && !isAutoNavActive) {
            isAutoNavActive = true;
            telemetry.addData("AutoNav", "ACTIVATED: Searching for Goal Tag...");
        } else if (gamepad1.b && isAutoNavActive) {
            isAutoNavActive = false;
            stopDriveMotors();
            telemetry.addData("AutoNav", "DEACTIVATED: Returning to Manual Drive.");
        }

        if (!isAutoNavActive) {
            return;
        }

        AprilTagDetection targetDetection = findGoalTag();

        if (targetDetection != null) {
            driveTowardsAprilTag(targetDetection);
        } else {
            stopDriveMotors();
            telemetry.addData("AutoNav Error", "Goal Tag ID " + ALLIANCE_GOAL_TAG_ID + " not visible.");
        }
    }

    /**
     * Helper method to find the Alliance Goal AprilTag.
     */
    private AprilTagDetection findGoalTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == ALLIANCE_GOAL_TAG_ID) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Uses Range, Bearing, and Elevation to drive the robot towards a detected AprilTag.
     */
    private void driveTowardsAprilTag(AprilTagDetection detection) {
        // P control for range (distance)
        double rangeError = detection.ftcPose.range - 6.0;
        double rangeDrive = Range.clip(rangeError * 0.1, -0.4, 0.4);

        // P control for bearing (strafe)
        double bearingError = detection.ftcPose.bearing;
        double strafeDrive = Range.clip(bearingError * 0.02, -0.4, 0.4);

        // P control for yaw (rotation)
        double yawError = detection.ftcPose.yaw;
        double turnDrive = Range.clip(yawError * 0.015, -0.3, 0.3);

        if (Math.abs(rangeError) > 0.5) { // Stop when within 0.5 inches
            motorLeftFront.setPower(rangeDrive + strafeDrive + turnDrive);
            motorLeftBack.setPower(rangeDrive - strafeDrive + turnDrive);
            motorRightFront.setPower(rangeDrive + strafeDrive - turnDrive);
            motorRightBack.setPower(rangeDrive - strafeDrive - turnDrive);

            telemetry.addData("AutoNav Status", "Tracking Goal Tag...");
            telemetry.addData("Range/Bearing/Elev", "%.1f, %.1f, %.1f", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);
        } else {
            stopDriveMotors();
            isAutoNavActive = false;
            telemetry.addData("AutoNav Status", "Reached Target Position!");
        }
    }

    /**
     * Helper method to stop all drive motors.
     */
    private void stopDriveMotors() {
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
    }

    /**
     * Final subsystem for all telemetry updates.
     */
    private void sendTelemetryUpdates() {
        telemetry.addData("Runtime", runtime.toString());
        telemetry.addData("Yaw (Field)", "%.1f deg", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("AutoNav Mode", isAutoNavActive ? "ACTIVE" : "INACTIVE (Manual)");

        telemetry.update();
    }
}
