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
package org.firstinspires.ftc.teamcode.Match.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

// *** APRILTAG IMPORTS ***
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.Locale;
// ****************************


@Autonomous(name="Preview Event Fusion Auto", group="Match", preselectTeleOp="TeleOpPreviewEvent")
//@Disabled
public class AutoPreviewEventFusion extends LinearOpMode {

    // --- AUTONOMOUS TIME CONSTANT ---
    private static final double AUTONOMOUS_TIMEOUT_S = 29.0; // Stop robot activity at 29.0 seconds

    // --- VISION & MOVEMENT CONSTANTS ---
    private static final double DESIRED_DISTANCE = 12.0;
    private static final double DISTANCE_TOLERANCE = 1.0;
    private static final double CAMERA_OFFSET_X = -3.5;
    private static final double CAMERA_OFFSET_Y = 0.0;
    private static final double SPEED_GAIN = 0.025;
    private static final double STRAFE_GAIN = 0.020;
    private static final double TURN_GAIN = 0.015;
    private static final double MAX_AUTO_SPEED = 0.6;
    private static final double MAX_AUTO_STRAFE = 0.6;
    private static final double MAX_AUTO_TURN = 0.4;
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;

    // --- POSITION CONTROL CONSTANTS ---
    private static final double DRIVE_P_GAIN = 0.05;
    private static final double STRAFE_P_GAIN = 0.05;
    private static final double HEADING_P_GAIN = 0.02;
    private static final double POSITION_TOLERANCE = 2.0; // inches
    private static final double HEADING_TOLERANCE = 5.0; // degrees
    private static final double MAX_MOVE_POWER = 0.8;
    private static final double MIN_MOVE_POWER = 0.1;

    // --- Global Position Variables (Estimated by AprilTag Localization) ---
    private double robotX = 0.0; // Robot X position (inches) relative to field origin
    private double robotY = 0.0; // Robot Y position (inches) relative to field origin
    private double robotHeading = 0.0; // Robot Heading (degrees) relative to field origin (Yaw)

    // --- AprilTag Vision Variables ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;
    private int targetAprilTagID = -1;
    private AprilTagMovementController movementController;
    // ---------------------------------

    // Enumerations for Alliance and Position
    enum Alliance { RED, BLUE }
    enum Position { POS1, POS2, POS3 }

    // Enumeration for the Finite State Machine (FSM)
    enum RobotState {
        SCAN_OBELISK,
        SCAN_GOAL,
        DRIVE_TO_GOAL,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        LEAVE_LAUNCH_LINE,
        COMPLETE
    }

    // Hardware for Mecanum Drivetrain
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;

    // Variables for Alliance and Position
    private Alliance alliance = Alliance.RED;
    private Position position = Position.POS1;

    // Initializing the FSM state
    private RobotState currentState = RobotState.SCAN_OBELISK;

    // A simple class to represent a Waypoint in the autonomous path
    private class Waypoint {
        public double x;
        public double y;
        public double heading;

        public Waypoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // Waypoint entities
    // NOTE: Coordinates should be in inches and relative to the field origin
    private final Waypoint bluePos1Waypoint = new Waypoint(-12.0, 12.0, 0.0); // Example: Start far from center, facing long way
    private final Waypoint bluePos2Waypoint = new Waypoint(-30.0, 30.0, 35.0); // New: Start closer to center
    private final Waypoint bluePos3Waypoint = new Waypoint(-36.0, 36.0, 45.0);  // New: Start far edge

    private final Waypoint redPos1Waypoint = new Waypoint(12.0, 12.0, 0.0); // Example: Start far from center, facing long way
    private final Waypoint redPos2Waypoint = new Waypoint(30.0, 30.0, -35.0); // New: Start closer to center
    private final Waypoint redPos3Waypoint = new Waypoint(36.0, 36.0, -45.0);  // New: Start far edge

    private final Waypoint redScanGoalPosition = new Waypoint(36.0, 36.0, 45.0);
    private final Waypoint blueScanGoalPosition = new Waypoint(-36.0, 36.0, -45.0);

    // NEW FINAL PARKING WAYPOINTS
    private final Waypoint redFinalWaypoint = new Waypoint(48.0, 24.0, 0.0); // Example Final Parking for Red
    private final Waypoint blueFinalWaypoint = new Waypoint(-48, 24.0, 0.0); // Example Final Parking for Blue


    @Override
    public void runOpMode() {
        // --- Hardware Initialization ---
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

        motorLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorEx.Direction.FORWARD);

        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // --- Vision Initialization ---
        initAprilTag();
        movementController = new AprilTagMovementController();

        // --- Driver Hub Pre-Match Selection ---
        telemetry.addData("Status", "Ready for Selection");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) {
                alliance = Alliance.BLUE;
            } else if (gamepad1.x) {
                alliance = Alliance.RED;
            }
            if (gamepad1.dpad_up) {
                position = Position.POS1;
            } else if (gamepad1.dpad_right) {
                position = Position.POS2;
            } else if (gamepad1.dpad_down) {
                position = Position.POS3;
            }
            telemetry.addData("Alliance", "Press B|O for Blue, X|□ for Red");
            telemetry.addData("Position", "Press D-pad Up/Right/Down for POS1/POS2/POS3");
            telemetry.addData("Current Selection", "Alliance: %s, Position: %s", alliance.toString(), position.toString());
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            Waypoint selectedWaypoint;

            // Select the starting waypoint based on user input
            if (alliance == Alliance.RED) {
                if (position == Position.POS1) {
                    selectedWaypoint = redPos1Waypoint;
                } else if (position == Position.POS2) {
                    selectedWaypoint = redPos2Waypoint;
                } else { // position == Position.POS3
                    selectedWaypoint = redPos3Waypoint;
                }
            } else { // Alliance.BLUE
                if (position == Position.POS1) {
                    selectedWaypoint = bluePos1Waypoint;
                } else if (position == Position.POS2) {
                    selectedWaypoint = bluePos2Waypoint;
                } else { // position == Position.POS3
                    selectedWaypoint = bluePos3Waypoint;
                }
            }

            // Set initial robot position estimate (could be refined by AprilTag later)
            robotX = selectedWaypoint.x;
            robotY = selectedWaypoint.y;
            robotHeading = selectedWaypoint.heading;

            // Log the selected starting position (Important for debugging)
            telemetry.addData("Starting Pose", "(%.1f, %.1f) @ %.1f", selectedWaypoint.x, selectedWaypoint.y, selectedWaypoint.heading);
            telemetry.update();

            runAutonomousRoutine(selectedWaypoint);
            sleep(1000);
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // --- Main Autonomous Routine with FSM ---
    private void runAutonomousRoutine(Waypoint startPoint) {
        telemetry.addData("Executing Path", "Starting from (%.1f, %.1f)", startPoint.x, startPoint.y);
        telemetry.update();

        visionPortal.resumeStreaming(); // Keep streaming on for continuous localization/detection

        // Main FSM loop. Loop continues until opMode is stopped, state is COMPLETE, or time runs out.
        while (opModeIsActive() && currentState != RobotState.COMPLETE && getRuntime() < AUTONOMOUS_TIMEOUT_S) {
            // Continuously update robot position using AprilTag detections (localization)
            updateRobotPosition();

            telemetry.addData("Current State", currentState.toString());
            telemetry.addData("Robot Pose (X, Y, H)", "(%.1f, %.1f, %.1f)", robotX, robotY, robotHeading);
            telemetry.addData("AprilTag ID", targetAprilTagID == -1 ? "N/A" : targetAprilTagID);
            telemetry.addData("Time Left", "%.1f", AUTONOMOUS_TIMEOUT_S - getRuntime());
            telemetry.update();

            switch (currentState) {
                case SCAN_OBELISK:
                    AprilTagDetection obeliskDetection = scanForAprilTag(-1);
                    targetAprilTagID = (obeliskDetection != null) ? obeliskDetection.id : -1;

                    telemetry.addData("Obelisk Tag ID", (targetAprilTagID != -1) ? targetAprilTagID : "NOT FOUND");
                    telemetry.update();
                    sleep(1000);

                    currentState = RobotState.SCAN_GOAL;
                    break;

                case SCAN_GOAL:
                    int goalTagId = (alliance == Alliance.RED) ? RED_GOAL_TAG_ID : BLUE_GOAL_TAG_ID;
                    Waypoint scanGoalWaypoint = (alliance == Alliance.RED) ? redScanGoalPosition : blueScanGoalPosition;

                    // Travel to the preset Waypoint for the Alliance's GOAL scan position
                    telemetry.addData("Status", "Driving to Scan Goal Waypoint (%.1f, %.1f)", scanGoalWaypoint.x, scanGoalWaypoint.y);
                    telemetry.update();
                    driveToWaypoint(scanGoalWaypoint); // Replaced time-based move

                    // Now scan for the tag from the precise location
                    AprilTagDetection goalDetection = getFirstDetection(goalTagId);
                    targetAprilTagID = (goalDetection != null) ? goalDetection.id : -1;

                    if (targetAprilTagID != -1) {
                        currentState = RobotState.DRIVE_TO_GOAL;
                    } else {
                        currentState = RobotState.LAUNCH; // Fallback if tag isn't visible
                    }
                    sleep(500);
                    break;

                case DRIVE_TO_GOAL:
                    telemetry.addLine("Driving to target using continuous AprilTag feedback...");

                    // Continuously check for the tag and drive toward the DESIRED_DISTANCE
                    while (opModeIsActive() && getRuntime() < AUTONOMOUS_TIMEOUT_S) {
                        AprilTagDetection currentDetection = getFirstDetection(targetAprilTagID);

                        if (currentDetection != null) {
                            double[] powers = movementController.calculatePowers(currentDetection);
                            moveRobot(powers[0], powers[1], powers[2]);

                            double rangeError = (movementController.getRobotCenterY(currentDetection) - DESIRED_DISTANCE);
                            telemetry.addData("Alignment", "Range Error: %.2f", rangeError);
                            telemetry.update();

                            if (Math.abs(rangeError) <= DISTANCE_TOLERANCE) {
                                moveRobot(0, 0, 0);
                                currentState = RobotState.LAUNCH;
                                break;
                            }
                        } else {
                            moveRobot(0, 0, 0);
                            currentState = RobotState.LAUNCH; // Proceed with next state
                            break;
                        }
                        sleep(10); // Small delay for the control loop
                    }
                    break;

                case LAUNCH:
                    launch(1.0);
                    currentState = RobotState.WAIT_FOR_LAUNCH;
                    break;

                case WAIT_FOR_LAUNCH:
                    if(isLaunchComplete()) {
                        currentState = RobotState.DRIVING_AWAY_FROM_GOAL;
                    }
                    break;

                case DRIVING_AWAY_FROM_GOAL:
                    // Simple move away from the goal, which is not position-critical
                    moveRobotBlocking(-0.5, 0, 0, 1.0);
                    currentState = RobotState.ROTATING;
                    break;

                case ROTATING:
                    // Simple time-based rotation, awaiting a full IMU implementation
                    moveRobotBlocking(0, 0, 0.5, 2.0); // Rotate CCW for 2.0s
                    currentState = RobotState.LEAVE_LAUNCH_LINE;
                    break;

                case LEAVE_LAUNCH_LINE:
                    // Determine the final parking Waypoint based on the alliance
                    Waypoint finalWaypoint = (alliance == Alliance.RED) ? redFinalWaypoint : blueFinalWaypoint;

                    telemetry.addData("Status", "LEAVE_LAUNCH_LINE: Driving to Final Waypoint (X: %.1f, Y: %.1f, H: %.1f)", finalWaypoint.x, finalWaypoint.y, finalWaypoint.heading);
                    telemetry.update();

                    // Non-time-based movement using AprilTag-derived position
                    driveToWaypoint(finalWaypoint);

                    currentState = RobotState.COMPLETE;
                    break;

                case COMPLETE:
                    break;
            }
        }

        visionPortal.stopStreaming(); // Stop streaming before end of OpMode

        // --- FINAL MOTOR SHUTDOWN ---
        moveRobot(0, 0, 0);
        telemetry.addData("Status", "Autonomous Routine Ended.");
        telemetry.addData("Final Time", "%.1f", getRuntime());
        telemetry.update();
    }

    // --- LOCALIZATION METHOD ---

    /**
     * Attempts to obtain the robot's current field position (X, Y, Heading)
     * using the latest AprilTag detection for the goal tags (20 or 24).
     */
    public void updateRobotPosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection bestDetection = null;

        for (AprilTagDetection detection : currentDetections) {
            // Only consider the goal tags 20 or 24 for localization
            if (detection.id == BLUE_GOAL_TAG_ID || detection.id == RED_GOAL_TAG_ID) {
                // Ensure the detection has a calculated robot pose (meaning it's in the tag library)
                if (detection.robotPose != null) {
                    bestDetection = detection;
                    break; // Use the first valid one found
                }
            }
        }

        if (bestDetection != null) {
            // Update global position estimates from the tag's robotPose
            robotX = bestDetection.robotPose.getPosition().x;
            robotY = bestDetection.robotPose.getPosition().y;
            // Yaw is the rotation around the Z (up) axis, which is the robot's heading
            robotHeading = bestDetection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
        }
    }

    // --- POSITION-BASED MOVEMENT HELPER ---

    /**
     * Drives the robot to a specific Waypoint (X, Y, Heading) using P-Control and
     * AprilTag Localization for position feedback. This function is blocking.
     */
    private void driveToWaypoint(Waypoint target) {
        double timeout = getRuntime() + 5.0; // Max 5 seconds for movement

        while (opModeIsActive() && getRuntime() < timeout) {
            // 1. Update Position
            updateRobotPosition();

            // 2. Calculate Errors
            double xError = target.x - robotX;
            double yError = target.y - robotY;
            double headingError = AngleUnit.normalizeDegrees(target.heading - robotHeading);

            // Calculate overall distance error (for stopping condition)
            double distanceError = Math.hypot(xError, yError);

            // 3. Apply P-Control to Errors
            // Convert field-centric error (xError, yError) to robot-centric drive powers (axial, lateral)
            // Note: This requires rotating the error by the negative robot heading (cos(-H) = cos(H), sin(-H) = -sin(H))
            double cosH = Math.cos(Math.toRadians(robotHeading));
            double sinH = Math.sin(Math.toRadians(robotHeading));

            double axialError = xError * sinH + yError * cosH;
            double lateralError = xError * cosH - yError * sinH;

            // P-Control gains
            double axial = Range.clip(axialError * DRIVE_P_GAIN, -MAX_MOVE_POWER, MAX_MOVE_POWER);
            double lateral = Range.clip(lateralError * STRAFE_P_GAIN, -MAX_MOVE_POWER, MAX_MOVE_POWER);
            double yaw = Range.clip(headingError * HEADING_P_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            // 4. Check for Completion
            if (distanceError < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE) {
                moveRobot(0, 0, 0);
                telemetry.addData("Status", "Waypoint Reached!");
                return;
            }

            // Apply minimum power if necessary to overcome friction
            if (Math.abs(axial) < MIN_MOVE_POWER && Math.abs(axial) > 0) axial = Math.copySign(MIN_MOVE_POWER, axial);
            if (Math.abs(lateral) < MIN_MOVE_POWER && Math.abs(lateral) > 0) lateral = Math.copySign(MIN_MOVE_POWER, lateral);


            // 5. Apply Powers
            moveRobot(axial, lateral, yaw);

            telemetry.addData("Target", "(%.1f, %.1f) @ %.1f", target.x, target.y, target.heading);
            telemetry.addData("Error (D, H)", "%.1f, %.1f", distanceError, headingError);
            telemetry.addData("Power (A, L, Y)", "%.2f, %.2f, %.2f", axial, lateral, yaw);
            telemetry.update();

            sleep(10);
        }

        // Ensure robot stops if timeout is reached
        moveRobot(0, 0, 0);
    }

    // --- VISION & LOW-LEVEL MOVEMENT HELPER METHODS ---

    private void initAprilTag() {
        // Use a builder to ensure proper configuration for localization if needed,
        // although easyCreateWithDefaults is used for simplicity here.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
        visionPortal.stopStreaming(); // Start stopped to allow pre-match selection
    }

    private AprilTagDetection getFirstDetection(int desiredId) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty()) { return null; }
        for (AprilTagDetection detection : currentDetections) {
            if (desiredId == -1 || detection.id == desiredId) { return detection; }
        }
        return null;
    }

    private AprilTagDetection scanForAprilTag(int desiredId) {
        AprilTagDetection detectedTag = null;
        long startTime = System.currentTimeMillis();
        long scanDuration = 3000;

        visionPortal.resumeStreaming();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < scanDuration) && (detectedTag == null)) {
            detectedTag = getFirstDetection(desiredId);

            if (detectedTag != null) {
                telemetry.addLine(String.format(Locale.US, "\n==== Tag Detected ===="));
                telemetry.addData("ID Found", detectedTag.id);
                telemetry.update();
                break;
            }
            telemetry.addData("Scanning", "Looking for AprilTags ID %d... (Time: %.1f)", desiredId, getRuntime());
            telemetry.update();
            sleep(20);
        }

        visionPortal.stopStreaming();
        return detectedTag;
    }

    /**
     * Non-blocking, low-level control of robot motion.
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        double frontLeftPower    =  axial - lateral - yaw;
        double frontRightPower   =  axial + lateral + yaw;
        double backLeftPower     =  axial + lateral - yaw;
        double backRightPower    =  axial - lateral + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        motorLeftFront.setPower(frontLeftPower);
        motorRightFront.setPower(frontRightPower);
        motorLeftBack.setPower(backLeftPower);
        motorRightBack.setPower(backRightPower);
    }

    /**
     * Blocking, time-based movement (used for simplified, non-position-critical steps).
     */
    private void moveRobotBlocking(double axial, double lateral, double yaw, double timeoutS) {
        if (!opModeIsActive()) return;

        moveRobot(axial, lateral, yaw);
        sleep((long) (timeoutS * 1000));
        moveRobot(0, 0, 0);
    }

    private void launch(double power) {
        // Placeholder
    }

    private boolean isLaunchComplete() {
        return true;
    }

    // --- AprilTag Alignment Controller Class (for close-range alignment) ---
    private class AprilTagMovementController {
        public double getRobotCenterY(AprilTagDetection detection) {
            return detection.ftcPose.y - CAMERA_OFFSET_Y;
        }

        public double[] calculatePowers(AprilTagDetection detection) {
            double robotCenterX = detection.ftcPose.x - CAMERA_OFFSET_X;
            double robotCenterY = getRobotCenterY(detection);

            double rangeError = (robotCenterY - DESIRED_DISTANCE);
            double strafeError = robotCenterX;
            double headingError = detection.ftcPose.bearing;

            double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double strafe = Range.clip(strafeError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            return new double[]{drive, strafe, turn};
        }
    }
}