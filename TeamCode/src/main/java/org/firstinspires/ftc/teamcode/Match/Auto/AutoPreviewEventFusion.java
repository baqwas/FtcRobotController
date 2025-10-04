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
    private final Waypoint redPos1Waypoint = new Waypoint(-36.0, 60.0, 90.0);
    private final Waypoint bluePos1Waypoint = new Waypoint(-36.0, -60.0, -90.0);
    private final Waypoint redScanGoalPosition = new Waypoint(36.0, 50.0, 0.0);
    private final Waypoint blueScanGoalPosition = new Waypoint(36.0, -50.0, 0.0);

    // NEW FINAL PARKING WAYPOINTS
    private final Waypoint redFinalWaypoint = new Waypoint(0.0, 36.0, 0.0); // Example Final Parking for Red
    private final Waypoint blueFinalWaypoint = new Waypoint(0.0, -36.0, 0.0); // Example Final Parking for Blue


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
            Waypoint selectedWaypoint = (alliance == Alliance.RED) ? redPos1Waypoint : bluePos1Waypoint;
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

        // Main FSM loop. Loop continues until opMode is stopped, state is COMPLETE, or time runs out.
        while (opModeIsActive() && currentState != RobotState.COMPLETE && getRuntime() < AUTONOMOUS_TIMEOUT_S) {
            telemetry.addData("Current State", currentState.toString());
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

                    // Placeholder: Move to a general area where the tag is visible
                    driveMecanum(0.5, 0, 0, 10, 2.0);

                    AprilTagDetection goalDetection = scanForAprilTag(goalTagId);
                    targetAprilTagID = (goalDetection != null) ? goalDetection.id : -1;

                    if (targetAprilTagID != -1) {
                        currentState = RobotState.DRIVE_TO_GOAL;
                    } else {
                        currentState = RobotState.LAUNCH; // Fallback
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
                    driveMecanum(-0.5, 0, 0, 4, 1.0);
                    currentState = RobotState.ROTATING;
                    break;

                case ROTATING:
                    driveMecanum(0, 0, 0.5, 90, 2.0);
                    currentState = RobotState.LEAVE_LAUNCH_LINE;
                    break;

                case LEAVE_LAUNCH_LINE:
                    // Determine the final parking Waypoint based on the alliance
                    Waypoint finalWaypoint = (alliance == Alliance.RED) ? redFinalWaypoint : blueFinalWaypoint;

                    telemetry.addData("Status", "LEAVE_LAUNCH_LINE: Driving to Final Waypoint (X: %.1f, Y: %.1f, H: %.1f)", finalWaypoint.x, finalWaypoint.y, finalWaypoint.heading);
                    telemetry.update();

                    // NOTE: This driveMecanum call is a placeholder movement.
                    // In a full implementation with odometry/RoadRunner, this would be replaced
                    // with a command to drive to the specific finalWaypoint coordinates:
                    // e.g., robot.driveTo(finalWaypoint.x, finalWaypoint.y, finalWaypoint.heading);
                    driveMecanum(0.5, 0, 0, 6, 2.0); // Example: Drive forward 6 inches off the line

                    currentState = RobotState.COMPLETE;
                    break;

                case COMPLETE:
                    break;
            }
        }

        // --- FINAL MOTOR SHUTDOWN ---
        moveRobot(0, 0, 0);
        telemetry.addData("Status", "Autonomous Routine Ended.");
        telemetry.addData("Final Time", "%.1f", getRuntime());
        telemetry.update();
    }

    // --- VISION & MOVEMENT HELPER METHODS ---

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
        visionPortal.stopStreaming();
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
                telemetry.addLine(String.format(Locale.US,
                        "RBE (in, deg, deg): %.1f, %.1f, %.1f",
                        detectedTag.ftcPose.range,
                        detectedTag.ftcPose.bearing,
                        detectedTag.ftcPose.elevation));
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
     * Move robot according to desired axes motions (X=forward, Y=strafe left, Yaw=CCW)
     */
    public void moveRobot(double x, double y, double yaw) {
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

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

    private void driveMecanum(double axial, double lateral, double yaw, double distance, double timeoutS) {
        if (!opModeIsActive()) return;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        motorLeftFront.setPower(leftFrontPower);
        motorRightFront.setPower(rightFrontPower);
        motorLeftBack.setPower(leftBackPower);
        motorRightBack.setPower(rightBackPower);

        sleep((long) (timeoutS * 1000));

        // Stop motors after placeholder movement
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
    }

    private void launch(double power) {
        // Placeholder
    }

    private boolean isLaunchComplete() {
        return true;
    }

    // --- AprilTag Alignment Controller Class ---
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

