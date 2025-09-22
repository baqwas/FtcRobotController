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

package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;

import org.firstinspires.ftc.teamcode.Holonomic.MecanumPathPlanner;
import org.firstinspires.ftc.teamcode.Holonomic.PurpleGreenDetector;
import org.firstinspires.ftc.teamcode.Holonomic.GridManager;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;

import java.util.List;

@Autonomous(name = "Mecanum: Dynamic Path Following", group = "Test")
public class MecanumPathFollowing extends LinearOpMode {

    private final int GRID_ROWS = 100;
    private final int GRID_COLS = 100;
    private final int START_ROW = 10;
    private final int START_COL = 10;
    private final int END_ROW = 90;
    private final int END_COL = 90;

    private OpenCvCamera camera;
    private PurpleGreenDetector visionPipeline;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    // Odometry-specific hardware
    private DcMotorEx odometryLeft;
    private DcMotorEx odometryRight;
    private DcMotorEx odometryStrafe;

    private final double DRIVE_POWER = 0.5;
    private final double POSITIONAL_TOLERANCE = 5.0;

    // Use a class to manage odometry
    private Odometry odometry;

    // Store current position
    private double currentX;
    private double currentY;

    private Datalogger datalogger;
    private Datalogger.GenericField currentXField;
    private Datalogger.GenericField currentYField;
    private Datalogger.GenericField frontLeftPowerField;
    private Datalogger.GenericField frontRightPowerField;
    private Datalogger.GenericField backLeftPowerField;
    private Datalogger.GenericField backRightPowerField;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        initializeVision();
        initializeDatalogger();

        // Initialize the odometry system
        odometry = new Odometry(odometryLeft, odometryRight, odometryStrafe);
        currentX = START_COL;
        currentY = START_ROW;
        odometry.setPosition(START_COL, START_ROW, 0); // Set initial position

        telemetry.addData("Status", "Hardware Initialized. Waiting for start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            MecanumPathPlanner pathPlanner = new MecanumPathPlanner(GRID_ROWS, GRID_COLS);
            telemetry.addData("Status", "Starting path following...");
            telemetry.update();

            while (opModeIsActive() && (Math.abs(currentX - END_COL) > POSITIONAL_TOLERANCE || Math.abs(currentY - END_ROW) > POSITIONAL_TOLERANCE)) {

                // Update robot's position and heading with odometry
                odometry.update();
                currentX = odometry.getX();
                currentY = odometry.getY();

                // 1. Detect obstacles and add to the grid
                telemetry.addData("Status", "Detecting obstacles...");
                telemetry.update();
                sleep(100);

                List<Point> obstacleCenters = visionPipeline.getDetectedObstacleCenters();
                GridManager gridManager = pathPlanner.getGridManager();
                gridManager.clearObstacles();

                if (!obstacleCenters.isEmpty()) {
                    telemetry.addData("Status", "Found %d obstacles. Adding to grid.", obstacleCenters.size());
                    for (Point center : obstacleCenters) {
                        int obstacleRow = (int) ((center.y / 480.0) * GRID_ROWS);
                        int obstacleCol = (int) ((center.x / 640.0) * GRID_COLS);
                        gridManager.addObstacle(obstacleRow, obstacleCol);
                    }
                } else {
                    telemetry.addData("Status", "No obstacles detected.");
                }

                // 2. Find a new path from the current position
                telemetry.addData("Status", "Finding new path from (%.2f, %.2f) to (%d, %d)...", currentY, currentX, END_ROW, END_COL);
                telemetry.update();
                List<int[]> path = pathPlanner.findPath((int) currentY, (int) currentX, END_ROW, END_COL);

                if (path == null || path.size() <= 1) {
                    telemetry.addData("Status", "No path found! Stopping motors.");
                    telemetry.update();
                    stopAllMotors();
                    sleep(500);
                    continue;
                }

                // 3. Drive to the next waypoint in the path
                int[] nextWaypoint = path.get(1);
                telemetry.addData("Following Path", "Driving to [%d, %d]", nextWaypoint[0], nextWaypoint[1]);
                telemetry.update();
                driveToPoint(nextWaypoint[0], nextWaypoint[1]);
            }

            stopAllMotors();
            telemetry.addData("Status", "Path complete! OpMode finished.");
            telemetry.update();
        }
    }

    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "motorRightBack");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Odometry hardware initialization (replace with your actual hardware names)
        odometryLeft = hardwareMap.get(DcMotorEx.class, "odometryLeft");
        odometryRight = hardwareMap.get(DcMotorEx.class, "odometryRight");
        odometryStrafe = hardwareMap.get(DcMotorEx.class, "odometryStrafe");

        odometryLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometryRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometryStrafe.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        odometryLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        odometryRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        odometryStrafe.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        visionPipeline = new PurpleGreenDetector();
        camera.setPipeline(visionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Status", "Camera failed to open with code: " + errorCode);
            }
        });
    }

    private void initializeDatalogger() {
        datalogger = new Datalogger.Builder()
                .setFilename("DynamicPathingData")
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                .setFields(
                        currentXField = new Datalogger.GenericField("CurrentX"),
                        currentYField = new Datalogger.GenericField("CurrentY"),
                        frontLeftPowerField = new Datalogger.GenericField("FrontLeftPower"),
                        frontRightPowerField = new Datalogger.GenericField("FrontRightPower"),
                        backLeftPowerField = new Datalogger.GenericField("BackLeftPower"),
                        backRightPowerField = new Datalogger.GenericField("BackRightPower")
                )
                .build();
    }

    private void driveToPoint(int targetRow, int targetCol) {
        // Now, this method only controls the motors and logs data
        while (opModeIsActive() && (Math.abs(currentX - targetCol) > POSITIONAL_TOLERANCE || Math.abs(currentY - targetRow) > POSITIONAL_TOLERANCE)) {
            // Update position from odometry
            odometry.update();
            currentX = odometry.getX();
            currentY = odometry.getY();

            double dx = targetCol - currentX;
            double dy = targetRow - currentY;
            double angleToTarget = Math.atan2(dy, dx);
            double strafePower = Math.cos(angleToTarget) * DRIVE_POWER;
            double forwardPower = Math.sin(angleToTarget) * DRIVE_POWER;
            double rotationPower = 0.0;

            frontLeft.setPower(forwardPower + strafePower + rotationPower);
            frontRight.setPower(forwardPower - strafePower - rotationPower);
            backLeft.setPower(forwardPower - strafePower + rotationPower);
            backRight.setPower(forwardPower + strafePower - rotationPower);

            // Log data
            currentXField.set(currentX);
            currentYField.set(currentY);
            frontLeftPowerField.set(frontLeft.getPower());
            frontRightPowerField.set(frontRight.getPower());
            backLeftPowerField.set(backLeft.getPower());
            backRightPowerField.set(backRight.getPower());
            datalogger.writeLine();
        }
        stopAllMotors();
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}