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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name = "Mecanum Kalman", group = "Autonomous")
public class MecanumKalman extends LinearOpMode {

    private MecanumDrive mecanumDrive;
    private OdometryKalman odometry;

    /*
    // Movement constants (tune these for your robot)
    // The following private fields are never used owing to changes in code
    // private final double DRIVE_SPEED = 0.5;
    // private final double TURN_SPEED = 0.5;
     */

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        mecanumDrive = new MecanumDrive(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight")
        );

        // Odometry hardware initialization (replace with your actual hardware names)
        // Odometry-specific hardware
        DcMotorEx odometryLeft = hardwareMap.get(DcMotorEx.class, "odometryLeft");
        DcMotorEx odometryRight = hardwareMap.get(DcMotorEx.class, "odometryRight");
        DcMotorEx odometryStrafe = hardwareMap.get(DcMotorEx.class, "odometryStrafe");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set odometry motor modes
        odometryLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometryRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometryStrafe.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometryLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        odometryRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        odometryStrafe.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize odometry system with IMU
        odometry = new OdometryKalman(odometryLeft, odometryRight, odometryStrafe, imu);

        telemetry.addData("Status", "Hardware Initialized. Waiting for start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Define the path using the new Waypoint class with a heading
            Waypoint[] path = {
                    new Waypoint(0, 24, 0),
                    new Waypoint(24, 24, 90),
                    new Waypoint(24, 48, 0)
            };

            // Iterate through each waypoint and drive to it
            for (Waypoint waypoint : path) {
                goToPosition(waypoint.x, waypoint.y, waypoint.heading);
            }

            telemetry.addData("Status", "Path complete! OpMode finished.");
            telemetry.update();
        }
    }

    /**
     * Navigates the robot to a specific (x, y) position and a target heading.
     * This is a simplified implementation for demonstration.
     */
    private void goToPosition(double targetX, double targetY, double targetHeading) {
        // in inches
        double POSITION_TOLERANCE = 1.0;
        // in degrees
        double HEADING_TOLERANCE = 1.0;
        while (opModeIsActive() && (
                Math.hypot(targetX - odometry.getX(), targetY - odometry.getY()) > POSITION_TOLERANCE ||
                        Math.abs(targetHeading - odometry.getHeading()) > HEADING_TOLERANCE
        )) {
            odometry.update();
            double currentX = odometry.getX();
            double currentY = odometry.getY();
            double currentHeading = odometry.getHeading();

            // Calculate drive and turn components
            double dx = targetX - currentX;
            double dy = targetY - currentY;
            double headingError = targetHeading - currentHeading;

            // Simple Proportional control for movement and turning
            double drivePower = Math.hypot(dx, dy);
            double turnPower = headingError / HEADING_TOLERANCE;
            drivePower = Math.min(drivePower, 1.0); // Cap max power

            // Convert to Mecanum powers
            double robotAngle = Math.atan2(dy, dx) - Math.toRadians(currentHeading);
            double strafePower = Math.sin(robotAngle) * drivePower;
            double forwardPower = Math.cos(robotAngle) * drivePower;

            mecanumDrive.setPower(forwardPower, strafePower, turnPower);

            telemetry.addData("Status", "Driving to target...");
            telemetry.addData("Current Pos", "X: %.2f, Y: %.2f, Heading: %.2f", currentX, currentY, currentHeading);
            telemetry.addData("Target Pos", "X: %.2f, Y: %.2f, Heading: %.2f", targetX, targetY, targetHeading);
            telemetry.update();
        }
        mecanumDrive.stop();
    }
}