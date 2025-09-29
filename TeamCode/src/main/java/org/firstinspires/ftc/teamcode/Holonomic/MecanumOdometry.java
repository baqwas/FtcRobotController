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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Mecanum Odometry", group = "Autonomous")
@Disabled
public class MecanumOdometry extends LinearOpMode {

    private MecanumDrive mecanumDrive;
    private Odometry odometry;

    // Odometry-specific hardware
    private DcMotorEx odometryLeft;
    private DcMotorEx odometryRight;
    private DcMotorEx odometryStrafe;

    // Movement constants (tune these for your robot)
    private final double DRIVE_SPEED = 0.5;
    private final double TURN_SPEED = 0.5;
    private final double POSITION_TOLERANCE = 1.0; // in inches
    private final double HEADING_TOLERANCE = 1.0; // in degrees

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
        odometryLeft = hardwareMap.get(DcMotorEx.class, "odometryLeft");
        odometryRight = hardwareMap.get(DcMotorEx.class, "odometryRight");
        odometryStrafe = hardwareMap.get(DcMotorEx.class, "odometryStrafe");

        // Set odometry motor modes
        odometryLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometryRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometryStrafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometryLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometryRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometryStrafe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize odometry system
        odometry = new Odometry(odometryLeft, odometryRight, odometryStrafe);

        telemetry.addData("Status", "Hardware Initialized. Waiting for start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example path:
            // 1. Drive forward 24 inches
            goToPosition(0, 24, 0);

            // 2. Strafe right 24 inches
            goToPosition(24, 24, 0);

            // 3. Turn 90 degrees
            goToPosition(24, 24, 90);

            telemetry.addData("Status", "Path complete! OpMode finished.");
            telemetry.update();
        }
    }

    /**
     * Navigates the robot to a specific (x, y) position and a target heading.
     * This is a simplified implementation for demonstration.
     */
    private void goToPosition(double targetX, double targetY, double targetHeading) {
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
