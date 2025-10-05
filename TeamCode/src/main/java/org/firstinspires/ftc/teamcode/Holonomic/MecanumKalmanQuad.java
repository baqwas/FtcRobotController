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

/*
 * This revised set of files adapts the odometry system to use the four drivetrain motors' encoders
 * instead of dedicated auxiliary encoders. This approach eliminates the need for separate odometry
 * wheels but may be less precise due to wheel slippage and other factors inherent to driven wheels.
 * The core logic of the Kalman filter in OdometryKalman.java is updated to process four encoder
 * inputs and calculate the robot's pose.
 */
package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
// **REMOVED: import com.qualcomm.hardware.bosch.BNO055IMU;**

// **ADDED: Universal IMU Interface**
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "Mecanum Kalman Quad", group = "Autonomous")
@Disabled
public class MecanumKalmanQuad extends LinearOpMode {

    private MecanumDrive mecanumDrive;
    private OdometryKalmanQuad odometry;
    /*
    // Movement constants (tune these for your robot)
    private final double DRIVE_SPEED = 0.5;
    private final double TURN_SPEED = 0.5;
     */

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drivetrain motors
        // Drivetrain motors (now also used for odometry)
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Set drivetrain motor modes
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse motors as needed
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // **UPDATED: Universal IMU Declaration and Initialization**
        // IMU hardware initialization
        IMU imu = hardwareMap.get(IMU.class, "imu");
        /*
        // Configure IMU Parameters
        IMU.Parameters parameters = new IMU.Parameters.Builder()
                .setAngleUnit(AngleUnit.DEGREES)
                .build();

        imu.initialize(parameters);
        */
        imu.resetYaw(); // Reset yaw to make the current direction 0 degrees

        telemetry.addData("Status", "IMU Initialized. Waiting for system set...");
        telemetry.update();

        // Initialize odometry system with all four drivetrain motors
        // NOTE: The OdometryKalmanQuad class must be updated separately to accept an IMU object.
        odometry = new OdometryKalmanQuad(frontLeft, frontRight, backLeft, backRight, imu);

        telemetry.addData("Status", "Hardware Initialized. Waiting for start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Define the path using the Waypoint class
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

    // Placeholder class definition for Waypoint (needed for compilation)
    private static class Waypoint {
        public double x;
        public double y;
        public double heading;

        public Waypoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // The OdometryKalmanQuad class is assumed to have been updated to accept a com.qualcomm.robotcore.hardware.IMU object
    // in its constructor and use the IMU's modern API (e.g., imu.getRobotYawPitchRollAngles().getYaw()).
}
