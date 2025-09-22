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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.List;

// Import your existing MecanumPathPlanner class and the new KalmanFilter
import org.firstinspires.ftc.teamcode.Holonomic.MecanumPathPlanner;
import org.firstinspires.ftc.teamcode.Holonomic.KalmanFilter;

/**
 * An advanced OpMode that uses a MecanumPathPlanner to follow a path with
 * PID control and a Kalman filter for robust sensor fusion from motor
 * encoders and an IMU.
 */
@Autonomous(name="Advanced Path Following with Kalman Filter", group="Test")
public class MecanumSensorFusion extends LinearOpMode {

    // --- Define your grid dimensions here ---
    private final int GRID_ROWS = 100;
    private final int GRID_COLS = 100;

    // --- Define your start and end coordinates here ---
    private final double START_X = 10;
    private final double START_Y = 10;
    private final double END_X = 90;
    private final double END_Y = 90;

    // Simulate an obstacle for pathfinding.
    private final int OBSTACLE_ROW = 50;
    private final int OBSTACLE_COL = 50;
    private final int OBSTACLE_SIZE = 10;

    // PID constants for movement control. These will need to be TUNED!
    private static final double Kp_POSITION = 0.1;
    private static final double Ki_POSITION = 0.001;
    private static final double Kd_POSITION = 0.01;

    // PID constants for heading/rotation control.
    private static final double Kp_HEADING = 0.05;
    private static final double Ki_HEADING = 0.001;
    private static final double Kd_HEADING = 0.005;

    // Tolerances for determining when the robot has reached its target.
    private static final double POSITIONAL_TOLERANCE = 2.0; // in grid units
    private static final double HEADING_TOLERANCE = 2.0; // in degrees

    // Robot hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;

    // Robot state and path planner
    private MecanumPathPlanner pathPlanner;
    private KalmanFilter kalmanFilter;
    private double lastFrontLeftTicks, lastFrontRightTicks, lastBackLeftTicks, lastBackRightTicks;

    // Ticks per inch and per degree for odometry calculations.
    // These values MUST be calibrated for your robot.
    private static final double TICKS_PER_INCH = 1000;
    private static final double TICKS_PER_DEGREE = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set motor directions based on your robot's physical layout
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to use encoders for precise control
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the IMU with parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        // --- Kalman Filter Initialization ---
        // State vector: [x, y, heading, vx, vy, v_heading]
        double[] initial_x = {START_X, START_Y, 0.0, 0.0, 0.0, 0.0};

        // Initial covariance (high uncertainty)
        double[][] initial_P = new double[6][6];
        for (int i = 0; i < 6; i++) {
            initial_P[i][i] = 1.0;
        }

        // State transition matrix (F)
        double[][] F = {
                {1, 0, 0, 1, 0, 0},
                {0, 1, 0, 0, 1, 0},
                {0, 0, 1, 0, 0, 1},
                {0, 0, 0, 1, 0, 0},
                {0, 0, 0, 0, 1, 0},
                {0, 0, 0, 0, 0, 1}
        };

        // Control input matrix (B)
        double[][] B = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0},
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        };

        // Process noise covariance (Q) - Tune these values!
        double[][] Q = new double[6][6];
        Q[0][0] = 0.001; // x noise
        Q[1][1] = 0.001; // y noise
        Q[2][2] = 0.0001; // heading noise
        Q[3][3] = 0.01;  // vx noise
        Q[4][4] = 0.01;  // vy noise
        Q[5][5] = 0.001; // v_heading noise

        // Observation matrix (H) - Maps state to measurement space
        // We measure x, y, and heading
        double[][] H = {
                {1, 0, 0, 0, 0, 0},
                {0, 1, 0, 0, 0, 0},
                {0, 0, 1, 0, 0, 0}
        };

        // Measurement noise covariance (R) - Tune these values!
        double[][] R = new double[3][3];
        R[0][0] = 0.01; // encoder x measurement noise
        R[1][1] = 0.01; // encoder y measurement noise
        R[2][2] = 0.001; // IMU heading measurement noise

        // Initialize the Kalman filter
        kalmanFilter = new KalmanFilter(initial_x, initial_P, F, B, Q, H, R);

        // Initialize Odometry
        lastFrontLeftTicks = frontLeft.getCurrentPosition();
        lastFrontRightTicks = frontRight.getCurrentPosition();
        lastBackLeftTicks = backLeft.getCurrentPosition();
        lastBackRightTicks = backRight.getCurrentPosition();

        // Pathfinding Setup
        pathPlanner = new MecanumPathPlanner(GRID_ROWS, GRID_COLS);
        telemetry.addData("Status", "Adding simulated obstacle...");
        for (int i = 0; i < OBSTACLE_SIZE; i++) {
            for (int j = 0; j < OBSTACLE_SIZE; j++) {
                pathPlanner.getGridManager().addObstacle(OBSTACLE_ROW + i, OBSTACLE_COL + j);
            }
        }
        telemetry.addData("Status", "Ready to find path.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            List<int[]> path = pathPlanner.findPath((int) START_Y, (int) START_X, (int) END_Y, (int) END_X);

            if (path == null) {
                telemetry.addData("Status", "No path found! Stopping OpMode.");
                telemetry.update();
                requestOpModeStop();
                return;
            }

            telemetry.addData("Status", "Path found. Following waypoints...");
            telemetry.update();
            sleep(1000);

            for (int[] waypoint : path) {
                double targetX = (double) waypoint[1];
                double targetY = (double) waypoint[0];

                telemetry.addData("Following Path", "Driving to [%.2f, %.2f]", targetX, targetY);
                telemetry.update();

                driveToPoint(targetX, targetY);
            }

            stopAllMotors();
            telemetry.addData("Status", "Path complete! OpMode finished.");
            telemetry.update();
        }
    }

    private void driveToPoint(double targetX, double targetY) {
        PIDController positionPid = new PIDController(Kp_POSITION, Ki_POSITION, Kd_POSITION);
        PIDController headingPid = new PIDController(Kp_HEADING, Ki_HEADING, Kd_HEADING);

        while (opModeIsActive()) {
            // Update the robot's pose using the Kalman filter
            updateRobotPoseWithKalman();

            // Get the estimated state from the filter
            double[] estimatedState = kalmanFilter.getEstimatedState();
            double currentX = estimatedState[0];
            double currentY = estimatedState[1];
            double currentHeading = estimatedState[2];

            double xError = targetX - currentX;
            double yError = targetY - currentY;
            double distanceError = Math.hypot(xError, yError);

            double targetHeading = Math.toDegrees(Math.atan2(yError, xError));
            double headingError = AngleUnit.normalizeDegrees(targetHeading - currentHeading);

            if (distanceError < POSITIONAL_TOLERANCE) {
                break;
            }

            double positionalCorrection = positionPid.calculate(distanceError);
            double headingCorrection = headingPid.calculate(headingError);

            double angle = Math.atan2(yError, xError);
            double forwardPower = Math.sin(angle) * positionalCorrection;
            double strafePower = Math.cos(angle) * positionalCorrection;

            frontLeft.setPower(forwardPower + strafePower + headingCorrection);
            frontRight.setPower(forwardPower - strafePower - headingCorrection);
            backLeft.setPower(forwardPower - strafePower + headingCorrection);
            backRight.setPower(forwardPower + strafePower - headingCorrection);

            telemetry.addData("Pose", "X: %.2f, Y: %.2f, Heading: %.2f", currentX, currentY, currentHeading);
            telemetry.addData("Errors", "Dist: %.2f, Heading: %.2f", distanceError, headingError);
            telemetry.update();
        }

        stopAllMotors();
    }

    /**
     * This method updates the robot's pose using the Kalman filter.
     * It performs a prediction step with encoder data and an update step
     * with IMU heading data.
     */
    private void updateRobotPoseWithKalman() {
        // Read encoder ticks
        double currentFrontLeftTicks = frontLeft.getCurrentPosition();
        double currentFrontRightTicks = frontRight.getCurrentPosition();
        double currentBackLeftTicks = backLeft.getCurrentPosition();
        double currentBackRightTicks = backRight.getCurrentPosition();

        // Calculate the change in ticks since the last update
        double deltaFrontLeft = currentFrontLeftTicks - lastFrontLeftTicks;
        double deltaFrontRight = currentFrontRightTicks - lastFrontRightTicks;
        double deltaBackLeft = currentBackLeftTicks - lastBackLeftTicks;
        double deltaBackRight = currentBackRightTicks - lastBackRightTicks;

        // Update last ticks for next iteration
        lastFrontLeftTicks = currentFrontLeftTicks;
        lastFrontRightTicks = currentFrontRightTicks;
        lastBackLeftTicks = currentBackLeftTicks;
        lastBackRightTicks = currentBackRightTicks;

        // Calculate odometry changes
        // These formulas are specific to your robot's configuration and may need tuning.
        double deltaX = (deltaFrontRight + deltaBackLeft) / (2.0 * TICKS_PER_INCH);
        double deltaY = (deltaFrontLeft + deltaBackRight) / (2.0 * TICKS_PER_INCH);
        double deltaHeading = (deltaFrontRight - deltaBackLeft) / (2.0 * TICKS_PER_DEGREE);

        // Create the control vector `u` for the prediction step.
        double[] u = {deltaX, deltaY, deltaHeading};

        // --- Prediction Step ---
        kalmanFilter.predict(u);

        // Get the current heading from the IMU
        double imuHeading = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

        // Create the measurement vector `z` for the update step.
        // We assume our measurements are the new x, y, and heading.
        double[] z = {
                kalmanFilter.getEstimatedState()[0] + deltaX,
                kalmanFilter.getEstimatedState()[1] + deltaY,
                imuHeading
        };

        // --- Update Step ---
        kalmanFilter.update(z);
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // A simple class to represent the robot's pose (position and heading).
    private static class Pose {
        double x;
        double y;
        double heading; // in degrees

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // A simple PID Controller class.
    private static class PIDController {
        private final double Kp, Ki, Kd;
        private double lastError = 0;
        private double integral = 0;

        public PIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public double calculate(double error) {
            integral += error;
            double derivative = error - lastError;
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            lastError = error;
            return output;
        }

        public void reset() {
            lastError = 0;
            integral = 0;
        }
    }
}
