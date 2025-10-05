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
 * LIABILITY, WHETHER IN AN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Holonomic;

// **REPLACED BNO055IMU with IMU**
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// **ADDED YawPitchRollAngles for Universal IMU data retrieval**
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// Removed: AxesOrder, AxesReference, Orientation (no longer necessary for IMU data retrieval)

public class OdometryKalmanQuad {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    // **CHANGED TYPE TO IMU**
    private IMU imu;

    private double x, y, theta;
    private int lastFrontLeftPos, lastFrontRightPos, lastBackLeftPos, lastBackRightPos;

    // Drivetrain-based Odometry constants (TUNE THESE FOR YOUR ROBOT)
    private final double TICKS_PER_INCH = 120; // Calibrate
    private final double TRACK_WIDTH = 15;     // Distance between left and right wheels in inches
    private final double WHEELBASE_WIDTH = 15; // Distance between front and back wheels in inches

    // Kalman Filter parameters (TUNE THESE)
    // Process noise: how much we trust our model (odometry)
    private final double Q_HEADING = 0.05;
    private final double Q_POSITION = 0.1;

    // Measurement noise: how much we trust our sensor (IMU)
    private final double R_HEADING = 0.005;
    private final double R_POSITION = 100000;

    // Filter state variables
    private double filteredX, filteredY, filteredTheta;
    private double P_theta; // Variance of our heading estimate

    // **CHANGED CONSTRUCTOR PARAMETER TYPE TO IMU**
    public OdometryKalmanQuad(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;
        /*
        // **UPDATED IMU Initialization using the Builder Pattern**
        IMU.Parameters imuParameters = new IMU.Parameters.Builder()
                .setAngleUnit(IMU.AngleUnit.RADIANS)
                .build();

        imu.initialize(imuParameters);
         */
        // **Explicitly call resetYaw() to set the starting heading to zero.**
        imu.resetYaw();

        // Initialize state
        this.x = 0;
        this.y = 0;
        this.theta = 0;

        // Initialize filtered state and variance
        this.filteredX = x;
        this.filteredY = y;
        this.filteredTheta = theta;
        this.P_theta = 1.0;

        // Reset and get initial encoder counts
        this.lastFrontLeftPos = frontLeft.getCurrentPosition();
        this.lastFrontRightPos = frontRight.getCurrentPosition();
        this.lastBackLeftPos = backLeft.getCurrentPosition();
        this.lastBackRightPos = backRight.getCurrentPosition();
    }

    public void update() {
        int currentFrontLeftPos = frontLeft.getCurrentPosition();
        int currentFrontRightPos = frontRight.getCurrentPosition();
        int currentBackLeftPos = backLeft.getCurrentPosition();
        int currentBackRightPos = backRight.getCurrentPosition();

        int deltaFrontLeft = currentFrontLeftPos - lastFrontLeftPos;
        int deltaFrontRight = currentFrontRightPos - lastFrontRightPos;
        int deltaBackLeft = currentBackLeftPos - lastBackLeftPos;
        int deltaBackRight = currentBackRightPos - lastBackRightPos;

        lastFrontLeftPos = currentFrontLeftPos;
        lastFrontRightPos = currentFrontRightPos;
        lastBackLeftPos = currentBackLeftPos;
        lastBackRightPos = currentBackRightPos;

        double deltaFrontLeftInches = deltaFrontLeft / TICKS_PER_INCH;
        double deltaFrontRightInches = deltaFrontRight / TICKS_PER_INCH;
        double deltaBackLeftInches = deltaBackLeft / TICKS_PER_INCH;
        double deltaBackRightInches = deltaBackRight / TICKS_PER_INCH;

        // --- PREDICTION STEP (using odometry) ---
        // Calculate change in X, Y, and Theta from Mecanum wheel encoders
        double deltaX = (deltaFrontLeftInches + deltaBackRightInches - deltaFrontRightInches - deltaBackLeftInches) / 4.0;
        double deltaY = (deltaFrontLeftInches + deltaFrontRightInches + deltaBackLeftInches + deltaBackRightInches) / 4.0;
        double deltaTheta = (deltaFrontLeftInches + deltaBackLeftInches - deltaFrontRightInches - deltaBackRightInches) / (2 * (TRACK_WIDTH + WHEELBASE_WIDTH));

        double predictedTheta = filteredTheta + deltaTheta;

        double headingChange = (filteredTheta + predictedTheta) / 2.0;
        double deltaXGlobal = deltaY * Math.cos(headingChange) - deltaX * Math.sin(headingChange);
        double deltaYGlobal = deltaY * Math.sin(headingChange) + deltaX * Math.cos(headingChange);

        double predictedX = filteredX + deltaXGlobal;
        double predictedY = filteredY + deltaYGlobal;

        P_theta += Q_HEADING;

        // --- UPDATE STEP (using IMU measurement) ---
        // **REPLACED getAngularOrientation with getRobotYawPitchRollAngles**
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double measuredTheta = angles.getYaw(AngleUnit.RADIANS);

        double K = P_theta / (P_theta + R_HEADING);
        filteredTheta = predictedTheta + K * (measuredTheta - predictedTheta);
        P_theta *= (1 - K);

        filteredX = predictedX;
        filteredY = predictedY;
    }

    public double getX() { return filteredX; }
    public double getY() { return filteredY; }
    public double getHeading() { return Math.toDegrees(filteredTheta); }
}