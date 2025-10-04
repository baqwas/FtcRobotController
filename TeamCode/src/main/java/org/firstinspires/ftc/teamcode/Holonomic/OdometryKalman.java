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

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IMU.Parameters;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// **REMOVED BNO055 NAVIGATION IMPORTS**
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles; // For retrieving angles from IMU

public class OdometryKalman {

    private final DcMotorEx leftEncoder;
    private final DcMotorEx rightEncoder;
    private final DcMotorEx strafeEncoder;

    private final IMU imu;    // **USING UNIVERSAL IMU INTERFACE**

    private int lastLeftPos, lastRightPos, lastStrafePos;

    /*
        The following variables are no longer used in the code
    private final double Q_POSITION = 0.1;

    private final double R_POSITION = 100000; // High value since we have no position measurement sensor
     */
    // Filter state variables
    private double filteredX, filteredY, filteredTheta;
    private double P_theta; // Variance of our heading estimate

    // **CONSTRUCTOR ACCEPTS IMU INTERFACE**
    public OdometryKalman(DcMotorEx leftEncoder, DcMotorEx rightEncoder, DcMotorEx strafeEncoder, IMU imu) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.strafeEncoder = strafeEncoder;
        this.imu = imu;
        /**
        // **CORRECTED IMU Initialization using the Builder Pattern**
        // The error likely occurred here if the SDK was outdated or the import was missing.
        // We use IMU.Parameters.Builder() and explicitly set the angle unit to RADIANS
        // since the odometry code expects it.
        com.qualcomm.robotcore.hardware.IMU.Parameters imuParameters =
                new com.qualcomm.robotcore.hardware.IMU.Parameters.Builder()
                .setAngleUnit(com.qualcomm.robotcore.hardware.IMU.AngleUnit.RADIANS)
                // If you are using an older SDK, you MUST update it to
                // fix the "Cannot resolve symbol 'Builder'" error.
                .build();

        imu.initialize(imuParameters);
         */
        // Explicitly call resetYaw() to set the starting heading to zero.
        imu.resetYaw();

        // Initialize state
        double x = 0;
        double y = 0;
        double theta = 0;

        // Initialize filtered state and variance
        this.filteredX = x;
        this.filteredY = y;
        this.filteredTheta = theta;
        this.P_theta = 1.0; // Start with a high uncertainty

        // Reset and get initial encoder counts
        this.lastLeftPos = leftEncoder.getCurrentPosition();
        this.lastRightPos = rightEncoder.getCurrentPosition();
        this.lastStrafePos = strafeEncoder.getCurrentPosition();
    }

    public void update() {
        int currentLeftPos = leftEncoder.getCurrentPosition();
        int currentRightPos = rightEncoder.getCurrentPosition();
        int currentStrafePos = strafeEncoder.getCurrentPosition();

        int deltaLeft = currentLeftPos - lastLeftPos;
        int deltaRight = currentRightPos - lastRightPos;
        int deltaStrafe = currentStrafePos - lastStrafePos;

        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;
        lastStrafePos = currentStrafePos;

        // Odometry constants (TUNE THESE FOR YOUR ROBOT)
        // Calibrate
        double TICKS_PER_INCH = 120;
        double deltaLeftInches = deltaLeft / TICKS_PER_INCH;
        double deltaRightInches = deltaRight / TICKS_PER_INCH;
        double deltaStrafeInches = deltaStrafe / TICKS_PER_INCH;

        // --- PREDICTION STEP (using odometry) ---
        // Predict new heading
        // Distance between left and right wheels in inches
        double TRACK_WIDTH = 15;
        double deltaTheta = (deltaRightInches - deltaLeftInches) / TRACK_WIDTH;
        double predictedTheta = filteredTheta + deltaTheta;

        // Predict new position based on the predicted heading
        double headingChange = (filteredTheta + predictedTheta) / 2.0;
        double deltaY = (deltaRightInches + deltaLeftInches) / 2.0;
        // Correction factor for strafe wheel (tune to fix arc)
        double STRAFE_CORRECTION = 1.0;
        double deltaX = deltaStrafeInches * STRAFE_CORRECTION;

        double deltaXGlobal = deltaY * Math.sin(headingChange) + deltaX * Math.cos(headingChange);
        double deltaYGlobal = deltaY * Math.cos(headingChange) - deltaX * Math.sin(headingChange);

        double predictedX = filteredX + deltaXGlobal;
        double predictedY = filteredY + deltaYGlobal;

        // Update covariance (our uncertainty grows)
        // Kalman Filter parameters (TUNE THESE)
        // Process noise: how much we trust our model (odometry)
        // High value for noisy odometry, low for accurate odometry
        double q_HEADING = 0.05;
        P_theta += q_HEADING;

        // --- UPDATE STEP (using IMU measurement) ---
        // **USE getRobotYawPitchRollAngles for Universal IMU**
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double measuredTheta = angles.getYaw(AngleUnit.RADIANS); // Use getYaw()

        // Calculate Kalman Gain (K)
        // Measurement noise: how much we trust our sensor (IMU)
        // Low value for accurate IMU, high for noisy IMU
        double r_HEADING = 0.005;
        double K = P_theta / (P_theta + r_HEADING);

        // Correct the predicted heading
        filteredTheta = predictedTheta + K * (measuredTheta - predictedTheta);

        // Update covariance (our uncertainty is reduced)
        P_theta *= (1 - K);

        // Since we don't have position-based measurements, we simply update our
        // filtered position with the predicted position. The IMU is only used to
        // correct the heading, which in turn makes the position calculation more accurate.
        filteredX = predictedX;
        filteredY = predictedY;
    }

    public double getX() { return filteredX; }
    public double getY() { return filteredY; }
    public double getHeading() { return Math.toDegrees(filteredTheta); }
}
