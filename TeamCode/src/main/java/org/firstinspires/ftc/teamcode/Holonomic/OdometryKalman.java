package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class OdometryKalman {

    private DcMotorEx leftEncoder, rightEncoder, strafeEncoder;
    private BNO055IMU imu;

    private double x, y, theta;
    private int lastLeftPos, lastRightPos, lastStrafePos;

    // Odometry constants (TUNE THESE FOR YOUR ROBOT)
    private final double TICKS_PER_INCH = 120; // Calibrate
    private final double TRACK_WIDTH = 15;     // Distance between left and right wheels in inches
    private final double STRAFE_CORRECTION = 1.0; // Correction factor for strafe wheel (tune to fix arc)

    // Kalman Filter parameters (TUNE THESE)
    // Process noise: how much we trust our model (odometry)
    private final double Q_HEADING = 0.05; // High value for noisy odometry, low for accurate odometry
    private final double Q_POSITION = 0.1;

    // Measurement noise: how much we trust our sensor (IMU)
    private final double R_HEADING = 0.005; // Low value for accurate IMU, high for noisy IMU
    private final double R_POSITION = 100000; // High value since we have no position measurement sensor

    // Filter state variables
    private double filteredX, filteredY, filteredTheta;
    private double P_theta; // Variance of our heading estimate

    public OdometryKalman(DcMotorEx leftEncoder, DcMotorEx rightEncoder, DcMotorEx strafeEncoder, BNO055IMU imu) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.strafeEncoder = strafeEncoder;
        this.imu = imu;

        // Initialize IMU parameters
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);

        // Initialize state
        this.x = 0;
        this.y = 0;
        this.theta = 0;

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

        double deltaLeftInches = deltaLeft / TICKS_PER_INCH;
        double deltaRightInches = deltaRight / TICKS_PER_INCH;
        double deltaStrafeInches = deltaStrafe / TICKS_PER_INCH;

        // --- PREDICTION STEP (using odometry) ---
        // Predict new heading
        double deltaTheta = (deltaRightInches - deltaLeftInches) / TRACK_WIDTH;
        double predictedTheta = filteredTheta + deltaTheta;

        // Predict new position based on the predicted heading
        double headingChange = (filteredTheta + predictedTheta) / 2.0;
        double deltaY = (deltaRightInches + deltaLeftInches) / 2.0;
        double deltaX = deltaStrafeInches * STRAFE_CORRECTION;

        double deltaXGlobal = deltaY * Math.sin(headingChange) + deltaX * Math.cos(headingChange);
        double deltaYGlobal = deltaY * Math.cos(headingChange) - deltaX * Math.sin(headingChange);

        double predictedX = filteredX + deltaXGlobal;
        double predictedY = filteredY + deltaYGlobal;

        // Update covariance (our uncertainty grows)
        P_theta += Q_HEADING;

        // --- UPDATE STEP (using IMU measurement) ---
        // Get the IMU's measured heading
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double measuredTheta = angles.firstAngle;

        // Calculate Kalman Gain (K)
        double K = P_theta / (P_theta + R_HEADING);

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
