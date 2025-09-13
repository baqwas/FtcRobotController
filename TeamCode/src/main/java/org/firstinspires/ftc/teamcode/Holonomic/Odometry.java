package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.hardware.DcMotorEx;
public class Odometry {
    private DcMotorEx leftEncoder, rightEncoder, strafeEncoder;

    private double x, y, theta;
    private int lastLeftPos, lastRightPos, lastStrafePos;

    // Odometry constants (tune these for your robot)
    private final double TICKS_PER_INCH = 120; // Example value, must be calibrated
    private final double TRACK_WIDTH = 15;     // Distance between left and right wheels
    private final double STRAFE_CORRECTION = 1.0; // Correction for strafe wheel

    public Odometry(DcMotorEx leftEncoder, DcMotorEx rightEncoder, DcMotorEx strafeEncoder) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.strafeEncoder = strafeEncoder;

        // Initialize position
        this.x = 0;
        this.y = 0;
        this.theta = 0;

        // Reset and get initial encoder counts
        this.lastLeftPos = leftEncoder.getCurrentPosition();
        this.lastRightPos = rightEncoder.getCurrentPosition();
        this.lastStrafePos = strafeEncoder.getCurrentPosition();
    }

    public void update() {
        int currentLeftPos = leftEncoder.getCurrentPosition();
        int currentRightPos = rightEncoder.getCurrentPosition();
        int currentStrafePos = strafeEncoder.getCurrentPosition();

        // Calculate change in ticks
        int deltaLeft = currentLeftPos - lastLeftPos;
        int deltaRight = currentRightPos - lastRightPos;
        int deltaStrafe = currentStrafePos - lastStrafePos;

        // Convert to inches
        double deltaLeftInches = deltaLeft / TICKS_PER_INCH;
        double deltaRightInches = deltaRight / TICKS_PER_INCH;
        double deltaStrafeInches = deltaStrafe / TICKS_PER_INCH;

        // Calculate change in heading (radians)
        double deltaTheta = (deltaRightInches - deltaLeftInches) / TRACK_WIDTH;

        // Calculate change in local position
        double deltaY = (deltaRightInches + deltaLeftInches) / 2.0;
        double deltaX = deltaStrafeInches * STRAFE_CORRECTION;

        // Update global position based on heading
        double deltaXGlobal = deltaY * Math.sin(theta) + deltaX * Math.cos(theta);
        double deltaYGlobal = deltaY * Math.cos(theta) - deltaX * Math.sin(theta);

        x += deltaXGlobal;
        y += deltaYGlobal;
        theta += deltaTheta;

        // Update last positions
        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;
        lastStrafePos = currentStrafePos;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() { return Math.toDegrees(theta); }
    public void setPosition(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
}
