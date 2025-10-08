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
  A basic program that:
  <ul>
  <li> travels in a linear movement</li>
  <li> turns 90 degrees when the touch sensor is pressed</li>
  <li> the turn direction is controlled with bumper buttons on the gamepad</li>
  <li> relies on IMU data to travel in a straight line as well to perform the turns</li>
  <li> displays a few status messages</li>
  </ul>
  @author modified by armw
 * @version 1.2 - Converted to Universal IMU Interface
 * @param none
 * @return none
 * @exception none
 * @see https://stemrobototics.cs.pdx.edu/node/7266
 * <p>
 * This program registers as Autonomous OpMode in the FtcRobotController app.
 * The robot travels forward in a linear movement. When the touch sensor is pressed
 * it backs up a little enable a 90 degree turn. The bumper buttons on gamepad2
 * select the direction of the turn - left or right.
 * The program relies on the IMU sensor in the REV Robotics Control Hub that
 * runs the FtcRobotController app.
 * </p>
 * <p>
 * forward travel:
 * ^                   ^
 * |                   |
 * 0 left front        2 right front
 * X
 * ^                   ^
 * |                   |
 * 1 left back         3 right back
 *
 * hard coded numbers to avoid the use of enum construct for such a simple program
 * motor positions:
 * <ul>
 * <li>0 = left front (or forward or fore)</li>
 * <li>1 = left back (or rear or aft)</li>
 * <li>2 = right front (or forward or fore)</li>
 * <li>3 = right back (or rear or aft)</li>
 *</ul>
 * Initialize the hardware variables. Note that the strings used here as parameters
 * to 'get' must correspond to the names assigned during the robot configuration
 * step (using the FTC Robot Controller app on the phone).
 * Moon Mechanics nomenclature options for motors:
 * <device><port|starboard>|<stern/aft>
 * <role><qualifier>
 * </p>
 * @see https://first-tech-challenge.github.io/SkyStone/com/qualcomm/robotcore/hardware/DcMotor.html
 *
 * @see https://docs.revrobotics.com/rev-control-system/sensors/encoders/motor-based-encoders
 * HD Hex Motor (REV-41-1291) Encoder Specifications
 * HD Hex Motor Reduction                  Bare Motor      40:1            20:1
 * Free speed, RPM                         6,000           150             300
 * Cycles per rotation of encoder shaft    28 (7 Rises)    28 (7 Rises)    28 (7 Rises)
 * Ticks per rotation of output shaft      28              1120            560
 * TICKS_PER_MOTOR_REV = 560            REV HD Hex UltraPlanetary 20:1 cartridge
 * DRIVE_GEAR_REDUCTION = 1.0
 * WHEEL_DIAMETER_MM = 75.0             REV Mecanum wheel
 * MM_TO_INCH = 0.03937008
 * TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * MM_TO_INCH * PI)
 * = 56.9887969189608
 * <p>
 * Hardware map
 * Device name      Control Hub setting
 * imu              I2C bus 0
 * motorLeftFront   port 0
 * motorLeftBack    port 1
 * motorRightFront  port 2
 * motorRightBack   port 3
 * sensorTouch      n/a
 * sensorLED        n/a
 * gamepad2         USB2
 * </p>
 */

package org.firstinspires.ftc.teamcode.Holonomic;

// *** Universal IMU Imports ***

import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Preview Event Mecanum Autonomous", group = "Match", preselectTeleOp="Preview Event TeleOp")
//@Disabled

public class MecanumAutonomous extends LinearOpMode
{
    private static final  String TAG = MecanumAutonomous.class.getSimpleName(); // for use in logging
    //Datalogger datalog = new Datalogger(TAG,null);
    // === HARDWARE DECLARATIONS ===
    private DcMotorEx motorLeftFront=null, motorLeftBack=null, motorRightFront=null, motorRightBack=null;
    private IMU imu;
    // === CONSTANTS & GAINS (TUNE THESE!) ===
    private static final double POSITION_TOLERANCE = 1.0; // Inches
    private static final double HEADING_TOLERANCE = 2.0;  // Degrees
    private static final double MAX_MOVE_POWER = 0.8;
    private static final double MAX_AUTO_TURN = 0.6;
    private static final double MIN_MOVE_POWER = 0.15;

    // PID Gains for Driving Straight (Axial Movement)
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KI = 0.0001;
    private static final double DRIVE_KD = 0.005;

    // PID Gains for Turning (Yaw Movement)
    private static final double TURN_KP = 0.03;
    private static final double TURN_KI = 0.0002;
    private static final double TURN_KD = 0.002;

    // P-Gain for Heading Correction while Driving (Yaw Correction)
    private static final double HEADING_CORRECTION_KP = 0.01;

    // Encoder Conversion Factors (YOU MUST MEASURE AND SET THESE)
    private static final double WHEEL_DIAMETER_INCHES = 86.0 / 25.4; // SWYFT Drive v2, Mecanum wheel dia = 86 mm
    private static final double TICKS_PER_MOTOR_REV = 4.0 * 7.0; // Example for REV 5203 series 312 RPM motor
    private static final double GEAR_RATIO = 12.7; // approx 475 rpm
    private static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * PI);

    // === ROBOT STATE (Simple Odometry/Position Tracking) ===
    // Note: A real system would use separate dead-wheel odometry for better accuracy.
    protected double robotX = 0;
    protected double robotY = 0;
    protected double robotHeading = 0; // Updated by getHeading()

    // --- WAYPOINT CLASS ---
    private static class Waypoint {
        public double x, y, heading;
        public Waypoint(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }

    // =====================================================================================
    //                                  MAIN OPMODE EXECUTION
    // =====================================================================================

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        // Set initial assumed position (start of autonomous)
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = getHeading(); // Read current IMU value

        telemetry.addData("Status", "Hardware Initialized & IMU Calibrated");
        telemetry.update();
        // --- AUTONOMOUS SEQUENCE ---
        Waypoint firstTarget = new Waypoint(0, 12, 0);  // Move 24 inches forward, end facing 90 deg
        Waypoint secondTarget = new Waypoint(12, 0, 0); // Move to (24, 24), end facing 0 deg

        waitForStart();

        if (opModeIsActive()) {
            // NOTE: The Waypoint position update logic below is simplified.
            // A professional FTC team would update robotX/Y/Heading continuously via Odometry.

            // 1. Execute First Waypoint
            driveToWaypoint_IMU_Encoders(firstTarget);

            // 2. After movement, update the robot's estimated position for the next move
            // This is a crude update! Rely on external Odometry for accuracy.
            robotX = firstTarget.x;
            robotY = firstTarget.y;
            robotHeading = firstTarget.heading;

            // 3. Execute Second Waypoint
            // driveToWaypoint_IMU_Encoders(secondTarget);

            // ... etc.
        }
    }

    // =====================================================================================
    //                                HARDWARE & UTILITY METHODS
    // =====================================================================================

    private void initializeHardware() {
        // --- Drive Motors ---
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

        motorLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorEx.Direction.FORWARD);

        resetDriveEncoders();
        setDriveRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // --- Universal IMU ---
        imu = hardwareMap.get(IMU.class, "imu");

        // Define Control Hub orientation (REQUIRED for Universal IMU)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);

        // Reset the yaw to make the current heading 0 degrees
        imu.resetYaw();
    }

    /**
     * Reads the robot's current heading (Yaw) from the IMU.
     * @return Normalized heading in degrees (-180 to 180).
     */
    protected double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return AngleUnit.normalizeDegrees(orientation.getYaw(AngleUnit.DEGREES));
    }

    private void setDriveRunMode(DcMotorEx.RunMode mode) {
        motorLeftFront.setMode(mode);
        motorLeftBack.setMode(mode);
        motorRightFront.setMode(mode);
        motorRightBack.setMode(mode);
    }

    protected void resetDriveEncoders() {
        setDriveRunMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setDriveRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
//    /**
//     * Calculates the average distance traveled using the four drive motor encoders.
//     * @return Average distance traveled in inches.
//     */
    protected double getDistanceTraveled() {
        double avgTicks = (abs(motorLeftFront.getCurrentPosition()) +
                abs(motorLeftBack.getCurrentPosition()) +
                abs(motorRightFront.getCurrentPosition()) +
                abs(motorRightBack.getCurrentPosition())) / 4.0;
        return avgTicks / TICKS_PER_INCH;
    }


    /**
     * Low-level control for Mecanum drive based on axial, lateral, and yaw power inputs.
     * axial (forward) is positive
     * lateral (strafe right) is positive
     * yaw (counter-clockwise rotation) is positive
     */
    protected void moveRobot(double axial, double lateral, double yaw) {
        double powerLeftFront = axial + lateral + yaw;
        double powerRightFront = axial - lateral - yaw;
        double powerLeftBack = axial - lateral + yaw;
        double powerRightBack = axial + lateral - yaw;

        // Normalize the values so no motor power exceeds MAX_MOVE_POWER
        double max = max(abs(powerLeftFront), abs(powerRightFront));
        max = max(max, abs(powerLeftBack));
        max = max(max, abs(powerRightBack));

        if (max > 1.0) {
            powerLeftFront /= max;
            powerRightFront /= max;
            powerLeftBack /= max;
            powerRightBack /= max;
        }

        motorLeftFront.setPower(powerLeftFront);
        motorRightFront.setPower(powerRightFront);
        motorLeftBack.setPower(powerLeftBack);
        motorRightBack.setPower(powerRightBack);
    }

    // =====================================================================================
    //                                  PID NAVIGATION
    // =====================================================================================
    /**
     * Executes point-to-point travel using IMU and Motor Encoders with PID control.
     */
    private void driveToWaypoint_IMU_Encoders(Waypoint target) {
        // --- 1. Calculate Relative Movement Targets ---
        // Calculate required change in X and Y from current position (robotX/Y)
        double relativeX = target.x - robotX;
        double relativeY = target.y - robotY;

        double targetDistance = hypot(relativeX, relativeY);

        // Calculate the absolute angle the robot needs to face (atan2 uses Y, X)
        // AngleUnit.DEGREES.fromUnit(AngleUnit.RADIANS, Math.atan2(relativeY, relativeX))
        double angleToTarget = toDegrees(atan2(relativeY, relativeX));

        // Calculate the turn needed from current heading (getHeading())
        double turnAngle = AngleUnit.normalizeDegrees(angleToTarget - getHeading());

        // --- 2. Execute Movement Sub-tasks ---

        // A. Turn to Face the Target (IMU PID)
        if (abs(turnAngle) > HEADING_TOLERANCE) {
            telemetry.addData("Status", "Turning to Face Target");
            telemetry.update();
            turnPID(turnAngle);
        }

        // B. Drive the Distance (Encoder/IMU PID)
        if (targetDistance > POSITION_TOLERANCE) {
            telemetry.addData("Status", "Driving Straight");
            telemetry.update();
            // The robot drives on the field heading it just turned to face (angleToTarget)
            driveStraightPID(targetDistance, angleToTarget);
        }

        // C. Final Turn to Target Heading (IMU PID)
        double finalTurn = AngleUnit.normalizeDegrees(target.heading - getHeading());
        if (abs(finalTurn) > HEADING_TOLERANCE) {
            telemetry.addData("Status", "Final Turn");
            telemetry.update();
            turnPID(finalTurn);
        }

        // 3. Stop the Robot
        moveRobot(0, 0, 0);
        telemetry.addData("Status", "Waypoint Reached!");
        telemetry.update();
        sleep(100);
    }

    /**
     * Executes a turn of a specified angle using IMU and PID control.
     * @param angleChange The relative angle to turn.
     */
    private void turnPID(double angleChange) {
        double targetHeading = AngleUnit.normalizeDegrees(getHeading() + angleChange);

        double lastError = 0;
        double integral = 0;
        double timeout = getRuntime() + 3.0;

        while (opModeIsActive() && getRuntime() < timeout) {
            double error = AngleUnit.normalizeDegrees(targetHeading - getHeading());

            // Check for completion
            if (abs(error) < HEADING_TOLERANCE && abs(lastError) < HEADING_TOLERANCE) break;

            // PID Calculations
            double dt = 0.01; // Assuming 10ms loop time
            integral += error * dt;
            double derivative = (error - lastError) / dt;

            double power = (error * TURN_KP) + (integral * TURN_KI) + (derivative * TURN_KD);

            // Apply power limiting and minimum power
            power = Range.clip(power, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            if (abs(power) < MIN_MOVE_POWER && abs(error) > HEADING_TOLERANCE) {
                power = copySign(MIN_MOVE_POWER, power);
            }

            moveRobot(0, 0, power); // Only applying Yaw power

            lastError = error;

            telemetry.addData("Turn Error", "%.2f", error);
            telemetry.addData("Turn Power", "%.2f", power);
            telemetry.update();
            sleep(10);
        }
        moveRobot(0, 0, 0);
    }

    /**
     * Drives the robot straight a specified distance using encoder PID, with IMU for heading correction.
     * @param distance The distance to travel (inches).
     * @param targetHeading The field-centric heading to maintain while driving.
     */
    private void driveStraightPID(double distance, double targetHeading) {
        resetDriveEncoders();

        double lastDriveError = 0;
        double driveIntegral = 0;
        double timeout = getRuntime() + 5.0;

        while (opModeIsActive() && getRuntime() < timeout) {
            double currentDistance = getDistanceTraveled();
            double driveError = distance - currentDistance;

            // Check for completion
            if (abs(driveError) < POSITION_TOLERANCE && abs(lastDriveError) < POSITION_TOLERANCE) break;

            // 1. Distance (Axial) PID Calculations
            double dt = 0.01;
            driveIntegral += driveError * dt;
            double driveDerivative = (driveError - lastDriveError) / dt;

            double axialPower = (driveError * DRIVE_KP) + (driveIntegral * DRIVE_KI) + (driveDerivative * DRIVE_KD);

            // 2. Heading (Yaw) Correction P-Control (uses IMU)
            double headingError = AngleUnit.normalizeDegrees(targetHeading - getHeading());
            double yawCorrection = Range.clip(headingError * HEADING_CORRECTION_KP, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            // Apply power limiting and minimum power to Axial drive
            axialPower = Range.clip(axialPower, -MAX_MOVE_POWER, MAX_MOVE_POWER);
            if (abs(axialPower) < MIN_MOVE_POWER && abs(driveError) > POSITION_TOLERANCE) {
                axialPower = copySign(MIN_MOVE_POWER, axialPower);
            }

            // Apply Powers (No lateral/strafe movement for driveStraight)
            moveRobot(axialPower, 0, yawCorrection);

            lastDriveError = driveError;

            telemetry.addData("Dist Error (in)", "%.2f", driveError);
            telemetry.addData("Heading Error (deg)", "%.2f", headingError);
            telemetry.update();
            sleep(10);
        }
        moveRobot(0, 0, 0);
    }
}