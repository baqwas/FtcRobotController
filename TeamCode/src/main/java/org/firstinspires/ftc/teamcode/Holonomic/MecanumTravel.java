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
  <li> rotates by 90 degrees after initiwhen the touch sensor is pressed</li>
  <li> the turn direction is controlled with bumper buttons on the gamepad</li>
  <li> relies on IMU data to travel in a straight line as well to perform the turns</li>
  <li> displays a few status messages</li>
  </ul>
  @author modified by armw
 * @version 1.1 - Converted to Universal IMU Interface
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
 * Universal IMU parameters for initialization:
 * The universal IMU interface uses the IMU class and configures the hub's orientation
 * for robot-centric angles.
 * </p>
 * https://en.wikipedia.org/wiki/PID_controller
 * https://www.csimn.com/CSI_pages/PIDforDummies.html
 *
 */

package org.firstinspires.ftc.teamcode.Holonomic;

import static java.lang.Thread.sleep;

// Old BNO055IMU is replaced by the new IMU interface
// import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
// New IMU-related imports
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Control.PIDController;

public class MecanumTravel {

    // Change BNO055IMU to IMU
    // YawPitchRollAngles is used to store the orientation data
    IMU imu;
    YawPitchRollAngles robotOrientation; // New variable to get orientation data
    double globalAngle, power = 0.30, correction, rotation;
    PIDController pidDrive, pidRotate;
    private DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    long travelTicks;
    double TICKS_PER_INCH;

    // Change BNO055IMU to IMU in the init method's parameters
    public void init(HardwareMap hwMap, IMU imux, DcMotorEx[] motorx, double COUNTS_PER_INCH) {
        for (int i = 0; i < motorx.length; i++) {
            motor[i] = motorx[i];
            motor[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        imu = imux;

        // *** Universal IMU Initialization ***
        // Create an orientation parameter object.
        // This is a default orientation for a Hub mounted flat with logo up and USB ports forward.
        // You should adjust these parameters (LogoFacingDirection, UsbFacingDirection)
        // to match your robot's actual Control/Expansion Hub mounting.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Initialize the IMU with the specified parameters
        imu.initialize(parameters);
        // **********************************

        TICKS_PER_INCH = COUNTS_PER_INCH;
    }

    /**
     * Function to set power to all defined motors in the arraylist motor[]
     * and to wait for 0.5 seconds. If the power
     * The stopping action is controlled by the setting chosen for the motor
     * based on vendor specification by the program during the initialization.
     *
     * @param power the value applied to the power setting of all motors in the arraylist
     */
    public void robotPower(double power) throws InterruptedException {
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(power);
        }
        sleep(500);
    }

    public long averageTicks() {
        return (motor[0].getCurrentPosition() + motor[1].getCurrentPosition() + motor[2].getCurrentPosition() + motor[3].getCurrentPosition()) / 4;
    }

    public void DeltaTravel(double distance) throws InterruptedException {
        int rotationDegrees;            // rotation angle specified/required, degrees
        travelTicks = (long) (TICKS_PER_INCH * distance);
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        // parameters for tank travel
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        //rotationDegrees = 90;

        while (averageTicks() < travelTicks)                // drive until targetTicks reached
        {
            correction = pidDrive.performPID(getAngle());      // Use gryo with PID for straight line travel

            motor[0].setPower(power - correction);
            motor[1].setPower(power - correction);
            motor[2].setPower(power + correction);
            motor[3].setPower(power + correction);

        }
        robotPower(0.0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     * For the universal IMU, this is done using imu.resetYaw().
     */
    private void resetAngle() {
        // Use the dedicated resetYaw() method for the universal IMU interface
        imu.resetYaw(); //
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // Use the new getRobotYawPitchRollAngles() method to get robot-centric angles
        robotOrientation = imu.getRobotYawPitchRollAngles(); //

        // The universal IMU interface provides robot-centric angles (Yaw, Pitch, Roll)
        // Yaw is the robot's heading around the Z-axis, which is the value needed for the turn.
        // Yaw angle is returned as a continuous value, so the old manual processing to track
        // cumulative angle (deltaAngle < -180, etc.) is not strictly needed for basic turn
        // control unless you want to track total accumulated rotation for more than one full turn
        // of the robot. However, since the original code was written to handle the BNO055's
        // $\pm 180^{\circ}$ wrap-around, we'll simplify and use the `resetYaw()` and the
        // continuous heading provided by `getYaw()` for this conversion.

        // When using the universal IMU, you can directly get the Yaw angle
        // in degrees or radians. The original code used degrees for the PID loop.
        double currentAngle = robotOrientation.getYaw(AngleUnit.DEGREES); //

        // Since the old code performed cumulative angle tracking, we need to maintain
        // that logic structure or rely on resetYaw() followed by a direct angle reading.
        // For a simple PID turn (which this is), we rely on resetYaw() setting the start
        // angle to 0, and currentAngle is the new heading.

        // Since resetYaw() is used in resetAngle(), currentAngle will be the heading
        // relative to the last reset.
        return currentAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * <p>
     * getAngle() returns + when rotating counter clockwise (left) and - when rotating
     * clockwise (right).
     * </p>
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power) throws InterruptedException {
        double leftPower, rightPower;

        resetAngle();                   // reset heading to 0 for tracking with IMU data
        // if degrees > 359 cap at 359 with same sign as original value
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        /*
         * start PID controller
         * monitor the turn angle with respect to the
         * target angle and reduce power when approaching the target angle
         * This action prevents the robot's momentum from overshooting
         * the turn after the power is disengaged.
         * The PID controller reports onTarget() = true when the difference
         * between turn angle and target angle is within
         * 1% of target (tolerance) which is about 1 degree.
         * This helps prevent overshoot which is
         * dependant on the motor and gearing configuration, starting power,
         * weight of the robot and the on target tolerance.
         * If the controller overshoots, it will reverse the sign of the output
         * turning the robot back toward the setpoint value.
         */

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        // Input range is set for the PID controller to manage the range of the IMU's output
        pidRotate.setInputRange(-180, 180); // Yaw is typically $\pm 180$ degrees
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // rotate until turn is completed.
        if (degrees < 0) {                               // On right turn we have to get off zero first.
            // When using imu.resetYaw(), the angle starts at 0, so the 'get off zero' logic might not be needed,
            // but we'll retain the `do-while` loop for PID control until it hits the target.
            // The original logic here seems to be flawed for an IMU returning 0, so it's adjusted
            // to just enter the PID loop.

            do {
                power = pidRotate.performPID(getAngle()); // -ve power on right turn (negative degrees)
                motor[0].setPower(-power); // Front Left
                motor[3].setPower(-power); // Back Right
                motor[1].setPower(power);  // Front Right
                motor[2].setPower(power);  // Back Left
            } while (!pidRotate.onTarget());

        } else {                          // left turn
            do {
                power = pidRotate.performPID(getAngle()); // +ve power on left turn (positive degrees)
                motor[0].setPower(-power); // Front Left
                motor[3].setPower(-power); // Back Right
                motor[1].setPower(power);  // Front Right
                motor[2].setPower(power);  // Back Left
            } while (!pidRotate.onTarget());
        }

        robotPower(0.0);            // turn off the motors
        rotation = getAngle();
        sleep(500);                 // time to let rotation stop; empirical value
        resetAngle();               // reset angle for use in next heading
    }
}