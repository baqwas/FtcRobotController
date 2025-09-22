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
 * @version 1.1
 * @param none
 * @return none
 * @exception none
 * @see https://stemrobotics.cs.pdx.edu/node/7266
 * <p>
 * This program registers as Autonomous OpMode in the FtcRobotController app.
 * The robot travels forward in a linear movement. When the touch sensor is pressed
 * it backs up a little enable a 90 degree turn. The bumper buttons on gamepad2
 * select the direction of the turn - left or right.
 * The program relies on the IMU sensor in the REV Robotics Control Hub that
 * runs the FtcRobotController app.
 * </p>
 * <p>
 * BNO055 IMU parameters for initialization:
 * @see https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/bosch/BNO055IMU.Parameters.html
 * BNO055IMU interface abstracts the functionality of the Bosch/Sensortec BNO055 Intelligent 9-axis absolute orientation sensor
 * The BNO055 can output the following sensor data (as described in AdaFruit Absolute Orientation Sensor).
 * - Absolute Orientation (Euler Vector, 100Hz) Three axis orientation data based on a 360° sphere
 * - Absolute Orientation (Quaterion, 100Hz) Four point quaternion output for more accurate data manipulation
 * - Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in rad/s
 * - Acceleration Vector (100Hz) Three axis of acceleration (gravity + linear motion) in m/s^2
 * - Magnetic Field Strength Vector (20Hz) Three axis of magnetic field sensing in micro Tesla (uT)
 * - Linear Acceleration Vector (100Hz) Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
 * - Gravity Vector (100Hz) Three axis of gravitational acceleration (minus any movement) in m/s^2
 * - Temperature (1Hz) Ambient temperature in degrees celsius
 * Of those, the first (the gravity-corrected absolute orientation vector) is arguably the most useful in FTC robot design.
 *
 * Calibration available are:
 * System - gyro, accelerometer and magnetometer
 * Gyro
 * Accelerometer
 * Magnetometer
 * </p>
 * https://en.wikipedia.org/wiki/PID_controller
 * https://www.csimn.com/CSI_pages/PIDforDummies.html
 *
 */

package org.firstinspires.ftc.teamcode.Holonomic;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Control.PIDController;

public class MecanumTravel {

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.30, correction, rotation;
    PIDController pidDrive, pidRotate;
    private DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    long travelTicks;
    double TICKS_PER_INCH;

    public void init(HardwareMap hwMap, BNO055IMU imux, DcMotorEx[] motorx, double COUNTS_PER_INCH) {
        for (int i = 0; i < motorx.length; i++) {
            motor[i] = motorx[i];
            motor[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        imu = imux;
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
     * Resets the cumulative angle tracking to zero and initializes the lastAngles entity
     * AxesReference - the axes reference in which the result will be expressed
     * EXTRINSIC | INTRINSIC
     * AxesOrder - the axes order in which the result will be expressed
     * AngleUnit - the angle units in which the result will be expressed
     * DEGREES | RADIANS
     */
    private void resetAngle() {
        // the absolute orientation of the sensor as a set three angles
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        // the absolute orientation of the sensor as a set three angles
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
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

        resetAngle();                   // reset heading for tracking with IMU data
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
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // rotate until turn is completed.
        if (degrees < 0) {                               // On right turn we have to get off zero first.
            // noinspection StatementWithEmptyBody
            while (getAngle() == 0) {
                motor[0].setPower(power);
                motor[3].setPower(power);
                motor[1].setPower(-power);
                motor[2].setPower(-power);
            }
            do {
                power = pidRotate.performPID(getAngle()); // -ve power on right turn
                motor[0].setPower(-power);
                motor[3].setPower(-power);
                motor[1].setPower(power);
                motor[2].setPower(power);
            } while (!pidRotate.onTarget());
        } else {                          // left turn
            do {
                power = pidRotate.performPID(getAngle()); // +ve power on left turn
                motor[0].setPower(-power);
                motor[3].setPower(-power);
                motor[1].setPower(power);
                motor[2].setPower(power);
            } while (!pidRotate.onTarget());
            robotPower(0.0);            // turn off the motors
            rotation = getAngle();
            sleep(500);                 // time to let rotation stop; empirical value
            resetAngle();               // reset angle for use in next heading
        }
    }
}

