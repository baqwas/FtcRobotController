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
 * forward travel:
 *       ^                   ^
 *       |                   |
 *       0 left front        2 right front
 *                 X
 *       ^                   ^
 *       |                   |
 *       1 left back         3 right back
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
 *      HD Hex Motor Reduction                  Bare Motor      40:1            20:1
 *      Free speed, RPM                         6,000           150             300
 *      Cycles per rotation of encoder shaft    28 (7 Rises)    28 (7 Rises)    28 (7 Rises)
 *      Ticks per rotation of output shaft      28              1120            560
 * TICKS_PER_MOTOR_REV = 560            REV HD Hex UltraPlanetary 20:1 cartridge
 * DRIVE_GEAR_REDUCTION = 1.0
 * WHEEL_DIAMETER_MM = 75.0             REV Mecanum wheel
 * MM_TO_INCH = 0.03937008
 * TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * MM_TO_INCH * PI)
 *                = 56.9887969189608
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;

@TeleOp(name = "Mecanum: Spin Direction", group = "Test")
//@Disabled

public class MecanumSpinDirection extends LinearOpMode {

    private static final  String TAG = MecanumSpinDirection.class.getSimpleName();
    Datalog datalog = new Datalog(TAG); // data logging for offline analysis/debugging

    static final double TICKS_PER_INCH = 56.9887969189608; // REV Robotics motor and wheel specific!
    TouchSensor touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .40, correction;
    boolean aButton, bButton, touched;
    private final ElapsedTime runtime = new ElapsedTime();
    // motor entities for drivetrain
    DcMotorEx[] motor = new DcMotorEx[]{
            hardwareMap.get(DcMotorEx.class, "motorLeftFront"),
            hardwareMap.get(DcMotorEx.class, "motorLeftBack"),
            hardwareMap.get(DcMotorEx.class, "motorRightFront"),
            hardwareMap.get(DcMotorEx.class, "motorRightBack")};
    int[] motorTicks = {0, 0, 0, 0};    // current tick count from encoder for the respective motors
    double[] motorPower = {0.0, 0.0, 0.0, 0.0};
    int drivetrainSteps = 0;

    /**
     * Function to backup the robot after a stop to ensure it does not
     * collide with any object. This function will require further tuning.
     */
    private void LinearTravel(double travelLength) {
        boolean travelCompleted = false;// = true when length has been traveled by robot
        double travelTicks = TICKS_PER_INCH * travelLength;

        while (opModeIsActive() && !travelCompleted)            // drive until end of period.
        {
            correction = checkDirection();  // Use gyro to drive in a straight line.

            telemetry.addData("1 IMU heading", lastAngles.firstAngle);
            telemetry.addData("2 Global heading", globalAngle);
            telemetry.addData("3 Correction", correction);
            telemetry.update();
            for (int i = 0; i < motor.length; i++) {
                motorTicks[i] = motor[i].getCurrentPosition();
            }
            if (motorTicks[0] > travelTicks) {
                travelCompleted = true;
            } else {

                motor[0].setPower(power - correction);
                motor[1].setPower(power + correction);
                motor[2].setPower(power + correction);
                motor[3].setPower(power - correction);

                aButton = gamepad1.a;       // allow teleop to change direction by 90 degrees
                bButton = gamepad1.b;
                //touched = touch.isPressed();
                if (/*touched || */aButton || bButton) {
                    backup();               // for a safer turn in the subsequent operation

                    if (/*touched || */aButton) rotate(-90, power); // turn 90 degrees right

                    if (bButton) rotate(90, power);             // turn 90 degrees left
                }
            }
        }
        fullStop();                     // turn the motors off
    }

    /**
     * Function to backup the robot after a stop to ensure it does not
     * collide with any object. This function will require further tuning.
     */
    private void backup() {
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(-0.4);
        }
        sleep(500);
        fullStop();
    }

    /**
     * Function to stop power to all defined motors in the arraylist motor[]
     * and to wait for 1 second.
     * The stopping action is controlled by the setting chosen for the motor
     * based on vendor specification by the program during the initialization.
     */
    private void fullStop() {
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);
        }
        sleep(500);
    }

    private void statusMessage(String caption, int number) {
        telemetry.addData(caption, "forward");
        telemetry.update();
        sleep(5000);
        motor[number].setPower(0.5);
        sleep(5000);
        motor[number].setPower(0.0);
        telemetry.addData(caption, "reverse");
        telemetry.update();
        motor[number].setPower(-0.5);
        sleep(5000);
        motor[number].setPower(0.0);
    }
    @Override
    public void runOpMode() throws InterruptedException   // called when init button is  pressed.
    {
        int motorNumber = 0;
        // default condition but play it safe anyway
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0); // setting a power level of zero will brake the motor
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // stops and then brakes, actively resisting any external force which attempts to turn the motor
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor
        }
        //touch = hardwareMap.touchSensor.get("sensorTouch");        // get a reference to touch sensor.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.NDOF;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imuGyro");        // Retrieve and initialize the IMU
        imu.initialize(parameters);

        double travelLength = 24.0;     // 12" linear

        telemetry.addData("Mode", "Calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())        // make sure the imu gyro is calibrated before continuing.
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Select Start");
        telemetry.addData("Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();                     // wait for Start button to be pressed for Linear OpMode

        telemetry.addData("Motor", "individual spin");
        telemetry.update();
        sleep(1000);
        // motor[0].setDirection(DcMotorSimple.Direction.REVERSE); // leftFrontMotor
        statusMessage("leftFront", 0);
        // motor[1].setDirection(DcMotorSimple.Direction.REVERSE); // leftFrontMotor
        statusMessage("leftBack", 1);
        // motor[2].setDirection(DcMotorSimple.Direction.REVERSE); // leftFrontMotor
        statusMessage("rightFront", 2);
        // motor[3].setDirection(DcMotorSimple.Direction.REVERSE); // leftFrontMotor
        statusMessage("rightBack", 3);
        telemetry.addData("Motor", "spin test complete");
        telemetry.update();
        sleep(60000);

        if (isStopRequested())          // has stopping opMode been requested
            return;
        while (opModeIsActive()) {
            // X axis for pivot turning
            // Y axis for forward/backward movement
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // normalize the power setting in the range {-1, 1}
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorPower[0] = (y + x + rx) / denominator;
            motorPower[1] = (y - x + rx) / denominator;
            motorPower[2] = (y - x - rx) / denominator;
            motorPower[3] = (y + x - rx) / denominator;

            for (int i = 0; i < motor.length; i++) {
                motor[i].setPower(motorPower[i]);
            }

            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            datalog.loopCounter.set(drivetrainSteps++);
            datalog.yaw.set(lastAngles.firstAngle);
            datalog.pitch.set(lastAngles.secondAngle);
            datalog.roll.set(lastAngles.thirdAngle);
            datalog.roll.set(globalAngle);
            datalog.writeLine();            // A timestamp is applied to the record when writing
        }
    }

    /**
     * Resets the cumulative angle tracking to zero and initializes the lastAngles entity
     */
    private void resetAngle() {
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
     * If the current cumulative heading angle is not zero then the robot is
     * not travel in a straight line. Use a simple proportional control to
     * correct the deviation.
     * <p>
     * The gain value is essentially the proportional control correction factor.
     * This value may require some tuning based on field experiments.
     * </p>
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
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
    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        resetAngle();                   // reset heading for tracking with IMU data


        if (degrees < 0) {                               // turn right
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {                               // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        motor[0].setPower(leftPower);   // set power to rotate
        motor[1].setPower(rightPower);
        motor[2].setPower(rightPower);
        motor[3].setPower(leftPower);
        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && getAngle() == 0) {
            }

            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else                                // left turn.
            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && getAngle() < degrees) {
            }

        fullStop();

        resetAngle();                          // reset angle tracking on new heading
    }

    /**
     * This class encapsulates all the fields that will go into the datalog.
     */
    private static class Datalog {
        /**
         * The underlying datalogger object - it cares only about an array of loggable fields
         * The fields for this OpCode are:
         * yaw, pitch, roll & battery - in that order
         */

        private final Datalogger datalogger;
        // The fields whose values will be written to the storage file
        // Note that order here is NOT important
        // The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField yaw = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField pitch = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField roll = new Datalogger.GenericField("Roll");
        public Datalogger.GenericField globalAngle = new Datalogger.GenericField("globalAngle");
        public Datalogger.GenericField deltaAngle = new Datalogger.GenericField("deltaAngle");
        public Datalogger.GenericField battery = new Datalogger.GenericField("Battery");

        /**
         * @param name the name of the file where the fields are written
         */
        public Datalog(String name) {
            datalogger = new Datalogger.Builder()   // Build the underlying datalog object

                    .setFilename(name)  // Pass through the filename
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS) // Request an automatic timestamp field
                    .setFields(         // specify the fields for logging in order expected in the log file
                            opModeStatus,
                            loopCounter,
                            yaw,
                            pitch,
                            roll,
                            globalAngle,
                            deltaAngle,
                            battery
                    )
                    .build();
        }

        /**
         * The operation to output one record of the fields to the storage file
         */
        public void writeLine() {
            datalogger.writeLine();
        }
    }
}