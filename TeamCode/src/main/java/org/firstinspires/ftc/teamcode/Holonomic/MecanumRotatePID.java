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
 * Rotations:
 *  Button      Button  Rotation
 *  square      X       45
 *  triangle    Y       135
 *  circle      B       180
 *  cross       A       90
 */

package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;

@TeleOp(name = "Mecanum: Rotate PID", group = "Test")
@Disabled

public class MecanumRotatePID extends LinearOpMode
{
    private static final  String TAG = MecanumRotatePID.class.getSimpleName();
    Datalog datalog = new Datalog(TAG); // data logging for offline analysis/debugging

    TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    PIDController           pidRotate;
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;
    int                     rotationSteps = 0;  // step counter for the iterations during the rotate movement
    // motor entities for drivetrain
    String[] motorLabels = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    private final DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    private final double[] motorPower = {0.0, 0.0, 0.0, 0.0};
    /**
     * Function to stop power to all defined motors in the arraylist motor[]
     * and to wait for 1 second.
     * The stopping action is controlled by the setting chosen for the motor
     * based on vendor specification by the program during the initialization.
     */
    private void fullStop() {
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);
            dcMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // see note, if any, on vendor specs for the corresponding motor
            dcMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        sleep(500);
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double GetAngle()
    {
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

        datalog.yaw.set(angles.firstAngle);
        datalog.pitch.set(angles.secondAngle);
        datalog.roll.set(angles.thirdAngle);
        datalog.globalAngle.set(globalAngle);
        datalog.deltaAngle.set(deltaAngle);

        datalog.writeLine();            // A timestamp is applied to the record when writing
        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero and initializes the lastAngles entity
     *     AxesReference - the axes reference in which the result will be expressed
     *                      EXTRINSIC | INTRINSIC
     *     AxesOrder - the axes order in which the result will be expressed
     *     AngleUnit - the angle units in which the result will be expressed
     *                      DEGREES | RADIANS
     */
    private void ResetAngle()
    {
        // the absolute orientation of the sensor as a set three angles
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * <p>
     * getAngle() returns + when rotating counter clockwise (left) and - when rotating
     * clockwise (right).
     * </p>
     * @param degrees Degrees to turn, + is left - is right
     */
    private void RotatePID(int degrees, double power) {
        double currentPower, leftPower, rightPower;

        ResetAngle();                   // reset heading for tracking with IMU data

        if (Math.abs(degrees) > 359)
            degrees = (int) Math.copySign(359, degrees); // upper limit @ 359 with original sign

        telemetry.addData("RotatePID", "degrees=%d, power=%.2f", degrees, power);
        telemetry.update();

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setOutputRange(0, power); // last lower limit was 0.7
        pidRotate.setInputRange(0, degrees);
        pidRotate.setTolerance(1);      // +- 1 degree
        pidRotate.enable();

        if (degrees < 0) {              // turn right
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {       // turn left
            leftPower = -power;
            rightPower = power;
        } else
            return;

        datalog.powerLeft.set(leftPower);
        datalog.powerRight.set(rightPower);

        // rotate until turn is completed
        if (degrees < 0) {                      // On right turn we have to get off zero first
            while (opModeIsActive() && GetAngle() == 0) {
                motor[0].setPower(leftPower);   // set power to rotate
                motor[1].setPower(leftPower);
                motor[2].setPower(rightPower);
                motor[3].setPower(rightPower);
                datalog.powerLeft.set(leftPower);
                sleep(50L);
            }
            do {
                currentPower = pidRotate.performPID(GetAngle());// -ve power for clockwise turn
                motor[0].setPower(-currentPower);// set power to rotate
                motor[1].setPower(-currentPower);
                motor[2].setPower(currentPower);
                motor[3].setPower(currentPower);
                datalog.currentPower.set(currentPower);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else
        {
            do {
                currentPower = pidRotate.performPID(GetAngle());// +ve power for counter clockwise turn
                motor[0].setPower(-currentPower);// set power to rotate
                motor[1].setPower(-currentPower);
                motor[2].setPower(+currentPower);
                motor[3].setPower(+currentPower);
                datalog.currentPower.set(currentPower);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        fullStop();

        telemetry.addData("All done", "degrees=%d, power=%.2f, currentpower=%.2f", degrees, power, currentPower);
        telemetry.update();

        ResetAngle();                               // reset angle tracking on new heading
    }

    @Override
    /*
     * basic runOpMode
     */
    public void runOpMode() throws InterruptedException   // called when init button is  pressed.
    {
        int rotationDegrees;                        // rotation angle specified/required, degrees

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/bosch/BNO055IMU.SensorMode.html
        parameters.mode                = BNO055IMU.SensorMode.NDOF;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        String deviceName = "imu";
        String rotations = "Rotations: X/□:45 A/X:90 Y/Δ:-45 B/O:-90";
        /* (first) device with specified class which is also an instance of the indicated class or interface
         * https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/bosch/BNO055IMU.html
         */
        imu = hardwareMap.get(BNO055IMU.class, deviceName);  // Retrieve the entity reference
        if (imu == null) {
            telemetry.addData("IMU", "device name " + deviceName + " not detected");
            telemetry.update();
            sleep(60000);                 // really need to stop the run!
        }
        /*
         * Initialize the sensor using the indicated set of parameters
         * This method can take a fairly long while, possibly several tens of milliseconds
         */
        if (!imu.initialize(parameters))            // N.B. if imu is not detected in the previous statement then the corresponding variable will null
            telemetry.addData("IMU", "Initialization unsuccessful");
        else
            telemetry.addData("Mode", "Calibrating...");
        telemetry.update();

        // Initialize the hardware variables
        // The strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        for (int i = 0; i < motor.length; i++) {
            motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]);
            /* motor stops and then brakes
             * actively resisting any external force
             * which attempts to turn the motor */
            motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            // run at any velocity with specified power level
            motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        // is the gyro is fully calibrated?
        while (!isStopRequested() && !imu.isGyroCalibrated())        // cannot continue until gyro is calibrated
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Calibration status", imu.getCalibrationStatus().toString());

        /*
         * The logical direction in which this motor operates: FORWARD | REVERSE
         * REV Robotics motors may need two of the following
         * four statements to be enabled - PLEASE TEST before 1st use!
         */
        // motor[0].setDirection(DcMotorEx.Direction.FORWARD); // motorLeftFront
        // motor[1].setDirection(DcMotorEx.Direction.FORWARD); // motorLeftBack
        motor[2].setDirection(DcMotorEx.Direction.REVERSE); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.REVERSE); // motorRightBack

        pidRotate = new PIDController(0.05, 0.005, 0.0005);

        telemetry.addLine(rotations);
        telemetry.addLine("Start: press PLAY");
        telemetry.update();

        waitForStart();                     // wait for Start button

        //rotationDegrees = 90;

        while (opModeIsActive())                // drive until end of TeleOps (for testing use only presently)
        {
            int rotationAngle = 0;
            //correction = checkDirection();      // Use gryo to maintain heading

            telemetry.addData("1 IMU heading", lastAngles.firstAngle);
            telemetry.addData("2 Global heading", globalAngle);
            telemetry.addData("3 Correction", correction);
            telemetry.addLine(rotations);
            telemetry.update();

            if (gamepad1.x) {           // square X button
                rotationAngle = 45;
            }
            else if (gamepad1.a) {           // cross A button
                rotationAngle = 90;
            }
            else if (gamepad1.y) {           // triangle Y button
                rotationAngle = -45;
            }
            else if (gamepad1.b) {           // circle B button
                rotationAngle = -90;
            }
            if (Math.abs(rotationAngle) > 0) {
                RotatePID(rotationAngle, power);
                sleep(100L);
            }
        }
        fullStop();
    }

    /**
     * If the current cumulative heading angle is not zero then the robot is
     * not travel in a straight line. Use a simple proportional control to
     * correct the deviation.
     * <p>
     *     The gain value is essentially the proportional control correction factor.
     *     This value may require some tuning based on field experiments.
     * </p>
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        double correction, angle, gain = .10;

        angle = GetAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }


    /**
     * This class encapsulates all the fields that will go into the datalog.
     */
    private static class Datalog
    {
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
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField yaw          = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField pitch        = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField roll         = new Datalogger.GenericField("Roll");
        public Datalogger.GenericField mode         = new Datalogger.GenericField("mode");
        public Datalogger.GenericField globalAngle  = new Datalogger.GenericField("globalAngle");
        public Datalogger.GenericField deltaAngle   = new Datalogger.GenericField("deltaAngle");
        public Datalogger.GenericField currentAngle = new Datalogger.GenericField("currentAngle");
        public Datalogger.GenericField currentPower = new Datalogger.GenericField("currentPower");
        public Datalogger.GenericField powerLeft    = new Datalogger.GenericField("powerLeft");
        public Datalogger.GenericField powerRight   = new Datalogger.GenericField("powerRight");
        //public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");
        /**
         *
         * @param name the name of the file where the fields are written
         */
        public Datalog(String name)
        {
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
                            powerLeft,
                            powerRight
                            //battery
                    )
                    .build();
        }

        /**
         *  The operation to output one record of the fields to the storage file
         */
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}