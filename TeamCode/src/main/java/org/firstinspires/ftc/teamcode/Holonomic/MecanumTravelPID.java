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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;

@TeleOp(name = "Mecanum: Travel PID", group = "Test")
@Disabled

public class MecanumTravelPID extends LinearOpMode {
    private static final  String TAG = MecanumTravelPID.class.getSimpleName(); // for use in logging
    Datalog datalog = new Datalog(TAG);
    TouchSensor touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Position position;
    Velocity velocity;

    double globalAngle, power = .30, correction, rotation;
    boolean buttonA, buttonB, touched;
    PIDController pidDrive, pidRotate;
    int rotationSteps = 0;  // step counter for the iterations during the rotate movement
    // motor entities for drivetrain
    String[] motorLabels = {
            "motorLeftFront",           // port 0 Control Hub
            "motorLeftBack",            // port 1 Control Hub
            "motorRightFront",          // port 2 Control Hub
            "motorRightBack"            // port 3 Control Hub
    };
    private final DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    private final double[] motorPower = {0.0, 0.0, 0.0, 0.0};
    private final int[] motorTicks = {0, 0, 0, 0};  // current ticks count for the respective motors

    /**
     * Function to set power to all defined motors in the arraylist motor[]
     * and to wait for 0.5 seconds. If the power
     * The stopping action is controlled by the setting chosen for the motor
     * based on vendor specification by the program during the initialization.
     *
     * @param power the value applied to the power setting of all motors in the arraylist
     */
    private void robotPower(double power) {
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(power);
        }
        sleep(500);
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
            dcMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // see note, if any, on vendor specs for the corresponding motor
            dcMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        sleep(500);
    }
    @Override
    /**
     * basic runOpMode fpr the OpMode
     * for testing purposes only
     * in testing
     */
    public void runOpMode() throws InterruptedException   // called when init button is  pressed.
    {
        int rotationDegrees;            // rotation angle specified/required, degrees
        int encoderLeftFront;           // current ticks counted for left front motor
        double travelDistance = 24.0;   // initial distance to clear 1st tile + tolerance buffer = 24" + tolerance buffer
        long travelTicks;
        //double TICKS_PER_INCH = 31.04667207; // REV Core Hex
        //double TICKS_PER_INCH = 56.98879692; // REV HD Hex
        //double TICKS_PER_INCH = 85.67563466; // goBILDA 5203 13.7
        double TICKS_PER_INCH = 45.283963; // goBILDA 5203 19.2

        telemetry.addData("imu", "calibrating ...");
        telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/bosch/BNO055IMU.SensorMode.html
        parameters.mode = BNO055IMU.SensorMode.NDOF;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        String deviceName = "imu";
        /* (first) device with specified class which is also an instance of the indicated class or interface
         * https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/bosch/BNO055IMU.html
         */
        imu = hardwareMap.get(BNO055IMU.class, deviceName);  // Retrieve the entity reference
        if (imu == null) {
            telemetry.addData("IMU", "device name " + deviceName + " not detected");
            telemetry.update();
            sleep(60000);    // really need to stop the run!
        }
        /*
         * Initialize the sensor using the indicated set of parameters
         * This method can take a fairly long while, possibly several tens of milliseconds
         */
        if (!imu.initialize(parameters)) // N.B. if imu is not detected in the previous statement then the corresponding variable will null
            telemetry.addData("IMU", "Initialization unsuccessful");
        else
            telemetry.addData("Mode", "Calibrating...");
        telemetry.update();


        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

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
        /*
         * The logical direction in which this motor operates: FORWARD | REVERSE
         * REV Robotics motors may need two of the following
         * four statements to be enabled - PLEASE TEST before 1st use!
         */
        // motor[0].setDirection(DcMotorSimple.Direction.FORWARD); // motorLeftFront
        // motor[1].setDirection(DcMotorSimple.Direction.FORWARD); // motorLeftBack
        motor[2].setDirection(DcMotorEx.Direction.REVERSE); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.REVERSE); // motorRightBack
        // is the gyro is fully calibrated?
        while (!isStopRequested() && !imu.isGyroCalibrated())        // cannot continue until gyro is calibrated
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Calibration status", imu.getCalibrationStatus().toString());
        encoderLeftFront = motor[0].getCurrentPosition(); // current ticks counted for left front motor

        travelTicks = (long) (TICKS_PER_INCH * travelDistance);
        telemetry.addData("Position, distance",  "%d, %d", encoderLeftFront, travelTicks);
        telemetry.addData("Start", "Press PLAY");
        telemetry.update();
        fullStop();
        waitForStart();                     // wait for Start button

        // parameters for tank travel
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        //rotationDegrees = 90;


        while (opModeIsActive() && Math.abs(encoderLeftFront) < travelTicks)                // drive until end of TeleOps (for testing use only presently)
        {
            correction = pidDrive.performPID(getAngle());      // Use gryo with PID for straight line travel

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);// abs orientation of sensor as a set three angles
            gravity  = imu.getGravity();// the acceleration vector of gravity relative to the IMU
            position  = imu.getPosition();// current position of sensor vy doubly integrating the observed sensor accelerations

            telemetry.addData("1 IMU heading", lastAngles.firstAngle);
            telemetry.addData("2 Global heading", globalAngle);
            telemetry.addData("3 Correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.addData("5 Encoder: target, current", "%d, %d", travelTicks, encoderLeftFront);

            // don't ask me why this works!!!
            motor[0].setPower(power - correction);
            motor[1].setPower(power - correction);
            motor[2].setPower(power + correction);
            motor[3].setPower(power + correction);

            buttonA = gamepad1.a;
            buttonB = gamepad1.b;
            if (buttonA || buttonB) {
                robotPower(-0.3);       // backup
                robotPower(0.0);        // stop
                if (buttonA) {
                    rotate(-90, 0.3);
                } else if (buttonB) {
                    rotate(90, 0.3);
                }
            }
            encoderLeftFront = motor[0].getCurrentPosition(); // current ticks counted for left front motor
            datalog.positionx.set(position.x);
            datalog.positiony.set(position.y);
            datalog.positionz.set(position.z);
            datalog.gx.set(gravity.xAccel);
            datalog.gy.set(gravity.yAccel);
            datalog.gz.set(gravity.zAccel);
            datalog.yaw.set(angles.firstAngle);
            datalog.pitch.set(angles.secondAngle);
            datalog.roll.set(angles.thirdAngle);
            datalog.writeLine();

            telemetry.addData("Position", encoderLeftFront);
            telemetry.update();
        }
        fullStop();
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

        datalog.yaw.set(angles.firstAngle);
        datalog.pitch.set(angles.secondAngle);
        datalog.roll.set(angles.thirdAngle);
        datalog.globalAngle.set(globalAngle);
        datalog.deltaAngle.set(deltaAngle);
        datalog.correction.set(correction);
        datalog.motor0.set(motor[0].getPower());
        datalog.motor1.set(motor[1].getPower());
        datalog.motor2.set(motor[2].getPower());
        datalog.motor3.set(motor[3].getPower());
        datalog.writeLine();            // A timestamp is applied to the record when writing

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
    private void rotate(int degrees, double power) {
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
            while (opModeIsActive() && getAngle() == 0) {
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
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else {                          // left turn
            do {
                power = pidRotate.performPID(getAngle()); // +ve power on left turn
                motor[0].setPower(-power);
                motor[3].setPower(-power);
                motor[1].setPower(power);
                motor[2].setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
            robotPower(0.0);            // turn off the motors
            rotation = getAngle();
            sleep(100L);                 // time to let rotation stop; empirical value
            resetAngle();               // reset angle for use in next heading
        }
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField yaw          = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField pitch        = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField roll         = new Datalogger.GenericField("Roll");
        public Datalogger.GenericField globalAngle  = new Datalogger.GenericField("GlobalAngle");
        public Datalogger.GenericField deltaAngle   = new Datalogger.GenericField("DeltaAngle");
        public Datalogger.GenericField correction   = new Datalogger.GenericField("Correction");
        public Datalogger.GenericField gx             = new Datalogger.GenericField("gravityX");
        public Datalogger.GenericField gy             = new Datalogger.GenericField("gravityX");
        public Datalogger.GenericField gz             = new Datalogger.GenericField("gravityX");
        public Datalogger.GenericField positionx      = new Datalogger.GenericField("X");
        public Datalogger.GenericField positiony      = new Datalogger.GenericField("Y");
        public Datalogger.GenericField positionz      = new Datalogger.GenericField("Z");
        public Datalogger.GenericField targetTicks  = new Datalogger.GenericField("TargetTicks");
        public Datalogger.GenericField motor0       = new Datalogger.GenericField("Motor0");
        public Datalogger.GenericField motor1       = new Datalogger.GenericField("Motor1");
        public Datalogger.GenericField motor2       = new Datalogger.GenericField("Motor2");
        public Datalogger.GenericField motor3       = new Datalogger.GenericField("Motor3");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            yaw,
                            pitch,
                            roll,
                            globalAngle,
                            deltaAngle,
                            correction,
                            targetTicks,
                            motor0,
                            motor1,
                            motor2,
                            motor3
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}