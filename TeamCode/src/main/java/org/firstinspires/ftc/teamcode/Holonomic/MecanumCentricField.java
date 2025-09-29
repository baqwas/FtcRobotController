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
 * Formula for rotating a vector in 2D
 * @see https://matthew-brett.github.io/teaching/rotation_2d.html
 */

package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;

@TeleOp(name = "Mecanum: Centric Field", group = "Test")
@Disabled

public class MecanumCentricField extends LinearOpMode {

    private static final  String TAG = MecanumCentricField.class.getSimpleName();
    Datalog datalog = new Datalog(TAG); // data logging for offline analysis/debugging

    static final double TICKS_PER_INCH = 56.9887969189608; // motor and wheel specific!
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.40, correction;
    private final ElapsedTime runtime = new ElapsedTime();
    // motor entities for drivetrain
    String[] motorLabels = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    int[] motorTicks = {0, 0, 0, 0};    // current tick count from encoder for the respective motors
    double[] motorPower = {0.0, 0.0, 0.0, 0.0};
    int drivetrainSteps = 0;

    @Override
    public void runOpMode() throws InterruptedException   // called when init button is  pressed.
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.NDOF;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");        // Retrieve and initialize the IMU
        imu.initialize(parameters);
        /*
         * Initialize the hardware variables
         * The strings used here must correspond
         * to the names assigned during the robot configuration step on the DS or RC devices.
         */
        for (int i = 0; i < motor.length; i++) {
            motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]);
            /*
             * motor stops and then brakes
             * actively resisting any external force
             * which attempts to turn the motor
             */
            motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            // run at any velocity with specified power level
            motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        /*
         * REV Robotics Mecanum drive-train with horizontally opposing motors needs
         * two motors to reverse the default spin direction
         * motor[0].setDirection(DcMotorSimple.Direction.FORWARD)
         * motor[1].setDirection(DcMotorSimple.Direction.FORWARD)
         * FORWARD is default - redundant here
         */
        motor[2].setDirection(DcMotorEx.Direction.REVERSE); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.REVERSE); // motorRightBack

        // default condition but play it safe anyway
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0); // setting a power level of zero will brake the motor
            dcMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // stops and then brakes, actively resisting any external force which attempts to turn the motor
            dcMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor
        }
        //touch = hardwareMap.touchSensor.get("sensorTouch");        // get a reference to touch sensor.
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

        telemetry.addData("Mode", "running");
        telemetry.update();
        sleep(1000);
        if (isStopRequested())          // has stopping opMode been requested
            return;
        while (opModeIsActive()) {
            /*
             * left stick X axis for lateral movement (i.e. right or left)
             * left stick Y axis for axial movement (i.e. forward or reverse)
             */
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            /*
             * empirically refine the scale factor below -> 1.n, n= 0.0.1 to 0.10
             * individual wheels may slip for many reasons
             * not all wheels will slip at the same time by the same amount
             * contact with field surface may be uneven
             * a roller or two on a Mecanum drive may have a slightly different surface property owing to several factors
             */
            double x = gamepad1.left_stick_x * 1.0; // counteract imperfect strafing by empirically refining m.n, 0.9, 0.95, 1.0, 1.05, 1.1
            // right stick X is for rotation (i.e. clockwise or counter-clockwise)
            double rx = gamepad1.right_stick_x;
            // heading from IMU is clockwise but our reference is counter-clockwise so change sign
            // note angle here is in radians NOT degrees to save a few CPU cycles during calcs
            double yaw = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

            double rotX = x * Math.cos(yaw) - y * Math.sin(yaw);
            double rotY = x * Math.sin(yaw) + y * Math.cos(yaw);
            // normalize the power setting in the range {-1, 1}
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorPower[0] = (rotY + rotX + rx) / denominator;
            motorPower[1] = (rotY - rotX + rx) / denominator;
            motorPower[2] = (rotY - rotX - rx) / denominator;
            motorPower[3] = (rotY + rotX - rx) / denominator;

            for (int i = 0; i < motor.length; i++) {
                motor[i].setPower(motorPower[i]);
            }

            datalog.loopCounter.set(drivetrainSteps++);
            // all angles are expressed in RADIANS for this OpMode
            datalog.yaw.set(yaw);
            datalog.pitch.set(lastAngles.secondAngle);
            datalog.roll.set(lastAngles.thirdAngle);
            datalog.roll.set(globalAngle);
            datalog.gamepadx.set(x);
            datalog.gamepady.set(y);
            datalog.gamepadrx.set(rx);
            datalog.rotx.set(rotX);
            datalog.roty.set(rotY);
            datalog.power0.set(motorPower[0]);
            datalog.power1.set(motorPower[1]);
            datalog.power2.set(motorPower[2]);
            datalog.power3.set(motorPower[3]);
            datalog.writeLine();            // A timestamp is applied to the record when writing

            // cone ingress - button Y/Triangle
            // raise slide by 6"
            // go forward by 2"
            // release gripper to 0.25
            // lower slide
            // close gripper to 0.5
            // raise slide by 6"
            // backaway by 2"

            // cone egress - button X/Cross
            // raise slide to junction height
            // go forward by 2"
            // release gripper to 0.25
            // close gripper to 0.5
            // backwaway by 2"

            // collision avoidance
            // stop
            // backaway by 4"
            // rotate left by 90
        }
    }

    /**
     * This class encapsulates all the fields that will go into the datalog.
     */
    private static class Datalog {
        /*
         * The underlying datalogger object - it cares only about an array of loggable fields
         * The fields for this OpCode are:
         * yaw, pitch, roll & battery - in that order
         */

        private final Datalogger datalogger;
        /*
         * The fields whose values will be written to the storage file
         * Note that order here is NOT important
         * The order is important in the setFields() call below
         */
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField yaw = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField pitch = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField roll = new Datalogger.GenericField("Roll");
        public Datalogger.GenericField globalAngle = new Datalogger.GenericField("globalAngle");
        public Datalogger.GenericField deltaAngle = new Datalogger.GenericField("deltaAngle");
        public Datalogger.GenericField gamepadx = new Datalogger.GenericField("gamepadY");
        public Datalogger.GenericField gamepady = new Datalogger.GenericField("gamepadX");
        public Datalogger.GenericField gamepadrx = new Datalogger.GenericField("gamepadRx");
        public Datalogger.GenericField rotx = new Datalogger.GenericField("rotX");
        public Datalogger.GenericField roty = new Datalogger.GenericField("rotY");
        public Datalogger.GenericField power0 = new Datalogger.GenericField("motorPower0");
        public Datalogger.GenericField power1 = new Datalogger.GenericField("motorPower1");
        public Datalogger.GenericField power2 = new Datalogger.GenericField("motorPower2");
        public Datalogger.GenericField power3 = new Datalogger.GenericField("motorPower3");
        //public Datalogger.GenericField battery = new Datalogger.GenericField("Battery");

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
                            gamepadx,
                            gamepady,
                            gamepadrx,
                            rotx,
                            roty,
                            power0,
                            power1,
                            power2,
                            power3
                            //battery
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