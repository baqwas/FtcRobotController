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
 * 4	Quadrature (Hall Effect)
 * 7	Cycles per revolution
 * 28	Ticks per motor internal shaft
 * 12.7	Gear ratio
 * 355.6	Motor shaft ticks
 * 86	Wheel diameter, mm
 * 270.176968208722	Wheel circumference, mm
 * 10.6368885121544	Wheel circumference, inches
 * 33.4308289114498	ticks/inch
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

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU; // Import Universal IMU

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles; // For Universal IMU angle retrieval

import org.firstinspires.ftc.teamcode.Utility.Datalogger;

@TeleOp(name = "Mecanum: Travel Strafe", group = "Test")
@Disabled

public class MecanumTravelStrafe extends LinearOpMode
{
    private static final  String TAG = MecanumTravelStrafe.class.getSimpleName(); // for use in logging
    Datalog datalog = new Datalog(TAG);
    //static final double     TICKS_PER_INCH = 56.9887969189608; // REV Robotics HD Hex motor & 75mm Mecanum wheel
    // static final double     TICKS_PER_INCH = 45.283963; // goBILDA 5203 19.2:1 Motor 96mm Mecanum wheel
    static final double     TICKS_PER_INCH = 33.4308289114498; // SWYFT Drive v2; goBILDA 5203 12.7:1 Motor 86mm Mecanum wheel

    TouchSensor             touch;
    // BNO055IMU imu; // Old BNO055 declaration
    IMU                     imu; // Universal IMU declaration
    Orientation             globalAngles = new Orientation(); // This Orientation object can be kept, but for Universal IMU we'll use YawPitchRollAngles
    YawPitchRollAngles      robotOrientation; // New object for Universal IMU angles
    // Orientation lastAngles = new Orientation(); // This can be removed or repurposed
    double                  globalAngle, initialPower = 0.40, correction;
    boolean                 aButton, bButton, touched;
    private final ElapsedTime runtime = new ElapsedTime();
    // motor entities for drivetrain
    String[] motorLabels = {
            "motorLeftFront",           // port 0 Control Hub
            "motorLeftBack",            // port 1 Control Hub
            "motorRightFront",          // port 2 Control Hub
            "motorRightBack"            // port 3 Control Hub
    };
    DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null}; // couldn't initialize hardwareMap here?!!
    int[] motorTicks = {0, 0, 0, 0};    // current tick count from encoder for the respective motors

    /**
     * Function to backup the robot after a stop to ensure it does not
     * collide with any object. This function will require further tuning
     * of the parameter value to the sleep function.
     * For REV Robotics Mecanum kit drivetrain, the clearance required is 3.5" from the obstacle
     * Suggested parameter changes to this function:
     * backup directions:
     * Value    Direction       Bearing, degrees         Obstacle(s) (e.g. walls)
     * 0        due north       0                           behind
     * 1        due east        90                          left
     * 2        due south east  135                         front and left
     * 3        due south       180                         front
     * 4        due south west  225                         front and right
     * 5        due west        270                         right obstacle
     */
    private void backup() {
        for (DcMotorEx dcMotor : motor)
        {
            dcMotor.setPower(-0.4);     // mode = 3
        }
        sleep(500);                     // adjust parameter value based on drive train dimensions and motors
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
            dcMotor.setPower(0.0);      // since the following RunMode varies with motors play it safe with this call
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // see note, if any, on vendor specs for the corresponding motor
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        sleep(500);                     // allow the motors to come to a full stop
    }

    /**
     * normalize the angle to that it returns a value between -pi and pi
     * @param radians the angle to be normalized
     * @return normalized angle in radians
     * usage: Math.toDegrees(angleWrap(Math.toRadians(359 - 1)))
     * 359 - 1 = 358
     * 358 > 180
     * 358 - 360 = -2
     * corrected angle is = -2
     */
    public double angleWrap(double radians)
    {
        while (radians > Math.PI)
        {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI)
        {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    /**
     * perform lateral holonomic movement for a specified distance
     *
     * rotational changes to keep heading constant are:
     * clockwise:
     * +ve left front (0) and left back (1)
     * -ve right front (2) and right back (3)
     * anti-clockwise:
     * -ve left front (0) and left back (1)
     * +ve right front (2) and right back (3)
     *
     * @param right true for movement to right, false for movement to left
     * @param distance the specified distance for holonomic travel
     * @param startPower the initial power to the motors for the movement
     */
    public void DriveStrafe(boolean right, double distance, double startPower)
    {
        /*
         * Kp
         * 0.1 is BAD, explore lower values by two orders of magnitude
         * 0.001 has slight improvement, go with lower values
         * 0.0005 need to assess on proper surface
         * 0.00025 ned to go lower
         * 0.00015 need to higher
         * 0.0001 opposite end of spectrum, need to go higher
         */

        double yawKp = 0.01;    // Proportional coefficient as in PID algorithm; will tweak with tests
        double[] motorPower = new double[]{startPower, startPower, startPower, startPower};
        double strafeCoeficient = 1.0; // empirical number that may need tweaking for greater accuracy
        boolean travelCompleted = false;// = true when length has been traveled by robot
        double pitch = 0;               // for pure strafe movement there is no pitch
        double lateral = startPower;
        double lateralError;
        long travelTicks = (long) (TICKS_PER_INCH * distance);
        double ticksError;
        double ticksKp = 0.00005;      // to correct the horizontal drift
        double yaw = 0, yawError;

        for (DcMotorEx dcMotor : motor) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // stops and then brakes, actively resisting any external force which attempts to turn the motor
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        while (opModeIsActive() && !travelCompleted) {
            for (int i = 0; i < motor.length; i++)              // for each motor
            {
                motorTicks[i] = motor[i].getCurrentPosition();  // have we travelled far enough?
            }
            ticksError = ((double) (travelTicks - ((motorTicks[0] + motorTicks[3]) - (motorTicks[1] + motorTicks[2])) / 2)) * ticksKp;
            //lateral = lateral * ticksError / travelTicks;

            // Update IMU angle retrieval for Universal IMU
            robotOrientation = imu.getRobotYawPitchRollAngles();
            double currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);

            // lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Old BNO055 call
            // yawError = (globalAngle - lastAngles.firstAngle) * yawKp; // Old BNO055 calculation
            yawError = (globalAngle - currentYaw) * yawKp; // Universal IMU calculation
            correction = ticksError;// + yawError;
            double denominator = Math.max(Math.abs(lateral) /*+ Math.abs(pitch)*/ + Math.abs(correction), 1);
            if (right) {
                motorPower[0] = lateral - yawError;//(-lateral + correction) / denominator;
                motorPower[1] = -lateral + yawError;//( lateral - correction) / denominator;
                motorPower[2] = -lateral + yawError;//( lateral + correction) / denominator;
                motorPower[3] = lateral - yawError;//(-lateral + correction) / denominator;
            } else {
                motorPower[0] = -lateral + yawError;//( lateral - correction) / denominator;
                motorPower[1] = lateral - yawError;//(-lateral + correction) / denominator;
                motorPower[2] = lateral - yawError;//(-lateral - correction) / denominator;
                motorPower[3] = -lateral + yawError;//( lateral - correction) / denominator;
            }

            if (Math.abs(motorTicks[0]) < Math.abs(travelTicks))                    // are we there yet?
            {
                for (int i = 0; i < motor.length; i++)              // for each motor
                {
                    motor[i].setPower(motorPower[i]);
                }
                datalog.yaw.set(currentYaw); // Use the current Yaw from Universal IMU
                datalog.pitch.set(pitch);
                datalog.roll.set(lateral);
                datalog.globalAngle.set(globalAngle);
                datalog.lastAngle.set(currentYaw); // Use the current Yaw for the 'lastAngle' log field
                datalog.ticksKp.set(ticksKp);
                datalog.yawKp.set(yawKp);
                datalog.ticksError.set(ticksError);
                datalog.yawError.set(yawError);
                datalog.correction.set(correction);
                datalog.targetTicks.set(travelTicks);
                datalog.motorTicks0.set(motorTicks[0]);
                datalog.motorPower0.set(motorPower[0]);
                datalog.motorTicks1.set(motorTicks[1]);
                datalog.motorPower1.set(motorPower[1]);
                datalog.motorTicks2.set(motorTicks[2]);
                datalog.motorPower2.set(motorPower[2]);
                datalog.motorTicks3.set(motorTicks[3]);
                datalog.motorPower3.set(motorPower[3]);
                // The logged timestamp is taken when writeLine() is called.
                datalog.writeLine();
            }
            else
            {
                travelCompleted = true; // Alhamdolillah! Travelled the specified linear length.
            }
        }
        fullStop();                     // all done!
    }

    @Override
    public void runOpMode() throws InterruptedException   // called when init button is  pressed.
    {
        // Initialize the hardware variables
        // The strings used here must correspond
        // to the names assigned during the robot configuration step on the Driver Hub
        for (int i = 0; i < motor.length; i++)
        {
            motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]); // motorLabels for user friendly messages elsewhere
        }
        // REV Robotics motors may need two of the following
        // four statements to be enabled - PLEASE TEST before 1st use!
        motor[0].setDirection(DcMotorEx.Direction.FORWARD); // motorLeftFront
        motor[1].setDirection(DcMotorEx.Direction.FORWARD); // motorLeftBack
        motor[2].setDirection(DcMotorEx.Direction.REVERSE); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.REVERSE); // motorRightBack
        // default condition but play it safe anyway
        for (DcMotorEx dcMotor : motor) {
            /*
             * motor stops and then brakes
             * actively resisting any external force
             * which attempts to turn the motor
             */
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // stops and then brakes, actively resisting any external force which attempts to turn the motor
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        //touch = hardwareMap.touchSensor.get("sensorTouch");        // get a reference to touch sensor.

        // Universal IMU Initialization
        /*
        IMU.Parameters parameters = new IMU.Parameters.Builder()
                // You can add more parameters here if needed, such as
                .setAccelerationIntegrationAlgorithm(new JustLoggingAccelerationIntegrator())
                // Set the angle unit for all angle methods (e.g., imu.getRobotYawPitchRollAngles())
                .setAngleUnit(IMU.AngleUnit.DEGREES)
                .build();
        */
        imu = hardwareMap.get(IMU.class, "imu"); // Retrieve and initialize the IMU (Universal IMU)
        // imu.initialize(parameters);

        double travelLength = 0.0;     // 12" linear, will parametrically evaluate other lengths too!
        double deltaTravel = 12.0;

        telemetry.addData("Mode", "Initializing IMU...");
        telemetry.update();

        // The BNO055 'isGyroCalibrated()' check is removed for the Universal IMU.
        // It's generally ready immediately or initialization handles the wait.
        // We will reset the Yaw angle to establish the starting heading.

        // while (!isStopRequested() && !imu.isGyroCalibrated())        // Old BNO055 calibration check
        // {
        //     sleep(50);                  // calibration time does not exceed ~3 secs in practice
        //     idle();
        // }

        // Reset yaw to zero out the starting heading
        imu.resetYaw();


        // telemetry.addData("Calibration status", imu.getCalibrationStatus().toString()); // Removed, not applicable to all Universal IMUs
        telemetry.addData("IMU Status", "Yaw Reset");
        telemetry.addData("Wheels", "goBILDA Mecanum 96mm");
        telemetry.addData("Start", "Press PLAY");
        telemetry.update();

        waitForStart();                 // wait for Start button

        telemetry.addData("Mode", "strafing...");
        telemetry.update();

        // Set the globalAngle after START, using the Universal IMU method
        robotOrientation = imu.getRobotYawPitchRollAngles();
        globalAngle = robotOrientation.getYaw(AngleUnit.DEGREES);

        for (int i = 0; i < 3; i++) {
            travelLength += deltaTravel;
            DriveStrafe(true, travelLength, 0.3);
            sleep(1000);                    // nominal delay to catch one's breath
            DriveStrafe(false, travelLength, 0.3);
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
        public Datalogger.GenericField lastAngle    = new Datalogger.GenericField("LastAngle");
        public Datalogger.GenericField ticksKp      = new Datalogger.GenericField("ticksKp");
        public Datalogger.GenericField yawKp        = new Datalogger.GenericField("yawKp");
        public Datalogger.GenericField yawError     = new Datalogger.GenericField("YawError");
        public Datalogger.GenericField ticksError   = new Datalogger.GenericField("TicksError");
        public Datalogger.GenericField correction   = new Datalogger.GenericField("Correction");
        public Datalogger.GenericField targetTicks  = new Datalogger.GenericField("TargetTicks");
        public Datalogger.GenericField motorTicks0   = new Datalogger.GenericField("MotorTicks0");
        public Datalogger.GenericField motorPower0   = new Datalogger.GenericField("MotorPower0");
        public Datalogger.GenericField motorTicks1   = new Datalogger.GenericField("MotorTicks1");
        public Datalogger.GenericField motorPower1   = new Datalogger.GenericField("MotorPower1");
        public Datalogger.GenericField motorTicks2   = new Datalogger.GenericField("MotorTicks2");
        public Datalogger.GenericField motorPower2   = new Datalogger.GenericField("MotorPower2");
        public Datalogger.GenericField motorTicks3   = new Datalogger.GenericField("MotorTicks3");
        public Datalogger.GenericField motorPower3   = new Datalogger.GenericField("MotorPower3");

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
                            lastAngle,
                            ticksKp,
                            yawKp,
                            yawError,
                            ticksError,
                            correction,
                            targetTicks,
                            motorTicks0,
                            motorPower0,
                            motorTicks1,
                            motorPower1,
                            motorTicks2,
                            motorPower2,
                            motorTicks3,
                            motorPower3
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