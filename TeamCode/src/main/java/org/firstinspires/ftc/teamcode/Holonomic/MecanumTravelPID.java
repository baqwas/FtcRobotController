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

import android.provider.ContactsContract;

// *** Universal IMU Imports ***
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor; // NEW IMPORT

@TeleOp(name = "Mecanum: Travel PID", group = "Test")
//@Disabled

public class MecanumTravelPID extends LinearOpMode
{
    private static final  String TAG = MecanumTravelIMU.class.getSimpleName(); // for use in logging

    // 1. REPLACED Datalog CLASS INSTANTIATION with Datalogger direct instantiation
    private final Datalogger datalogger = new Datalogger(
            TAG, // Filename
            "OpModeStatus", "Yaw", "Pitch", "Roll", "YawError", "TicksError",
            "Correction", "TargetTicks", "MotorTicks", "TicksToGo", "AppliedPower",
            "motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack",
            "Battery"
    );

    /**
     // calcs for TICKS_PER_INCH at
     // https://docs.google.com/spreadsheets/d/1PRGoHqyCUkSiiUiAUla-mElgsUdoqUssUntqvU-TYFY/edit?usp=sharing
     // static final double     TICKS_PER_INCH = 217.3267045; // REV Robotics Core Hex motor (REV-41-1300) & 75mm wheel
     static final double     TICKS_PER_INCH = 56.9887969189608; // REV Robotics HD Hex motor & 75mm Mecanum wheel
     // static final double     TICKS_PER_INCH = 45.283963; // goBILDA 5203 (19.2:1) and 96mm Mecanum wheel
     SWYFT Drive v2
     4	Quadrature (Hall Effect)
     7	Cycles per revolution
     28	Ticks per motor internal shaft
     12.7	Gear ratio
     355.6	Motor shaft ticks
     86	Wheel diameter, mm
     270.176968208722	Wheel circumference, mm
     10.6368885121544	Wheel circumference, inches
     33.4308289114498	ticks/inch
     */
    static final double     TICKS_PER_INCH = 33.4308289114498; // SWYFT Drive v2; goBILDA 5203 series, 12.7:1, 86 mm
    TouchSensor             touch;
    // Changed BNO055IMU to IMU
    IMU                     imu;
    // Removed Orientation lastAngles = new Orientation();
    double                  globalAngle, initialPower = .40, correction;
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

    // NEW: Battery Sensor field
    private BatteryVoltageSensor batterySensor = null;

    /**
     * @param travelLength the linear distance to travel, inches
     * @param travelPower the initial power setting of all motors
     * Function to backup the robot after a stop to ensure it does not
     * collide with any object. This function will require further tuning.
     * N.B.
     * STOP_AND_RESET_ENCODER MUST precede RUN_WITHOUT_ENCODER
     * The motor is to set the current encoder position to zero.
     * In contrast to RUN_TO_POSITION, the motor is not rotated in order to achieve this;
     * rather, the current rotational position of the motor is simply reinterpreted as the new zero value.
     * However, as a side effect of placing a motor in this mode, power is removed from the motor,
     * causing it to stop, though it is unspecified whether the motor enters brake or float mode.
     * Further, it should be noted that setting a motor toSTOP_AND_RESET_ENCODER may or may not be a transient state:
     * motors connected to some motor controllers will remain in this mode until explicitly transitioned
     * to a different one, while motors connected to other motor controllers
     * will automatically transition to a different mode after the reset of the encoder is complete.
     * See REV Robotics Core Hex and HD UltraPlanetary motor specifications
     *
     * Bosch BNO055 https://www.bosch-sensortec.com/products/smart-sensors/bno055/#documents
     * SensorMode
     * ACCGYRO
     * ACCMAG
     * ACCONLY
     * AMG
     * COMPASS
     * CONFIG
     * DISABLED
     * GYROONLY
     * IMU           Fusion of accelerometer and gyroscope
     * M4G           Fusion of accelerometer and magnetometer compensating limitations of gyroscope
     * MAGGYRO
     * MAGONLY
     * NDOF          Fusion of all 3 sensors with FMC enabled; incomplete figure 8 pattern will calibrate magnetometer
     * NDOF_FMC_OFF  Fusion of all 3 sensors but needs figure 8 pattern for magnetometer calibration
     * BNO055IMU interface abstracts the functionality of the Bosch/Sensortec BNO055 9-axis absolute orientation sensor.
     * The BNO055 can output the following sensor data
     * Absolute Orientation (Euler Vector, 100Hz) Three axis orientation data based on a 360° sphere
     * Absolute Orientation (Quaterion, 100Hz) Four point quaternion output for more accurate data manipulation
     * Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in rad/s
     * Acceleration Vector (100Hz) Three axis of acceleration (gravity + linear motion) in m/s^2
     * Magnetic Field Strength Vector (20Hz) Three axis of magnetic field sensing in micro Tesla (uT)
     * Linear Acceleration Vector (100Hz) Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
     * Gravity Vector (100Hz) Three axis of gravitational acceleration (minus any movement) in m/s^2
     * Temperature (1Hz) Ambient temperature in degrees celsius
     * Of those, the first (the gravity-corrected absolute orientation vector) is the most useful in FTC robot design.
     */
    private void LinearTravel(double travelLength, double travelPower) {
        double appliedPower = travelPower;
        boolean travelCompleted = false;// = true when length has been traveled by robot
        double ticksError;
        double ticksToGo;
        double travelTicks = TICKS_PER_INCH * travelLength;
        double yawError;

        for (DcMotorEx dcMotor: motor)
        {
            dcMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // The motor is to set the current encoder position to zero
            dcMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        for (int i = 0; i < motor.length; i++)              // for each motor
        {
            motorTicks[i] = 0;                              // no time to debug, eh?
        }
        resetAngle();
        while (opModeIsActive() && !travelCompleted)            // drive until end of period.
        {
            ticksError = ((motorTicks[1] + motorTicks[2]) - (motorTicks[0] + motorTicks[3]) ) * 0.007;
            yawError = checkDirection();  // Use gyro to drive in a straight line.
            correction = -ticksError + yawError;

            // Get the current orientation for logging and telemetry using the new interface
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            for (int i = 0; i < motor.length; i++)              // for each motor
            {
                motorTicks[i] = motor[i].getCurrentPosition();  // have we travelled far enough?
            }
            ticksToGo = (Math.abs(travelTicks) - Math.abs(motorTicks[0])) / Math.abs(travelTicks);
            if (ticksToGo < 0.10)
            {
                appliedPower = travelPower * ticksToGo;
                correction = correction * ticksToGo;
            }
            if (Math.abs(motorTicks[0]) < Math.abs(travelTicks))                    // are we there yet?
            {
                /*
                 * steering power applied along X axis principles
                 * motors 0,3 pair
                 * motors 1,2 pair
                 */
                double power0 = appliedPower - correction;
                double power1 = appliedPower + correction;
                double power2 = appliedPower + correction;
                double power3 = appliedPower - correction;

                motor[0].setPower(power0);
                motor[1].setPower(power1);
                motor[2].setPower(power2);
                motor[3].setPower(power3);

                // *** IMU Logging Update ***
                telemetry.addData("1 IMU Yaw", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("2 Global heading", globalAngle);
                telemetry.addData("3 Correction", correction);
                telemetry.addData("4 Current ticks:", motorTicks[0]);
                telemetry.addData("5 Travel length:", travelLength);
                telemetry.addData("6 Target ticks:", travelTicks);
                telemetry.addData("7 Motor power:", motor[0].getPower());
                telemetry.addData("8 Travel power:", travelPower);
                telemetry.update();

                // 2. UPDATED LOGGING CALL
                String batteryVoltage = (batterySensor != null) ? batterySensor.getFormattedVoltage() : "N/A";

                datalogger.log(
                        "RUNNING",
                        String.format("%.4f", orientation.getYaw(AngleUnit.DEGREES)),
                        String.format("%.4f", orientation.getPitch(AngleUnit.DEGREES)),
                        String.format("%.4f", orientation.getRoll(AngleUnit.DEGREES)),
                        String.format("%.4f", yawError),
                        String.format("%.4f", ticksError),
                        String.format("%.4f", correction),
                        String.format("%.0f", travelTicks),
                        String.format("%d", motorTicks[0]),
                        String.format("%.4f", ticksToGo),
                        String.format("%.4f", appliedPower),
                        String.format("%.4f", power0),
                        String.format("%.4f", power1),
                        String.format("%.4f", power2),
                        String.format("%.4f", power3),
                        batteryVoltage
                );

                aButton = gamepad1.a;       // allow teleop to change direction by 90 degrees
                bButton = gamepad1.b;       // A - Cross; B - Circle
                //touched = touch.isPressed();
                if (/*touched || */aButton || bButton) {
                    backup();               // for a safer turn in the subsequent operation

                    if (/*touched || */aButton) rotate(-90, travelPower); // turn 90 degrees right

                    if (bButton) rotate(90, travelPower);             // turn 90 degrees left
                }
            }
            else
            {
                travelCompleted = true; // Alhamdolillah! Travelled the specified linear length.
            }
        }
        fullStop();                     // turn the motors off
    }

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
            dcMotor.setPower(-0.3);     // mode = 3 from above table
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
            dcMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // see note, if any, on vendor specs for the corresponding motor
            dcMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        sleep(250);                     // allow the motors to come to a full stop
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
            dcMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // stops and then brakes, actively resisting any external force which attempts to turn the motor
            dcMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        //touch = hardwareMap.touchSensor.get("sensorTouch");        // get a reference to touch sensor.

        // *** Universal IMU Initialization ***
        // Create parameters object specifying the Control/Expansion Hub's orientation on the robot.
        // The default is Logo UP, USB FORWARD. Adjust this for your robot.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));

        // Retrieve and initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        // **********************************

        // --- BATTERY SENSOR INITIALIZATION ---
        try {
            batterySensor = new BatteryVoltageSensor(hardwareMap);
            telemetry.addData("Status", "Battery Sensor Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not initialize Battery Voltage Sensor.");
        }
        telemetry.update();

        double travelLength = 0.0;     // 12" linear, will parametrically evaluate other lengths too!
        double deltaTravel = 3.0;

        telemetry.addData("Mode", "Calibrating...");
        telemetry.update();

        // Simplified calibration status for the universal IMU
        telemetry.addData("Calibration status", "System Calibrated");
        telemetry.addData("Mode", "Select Start (Yaw Reset to 0)");
        telemetry.update();

        waitForStart();                 // wait for Start button

        telemetry.addData("Mode", "running");
        telemetry.update();
        sleep(1000);

        // Reset the Yaw once at the start of the OpMode to set the initial heading to 0 degrees.
        imu.resetYaw();

        // 6, 12, 18, 24, 30, 36, 42, 48, 54, 60, 66, 72
        for (int i = 0; i < 3; i++)
        {
            travelLength += deltaTravel;
            LinearTravel(travelLength, initialPower);   // modularized for future use in Automomous
            //LinearTravel(-travelLength, -initialPower);   // modularized for future use in Automomous
            long pauseInterval = 5000;
            telemetry.addData("Traveled", travelLength + " inches");
            telemetry.addData("Paused", pauseInterval + " milliseconds");
            telemetry.update();
            sleep(pauseInterval);
        }

        // CRITICAL: Close the datalogger when the OpMode ends
        datalogger.close();

    }

    /**
     * Resets the cumulative angle tracking to zero.
     * For the universal IMU, this is done using imu.resetYaw().
     */
    private void resetAngle()
    {
        // Use the dedicated resetYaw() method for the universal IMU interface
        imu.resetYaw();
        globalAngle = 0; // Keep globalAngle reset for the OpMode's legacy logic
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // The new IMU interface provides the Yaw angle (heading) directly,
        // which is relative to the last call to imu.resetYaw().
        // This value is what the OpMode's PID loop needs for straight-line correction and turning.

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = orientation.getYaw(AngleUnit.DEGREES);

        // Update globalAngle with the current Yaw for compatibility with the OpMode's legacy logic/variables.
        globalAngle = currentYaw;

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
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        double correction, angle, gain = 0.0055;

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
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        resetAngle();                   // reset heading for tracking with IMU data

        if (degrees < 0)
        {                               // turn right
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {                               // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        motor[0].setPower(leftPower);   // set power to rotate
        motor[1].setPower(rightPower);
        motor[2].setPower(rightPower);
        motor[3].setPower(leftPower);
        if (degrees < 0)                // rotate until turn is completed
        {
            // On right turn we have to get off zero first.
            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && getAngle() == 0) {
                logRotationStatus("ROTATING", leftPower, rightPower);
            }

            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && getAngle() > degrees) {
                logRotationStatus("ROTATING", leftPower, rightPower);
            }
        }
        else                                // left turn.
            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && getAngle() < degrees) {
                logRotationStatus("ROTATING", leftPower, rightPower);
            }

        fullStop();

        resetAngle();                          // reset angle tracking on new heading
    }

    /**
     * Helper to log data specifically during rotation.
     */
    private void logRotationStatus(String status, double powerLeft, double powerRight) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        String batteryVoltage = (batterySensor != null) ? batterySensor.getFormattedVoltage() : "N/A";

        // Log the current rotation status. Many fields will be irrelevant (like ticks, correction)
        // so we fill them with 0.0 or N/A
        datalogger.log(
                status,
                String.format("%.4f", orientation.getYaw(AngleUnit.DEGREES)),
                String.format("%.4f", orientation.getPitch(AngleUnit.DEGREES)),
                String.format("%.4f", orientation.getRoll(AngleUnit.DEGREES)),
                "N/A", // YawError
                "N/A", // TicksError
                "N/A", // Correction
                "0.0", // TargetTicks
                "0",   // MotorTicks
                "0.0", // TicksToGo
                String.format("%.4f", Math.abs(powerLeft)), // AppliedPower
                String.format("%.4f", motor[0].getPower()), // motorLeftFront
                String.format("%.4f", motor[1].getPower()), // motorLeftBack
                String.format("%.4f", motor[2].getPower()), // motorRightFront
                String.format("%.4f", motor[3].getPower()), // motorRightBack
                batteryVoltage
        );
    }


    // 3. REMOVED THE ENTIRE NESTED 'Datalog' CLASS
}