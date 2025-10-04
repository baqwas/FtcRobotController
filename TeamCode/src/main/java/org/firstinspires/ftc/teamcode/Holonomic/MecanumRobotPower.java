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
 *  4	Quadrature (Hall Effect)
 *  7	Cycles per revolution
 *  28	Ticks per motor internal shaft
 *  12.7	Gear ratio
 *  355.6	Motor shaft ticks
 *  86	Wheel diameter, mm
 *  270.176968208722	Wheel circumference, mm
 *  10.6368885121544	Wheel circumference, inches
 *   33.4308289114498	ticks/inch
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

// **REPLACED BNO055IMU with IMU**
import com.qualcomm.robotcore.hardware.IMU;
// Removed: import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// **ADDED YawPitchRollAngles for Universal IMU data retrieval**
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// Removed: AxesOrder, AxesReference, Orientation

@TeleOp(name = "Mecanum: Robot Power", group = "Test")
@Disabled

public class MecanumRobotPower extends LinearOpMode
{
    private static final  String TAG = MecanumRobotPower.class.getSimpleName(); // for use in logging
    //Datalog datalog = new Datalog(TAG);
    /*
    // calcs for TICKS_PER_INCH at
    // https://docs.google.com/spreadsheets/d/1PRGoHqyCUkSiiUiAUla-mElgsUdoqUssUntqvU-TYFY/edit?usp=sharing
    // static final double     TICKS_PER_INCH = 217.3267045; // REV Robotics Core Hex motor (REV-41-1300) & 75mm wheel
    static final double     TICKS_PER_INCH = 56.9887969189608; // REV Robotics HD Hex motor & 75mm Mecanum wheel
    // static final double     TICKS_PER_INCH = 45.283963; // goBILDA 5203 (19.2:1) and 96mm Mecanum wheel
    */
    static final double     TICKS_PER_INCH = 33.4308289114498; // SWYFT Drive v2; goBILDA 5203 series, 12.7:1; 86 mm wheels

    TouchSensor             touch;
    // **CHANGED IMU type**
    IMU                     imu;
    // **CHANGED to YawPitchRollAngles (initialization is dummy data)**
    YawPitchRollAngles      lastAngles = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

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

    private void robotPower(double power, int delay) {
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(power);      // since the following RunMode varies with motors play it safe with this call
        }
        sleep(delay);                     // allow the motors to come to a full stop
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);      // since the following RunMode varies with motors play it safe with this call
        }
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

        // **UPDATED: Universal IMU Parameters and Initialization**
        imu = hardwareMap.get(IMU.class, "imu");        // Retrieve and initialize the IMU
        /*
        IMU.Parameters parameters = new IMU.Parameters.Builder()
                .setAngleUnit(AngleUnit.DEGREES)
                .setAccelUnit(IMU.AccelUnit.METERS_PERSEC_PERSEC)
                .build();


        imu.initialize(parameters);                     // Initialize the sensor using the indicated set of parameters
        */

        imu.resetYaw();                                 // Reset yaw to make the current direction 0 degrees
        // The execution of this method can take a fairly long while,
        // possibly several tens of milliseconds

        double travelLength = 0.0;     // 12" linear, will parametrically evaluate other lengths too!
        double deltaTravel = 3.0;

        telemetry.addData("Mode", "Calibrating...");
        telemetry.update();

        // **UPDATED: Check if IMU is ready (isSystemSet() is the equivalent of isGyroCalibrated() for the U-IMU)**
        while (!isStopRequested())        // make sure the imu system is set before continuing.
        {
            sleep(50);                  // calibration time does not exceed ~3 secs in practice
            idle();
        }

        // Removed: imu.getCalibrationStatus().toString()
        telemetry.addData("Status", "IMU System Set");
        telemetry.addData("Mode", "Select Start");
        telemetry.update();

        waitForStart();                 // wait for Start button

        telemetry.addData("Mode", "running");
        telemetry.update();
        sleep(1000);

        if (isStopRequested())          // Added check for OpMode stop
            return;

        while (opModeIsActive()) {
            int i = 250;
            telemetry.addData("Delay", i);
            telemetry.update();
            robotPower(0.3, i);
            sleep(10000);
            i = 500;
            telemetry.addData("Delay", i);
            telemetry.update();
            robotPower(0.3, i);
            sleep(10000);
            i = 750;
            telemetry.addData("Delay", i);
            telemetry.update();
            robotPower(0.3, i);
            sleep(10000);
            i = 1000;
            telemetry.addData("Delay", i);
            telemetry.update();
            robotPower(0.3, i);
            sleep(10000);
            i = 1250;
            telemetry.addData("Delay", i);
            telemetry.update();
            robotPower(0.3, i);
            sleep(10000);
            i = 1500;
            telemetry.addData("Delay", i);
            telemetry.update();
            robotPower(0.3, i);
            sleep(10000);
            i = 2000;
            telemetry.addData("Delay", i);
            telemetry.update();
            robotPower(0.3, i);
        };

    }
}
