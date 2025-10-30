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
  <li> uses mecanum wheels in robot centric mode </li>
  <li> the drive is controlled by the left stick on gamepad1</li>
  <li> turning is controlled by the right stick on gamepad1</li>
  <li> relies on IMU data to travel in a straight line as well to perform the turns</li>
  <li> displays a few status messages</li>
  </ul>
  @author modified by armw
 * @version 1.1
 * @param none
 * @return none
 * @exception none
 * <p>
 * This program registers as Teleop OpMode in the FtcRobotController app.
 * The robot drive is relative to the robot's current orientation.
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

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor;

@TeleOp(name = "Mecanum: Centric Robot", group = "Test")
@Disabled

public class MecanumCentricRobot extends LinearOpMode {

    private static final String TAG = MecanumCentricRobot.class.getSimpleName();

    // 1. REPLACED Datalog CLASS INSTANTIATION with Datalogger direct instantiation
    private final Datalogger datalogger = new Datalogger(
            TAG, // Filename
            "OpModeStatus", "Loop Counter", "Yaw", "Pitch", "Roll",
            "Gamepad X", "Gamepad Y", "Gamepad Rx", "Strafe Factor",
            "motorPower0", "motorPower1", "motorPower2", "motorPower3",
            "Battery"
    );

    static final double TICKS_PER_INCH = 33.4308289114498; // SWYFT Drive v2; goBILDA 5203 12.7:1, 86 mm wheel

    IMU imu;
    private final ElapsedTime runtime = new ElapsedTime();
    // The Universal IMU uses YawPitchRollAngles
    YawPitchRollAngles lastAngles = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
    double globalAngle, power = 0.40, correction;

    // ADDED: Battery Sensor field
    private BatteryVoltageSensor batterySensor = null;

    // motor entities for drivetrain
    String[] motorLabels = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    double[] motorPower = {0.0, 0.0, 0.0, 0.0};
    int drivetrainSteps = 0;

    @Override
    public void runOpMode() throws InterruptedException   // called when init button is  pressed.
    {
        // **UPDATED: Hardware map retrieval to use IMU interface**
        imu = hardwareMap.get(IMU.class, "imu");

        // **ADDED: Explicitly reset yaw**
        imu.resetYaw();

        // --- MOTOR INITIALIZATION ---
        /*
         * Initialize the hardware variables
         * The strings used here must correspond
         * to the names assigned during the robot configuration step on the DS or RC devices.
         */
        for (int i = 0; i < motor.length; i++) {
            motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]);
            motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        motor[2].setDirection(DcMotorEx.Direction.REVERSE); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.REVERSE); // motorRightBack

        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);
            dcMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            dcMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // --- BATTERY SENSOR INITIALIZATION ---
        try {
            batterySensor = new BatteryVoltageSensor(hardwareMap);
            telemetry.addData("Status", "Battery Sensor Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not initialize Battery Voltage Sensor.");
        }

        telemetry.addData("Mode", "Calibrating...");
        telemetry.update();

        // **UPDATED: Check if IMU is ready (isSystemSet() is the equivalent of isGyroCalibrated() for the U-IMU)**
        while (!isStopRequested())        // make sure the imu gyro is ready before continuing.
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "Select Start");
        telemetry.addData("Status", "IMU System Set");
        telemetry.update();

        waitForStart();                     // wait for Start button to be pressed for Linear OpMode

        telemetry.addData("Mode", "running");
        telemetry.update();
        sleep(1000);

        if (isStopRequested()) {         // has stopping opMode been requested
            datalogger.close();
            return;
        }

        while (opModeIsActive()) {
            /*
             * left stick X axis for lateral movement (i.e. right or left)
             * left stick Y axis for axial movement (i.e. forward or reverse)
             */
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            /*
             * empirically refine the scale factor below -> 1.n, n= 0.0.1 to 0.10
             */
            double x = gamepad1.left_stick_x * 1.10;
            // right stick X is for rotation (i.e. clockwise or counter-clockwise)
            double rx = gamepad1.right_stick_x;

            // **UPDATED: Get Yaw angle from Universal IMU**
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw(AngleUnit.DEGREES);
            double pitch = angles.getPitch(AngleUnit.DEGREES);
            double roll = angles.getRoll(AngleUnit.DEGREES);

            // normalize the power setting in the range {-1, 1}
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorPower[0] = (y + x + rx) / denominator; // Left Front
            motorPower[1] = (y - x + rx) / denominator; // Left Back
            motorPower[2] = (y - x - rx) / denominator; // Right Front
            motorPower[3] = (y + x - rx) / denominator; // Right Back

            for (int i = 0; i < motor.length; i++) {
                motor[i].setPower(motorPower[i]);
            }

            // 2. REPLACED OLD DATALOG LOGIC WITH NEW API CALL
            String batteryVoltage = (batterySensor != null) ? batterySensor.getFormattedVoltage() : "N/A";

            datalogger.log(
                    "RUNNING",                                          // OpModeStatus
                    String.valueOf(drivetrainSteps++),                  // Loop Counter
                    String.format("%.4f", yaw),
                    String.format("%.4f", pitch),
                    String.format("%.4f", roll),
                    String.format("%.4f", x),                           // Gamepad X (strafe input)
                    String.format("%.4f", y),                           // Gamepad Y (axial input)
                    String.format("%.4f", rx),                          // Gamepad Rx (rotation input)
                    String.format("%.4f", 1.10),                        // Strafe Factor
                    String.format("%.4f", motorPower[0]),
                    String.format("%.4f", motorPower[1]),
                    String.format("%.4f", motorPower[2]),
                    String.format("%.4f", motorPower[3]),
                    batteryVoltage
            );

            telemetry.addData("Yaw", "%.2f", yaw);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        // CRITICAL: Close the datalogger when the OpMode ends
        datalogger.close();
    }
    // 3. REMOVED THE ENTIRE NESTED 'Datalog' CLASS (was at the end of the file)
}