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
  <li> travels a linear distance using IMU for heading correction</li>
  <li> drives all motors at a fixed power and direction</li>
  <li> uses encoders for distance tracking </li>
  <li> displays a few status messages</li>
  </ul>
  @author modified by armw
 * @version 1.1
 * @param none
 * @return none
 * @exception none
 * <p>
 * This program registers as Autonomous OpMode in the FtcRobotController app.
 * The robot travels forward a specified linear distance (24 inches in this example).
 * The program relies on the IMU sensor in the REV Robotics Control Hub that
 * runs the FtcRobotController app for steering correction.
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
 * </p>
 */

package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor;

import java.util.Objects;

@Autonomous(name = "Mecanum: Travel IMU", group = "Test")
@Disabled

public class MecanumTravelIMU extends LinearOpMode {

    private static final String TAG = MecanumTravelIMU.class.getSimpleName();

    // 1. REPLACED Datalog CLASS INSTANTIATION with Datalogger direct instantiation
    private final Datalogger datalogger = new Datalogger(
            TAG, // Filename
            "OpModeStatus", "Yaw", "Pitch", "Roll", "Yaw Error", "Ticks Error",
            "Correction", "Target Ticks", "Current Ticks",
            "LeftFront Power", "LeftBack Power", "RightFront Power", "RightBack Power",
            "Battery"
    );

    static final double TICKS_PER_INCH = 56.9887969189608; // REV Robotics motor and wheel specific!

    // IMU and Angle tracking variables
    private IMU imu = null;
    YawPitchRollAngles lastAngles = null; // Initialized to null for safety check

    // ADDED: Battery Sensor field
    private BatteryVoltageSensor batterySensor = null;

    double globalAngle = 0.0, correction, yawError, ticksError;
    private final ElapsedTime runtime = new ElapsedTime();
    double travelLength = 24.0;
    double travelPower = 0.40;
    double targetTicks;
    int motorTicks;

    // motor entities for drivetrain
    private DcMotorEx[] motor = new DcMotorEx[4];
    private static final String[] MOTOR_NAMES = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    private boolean motorsInitialized = true; // Flag to track initialization success


    /**
     * Function to travel a linear distance.
     */
    private void LinearTravel(double travelLength, double travelPower) {
        if (!motorsInitialized) return; // Exit if drive hardware failed initialization

        boolean travelCompleted = false; // = true when length has been traveled by robot
        targetTicks = TICKS_PER_INCH * travelLength;

        // Reset IMU angle for current movement
        resetAngle();

        while (opModeIsActive() && !travelCompleted)            // drive until end of period.
        {
            // Use gyro to drive in a straight line.
            correction = checkDirection();
            yawError = -getAngle(); // Error is the negative of the current angle (Proportional control)

            // Get current position (using one motor as a reference)
            motorTicks = Objects.requireNonNull(motor[0]).getCurrentPosition();

            // Calculate the error in ticks
            ticksError = targetTicks - motorTicks;

            if (motorTicks >= targetTicks) {
                travelCompleted = true;
            } else {
                // Apply power with steering correction
                motor[0].setPower(travelPower - correction); // Left Front
                motor[1].setPower(travelPower - correction); // Left Back
                motor[2].setPower(travelPower + correction); // Right Front
                motor[3].setPower(travelPower + correction); // Right Back
            }

            // Datalogging (Using new Datalogger API)
            String batteryVoltage = (batterySensor != null) ? batterySensor.getFormattedVoltage() : "N/A";

            datalogger.log(
                    "RUNNING",
                    String.format("%.4f", yawError), // Reusing yawError for current yaw
                    "N/A", // Pitch (Not tracked in this OpMode)
                    "N/A", // Roll (Not tracked in this OpMode)
                    String.format("%.4f", yawError), // Yaw Error (Same as -getAngle())
                    String.format("%.4f", ticksError),
                    String.format("%.4f", correction),
                    String.format("%.0f", targetTicks),
                    String.format("%d", motorTicks),
                    String.format("%.4f", motor[0].getPower()),
                    String.format("%.4f", motor[1].getPower()),
                    String.format("%.4f", motor[2].getPower()),
                    String.format("%.4f", motor[3].getPower()),
                    batteryVoltage
            );


            telemetry.addData("1 IMU heading", (lastAngles != null) ? lastAngles.getYaw(AngleUnit.DEGREES) : "N/A");
            telemetry.addData("2 Global heading", globalAngle);
            telemetry.addData("3 Correction", correction);
            telemetry.addData("4 Motor Ticks", motorTicks);
            telemetry.addData("5 Target Ticks", targetTicks);
            telemetry.update();
        }
        fullStop();                     // turn the motors off
    }


    /**
     * Function to stop power to all defined motors.
     */
    private void fullStop() {
        for (DcMotorEx dcMotor : motor) {
            if (dcMotor != null) {
                dcMotor.setPower(0.0);
            }
        }
        // Set to STOP_AND_RESET_ENCODER to prepare for the next movement
        for (DcMotorEx dcMotor : motor) {
            if (dcMotor != null) {
                dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        // Then set back to RUN_USING_ENCODER (as is typical for autonomous with corrections)
        for (DcMotorEx dcMotor : motor) {
            if (dcMotor != null) {
                dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }


    @Override
    public void runOpMode() throws InterruptedException   // called when init button is  pressed.
    {
        // --- SAFE MOTOR INITIALIZATION ---
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            try {
                motor[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
                motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Motor %s not found. Check configuration!", MOTOR_NAMES[i]);
                motorsInitialized = false;
            }
        }

        // Set direction for right-side motors
        motor[2].setDirection(DcMotor.Direction.REVERSE);
        motor[3].setDirection(DcMotor.Direction.REVERSE);

        // --- SAFE IMU INITIALIZATION ---
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
            imu.initialize(parameters);
            imu.resetYaw();
            telemetry.addData("Status", "IMU Initialized and Yaw Reset");
        } catch (Exception e) {
            telemetry.addData("ERROR", "IMU (name 'imu') not found. Check configuration!");
        }

        // --- BATTERY SENSOR INITIALIZATION ---
        try {
            batterySensor = new BatteryVoltageSensor(hardwareMap);
            telemetry.addData("Status", "Battery Sensor Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not initialize Battery Voltage Sensor.");
        }
        telemetry.update();

        // Critical check: stop if drive motors are missing
        if (!motorsInitialized) {
            telemetry.addData("FATAL ERROR", "Drive motors missing. OpMode cannot run.");
            telemetry.update();
            waitForStart();
            datalogger.close();
            return; // Exit if critical hardware is missing
        }

        waitForStart();                     // wait for Start button to be pressed for Linear OpMode

        if (isStopRequested()) {
            datalogger.close();
            return;
        }

        // The main movement call
        LinearTravel(travelLength, travelPower);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

        // CRITICAL: Close the datalogger when the OpMode ends
        datalogger.close();
    }

    /**
     * Resets the cumulative angle tracking to zero and initializes the lastAngles entity
     */
    private void resetAngle() {
        if (imu != null) {
            lastAngles = imu.getRobotYawPitchRollAngles();
        } else {
            telemetry.addData("ERROR", "IMU not initialized. Cannot reset angle.");
        }
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // Return 0.0 if IMU or angle data is not available
        if (imu == null || lastAngles == null) return 0.0;

        // We experimentally determined the Z axis is the axis we want to use for heading angle (Yaw).
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        // Safety check if the new angle reading is null
        if (angles == null) return globalAngle;

        double deltaAngle = angles.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

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
     * not traveling in a straight line. Use a simple proportional control to
     * correct the deviation.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        if (imu == null) return 0.0; // Cannot check direction without IMU

        double correction, angle, gain = 0.04; // Proportional gain for steering. May need tuning.

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    // 3. REMOVED THE ENTIRE NESTED 'Datalog' CLASS (was at the end of the file)
}