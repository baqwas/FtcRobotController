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
  <li> rotates by 90 degrees after initialization. The rotation angle is controlled with bumper buttons on the gamepad.</li>
  <li> relies on IMU data to perform the turns.</li>
  <li> displays a few status messages.</li>
  </ul>
  @author modified by armw
 * @version 1.2 - Converted to Universal IMU Interface and simplified Datalogger
 * @param none
 * @return none
 * @exception none
 * @see https://stemrobototics.cs.pdx.edu/node/7266
 * <p>
 * This program registers as Autonomous OpMode in the FtcRobotController app.
 * The robot rotates to a target angle. The program relies on the IMU sensor
 * in the REV Robotics Control Hub that runs the FtcRobotController app.
 * </p>
 * <p>
 * Rotations:
 * Button      Rotation
 * square (X)  45 degrees
 * triangle (Y) 135 degrees
 * circle (B)  180 degrees
 * cross (A)   90 degrees
 * left bumper rotates left, right bumper rotates right
 * </p>
 */
package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor;

import java.util.Objects;

@Autonomous(name = "Mecanum: Rotate IMU", group = "Test")
@Disabled
public class MecanumRotateIMU extends LinearOpMode {

    private static final String TAG = MecanumRotateIMU.class.getSimpleName();

    // 1. REPLACED Datalog CLASS INSTANTIATION with Datalogger direct instantiation
    private final Datalogger datalogger = new Datalogger(
            TAG, // Filename
            "OpModeStatus", "Loop Counter", "Yaw", "Pitch", "Roll", "Global Angle", "Delta Angle",
            "Power Left", "Power Right", "Battery"
    );

    // IMU and Angle tracking variables
    private IMU imu = null;
    YawPitchRollAngles lastAngles = null;
    double globalAngle = 0.0, deltaAngle = 0.0;
    private final ElapsedTime runtime = new ElapsedTime();
    double power = 0.40; // Default rotation power

    // ADDED: Battery Sensor field
    private BatteryVoltageSensor batterySensor = null;

    // motor entities for drivetrain
    private DcMotorEx[] motor = new DcMotorEx[4];
    private static final String[] MOTOR_NAMES = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    private boolean motorsInitialized = true; // Flag to track initialization success
    private int loopCounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- SAFE MOTOR INITIALIZATION ---
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            try {
                motor[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
                motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Motor %s not found. Check configuration!", MOTOR_NAMES[i]);
                motorsInitialized = false;
            }
        }
        // Set direction for right-side motors
        if (motorsInitialized) {
            motor[2].setDirection(DcMotor.Direction.REVERSE);
            motor[3].setDirection(DcMotor.Direction.REVERSE);
        }

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

        waitForStart();

        if (isStopRequested()) {
            datalogger.close();
            return;
        }

        while (opModeIsActive()) {
            // Check gamepad2 for rotation command
            int degreesToRotate = 0;

            if (gamepad1.x) degreesToRotate = 45;
            else if (gamepad1.y) degreesToRotate = 135;
            else if (gamepad1.b) degreesToRotate = 180;
            else if (gamepad1.a) degreesToRotate = 90;

            if (degreesToRotate != 0) {
                if (gamepad1.right_bumper) {
                    rotate(-degreesToRotate, power); // Negative for right turn
                } else if (gamepad1.left_bumper) {
                    rotate(degreesToRotate, power); // Positive for left turn
                }
            }

            // Log current status (even when stopped)
            logDatalogger("IDLE");

            telemetry.addData("Global Angle", "%.2f", globalAngle);
            telemetry.addData("Status", "Waiting for command");
            telemetry.update();

            idle();
        }

        // CRITICAL: Close the datalogger when the OpMode ends
        datalogger.close();
    }

    /**
     * Resets the cumulative angle tracking to zero and initializes the lastAngles entity
     */
    private void resetAngle() {
        if (imu != null) {
            imu.resetYaw();
            lastAngles = imu.getRobotYawPitchRollAngles();
        }
        globalAngle = 0;
        deltaAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // Return 0.0 if IMU or angle data is not available
        if (imu == null || lastAngles == null) return globalAngle;

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        if (angles == null) return globalAngle;

        deltaAngle = angles.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right in degrees.
     *
     * @param degrees amount to turn.
     * @param power   power of motors applied
     * param isRight true if turn direction is right
     */
    private void rotate(int degrees, double power) {
        if (imu == null || !motorsInitialized) return; // Cannot rotate without IMU or motors

        double leftPower, rightPower;

        resetAngle(); // reset heading for tracking with IMU data

        if (degrees < 0) {              // turn right
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {       // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // Apply power
        if (motor[0] != null) motor[0].setPower(leftPower); // Left Front
        if (motor[1] != null) motor[1].setPower(leftPower); // Left Back (Adjusted for rotation)
        if (motor[2] != null) motor[2].setPower(rightPower); // Right Front
        if (motor[3] != null) motor[3].setPower(rightPower); // Right Back (Adjusted for rotation)

        // rotate until turn is completed.
        if (degrees < 0) { // Right Turn (Negative degrees)
            // Wait until IMU moves off zero, then until angle is reached
            while (opModeIsActive() && getAngle() == 0) {
                logDatalogger("ROTATING");
            }
            while (opModeIsActive() && getAngle() > degrees) {
                logDatalogger("ROTATING");
            }
        } else { // Left Turn (Positive degrees)
            while (opModeIsActive() && getAngle() < degrees) {
                logDatalogger("ROTATING");
            }
        }

        fullStop();

        resetAngle(); // reset angle tracking on new heading
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
    }

    /**
     * Internal helper to log the current robot state.
     */
    private void logDatalogger(String status) {
        double currentYaw = (lastAngles != null) ? lastAngles.getYaw(AngleUnit.DEGREES) : 0.0;
        double currentPitch = (lastAngles != null) ? lastAngles.getPitch(AngleUnit.DEGREES) : 0.0;
        double currentRoll = (lastAngles != null) ? lastAngles.getRoll(AngleUnit.DEGREES) : 0.0;
        double powerLeft = (motor[0] != null) ? motor[0].getPower() : 0.0;
        double powerRight = (motor[2] != null) ? motor[2].getPower() : 0.0;
        String batteryVoltage = (batterySensor != null) ? batterySensor.getFormattedVoltage() : "N/A";

        datalogger.log(
                status,
                String.valueOf(loopCounter++),
                String.format("%.4f", currentYaw),
                String.format("%.4f", currentPitch),
                String.format("%.4f", currentRoll),
                String.format("%.4f", globalAngle),
                String.format("%.4f", deltaAngle),
                String.format("%.4f", powerLeft),
                String.format("%.4f", powerRight),
                batteryVoltage
        );
    }

    // 2. REMOVED THE ENTIRE NESTED 'Datalog' CLASS
}