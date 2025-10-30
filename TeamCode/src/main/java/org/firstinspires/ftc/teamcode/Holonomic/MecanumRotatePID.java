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
  <li> rotates by a set degrees (90 degrees after init)</li>
  <li> the turn direction is controlled with bumper buttons on the gamepad</li>
  <li> relies on IMU data to perform the turns using a PID controller</li>
  <li> displays a few status messages</li>
  </ul>
 */

package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Control.PIDController;

/**
 * Mecanum wheels rotate a precise number of degrees using an IMU and PID.
 */
@Autonomous(name = "Mecanum Rotate PID", group = "Final")
@Disabled
public class MecanumRotatePID extends LinearOpMode {
    private static final String TAG = MecanumRotatePID.class.getSimpleName();
    private final Datalog datalog = new Datalog(TAG);

    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;
    private DcMotorEx[] motor = new DcMotorEx[4];

    private TouchSensor touchSensor;
    private IMU imu = null;

    // IMU Data Fields
    private YawPitchRollAngles angles = null;
    private Orientation imuOrientation;

    // PID Controller for Rotation
    private PIDController pidRotate;
    // Rotation PID Constants
    private final double P_ROT = 0.015;
    private final double I_ROT = 0.00000;
    private final double D_ROT = 0.0;

    // Constants
    private final double ROTATE_SPEED = 0.25;
    private final double TARGET_ANGLE = 90.0; // The angle to rotate to

    // Telemetry and Logging fields
    private double yaw = 0.0;
    private double pitch = 0.0;
    private double roll = 0.0;
    private double currentAngle = 0.0;
    private double globalAngle = 0.0;
    private double lastAngle = 0.0;
    private int loopCounter = 0;
    private double powerLeft = 0.0;
    private double powerRight = 0.0;

    @Override
    public void runOpMode() {
        initHardware();
        pidRotate = new PIDController(P_ROT, I_ROT, D_ROT);

        telemetry.addData("Status", "Hardware Initialized. Waiting for start.");
        telemetry.update();

        // Loop to log data and wait for START
        while (!opModeIsActive()) {
            updateIMUData();
            telemetry.addData(">", "Robot Ready. Press PLAY.");
            telemetry.addData("Yaw/Pitch/Roll", "%.1f / %.1f / %.1f", yaw, pitch, roll);
            telemetry.update();

            // Corrected previous error (mismatched arguments) and kept this form.
            logDatalogger("IDLE", 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        // --- START OF OPMODE ---
        resetAngle();

        // Rotate 90 degrees left (negative direction)
        rotate(-TARGET_ANGLE, ROTATE_SPEED);

        fullStop();
        // The Datalog.close() method is part of the AutoCloseable interface implemented in the Datalog inner class
        datalog.close();
    }

    private void initHardware() {
        try {
            motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
            motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
            motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
            motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

            motor[0] = motorLeftFront;
            motor[1] = motorLeftBack;
            motor[2] = motorRightFront;
            motor[3] = motorRightBack;

            for (DcMotorEx m : motor) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
            motorRightFront.setDirection(DcMotor.Direction.FORWARD);
            motorRightBack.setDirection(DcMotor.Direction.FORWARD);

        } catch (Exception e) {
            telemetry.addData("ERROR", "Motor initialization failed.");
        }

        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
            imu.initialize(parameters);
            imu.resetYaw();
        } catch (Exception e) {
            telemetry.addData("ERROR", "IMU 'imu' not found.");
            imu = null;
        }

        try {
            touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        } catch (Exception e) {
            telemetry.addData("WARNING", "Touch sensor 'sensor_touch' not found.");
            touchSensor = null;
        }
    }

    private void fullStop() {
        for (DcMotorEx m : motor) {
            if (m != null) {
                m.setPower(0);
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    private void updateIMUData() {
        if (imu != null) {
            angles = imu.getRobotYawPitchRollAngles();
            imuOrientation = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            yaw = angles.getYaw(AngleUnit.DEGREES);
            pitch = angles.getPitch(AngleUnit.DEGREES);
            roll = angles.getRoll(AngleUnit.DEGREES);
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngle = getAngle();
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. +ve is left, -ve is right.
     */
    private double getAngle() {
        updateIMUData(); // Update yaw/pitch/roll
        currentAngle = yaw;
        double deltaAngle = currentAngle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngle = currentAngle;

        return globalAngle;
    }

    /**
     * Rotate left or right a specified number of degrees.
     * @param degrees  Degrees to turn, +ve is left, -ve is right
     * @param power    Maximum power to apply (0 to 1.0)
     */
    private void rotate(double degrees, double power) {
        if (imu == null || motorLeftFront == null) return;

        resetAngle();
        pidRotate.setPID(P_ROT, I_ROT, D_ROT);
        pidRotate.setSetpoint(degrees);
        pidRotate.setOutputRange(-power, power);
        pidRotate.setTolerance(2.0); // 2 degree tolerance
        pidRotate.enable();

        // Continue rotating while OpMode is active and PID is not on target
        while (opModeIsActive() && !pidRotate.onTarget()) {
            double error = pidRotate.getSetpoint() - getAngle();
            double correction = pidRotate.performPID(getAngle());

            powerLeft = correction;
            powerRight = -correction;

            // Apply minimum power to overcome friction
            if (Math.abs(correction) < 0.1) {
                if (degrees > 0) { // Target left
                    powerLeft = 0.1;
                    powerRight = -0.1;
                } else { // Target right
                    powerLeft = -0.1;
                    powerRight = 0.1;
                }
            }

            // Re-check for overshoot to brake
            if (degrees < 0 && getAngle() < degrees) { // Overshot right
                fullStop();
                break;
            } else if (degrees > 0 && getAngle() > degrees) { // Overshot left
                fullStop();
                break;
            }

            motorLeftFront.setPower(powerLeft);
            motorLeftBack.setPower(powerLeft);
            motorRightFront.setPower(powerRight);
            motorRightBack.setPower(powerRight);

            telemetry.addData("Yaw", "%.2f", yaw);
            telemetry.addData("Target/Error", "%.2f / %.2f", degrees, error);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.update();

            // Corrected call (6 arguments)
            logDatalogger("ROTATING", yaw, degrees, error, powerLeft, powerRight);
        }

        // Stop all motion
        fullStop();
        resetAngle();
    }

    /**
     * This method sets all fields in the datalogger for one line
     * The signature is kept for compatibility with the calls in runOpMode and rotate().
     */
    private void logDatalogger(String opModeStatus, double currentYaw, double targetAngle, double error, double leftPower, double rightPower) {
        // Increment the counter only when logging
        loopCounter++;

        // Pass all the required data points to the Datalog inner class, which handles formatting and logging.
        datalog.log(
                opModeStatus,
                loopCounter,
                currentYaw,
                pitch,
                roll,
                globalAngle,
                error,
                leftPower,
                rightPower
        );
    }

    // --- REPLACED DATALOG CLASS (Uses modern simplified Datalogger) ---

    /**
     * This class encapsulates all the fields that will go into the datalog.
     * It uses the modern simplified Datalogger approach, removing GenericField.
     */
    public static class Datalog implements AutoCloseable {
        private final Datalogger datalogger;

        // The headers, strictly maintaining the order for the log file
        private static final String[] HEADERS = new String[]{
                "OpModeStatus", "LoopCounter", "Yaw", "Pitch", "Roll",
                "globalAngle", "deltaAngle", "powerLeft", "powerRight"
        };

        /**
         * @param name the name of the file where the fields are written
         */
        public Datalog(String name) {
            // Use the simplified Datalogger constructor, passing all headers
            // The Datalogger class is imported from org.firstinspires.ftc.teamcode.Utility.Datalogger
            this.datalogger = new Datalogger(name, HEADERS);
        }

        /**
         * Logs a single line of data by manually formatting the values.
         */
        public void log(String opModeStatus, int loopCounter, double yaw, double pitch, double roll,
                        double globalAngle, double deltaAngle, double powerLeft, double powerRight) {

            // Manually construct the string array for the Datalogger.log() method, applying formatting here.
            datalogger.log(
                    opModeStatus,
                    String.valueOf(loopCounter),
                    String.format("%.2f", yaw),
                    String.format("%.2f", pitch),
                    String.format("%.2f", roll),
                    String.format("%.2f", globalAngle),
                    String.format("%.2f", deltaAngle),
                    String.format("%.2f", powerLeft),
                    String.format("%.2f", powerRight)
            );
        }

        /**
         * Closes the underlying file writer.
         */
        @Override
        public void close() {
            datalogger.close();
        }
    }
}