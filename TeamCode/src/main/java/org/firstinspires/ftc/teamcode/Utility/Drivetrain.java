package org.firstinspires.ftc.teamcode.Utility;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Added to enable OpMode context

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.retired.Auto.ToTeamProp;

/*
 * Copyright (c) 2019 ParkCircus Productions. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Drivetrain encapsulates the mechanical and control aspects of a Mecanum drive.
 * It is designed to be used by a LinearOpMode.
 */
public class Drivetrain {
    private static final String TAG = Drivetrain.class.getSimpleName();
    private final LinearOpMode opMode; // Reference to the calling OpMode

    // Motor and Control variables
    private final DcMotorEx[] motor = new DcMotorEx[4];
    private final String[] motorLabels = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    private IMU imu_ = null;
    private final Datalog datalog;
    private final BatteryVoltageSensor batterySensor; // Added for logging

    // IMU State tracking
    private double globalAngle = 0;
    private YawPitchRollAngles lastAngles = null;

    // Motor & Encoder Constants (Adjust as necessary)
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // goBILDA 5203 series
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // P-Controller Gain for Straight Drive (Adjust as necessary)
    private static final double HEADING_GAIN = 0.01;

    /**
     * Initializes the Drivetrain and its hardware components.
     *
     * @param opMode The calling LinearOpMode.
     * @param filename The base filename for the Datalogger.
     */
    public Drivetrain(LinearOpMode opMode, String filename) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Initialize IMU
        try {
            imu_ = hardwareMap.get(IMU.class, "imu");
            // Define hub orientation
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu_.initialize(new IMU.Parameters(orientationOnRobot));
            imu_.resetYaw();
        } catch (Exception e) {
            opMode.telemetry.addData("ERROR", "IMU 'imu' not found.");
        }

        // Initialize Motors
        for (int i = 0; i < motor.length; i++) {
            try {
                motor[i] = (DcMotorEx) hardwareMap.get(motorLabels[i]);
                motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                motor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                motor[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // Use encoders for precise control
            } catch (Exception e) {
                opMode.telemetry.addData("ERROR", "Motor %s not found.", motorLabels[i]);
            }
        }

        // Set Directions (Example Configuration - Adjust as needed)
        motor[0].setDirection(DcMotorEx.Direction.REVERSE); // motorLeftFront
        motor[1].setDirection(DcMotorEx.Direction.REVERSE); // motorLeftBack
        motor[2].setDirection(DcMotorEx.Direction.FORWARD); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.FORWARD); // motorRightBack

        // Initialize Battery Sensor
        batterySensor = new BatteryVoltageSensor(hardwareMap);

        // Initialize Datalogger
        datalog = new Datalog(filename);

        opMode.telemetry.addData("Drivetrain", "Initialized");
        opMode.telemetry.update();
    }


    // --- MOVEMENT METHODS (Simplified Examples) ---

    /**
     * Drives the robot forward/backward using encoders and IMU correction.
     * @param speed Target motor power (0.0 to 1.0).
     * @param distanceInches Distance to travel.
     * @param timeoutS Max time.
     */
    public void driveStraight(double speed, double distanceInches, double timeoutS) {
        if (!opMode.opModeIsActive()) return;

        int targetTicks = (int)(distanceInches * COUNTS_PER_INCH);

        // Set motor targets based on current position and calculated ticks
        int[] targetPositions = new int[4];
        for (int i = 0; i < motor.length; i++) {
            targetPositions[i] = motor[i].getCurrentPosition() + targetTicks;
            motor[i].setTargetPosition(targetPositions[i]);
            motor[i].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor[i].setPower(Math.abs(speed));
        }

        double startHeading = getHeading();
        long startTime = System.currentTimeMillis();

        while (opMode.opModeIsActive() &&
                (System.currentTimeMillis() < startTime + timeoutS * 1000) &&
                (motor[0].isBusy() || motor[2].isBusy())) {

            double currentHeading = getHeading();
            double headingError = startHeading - currentHeading;
            double correction = org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES.normalize(headingError) * HEADING_GAIN;

            double leftPower = speed + correction;
            double rightPower = speed - correction;

            // Clip power to the operating range
            leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
            rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

            motor[0].setPower(leftPower);
            motor[1].setPower(leftPower);
            motor[2].setPower(rightPower);
            motor[3].setPower(rightPower);

            // Log the state
            logDriveState("DRIVE", speed, distanceInches,
                    motor[0].getCurrentPosition(), motor[1].getCurrentPosition(),
                    motor[2].getCurrentPosition(), motor[3].getCurrentPosition(),
                    leftPower, rightPower, currentHeading);

            opMode.telemetry.addData("Status", "Driving Straight");
            opMode.telemetry.addData("Correction", "%.2f", correction);
            opMode.telemetry.update();
        }

        stopAndReset();
    }

    /**
     * Rotates the robot to a target angle.
     * @param targetAngle The absolute heading in degrees (e.g., 90, 0, -45).
     * @param power The motor power for rotation.
     */
    public void rotateToAngle(double targetAngle, double power) {
        if (!opMode.opModeIsActive()) return;

        double currentHeading = getHeading();
        double angleError = targetAngle - currentHeading;
        double normalizedError = org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES.normalize(angleError);

        while (opMode.opModeIsActive() && Math.abs(normalizedError) > 2.0) { // Rotate until within 2 degrees tolerance
            currentHeading = getHeading();
            angleError = targetAngle - currentHeading;
            normalizedError = org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES.normalize(angleError);

            // Simple P-like control for rotation power
            double rotationPower = power * Math.signum(normalizedError);

            motor[0].setPower(-rotationPower);
            motor[1].setPower(-rotationPower);
            motor[2].setPower(rotationPower);
            motor[3].setPower(rotationPower);

            // Log the state
            logDriveState("ROTATE", rotationPower, 0.0,
                    motor[0].getCurrentPosition(), motor[1].getCurrentPosition(),
                    motor[2].getCurrentPosition(), motor[3].getCurrentPosition(),
                    -rotationPower, rotationPower, currentHeading);

            opMode.telemetry.addData("Status", "Rotating");
            opMode.telemetry.addData("Error", "%.2f", normalizedError);
            opMode.telemetry.update();

            sleep(10); // Throttle loop
        }

        stopAndReset();
    }


    // --- UTILITY METHODS ---

    /**
     * Gets the current yaw angle from the IMU, normalized to -180 to 180.
     * @return Current heading in degrees.
     */
    public double getHeading() {
        if (imu_ == null) return 0.0;
        YawPitchRollAngles angles = imu_.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Stops all motors and resets their encoders.
     */
    public void stopAndReset() {
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);
            dcMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Closes the datalogger. Must be called at the end of the OpMode.
     */
    public void closeDatalogger() {
        if (datalog != null) {
            datalog.close();
        }
    }

    // --- DATALOGGING METHODS ---

    /**
     * Internal helper to log the current robot state.
     */
    private void logDriveState(String status, double setSpeed, double setDistance,
                               int ticksLF, int ticksLB, int ticksRF, int ticksRB,
                               double powerLeft, double powerRight, double heading) {
        String batteryVoltage = batterySensor.getFormattedVoltage();

        // Use the Datalog utility class's log method
        datalog.log(
                status,
                String.format("%.2f", setSpeed),
                String.format("%.2f", setDistance),
                String.valueOf(ticksLF),
                String.valueOf(ticksLB),
                String.valueOf(ticksRF),
                String.valueOf(ticksRB),
                String.format("%.2f", powerLeft),
                String.format("%.2f", powerRight),
                String.format("%.2f", heading),
                batteryVoltage
        );
    }


    // --- UPDATED DATALOG CLASS (Replaces GenericField pattern) ---

    /**
     * This class encapsulates the Datalogger fields and methods for Drivetrain.
     * It uses the modern Datalogger constructor and log() method.
     */
    public static class Datalog implements AutoCloseable {
        private final Datalogger datalogger;

        /**
         * @param name The base name of the log file.
         */
        public Datalog(String name) {
            // Define the headers based on the data logged in logDriveState()
            this.datalogger = new Datalogger(
                    name,
                    "OpModeStatus",
                    "SetSpeed",
                    "SetDistance",
                    "TicksLF",
                    "TicksLB",
                    "TicksRF",
                    "TicksRB",
                    "PowerLeft",
                    "PowerRight",
                    "Heading",
                    "Battery"
            );
        }

        /**
         * The operation to output one record of the fields to the storage file.
         * The values must be passed in the order defined in the constructor.
         */
        public void log(String... values) {
            datalogger.log(values);
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