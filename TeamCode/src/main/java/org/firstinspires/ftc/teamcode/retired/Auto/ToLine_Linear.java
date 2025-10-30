/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.retired.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor; // Added for the Line Tracking use case
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Universal IMU Imports
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor;

/**
 * Mecanum wheels drive forward using encoders until a line is detected by a color sensor.
 */
@Autonomous(name="Mecanum: To Line Linear", group="Encoder")
@Disabled
public class ToLine_Linear extends LinearOpMode {

    private static final String TAG = ToLine_Linear.class.getSimpleName();

    // 1. REPLACED Datalog CLASS INSTANTIATION with Datalogger direct instantiation
    private final Datalogger datalogger = new Datalogger(
            TAG, // Filename
            "OpModeStatus", "SetSpeed", "Direction", "Distance",
            "TicksLF", "TicksLB", "TicksRF", "TicksRB",
            "CurrentSpeedLF", "CurrentSpeedLB", "CurrentSpeedRF", "CurrentSpeedRB",
            "Heading", "LineDetected", "Red", "Green", "Blue", "Battery" // Added Color and Line fields
    );

    // Motor and Encoder Constants
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // goBILDA 5203 series yellow jacket motor
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_MM    = 86.0;
    private static final double MM_TO_INCH           = 0.0393701;
    private static final double WHEEL_DIAMETER_INCHES= WHEEL_DIAMETER_MM * MM_TO_INCH;
    private static final double COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final int MAX_TICKS = (int)(48.0 * COUNTS_PER_INCH); // Max travel of 48 inches (just a safety limit)
    private static final int LINE_THRESHOLD_RED = 250; // Example threshold for Red line detection

    // Motor and Sensor control variables
    private DcMotorEx[] motor = new DcMotorEx[4];
    private static final String[] MOTOR_NAMES = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    private ColorSensor colorSensor = null; // Assumes a sensor named "sensor_color"

    // IMU and Angle tracking variables
    private IMU imu = null;

    // Battery Sensor field
    private BatteryVoltageSensor batterySensor = null;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // --- HARDWARE INITIALIZATION ---
        telemetry.addData("Status", "Initializing Hardware...");
        telemetry.update();

        // Initialize Motors
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            try {
                motor[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
                motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Motor %s not found. Check configuration!", MOTOR_NAMES[i]);
            }
        }
        motor[0].setDirection(DcMotor.Direction.FORWARD);
        motor[1].setDirection(DcMotor.Direction.FORWARD);
        motor[2].setDirection(DcMotor.Direction.REVERSE);
        motor[3].setDirection(DcMotor.Direction.REVERSE);

        // Initialize Color Sensor
        try {
            colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
            telemetry.addData("Status", "Color Sensor Initialized");
        } catch (Exception e) {
            telemetry.addData("WARNING", "Color sensor 'sensor_color' not found.");
            colorSensor = null;
        }

        // Initialize IMU
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


        // Wait for the start button
        telemetry.addData("Status", "Initialized. Press PLAY.");
        telemetry.update();
        waitForStart();
        runtime.reset();

        if (isStopRequested()) {
            datalogger.close();
            return;
        }

        // --- AUTONOMOUS SEQUENCE ---
        driveToLine(0.4, 4.0); // Drive at 0.4 power, max 4.0 seconds

        // Final Stop
        fullStop();
        logDatalogger("STOPPED", 0.0, 0.0, 0.0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0, 0, 0);

        // CRITICAL: Close the datalogger when the OpMode ends
        datalogger.close();
    }

    /**
     * Drive the robot forward using encoders until a line is detected or max distance/time is reached.
     * @param speed The target motor power (0.0 to 1.0)
     * @param timeoutS Maximum time the movement can take.
     */
    private void driveToLine(double speed, double timeoutS) {
        if (!opModeIsActive()) return;

        int startPosition = motor[0].getCurrentPosition();
        int targetPosition = startPosition + MAX_TICKS;

        // Set Target Position (Max distance safety)
        motor[0].setTargetPosition(targetPosition);
        motor[1].setTargetPosition(targetPosition);
        motor[2].setTargetPosition(targetPosition);
        motor[3].setTargetPosition(targetPosition);

        // Run to position
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Reset the timeout time
        runtime.reset();

        // Start motion
        motor[0].setPower(speed);
        motor[1].setPower(speed);
        motor[2].setPower(speed);
        motor[3].setPower(speed);

        boolean lineDetected = false;

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && motor[0].isBusy()) {

            int red = 0, green = 0, blue = 0;
            if (colorSensor != null) {
                red = colorSensor.red();
                green = colorSensor.green();
                blue = colorSensor.blue();
                if (red > LINE_THRESHOLD_RED) {
                    lineDetected = true;
                }
            }

            if (lineDetected) {
                break; // Stop when the line is detected
            }

            // Get current heading for logging
            double currentHeading = 0.0;
            if (imu != null) {
                YawPitchRollAngles currentOrientation = imu.getRobotYawPitchRollAngles();
                currentHeading = currentOrientation.getYaw(AngleUnit.DEGREES);
            }

            // Log data
            double currentDistance = (motor[0].getCurrentPosition() - startPosition) / COUNTS_PER_INCH;

            logDatalogger("TO_LINE", speed, currentDistance, 1.0,
                    motor[0].getCurrentPosition(), motor[1].getCurrentPosition(),
                    motor[2].getCurrentPosition(), motor[3].getCurrentPosition(),
                    motor[0].getVelocity(), motor[1].getVelocity(),
                    motor[2].getVelocity(), motor[3].getVelocity(),
                    currentHeading, lineDetected, red, green, blue);

            telemetry.addData("Status", "Driving to line...");
            telemetry.addData("Current Pos", "%7d", motor[0].getCurrentPosition());
            telemetry.addData("Color (R/G/B)", "%d / %d / %d", red, green, blue);
            telemetry.update();
        }

        // Stop all motion
        fullStop();

        telemetry.addData("Status", lineDetected ? "Line Detected!" : "Timeout Reached");
        telemetry.update();
        sleep(500);
    }

    /**
     * Function to stop power to all defined motors.
     */
    private void fullStop() {
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Revert to running with encoder for next move
        }
    }

    /**
     * Internal helper to log the current robot state.
     */
    private void logDatalogger(String status, double setSpeed, double distance, double direction,
                               int ticksLF, int ticksLB, int ticksRF, int ticksRB,
                               double velocityLF, double velocityLB, double velocityRF, double velocityRB,
                               double heading, boolean lineDetected, int red, int green, int blue) {
        String batteryVoltage = (batterySensor != null) ? batterySensor.getFormattedVoltage() : "N/A";

        // 2. UPDATED LOGGING CALL
        datalogger.log(
                status,
                String.format("%.2f", setSpeed),
                String.format("%.2f", direction),
                String.format("%.2f", distance),
                String.valueOf(ticksLF),
                String.valueOf(ticksLB),
                String.valueOf(ticksRF),
                String.valueOf(ticksRB),
                String.format("%.2f", velocityLF),
                String.format("%.2f", velocityLB),
                String.format("%.2f", velocityRF),
                String.format("%.2f", velocityRB),
                String.format("%.2f", heading),
                String.valueOf(lineDetected),
                String.valueOf(red),
                String.valueOf(green),
                String.valueOf(blue),
                batteryVoltage
        );
    }

    // 3. REMOVED THE ENTIRE NESTED 'Datalog' CLASS
}