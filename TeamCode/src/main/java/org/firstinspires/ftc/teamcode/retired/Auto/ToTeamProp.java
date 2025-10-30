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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Universal IMU Imports
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor;

/**
 * Autonomous OpMode to drive towards the Team Prop using encoders and IMU heading correction.
 */
@Autonomous(name="Autonomous: To Team Prop", group="Final")
@Disabled // Assume this is disabled until configured
public class ToTeamProp extends LinearOpMode {

    private static final String TAG = ToTeamProp.class.getSimpleName();

    // 1. REPLACED Datalog CLASS INSTANTIATION with Datalogger direct instantiation
    // Fields are based on an encoder-based drive with P-control correction.
    private final Datalogger datalogger = new Datalogger(
            TAG, // Filename
            "OpModeStatus", "SetSpeed", "Direction", "Distance",
            "TicksLF", "TicksLB", "TicksRF", "TicksRB",
            "CurrentSpeedLF", "CurrentSpeedLB", "CurrentSpeedRF", "CurrentSpeedRB",
            "Heading", "Battery"
    );

    // Motor and Encoder Constants
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // goBILDA 5203 series yellow jacket motor
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_MM    = 86.0;
    private static final double MM_TO_INCH           = 0.0393701;
    private static final double WHEEL_DIAMETER_INCHES= WHEEL_DIAMETER_MM * MM_TO_INCH;
    private static final double COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Motor control variables
    private DcMotorEx[] motor = new DcMotorEx[4];
    private static final String[] MOTOR_NAMES = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};

    // IMU and Angle tracking variables
    private IMU imu = null;
    private double currentHeading = 0.0;
    private double targetHeading = 0.0;

    // P-Controller Gain for Straight Drive (Requires Tuning)
    private static final double HEADING_GAIN = 0.01;

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

        // Set initial heading and perform drive movements
        if (imu != null) {
            targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        // --- AUTONOMOUS SEQUENCE (Example) ---
        // Drive to the location of the Team Prop
        encoderDrive(0.6, 28.0, 28.0, 4.0); // Drive forward 28 inches

        // Final Stop
        fullStop();
        logDatalogger("STOPPED", 0.0, 0.0, 0.0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0);

        // CRITICAL: Close the datalogger when the OpMode ends
        datalogger.close();
    }

    /**
     * Drive the robot in a straight line using encoders for distance and IMU for heading correction.
     * @param speed      The target motor power (0.0 to 1.0)
     * @param leftInches  Distance to travel for left wheels in inches.
     * @param rightInches Distance to travel for right wheels in inches.
     * @param timeoutS   Maximum time the movement can take.
     */
    private void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        if (!opModeIsActive()) return;

        int newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget;

        // Calculate target positions
        newLeftFrontTarget = motor[0].getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newLeftBackTarget = motor[1].getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightFrontTarget = motor[2].getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newRightBackTarget = motor[3].getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        // Set Target Position
        motor[0].setTargetPosition(newLeftFrontTarget);
        motor[1].setTargetPosition(newLeftBackTarget);
        motor[2].setTargetPosition(newRightFrontTarget);
        motor[3].setTargetPosition(newRightBackTarget);

        // Run to position
        for (DcMotorEx dcMotor : motor) {
            dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Reset the timeout time
        runtime.reset();

        // Start motion
        double absSpeed = Math.abs(speed);
        motor[0].setPower(absSpeed);
        motor[1].setPower(absSpeed);
        motor[2].setPower(absSpeed);
        motor[3].setPower(absSpeed);

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (motor[0].isBusy() && motor[2].isBusy())) {

            double headingError = 0.0;
            if (imu != null) {
                // Get current heading
                YawPitchRollAngles currentOrientation = imu.getRobotYawPitchRollAngles();
                currentHeading = currentOrientation.getYaw(AngleUnit.DEGREES);

                // Calculate correction to stay straight (P-control)
                headingError = targetHeading - currentHeading;
            }
            double correction = Range.clip(headingError * HEADING_GAIN, -1.0, 1.0);

            // Apply correction to motor powers
            double leftPower, rightPower;

            // Motor power must be applied relative to the direction of travel
            if (leftInches < 0 || rightInches < 0) { // Moving backward
                leftPower = -absSpeed + correction;
                rightPower = -absSpeed - correction;
            } else { // Moving forward
                leftPower = absSpeed + correction;
                rightPower = absSpeed - correction;
            }

            // Clip power to the operating range (e.g., -1.0 to 1.0)
            leftPower = Range.clip(leftPower, -1.0, 1.0);
            rightPower = Range.clip(rightPower, -1.0, 1.0);

            // Set power
            motor[0].setPower(leftPower);
            motor[1].setPower(leftPower);
            motor[2].setPower(rightPower);
            motor[3].setPower(rightPower);

            // Log data
            double avgDistance = (leftInches + rightInches) / 2.0;
            logDatalogger("ENCODER_P", speed, avgDistance, leftInches > 0 ? 1.0 : -1.0,
                    motor[0].getCurrentPosition(), motor[1].getCurrentPosition(),
                    motor[2].getCurrentPosition(), motor[3].getCurrentPosition(),
                    motor[0].getVelocity(), motor[1].getVelocity(),
                    motor[2].getVelocity(), motor[3].getVelocity(),
                    currentHeading);

            telemetry.addData("Path", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
            telemetry.addData("Current", "LF:%7d LB:%7d RF:%7d RB:%7d",
                    motor[0].getCurrentPosition(), motor[1].getCurrentPosition(),
                    motor[2].getCurrentPosition(), motor[3].getCurrentPosition());
            telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.update();
        }

        // Stop all motion
        fullStop();
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
                               double heading) {
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
                batteryVoltage
        );
    }

    // 3. REMOVED THE ENTIRE NESTED 'Datalog' CLASS
}