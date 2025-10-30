/*
 * MIT License
 *
 * Copyright (c) 2024 ParkCircus Productions; All Rights Reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the \"Software\"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
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
 * \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor;

/**
 * Mecanum wheels travel using IMU and a touch sensor to trigger the logging of
 * motion, IMU data, and motor power.
 */
@TeleOp(name = "Test Drive Strafe", group = "Final")
public class TestDriveStrafe extends LinearOpMode {
    private static final String TAG = TestDriveStrafe.class.getSimpleName();
    // Datalogger implementation uses the modern simplified approach
    private final Datalog datalog = new Datalog(TAG);
    private final ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    private IMU imu = null;

    // Motors
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;

    // Touch Sensor
    private TouchSensor touchSensor;

    // IMU Data Fields (AngularVelocity, Position, Velocity, and Acceleration objects are now unused)
    private YawPitchRollAngles angles = null;
    private Orientation imuOrientation;
    private AngularVelocity angularVelocity;
    private Position imuPosition;
    private Velocity imuVelocity;
    private Acceleration imuAcceleration;

    // Control Variables
    private double powerFactor = 0.5; // Overall power reduction factor
    private boolean lastA = false;
    private boolean lastB = false;

    // Datalog fields are now simple data types
    private String opModeStatus = "INIT";
    private double acquisitionTime = 0.0;
    private double yaw = 0.0;
    private double pitch = 0.0;
    private double roll = 0.0;
    private double globalAngle = 0.0;
    private double deltaAngle = 0.0;
    private double correction = 0.0;
    private double positionX = 0.0;
    private double positionY = 0.0;
    private double positionZ = 0.0;
    // velocityX/Y/Z will now log 0.0
    private double velocityX = 0.0;
    private double velocityY = 0.0;
    private double velocityZ = 0.0;
    // accelx/y/z will now log 0.0
    private double accelx = 0.0;
    private double accely = 0.0;
    private double accelz = 0.0;
    private int ticks0 = 0;
    private int ticks1 = 0;
    private int ticks2 = 0;
    private int ticks3 = 0;
    private double powerLeftFront = 0.0;
    private double powerLeftBack = 0.0;
    private double powerRightFront = 0.0;
    private double powerRightBack = 0.0;
    private double motor0 = 0.0;
    private double motor1 = 0.0;
    private double motor2 = 0.0;
    private double motor3 = 0.0;

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Hardware Initialized. Waiting for start.");
        telemetry.update();

        waitForStart();
        runtime.reset();
        opModeStatus = "RUNNING";

        while (opModeIsActive()) {
            // --- DATA ACQUISITION ---
            updateIMUData();
            updateMotorData();

            // --- GAMEPAD INPUT ---
            handleDriveInput();
            handleLoggingInput();

            // --- LOGGING ---
            datalog.writeLine(opModeStatus, acquisitionTime, yaw, pitch, roll, globalAngle, deltaAngle, correction,
                    positionX, positionY, positionZ, velocityX, velocityY, velocityZ, accelx, accely, accelz,
                    ticks0, ticks1, ticks2, ticks3, powerLeftFront, powerLeftBack, powerRightFront, powerRightBack,
                    motor0, motor1, motor2, motor3);

            // --- TELEMETRY ---
            telemetry.addData("Status", opModeStatus);
            telemetry.addData("Yaw/Pitch/Roll", "%.1f / %.1f / %.1f", yaw, pitch, roll);
            telemetry.addData("Touch Sensor", touchSensor != null ? (touchSensor.isPressed() ? "PRESSED" : "NOT PRESSED") : "N/A");
            telemetry.addData("LeftFront Pwr/Ticks", "%.2f / %d", powerLeftFront, ticks0);
            telemetry.addData("RightFront Pwr/Ticks", "%.2f / %d", powerRightFront, ticks2);
            telemetry.update();
        }

        // --- CLEANUP ---
        fullStop();
        datalog.close();
    }

    private void initHardware() {
        try {
            motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
            motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
            motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
            motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

            DcMotorEx[] motors = {motorLeftFront, motorLeftBack, motorRightFront, motorRightBack};
            for (DcMotorEx motor : motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        DcMotorEx[] motors = {motorLeftFront, motorLeftBack, motorRightFront, motorRightBack};
        for (DcMotorEx motor : motors) {
            if (motor != null) {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    private void updateIMUData() {
        if (imu != null) {
            // These orientation methods should be compatible with most IMU implementations
            angles = imu.getRobotYawPitchRollAngles();
            imuOrientation = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // FIX: All velocity and acceleration methods on 'imu' are unsupported in this SDK version.
            // We set the log variables to 0.0 and skip the calls to prevent compilation errors.
            positionX = 0.0;
            positionY = 0.0;
            positionZ = 0.0;

            yaw = angles.getYaw(AngleUnit.DEGREES);
            pitch = angles.getPitch(AngleUnit.DEGREES);
            roll = angles.getRoll(AngleUnit.DEGREES);

            // Set all velocity and acceleration log fields to 0.0
            velocityX = 0.0;
            velocityY = 0.0;
            velocityZ = 0.0;

            accelx = 0.0;
            accely = 0.0;
            accelz = 0.0;

            // Use OpMode runtime as a placeholder timestamp
            acquisitionTime = runtime.seconds();
        }
    }

    private void updateMotorData() {
        if (motorLeftFront != null) {
            ticks0 = motorLeftFront.getCurrentPosition();
            ticks1 = motorLeftBack.getCurrentPosition();
            ticks2 = motorRightFront.getCurrentPosition();
            ticks3 = motorRightBack.getCurrentPosition();

            powerLeftFront = motorLeftFront.getPower();
            powerLeftBack = motorLeftBack.getPower();
            powerRightFront = motorRightFront.getPower();
            powerRightBack = motorRightBack.getPower();

            motor0 = motorLeftFront.getVelocity();
            motor1 = motorLeftBack.getVelocity();
            motor2 = motorRightFront.getVelocity();
            motor3 = motorRightBack.getVelocity();
        }
    }

    private void handleDriveInput() {
        // Mecanum drive
        double y = -gamepad1.left_stick_y; // Remember, this is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x; // Rotation

        // Denominator is the largest absolute value of the four powers, which keeps all power values within [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double lfPower = (y + x + rx) / denominator * powerFactor;
        double lbPower = (y - x + rx) / denominator * powerFactor;
        double rfPower = (y - x - rx) / denominator * powerFactor;
        double rbPower = (y + x - rx) / denominator * powerFactor;

        // Apply power
        if (motorLeftFront != null) {
            motorLeftFront.setPower(lfPower);
            motorLeftBack.setPower(lbPower);
            motorRightFront.setPower(rfPower);
            motorRightBack.setPower(rbPower);
        }
    }

    private void handleLoggingInput() {
        // Toggle power factor with A and B
        boolean currentA = gamepad1.a;
        boolean currentB = gamepad1.b;

        if (currentA && !lastA) {
            powerFactor = 0.5;
        } else if (currentB && !lastB) {
            powerFactor = 1.0;
        }

        lastA = currentA;
        lastB = currentB;

        // If the touch sensor is pressed, change status for log
        if (touchSensor != null && touchSensor.isPressed()) {
            opModeStatus = "TOUCH_PRESSED";
        } else {
            opModeStatus = "RUNNING";
        }
    }


    // --- UPDATED DATALOG CLASS (Uses modern Datalogger) ---

    /**
     * This class encapsulates all the fields that will go into the datalog.
     * It uses the modern Datalogger constructor and log() method.
     */
    public static class Datalog implements AutoCloseable {
        private final Datalogger datalogger;

        // The headers, strictly maintaining the order from the original GenericField setup
        private static final String[] HEADERS = new String[]{
                "OpModeStatus", "Timestamp", "Yaw", "Pitch", "Roll",
                "globalAngle", "deltaAngle", "correction", "positionX", "positionY",
                "positionZ", "velocityX", "velocityY", "velocityZ", "accelx",
                "accely", "accelz", "ticks0", "ticks1", "ticks2", "ticks3",
                "powerLeftFront", "powerLeftBack", "powerRightFront", "powerRightBack",
                "motor0", "motor1", "motor2", "motor3"
        };

        /**
         * @param name the name of the file where the fields are written
         */
        public Datalog(String name) {
            // Use the simplified Datalogger constructor, passing all headers
            this.datalogger = new Datalogger(name, HEADERS);
        }

        /**
         * The operation to output one record of the fields to the storage file.
         * The values must be passed in the order defined in HEADERS.
         */
        public void writeLine(String opModeStatus, double acquisitionTime, double yaw, double pitch, double roll,
                              double globalAngle, double deltaAngle, double correction, double positionX, double positionY,
                              double positionZ, double velocityX, double velocityY, double velocityZ, double accelx,
                              double accely, double accelz, int ticks0, int ticks1, int ticks2, int ticks3,
                              double powerLeftFront, double powerLeftBack, double powerRightFront, double powerRightBack,
                              double motor0, double motor1, double motor2, double motor3) {

            // Manually construct the string array for the log() method
            datalogger.log(
                    opModeStatus,
                    String.format("%.3f", acquisitionTime),
                    String.format("%.2f", yaw),
                    String.format("%.2f", pitch),
                    String.format("%.2f", roll),
                    String.format("%.2f", globalAngle),
                    String.format("%.2f", deltaAngle),
                    String.format("%.2f", correction),
                    String.format("%.3f", positionX),
                    String.format("%.3f", positionY),
                    String.format("%.3f", positionZ),
                    String.format("%.3f", velocityX),
                    String.format("%.3f", velocityY),
                    String.format("%.3f", velocityZ),
                    String.format("%.3f", accelx),
                    String.format("%.3f", accely),
                    String.format("%.3f", accelz),
                    String.valueOf(ticks0),
                    String.valueOf(ticks1),
                    String.valueOf(ticks2),
                    String.valueOf(ticks3),
                    String.format("%.2f", powerLeftFront),
                    String.format("%.2f", powerLeftBack),
                    String.format("%.2f", powerRightFront),
                    String.format("%.2f", powerRightBack),
                    String.format("%.0f", motor0),
                    String.format("%.0f", motor1),
                    String.format("%.0f", motor2),
                    String.format("%.0f", motor3)
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