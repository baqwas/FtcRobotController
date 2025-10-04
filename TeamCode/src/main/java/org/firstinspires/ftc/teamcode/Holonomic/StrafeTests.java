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

/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Holonomic;

// **REPLACED BNO055IMU with IMU**
import com.qualcomm.robotcore.hardware.IMU;
// Removed: import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// **ADDED YawPitchRollAngles for Universal IMU data retrieval**
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 */
@TeleOp(name = "Strafe: Left or Right - must be preset", group = "Test")
@Disabled

public class StrafeTests extends LinearOpMode {

    private static final  String TAG = StrafeTests.class.getSimpleName(); // for use in logging

    // Declare OpMode members
    private final ElapsedTime strafeTime = new ElapsedTime();

    // **CHANGED TYPE TO IMU**
    IMU imu;

    // motor entities for drivetrain
    String[] motorLabels = {
            "motorLeftFront",           // port 0 Control Hub
            "motorLeftBack",            // port 1 Control Hub
            "motorRightFront",          // port 2 Control Hub
            "motorRightBack"            // port 3 Control Hub
    };
    private final DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};

    public ElapsedTime myRuntime = new ElapsedTime();

    /**
     * mecanum strafe basic fixed test
     * @param left set to true for strafe left else false for strafe right
     */
    private void strafe(boolean left) {

        double rx, x, y = 0.0;
        double vx = 0.0, vy = 0.0, omegaz = 0.0;
        long runTime;

        if (left) {
            x = -0.3;
            runTime = 2200;
        } else {
            x = 0.3;
            runTime = 2200;
        }

        // **UPDATED: Use YawPitchRollAngles and getYaw() for heading**
        double initHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        strafeTime.reset();

        while (opModeIsActive() && !isStopRequested() && strafeTime.seconds() < runTime) {

            // **UPDATED: Use YawPitchRollAngles and getYaw() for current heading**
            double currentHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double botHeading = currentHeading - initHeading;

            rx = -botHeading * 0.1;

            double rotX = x * Math.cos(Math.toRadians(botHeading)) - y * Math.sin(Math.toRadians(botHeading));
            double rotY = x * Math.sin(Math.toRadians(botHeading)) + y * Math.cos(Math.toRadians(botHeading));

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // getVelocity() // current velocity of the motor, in ticks per second
            // getVelocity(AngleUnit.RADIANS) // current velocity of the motor, in angular units per second
            int v1 = motor[0].getCurrentPosition();  // current reading of the encoder
            int v2 = motor[1].getCurrentPosition();   // current reading of the encoder
            int v3 = motor[2].getCurrentPosition(); // current reading of the encoder
            int v4 = motor[3].getCurrentPosition();  // current reading of the encoder

            double va_fl = motor[0].getVelocity(AngleUnit.RADIANS); // angular velocity
            double va_rl = motor[1].getVelocity(AngleUnit.RADIANS);
            double va_fr = motor[2].getVelocity(AngleUnit.RADIANS);
            double va_rr = motor[3].getVelocity(AngleUnit.RADIANS);

            telemetry.addData("Heading", "%.2f", botHeading);
            telemetry.addData("Encoder", "%d, %d, %d, %d", v1, v2, v3, v4);
            telemetry.addData("Power", "%.2f %.2f, %.2f, %.2f", frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.addData("Omega", "%.4f %.4f, %.4f, %.4f", va_fl, va_rl, va_fr, va_rr);

            vx += (va_fl + va_rl + va_fr + va_rr) * 1.4 / 4.0;
            vy += (-va_fl + va_rl + va_fr - va_rr) * 1.4 / 4.0;
            omegaz += (-va_fl - va_rl + va_fr + va_rr) * 1.4 / (4.0 * 13.0);

            telemetry.addData("Odometry", "%.4f %.4f, %.4f", vx, vy, omegaz);
            telemetry.update();

            motor[0].setPower(frontLeftPower);
            motor[1].setPower(backLeftPower);
            motor[2].setPower(frontRightPower);
            motor[3].setPower(backRightPower);
        }

        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);
        }
    }

    @Override
    public void runOpMode() {

        telemetry.addData("Hardware", "Initialization...");
        telemetry.update();
        // Initialize the hardware variables
        // The strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        for (int i = 0; i < motor.length; i++) {
            motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]);
            /* motor stops and then brakes
             * actively resisting any external force
             * which attempts to turn the motor */
            motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            // run at any velocity with specified power level
            motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        /*
         * The logical direction in which this motor operates: FORWARD | REVERSE
         * REV Robotics motors may need two of the following
         * four statements to be enabled - PLEASE TEST before 1st use!
         */
        // motor[0].setDirection(DcMotorSimple.Direction.FORWARD); // motorLeftFront
        // motor[1].setDirection(DcMotorSimple.Direction.FORWARD); // motorLeftBack
        motor[2].setDirection(DcMotorEx.Direction.REVERSE); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.REVERSE); // motorRightBack

        telemetry.addData("Mode", "Calibrating...");
        telemetry.update();
        /*
        // **UPDATED: Universal IMU Initialization**
        IMU.Parameters parameters = new IMU.Parameters.Builder()
                // The Universal IMU defaults to SensorMode.NDOF, so this is handled implicitly.
                .setAngleUnit(IMU.AngleUnit.DEGREES) // AngleUnit is now IMU.AngleUnit
                .setAccelUnit(IMU.AccelUnit.METERS_PERSEC_PERSEC) // AccelUnit is now IMU.AccelUnit
                // loggingEnabled is not a property of the new Parameters object
                .build();

        // **UPDATED: Hardware map retrieval to use IMU interface**
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
         */
        // **ADDED: Explicitly reset yaw to zero the heading**
        imu.resetYaw();

          telemetry.addData("Mode", "Select Start");
        // Removed imu.getCalibrationStatus() as it is not a part of the Universal IMU API
        telemetry.addData("Status", "IMU System Set");
        telemetry.addData("Start", "Select PLAY");
        telemetry.update();
        // Driver expected to press PLAY after announcer blasts 3-2-1-GO
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Left", "20 secs delay after move");
            telemetry.update();
            strafe(true);
            sleep(20000);
            telemetry.addData("Right", "Reposition after move");
            telemetry.update();
            strafe(false);
            sleep(20000);
        }
    }
}