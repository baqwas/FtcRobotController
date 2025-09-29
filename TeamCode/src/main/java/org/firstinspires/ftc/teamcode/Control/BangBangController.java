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
package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;

/**
 * This OpMode demonstrates a simple bang-bang control system for a flywheel motor,
 * with enhanced data logging to analyze performance using a structured Datalogger utility
 * Bang-bang control is a very simple form of feedback control where the actuator
 * is either fully on or fully off, depending on whether a setpoint is met.
 * This example uses the concept of a "target velocity" for the flywheel. The code
 * will continuously check the motor's current velocity. If the current velocity
 * is below the target, the motor will be set to full power (1.0). If the current
 * velocity is at or above the target, the motor will be turned off (0.0).
 * This is a simple but effective way to maintain a target speed.
 * For more advanced and stable control, you would use a PID controller.
 */
@TeleOp(name = "Bang-Bang Control", group = "Control")
@Disabled
public class BangBangController extends OpMode {

    // Declare a motor object. We use DcMotorEx for velocity control.
    private DcMotorEx flywheelMotor = null;

    // Define the target velocity in ticks per second.
    // FTCRobotController SDK typically uses encoder ticks for velocity.
    // You will need to determine the appropriate value for your specific motor and gear ratio.
    private final double TARGET_VELOCITY = 2000; // Example target in ticks/sec

    // A timer to display some information on the telemetry.
    private ElapsedTime runtime = new ElapsedTime();

    // Data logging variables using the provided Datalogger utility.
    private Datalog datalog;

    @Override
    public void init() {
        // Get the motor from the hardware map.
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        // Set the motor direction. You might need to change this depending on your robot's wiring.
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

        // This is crucial for velocity control. The motor will use the built-in
        // encoder to try and maintain a set velocity.
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the zero power behavior to BRAKE. This helps the motor stop faster.
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the new Datalogger
        datalog = new Datalog("FlywheelBangBang_Data");
        datalog.opModeStatus.set("Initialized");
        datalog.targetVelocity.set(TARGET_VELOCITY);
        datalog.writeLine();

        // Tell the user everything is ready.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target Velocity", "%.2f ticks/sec", TARGET_VELOCITY);
    }

    @Override
    public void loop() {
        // Get the current velocity of the flywheel motor.
        double currentVelocity = flywheelMotor.getVelocity();
        double motorPower;

        // Implement the bang-bang control logic.
        if (currentVelocity < TARGET_VELOCITY) {
            // If the current velocity is below the target, turn the motor on at full power.
            motorPower = 1.0;
            datalog.opModeStatus.set("Running (motor ON)");
        } else {
            // If the current velocity is at or above the target, turn the motor off.
            motorPower = 0.0;
            datalog.opModeStatus.set("Running (motor OFF)");
        }

        flywheelMotor.setPower(motorPower);

        // Update and log the data using the Datalog utility.
        datalog.currentVelocity.set(currentVelocity);
        datalog.motorPower.set(motorPower);
        datalog.writeLine();

        // Add useful information to the telemetry.
        telemetry.addData("Status", datalog.opModeStatus);
        telemetry.addData("Current Velocity", "%.2f ticks/sec", currentVelocity);
        telemetry.addData("Motor Power", "%.2f", motorPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop the motor when the OpMode is stopped.
        flywheelMotor.setPower(0);
        // The Datalogger utility automatically handles closing the file.
        datalog.opModeStatus.set("Stopped");
        datalog.writeLine();
    }

    /**
     * This class encapsulates all the fields that will go into the datalog.
     * It mirrors the Datalog class from the provided example.
     */
    public static class Datalog {
        // The underlying datalogger object
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField currentVelocity = new Datalogger.GenericField("CurrentVelocity");
        public Datalogger.GenericField targetVelocity = new Datalogger.GenericField("TargetVelocity");
        public Datalogger.GenericField motorPower = new Datalogger.GenericField("MotorPower");

        public Datalog(String name) {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()
                    .setFilename(name)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    .setFields(
                            opModeStatus,
                            currentVelocity,
                            targetVelocity,
                            motorPower
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }
}
