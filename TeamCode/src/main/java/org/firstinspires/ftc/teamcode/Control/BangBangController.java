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

package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor; // NEW IMPORT

/**
 * BangBangController is a simple demonstration of an on-off control system
 * for maintaining a target velocity on a motor. It also demonstrates the use
 * of the updated Datalogger class.
 */
@TeleOp(name="Control: BangBang", group="Concept")
@Disabled
public class BangBangController extends LinearOpMode {

    private static final String TAG = BangBangController.class.getSimpleName();

    // 1. REPLACED Datalog CLASS INSTANTIATION with Datalogger direct instantiation
    private final Datalogger datalogger = new Datalogger(
            TAG, // Filename
            "OpModeStatus", "CurrentVelocity", "TargetVelocity", "MotorPower", "Battery"
    );

    // Motor and Control variables
    private DcMotorEx motor = null;
    private final ElapsedTime runtime = new ElapsedTime();
    private double targetVelocity = 1000; // Target velocity in ticks/sec

    // NEW: Battery Sensor field
    private BatteryVoltageSensor batterySensor = null;

    @Override
    public void runOpMode() {

        // --- HARDWARE INITIALIZATION ---
        telemetry.addData("Status", "Initializing Hardware...");
        telemetry.update();

        try {
            motor = hardwareMap.get(DcMotorEx.class, "motor");
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            // Set motor to RUN_USING_ENCODER to enable velocity control (even if not using a built-in PID)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Motor 'motor' not found. Check configuration.");
            telemetry.update();
            sleep(3000);
            return;
        }

        // --- BATTERY SENSOR INITIALIZATION ---
        try {
            batterySensor = new BatteryVoltageSensor(hardwareMap);
            telemetry.addData("Status", "Battery Sensor Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not initialize Battery Voltage Sensor.");
        }
        telemetry.update();

        telemetry.addData("Status", "Initialized. Press PLAY.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) {
            datalogger.close();
            return;
        }

        while (opModeIsActive()) {

            // --- USER INPUT FOR TARGET VELOCITY ---
            if (gamepad1.dpad_up) {
                targetVelocity += 50; // Increase target velocity
            } else if (gamepad1.dpad_down) {
                targetVelocity -= 50; // Decrease target velocity
            }
            // Clamp target velocity to a reasonable range (e.g., 0 to 2500)
            targetVelocity = Math.max(0, Math.min(2500, targetVelocity));

            // --- BANG-BANG CONTROL LOGIC ---
            double currentVelocity = motor.getVelocity();
            double motorPower;

            if (currentVelocity < targetVelocity) {
                motorPower = 1.0; // Full power ON
            } else {
                motorPower = 0.0; // Full power OFF
            }

            // Apply Power
            motor.setPower(motorPower);

            // --- TELEMETRY ---
            telemetry.addData("Status", "Running");
            telemetry.addData("Target Velocity (ticks/s)", "%.0f", targetVelocity);
            telemetry.addData("Current Velocity (ticks/s)", "%.2f", currentVelocity);
            telemetry.addData("Motor Power", "%.2f", motorPower);
            telemetry.update();

            // --- DATALOGGING ---
            String batteryVoltage = (batterySensor != null) ? batterySensor.getFormattedVoltage() : "N/A";

            // 2. UPDATED LOGGING CALL
            datalogger.log(
                    "RUNNING",
                    String.format("%.2f", currentVelocity),
                    String.format("%.0f", targetVelocity),
                    String.format("%.2f", motorPower),
                    batteryVoltage
            );
        }

        // Stop motor and close datalogger when OpMode stops
        motor.setPower(0);

        // CRITICAL: Close the datalogger when the OpMode ends
        datalogger.close();
    }
    // 3. REMOVED THE ENTIRE NESTED 'Datalog' CLASS (was at the end of the file)
}
