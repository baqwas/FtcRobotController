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

import org.firstinspires.ftc.teamcode.Utility.Datalogger;

/**
 * Autonomous program which drives a set distance using encoders
 * and the RUN_TO_POSITION motor mode.
 */
@Autonomous(name="ByEncoder_Linear", group="Retired")
@Disabled
public class ByEncoder_Linear extends LinearOpMode {

    private static final String TAG = ByEncoder_Linear.class.getSimpleName();
    private final Datalog datalog = new Datalog(TAG);

    private DcMotorEx  motorLeftFront   = null;
    private DcMotorEx  motorLeftBack    = null;
    private DcMotorEx  motorRightFront  = null;
    private DcMotorEx  motorRightBack   = null;

    private DcMotorEx[] motor = new DcMotorEx[4];
    private final String[] motorLabels = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};

    // Constants for drivetrain calculations
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // REV HD Hex Motor 20:1
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;      // No external gearing
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // GoBilda Mecanum Wheels
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     DRIVE_TIMEOUT_SECONDS   = 10.0;

    // Telemetry fields
    private double currentHeading = 0.0;
    private int loopCounter = 0;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        try {
            motorLeftFront  = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
            motorLeftBack   = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
            motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
            motorRightBack  = hardwareMap.get(DcMotorEx.class, "motorRightBack");

            motor[0] = motorLeftFront;
            motor[1] = motorLeftBack;
            motor[2] = motorRightFront;
            motor[3] = motorRightBack;

            // Set motor directions (assuming a standard setup where left motors are reversed)
            motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
            motorRightFront.setDirection(DcMotor.Direction.FORWARD);
            motorRightBack.setDirection(DcMotor.Direction.FORWARD);

            for (DcMotorEx m : motor) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

        } catch (Exception e) {
            telemetry.addData("Error", "Motor initialization failed: " + e.getMessage());
        }

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        waitForStart();

        // Step 1: Drive forward 24 inches
        encoderDrive(DRIVE_SPEED, 24, 24, DRIVE_TIMEOUT_SECONDS, "FORWARD");

        // Step 2: Drive backward 12 inches
        encoderDrive(DRIVE_SPEED, -12, -12, DRIVE_TIMEOUT_SECONDS, "BACKWARD");

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // FIX: Replaced the incorrect 13-argument call with a 5-argument call to the new logDatalogger signature.
        logDatalogger("STOPPED", 0.0, "N/A", 0.0, 0.0);

        sleep(1000);  // pause to display final telemetry
        datalog.close();
    }

    /*
     * Method to perform a safe drive operation using RUN_TO_POSITION.
     *
     * @param speed: Desired maximum motor speed (0.0 to 1.0).
     * @param leftInches: Target distance for the left side in inches (+ve for FORWARD, -ve for BACKWARD).
     * @param rightInches: Target distance for the right side in inches (+ve for FORWARD, -ve for BACKWARD).
     * @param timeoutS: Time limit for the operation in seconds.
     * @param direction: A string indicating the travel direction for logging.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS, String direction) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {

            // Calculate the target positions
            newLeftFrontTarget = motorLeftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget  = motorLeftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = motorRightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget  = motorRightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            // Set Target Position
            motorLeftFront.setTargetPosition(newLeftFrontTarget);
            motorLeftBack.setTargetPosition(newLeftBackTarget);
            motorRightFront.setTargetPosition(newRightFrontTarget);
            motorRightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            for (DcMotorEx m : motor) {
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Reset the timeout timer
            runtime.reset();

            // Set the motor power
            for (DcMotorEx m : motor) {
                m.setPower(Math.abs(speed));
            }

            // Keep looping while the motors are running and OpMode is active
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeftFront.isBusy() && motorRightFront.isBusy())) {

                // Log and Telemetry updates
                // FIX: Updated call to match the new 5-argument logDatalogger signature
                logDatalogger("RUNNING", speed, direction, leftInches, speed);

                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeftFront.getCurrentPosition(),
                        motorRightFront.getCurrentPosition());
                telemetry.addData("Time/Speed", "%.2f / %.2f", runtime.seconds(), speed);
                telemetry.update();
            }

            // Stop all motion;
            for (DcMotorEx m : motor) {
                m.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotorEx m : motor) {
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    /**
     * This method logs a single line of data using a simplified set of parameters.
     * All other log fields are retrieved internally.
     */
    private void logDatalogger(String opModeStatus, double setSpeed, String direction, double distance, double currentSpeed) {
        // Increment the counter only when logging
        loopCounter++;

        // Pass all the required data points to the Datalog inner class, which handles formatting and logging.
        datalog.log(
                opModeStatus,
                loopCounter,
                setSpeed,
                direction,
                distance,
                motor[0].getCurrentPosition(),
                motor[1].getCurrentPosition(),
                motor[2].getCurrentPosition(),
                motor[3].getCurrentPosition(),
                currentSpeed,
                currentSpeed,
                currentSpeed,
                currentSpeed,
                currentHeading
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
                "OpModeStatus", "LoopCounter", "SetSpeed", "Direction", "Distance",
                "Ticks0", "Ticks1", "Ticks2", "Ticks3",
                "CurrentSpeed0", "CurrentSpeed1", "CurrentSpeed2", "CurrentSpeed3",
                "Heading"
        };

        /**
         * @param name filename for output log
         */
        public Datalog(String name)
        {
            // Initialize the simplified Datalogger with the file name and the headers array
            this.datalogger = new Datalogger(name, HEADERS);
        }

        /**
         * Logs a single line of data by manually formatting the values.
         */
        public void log(String opModeStatus, int loopCounter, double setSpeed, String direction, double distance,
                        int ticks0, int ticks1, int ticks2, int ticks3,
                        double currentSpeed0, double currentSpeed1, double currentSpeed2, double currentSpeed3,
                        double heading) {

            // Manually construct the string array for the Datalogger.log() method, applying formatting here.
            datalogger.log(
                    opModeStatus,
                    String.valueOf(loopCounter),
                    String.format("%.2f", setSpeed),
                    direction,
                    String.format("%.2f", distance),
                    String.valueOf(ticks0),
                    String.valueOf(ticks1),
                    String.valueOf(ticks2),
                    String.valueOf(ticks3),
                    String.format("%.2f", currentSpeed0),
                    String.format("%.2f", currentSpeed1),
                    String.format("%.2f", currentSpeed2),
                    String.format("%.2f", currentSpeed3),
                    String.format("%.2f", heading)
            );
        }

        /**
         * Closes the datalogger. Implements the AutoCloseable interface for use in try-with-resources.
         */
        @Override
        public void close() {
            datalogger.close();
        }
    }
}