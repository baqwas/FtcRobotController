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
  <li> travels in a linear movement</li>
  <li> turns 90 degrees when the touch sensor is pressed</li>
  <li> the turn direction is controlled with bumper buttons on the gamepad</li>
  <li> relies on IMU data to travel in a straight line as well to perform the turns</li>
  <li> displays a few status messages</li>
  </ul>
  @author modified by armw
 * @version 1.2 - Converted to Universal IMU Interface
 * @param none
 * @return none
 * @exception none
 * @see https://stemrobototics.cs.pdx.edu/node/7266
 * <p>
 * This program registers as Autonomous OpMode in the FtcRobotController app.
 * The robot travels forward in a linear movement. When the touch sensor is pressed
 * it backs up a little enable a 90 degree turn. The bumper buttons on gamepad2
 * select the direction of the turn - left or right.
 * The program relies on the IMU sensor in the REV Robotics Control Hub that
 * runs the FtcRobotController app.
 * </p>
 * <p>
 * forward travel:
 * ^                   ^
 * |                   |
 * 0 left front        2 right front
 * X
 * ^                   ^
 * |                   |
 * 1 left back         3 right back
 *
 * hard coded numbers to avoid the use of enum construct for such a simple program
 * motor positions:
 * <ul>
 * <li>0 = left front (or forward or fore)</li>
 * <li>1 = left back (or rear or aft)</li>
 * <li>2 = right front (or forward or fore)</li>
 * <li>3 = right back (or rear or aft)</li>
 *</ul>
 * Initialize the hardware variables. Note that the strings used here as parameters
 * to 'get' must correspond to the names assigned during the robot configuration
 * step (using the FTC Robot Controller app on the phone).
 * Moon Mechanics nomenclature options for motors:
 * <device><port|starboard>|<stern/aft>
 * <role><qualifier>
 * </p>
 * @see https://first-tech-challenge.github.io/SkyStone/com/qualcomm/robotcore/hardware/DcMotor.html
 *
 * @see https://docs.revrobotics.com/rev-control-system/sensors/encoders/motor-based-encoders
 * HD Hex Motor (REV-41-1291) Encoder Specifications
 * HD Hex Motor Reduction                  Bare Motor      40:1            20:1
 * Free speed, RPM                         6,000           150             300
 * Cycles per rotation of encoder shaft    28 (7 Rises)    28 (7 Rises)    28 (7 Rises)
 * Ticks per rotation of output shaft      28              1120            560
 * TICKS_PER_MOTOR_REV = 560            REV HD Hex UltraPlanetary 20:1 cartridge
 * DRIVE_GEAR_REDUCTION = 1.0
 * WHEEL_DIAMETER_MM = 75.0             REV Mecanum wheel
 * MM_TO_INCH = 0.03937008
 * TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * MM_TO_INCH * PI)
 * = 56.9887969189608
 * <p>
 * Hardware map
 * Device name      Control Hub setting
 * imu              I2C bus 0
 * motorLeftFront   port 0
 * motorLeftBack    port 1
 * motorRightFront  port 2
 * motorRightBack   port 3
 * sensorTouch      n/a
 * sensorLED        n/a
 * gamepad2         USB2
 * </p>
 */

package org.firstinspires.ftc.teamcode.Match.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// AprilTag Imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name = "Meet 1 Auto", group = "Match", preselectTeleOp="TeleOpPreviewEvent")
//@Disabled
public class AutoMeet1 extends LinearOpMode {

    // --- FSM State Definitions ---
    enum AutoState {
        TRAVEL, // New: Uses AprilTags for navigation
        LAUNCH,
        LEAVE,
        AUTO_COMPLETE
    }

    // --- Drivetrain Hardware Components ---
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;

    // --- Vision Components ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- Configuration Constants ---
    // Encoder constants for strafeLeft()
    private static final double TICKS_PER_INCH = 30.0;
    private static final double STRAFE_DISTANCE_INCHES = 12.0;
    private static final double STRAFE_SPEED = 0.5;

    // AprilTag Navigation Constants
    private static final int TARGET_TAG_ID = 5; // Example target ID
    private static final double DESIRED_DISTANCE = 6.0; // Distance in inches to stop from the tag
    private static final double CAMERA_OFFSET_X = 0.0; // Camera X-offset in inches (left/right)
    private static final double CAMERA_OFFSET_Y = 0.0; // Camera Y-offset in inches (forward/back)
    private static final double TRAVEL_TIMEOUT_SECONDS = 5.0; // NEW: Timeout for tag navigation

    // Drive Gains (P-Controller terms)
    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.01;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double HEADING_THRESHOLD = 0.5; // Acceptable degree error for turn
    private static final double RANGE_THRESHOLD = 0.5;    // Acceptable inch error for range

    private AutoState current_state = AutoState.TRAVEL;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware (Drivetrain and Vision)
        try {
            motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
            motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
            motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
            motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

            // Motor Directions
            motorLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotorEx.Direction.REVERSE);
            motorRightFront.setDirection(DcMotorEx.Direction.FORWARD);
            motorRightBack.setDirection(DcMotorEx.Direction.FORWARD);

            // Set all motors to brake mode
            motorLeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motorLeftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motorRightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motorRightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        } catch (Exception e) {
            telemetry.addData("Error", "Hardware initialization failed. Check config names: " + e.getMessage());
            telemetry.update();
            sleep(3000);
            // Must return/stop if hardware fails
            return;
        }

        // Initialize AprilTag vision
        initAprilTag();

        // 2. Setup Encoders and Telemetry
        resetEncoders();

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData("FSM Start", current_state);
        telemetry.addData("Target Tag", TARGET_TAG_ID);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // --- FSM Processing Loop ---
        while (opModeIsActive() && current_state != AutoState.AUTO_COMPLETE) {

            switch (current_state) {

                case TRAVEL:
                    // Modular call to drive using AprilTags
                    // FIX: Splitting this complex telemetry call to avoid the 'Cannot resolve method' error.
                    telemetry.addData("FSM State", "1. TRAVEL: Driving to AprilTag");
                    telemetry.addData("Target (ID/Range)", "%d / %.1f in", TARGET_TAG_ID, DESIRED_DISTANCE);
                    telemetry.update();

                    // This helper function handles the drive and state transition upon success or timeout
                    driveToAprilTag();
                    break;

                case LAUNCH:
                    // Placeholder for activating the launcher
                    telemetry.addData("FSM State", "2. LAUNCH: Performing artifact launch sequence...");
                    telemetry.update();
                    sleep(1000);

                    // Transition to the next state
                    current_state = AutoState.LEAVE;
                    break;

                case LEAVE:
                    // Core Action: Execute the strafing movement
                    telemetry.addData("FSM State", "3. LEAVE: Strafing away from target (%.1f in, Pwr %.2f)", STRAFE_DISTANCE_INCHES, STRAFE_SPEED);
                    telemetry.update();

                    // The strafeLeft function is synchronous (blocks until finished)
                    strafeLeft(STRAFE_DISTANCE_INCHES, STRAFE_SPEED);

                    // Transition to the final state
                    current_state = AutoState.AUTO_COMPLETE;
                    break;

                default:
                    // Safety break for unexpected state
                    current_state = AutoState.AUTO_COMPLETE;
                    break;
            }
        }

        // --- Final State Handling (AUTO_COMPLETE) ---
        if (opModeIsActive()) {
            // Stop all drive motors
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            telemetry.addData("FSM State", "4. AUTO_COMPLETE: Routine finished.");
            telemetry.addData("Total Time", "%.2f s", runtime.seconds());
            telemetry.update();
            sleep(1000); // Display final telemetry
        }

        // Ensure vision resources are closed when OpMode is done
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initializes the AprilTag processor and Vision Portal.
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Build the Vision Portal, using the back camera
        visionPortal = new VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Helper function for the TRAVEL state. Drives the robot to the specified
     * DESIRED_DISTANCE from the TARGET_TAG_ID using vision-based P-control.
     * Blocks until the target is reached, OpMode stops, or a timeout occurs.
     */
    private void driveToAprilTag() {

        ElapsedTime travelTimer = new ElapsedTime();
        boolean targetReached = false;

        // Loop continues until target reached, OpMode ends, or timeout is hit
        while (opModeIsActive() && !targetReached && travelTimer.seconds() < TRAVEL_TIMEOUT_SECONDS) {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            AprilTagDetection targetDetection = null;

            // Search for the desired tag
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == TARGET_TAG_ID) {
                    targetDetection = detection;
                    break;
                }
            }

            if (targetDetection != null) {
                // We see the tag! Proceed with P-control driving.

                double rangeError = targetDetection.ftcPose.range - DESIRED_DISTANCE;
                double headingError = targetDetection.ftcPose.yaw;
                double yawError = targetDetection.ftcPose.bearing;

                // Check for completion criteria
                if (Math.abs(rangeError) < RANGE_THRESHOLD && Math.abs(headingError) < HEADING_THRESHOLD) {
                    // Success! Transition state and exit loop.
                    targetReached = true;
                    current_state = AutoState.LAUNCH; // <--- This is the transition you asked about.
                    telemetry.addData("1. TRAVEL: Tag Reached!", "Moving to LAUNCH.");
                    break;
                }

                // --- Calculate Motor Powers (P-Control) ---
                double drive = rangeError * SPEED_GAIN;
                // Yaw error controls strafing; add the fixed camera offset as a constant strafe bias
                double strafe = -(yawError * STRAFE_GAIN) + CAMERA_OFFSET_X;
                double turn = headingError * TURN_GAIN;

                // Cap the power to the maximum speed
                drive = Math.max(-MAX_AUTO_SPEED, Math.min(drive, MAX_AUTO_SPEED));
                strafe = Math.max(-MAX_AUTO_SPEED, Math.min(strafe, MAX_AUTO_SPEED));
                turn = Math.max(-MAX_AUTO_SPEED, Math.min(turn, MAX_AUTO_SPEED));

                // Apply power to the motors (Mecanum equations)
                motorLeftFront.setPower(drive + strafe + turn);
                motorRightFront.setPower(drive - strafe - turn);
                motorLeftBack.setPower(drive - strafe + turn);
                motorRightBack.setPower(drive + strafe - turn);

                // Telemetry Updates
                // FIX: Clarifying the telemetry addData call by adding the format string argument explicitly
                telemetry.addData("1. TRAVEL: Tracking Tag", "ID %d", targetDetection.id);
                telemetry.addData("Target Range", "%.1f in", DESIRED_DISTANCE);
                telemetry.addData("Current Range", "%.2f in", targetDetection.ftcPose.range);
                telemetry.addData("Errors (R/Y/B)", "%.2f / %.2f / %.2f", rangeError, headingError, yawError);
                telemetry.addData("Drive (D/S/T)", "%.2f / %.2f / %.2f", drive, strafe, turn);

            } else {
                // Tag not detected, stop movement and keep searching.
                motorLeftFront.setPower(0);
                motorLeftBack.setPower(0);
                motorRightFront.setPower(0);
                motorRightBack.setPower(0);
                //telemetry.addData("1. TRAVEL: Searching for Tag %d (Time left: %.1f)", TARGET_TAG_ID, TRAVEL_TIMEOUT_SECONDS - travelTimer.seconds());
                telemetry.addData("1. TRAVEL: Searching for Tag %d (Time left: %.1f)", TARGET_TAG_ID);

                sleep(20);
            }
            telemetry.update();
        }

        // --- Final Motor Stop and Timeout Handling ---
        // Stop all drive motors definitively after loop exit
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);

        // If the loop finished due to timeout and not reaching the target, transition to a safe state
        if (opModeIsActive() && !targetReached) {
            telemetry.addData("FSM State", "1. TRAVEL: TIMEOUT! %.1f seconds elapsed. Skipping LAUNCH, moving to LEAVE.", TRAVEL_TIMEOUT_SECONDS);
            current_state = AutoState.LEAVE;
        }
    }


    /**
     * Resets the motor encoders to 0 and sets the run mode to RUN_USING_ENCODER.
     */
    private void resetEncoders() {
        // Set to STOP_AND_RESET_ENCODER first
        motorLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Then set to RUN_USING_ENCODER for normal operations
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Executes an accurate strafe left movement based on encoder counts at a specified power.
     * This function blocks until the movement is complete.
     * @param inches The distance in inches to strafe left.
     * @param power The motor power to apply (0.0 to 1.0)
     */
    private void strafeLeft(double inches, double power) {
        // Calculate the target position change in encoder ticks
        int targetTicks = (int) (inches * TICKS_PER_INCH);

        // Get the current position of all four motors
        int currentLF = motorLeftFront.getCurrentPosition();
        int currentLB = motorLeftBack.getCurrentPosition();
        int currentRF = motorRightFront.getCurrentPosition();
        int currentRB = motorRightBack.getCurrentPosition();

        // Determine target positions for STRAFE LEFT movement
        int targetPositionLF = currentLF - targetTicks; // Move 'reverse' (backward wheel direction)
        int targetPositionLB = currentLB + targetTicks; // Move 'forward' (forward wheel direction)
        int targetPositionRF = currentRF + targetTicks; // Move 'forward' (forward wheel direction)
        int targetPositionRB = currentRB - targetTicks; // Move 'reverse' (backward wheel direction)

        // Set the new target positions
        motorLeftFront.setTargetPosition(targetPositionLF);
        motorLeftBack.setTargetPosition(targetPositionLB);
        motorRightFront.setTargetPosition(targetPositionRF);
        motorRightBack.setTargetPosition(targetPositionRB);

        // Switch to RUN_TO_POSITION mode
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Set power for all motors (using the new 'power' parameter)
        double absPower = Math.abs(power);
        motorLeftFront.setPower(absPower);
        motorLeftBack.setPower(absPower);
        motorRightFront.setPower(absPower);
        motorRightBack.setPower(absPower);

        // Wait for all motors to reach their target positions
        while (opModeIsActive() &&
                (motorLeftFront.isBusy() || motorLeftBack.isBusy() ||
                        motorRightFront.isBusy() || motorRightBack.isBusy())) {

            // Display current motor status
            telemetry.addData("FSM State", "3. LEAVE: Executing Strafe");
            telemetry.addData("Target Ticks", "%d", targetTicks);
            telemetry.addData("LF/RF Pos", "%7d/%7d", motorLeftFront.getCurrentPosition(), motorRightFront.getCurrentPosition());
            telemetry.addData("LB/RB Pos", "%7d/%7d", motorLeftBack.getCurrentPosition(), motorRightBack.getCurrentPosition());
            telemetry.addData("Applied Power", "%.2f", absPower);
            telemetry.update();
        }

        // Stop all motion by setting power back to 0
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);

        // Change back to RUN_USING_ENCODER
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
