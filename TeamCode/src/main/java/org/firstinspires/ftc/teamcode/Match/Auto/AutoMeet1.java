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
 * TICKS_PER_INCH = 33.4308289114498; // SWYFT Drive v2; goBILDA 5203 series, 12.7:1, 86 mm
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU; // NEW: Universal IMU Interface
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; // Required for setup

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;// For reading IMU data
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

// AprilTag Imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

// --- Datalogger Imports ---
import org.firstinspires.ftc.teamcode.Utility.Datalogger;
// --- IMU Universal Imports ---
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; // NEW: For IMU mounting config
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;       // NEW: To get angles from IMU
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES; // Static import for clarity
// --------------------------

@Autonomous(name = "Meet 1 Auto", group = "Match", preselectTeleOp="TeleOpPreviewEvent")
//@Disabled
public class AutoMeet1 extends LinearOpMode {
    private final String TAG = this.getClass().getSimpleName();
    private Datalogger datalogger = null;

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

    // CHANGED: Use the Universal IMU interface
    private IMU imu = null;

    private TouchSensor touchSensor = null; // Touch sensor for state transition

    // --------------------------
    // --- Vision Components ---
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;     // Set to -1 for ANY tag.
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // --- Configuration Constants ---
    // Encoder constants for strafeLeft()
    private static final double TICKS_PER_INCH = 33.4308289114498;
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
        //datalog = new Datalog(TAG); // Datalog initialization
        // --- A. Hardware Mapping ---
        // Assuming your IMU is named "imu" in the configuration
        imu = hardwareMap.get(IMU.class, "imu");

        // --- B. Define Hub Orientation on Robot ---
        // 1. Create the orientation object using the specified directions
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                // Logo Direction: The side of the hub with the REV logo
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                // USB Direction: The side of the hub with the USB-C port
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        // --- C. Create and Apply IMU Parameters ---
        // 2. Create the IMU Parameters object and pass the orientation
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        // 3. Initialize the IMU with the parameters
        // This process automatically calibrates and configures the sensor.
        imu.initialize(parameters);
        // 4. Reset the Yaw (Heading) to 0 degrees at the start of autonomous
        imu.resetYaw();
        telemetry.addData("IMU Status", "Initialized with Logo FORWARD, USB UP");
        telemetry.update();

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

        // --- DATALOGGER INITIALIZATION (NEW) ---
        try {
            // 1. Initialize the Datalogger with a filename and column headers
            datalogger = new Datalogger(
                    "AutoMeet1_Run",
                    "Elapsed_Time_s",
                    "FSM_State",
                    "LF_Motor_Power",
                    "Global_Angle"
            );
        } catch (RuntimeException e) {
            telemetry.addData("ERROR", "Datalogger Init Failed: " + e.getMessage());
            datalogger = null; // Ensure datalogger is null if setup failed
        }
        // ---------------------------------------

        waitForStart();
        runtime.reset();

        // Ensure the logger is closed if the OpMode is stopped abruptly
        try (Datalogger logOnClose = datalogger) {
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
                        // Transition to the next state
                        current_state = AutoState.LAUNCH;
                        break;

                    case LAUNCH:
                        // Placeholder for activating the launcher
                        telemetry.addData("FSM State", "2. LAUNCH: Performing artifact launch sequence...");
                        telemetry.update();
                        sleep(1000);
                        /*
                        launcherLeft.setPower(1.0);
                        launcherRight.setPower(1.0);
                        sleep(2000);
                        launcherLeft.setPower(0.0);
                        launcherRight.setPower(0.0);
                         */
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
        } // The 'try-with-resources' block ensures logOnClose.close() is called when the OpMode ends


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
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /**
     * Helper function for the TRAVEL state. Drives the robot to the specified
     * DESIRED_DISTANCE from the TARGET_TAG_ID using vision-based P-control.
     * Blocks until the target is reached, OpMode stops, or a timeout occurs.
     */
    private void driveToAprilTag() {

        boolean targetFound     = false;
        double  drive           = 0;
        double  strafe          = 0;
        double  turn            = 0;
        ElapsedTime travelTimer = new ElapsedTime();
        boolean targetReached = false;

        desiredTag  = null;

        // Loop continues until target reached, OpMode ends, or timeout is hit
        while (opModeIsActive() && !targetReached && travelTimer.seconds() < TRAVEL_TIMEOUT_SECONDS) {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    }
                }
            }

            if (desiredTag != null) {
                // We see the tag! Proceed with P-control driving.
                telemetry.addData("\n>","Driving to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Camera Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Camera Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Camera Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

                // We now get the camera's position relative to the tag, and use our known
                // camera offsets to find the robot's center relative to the tag.
                // NOTE: ftcPose.x is the left-right distance, and ftcPose.y is the forward-backward distance.
                double robotCenterX = desiredTag.ftcPose.x - CAMERA_OFFSET_X;
                double robotCenterY = desiredTag.ftcPose.y - CAMERA_OFFSET_Y;

                // Calculate the errors we need to correct
                double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                double headingError = desiredTag.ftcPose.yaw;
                double yawError = desiredTag.ftcPose.bearing;


                // Use the speed and turn "gains" to calculate the motor powers.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(headingError * STRAFE_GAIN, -HEADING_THRESHOLD, HEADING_THRESHOLD);
                turn   = Range.clip(headingError * TURN_GAIN, -RANGE_THRESHOLD, RANGE_THRESHOLD);

                telemetry.addData("Robot","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                telemetry.addData("Robot Position", "(X: %5.2f, Y: %5.2f)", robotCenterX, robotCenterY);

            } else {
                // If the target is not found, stop the robot.
                drive = 0;
                strafe = 0;
                turn = 0;
                telemetry.addData("Status", "Target Not Found - Stopping");
            }
            telemetry.update();


            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double powerLeftFront    =  x - y - yaw;
        double powerRightFront   =  x + y + yaw;
        double powerLeftBack     =  x + y - yaw;
        double powerRightBack    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(powerLeftFront), Math.abs(powerRightFront));
        max = Math.max(max, Math.abs(powerLeftBack));
        max = Math.max(max, Math.abs(powerRightBack));

        if (max > 1.0) {
            powerLeftFront /= max;
            powerRightFront /= max;
            powerLeftBack /= max;
            powerRightBack  /= max;
        }

        // Send powers to the wheels.
        motorLeftFront.setPower(powerLeftFront);
        motorRightFront.setPower(powerRightFront);
        motorLeftBack.setPower(powerLeftBack);
        motorRightBack.setPower(powerRightBack);
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
