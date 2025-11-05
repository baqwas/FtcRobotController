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

package org.firstinspires.ftc.teamcode.Match.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.text.SimpleDateFormat;
import java.util.Date;

@Autonomous(name="Meet 1 Auto", group="Match", preselectTeleOp="Preview Event TeleOp")

public class AutoMeet1 extends LinearOpMode {
    private static final String TAG = AutoMeet1.class.getSimpleName();
    private static final Logger log = LoggerFactory.getLogger(AutoMeet1.class);
    // --- Logging Declaration ---
    private Datalog datalog = null;
    private double targetHeading = 0.0; // Current target heading for the robot
    private String opModeStatus = "INIT";
    // Enumerations for Alliance and Position
    enum Alliance {
        RED,
        BLUE
    }

    enum Position {
        POS1,
        POS2,
        POS3
    }

    // Enumeration for the Finite State Machine (FSM)
    enum RobotState {
        SCAN_OBELISK,
        SCAN_GOAL,
        LAUNCH,
        TRAVEL,
        LEAVE,
        COMPLETE
    }

    // Hardware for Mecanum Drivetrain
    // --- Hardware Declarations (Using components expected from the original code) ---
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;
    private IMU imu = null; // Assuming IMU is used for heading
    private TouchSensor touchSensor = null;
    private VoltageSensor batterySensor;
    static final double TICKS_PER_MOTOR_REV = 28.0; // SWYFT v2 drive
    static final double WHEEL_DIAMETER_INCHES = 3.3856; // 86 mm
    static final double DRIVE_GEAR_REDUCTION = 12.7;
    static final double TICKS_PER_INCH =
            (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    // private static final double TICKS_PER_INCH = 33.4308289114498; // SWYFT Drive v2; goBILDA 5203 series, 12.7:1, 86 mm
    // --- PI CONTROL CONSTANTS (Suggested initial values) ---


    // --- PI CONTROL STATE VARIABLES ---
    private double integralSum = 0.0;
    private double lastTime = 0.0; // Used to calculate Delta Time (dt)
    static final double DRIVE_SPEED = 0.5;
    private static final double HEADING_GAIN = 0.03;      // Kp for translation
    private static final double INTEGRAL_GAIN = 0.005;    // Ki for translation
    private static final double MAX_INTEGRAL_SUM = 0.3;   // Anti-windup limit
    // --- ROTATION PI CONTROL CONSTANTS ---
    private static final double TURN_GAIN = 0.02;         // Kp for rotation
    private static final double TURN_INTEGRAL_GAIN = 0.002; // Ki for rotation
    private static final double HEADING_TOLERANCE = 1.0;  // Stop tolerance in degrees for turning

    // Variables for Alliance and Position
    private Alliance alliance = Alliance.RED; // Default to Red Alliance
    private Position position = Position.POS1; // Default to Position 1

    // Initializing the FSM state
    private RobotState currentState = RobotState.LEAVE;

    // --- VISION COMPONENTS ---
    private static final int DESIRED_TAG_ID = -1;     // Set to -1 for ANY tag.
    private VisionPortal visionPortal = null; // Initialize to null for safety
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // A simple class to represent a Waypoint in the autonomous path
    private class Waypoint {
        public double x;
        public double y;
        public double heading;

        public Waypoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // Waypoint entities for each starting position.
    // Replace these placeholder values with your actual coordinates.
    private final Waypoint redPos1Waypoint = new Waypoint(-36.0, 60.0, 90.0);
    private final Waypoint redPos2Waypoint = new Waypoint(-12.0, 60.0, 90.0);
    private final Waypoint redPos3Waypoint = new Waypoint(12.0, 60.0, 90.0);

    private final Waypoint bluePos1Waypoint = new Waypoint(-36.0, -60.0, -90.0);
    private final Waypoint bluePos2Waypoint = new Waypoint(-12.0, -60.0, -90.0);
    private final Waypoint bluePos3Waypoint = new Waypoint(12.0, -60.0, -90.0);

    @Override
    public void runOpMode() {
        // --- Hardware Initialization ---
        opModeStatus = currentState.toString();
        // 1. Initialize the Datalogger with a filename and headers
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
        // 2. Get the current system date and time.
        String timestamp = dateFormat.format(new Date());
        // 3. Define the base filename and append the timestamp
        String datalogFilename = TAG + "_" + timestamp;
        datalog = new Datalog(datalogFilename);
        // ---------------------------------------------------------------------------------
        // FIX: Robust Battery Sensor Initialization
        // The original code: batterySensor = hardwareMap.voltageSensor.iterator().next();
        // can throw a NoSuchElementException if the collection is empty.
        // We ensure the collection is not empty before accessing the sensor.
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            batterySensor = hardwareMap.voltageSensor.iterator().next();
        } else {
            // Set to null and log a warning if no sensor is found, preventing NullPointerException later.
            batterySensor = null;
            telemetry.addData("WARNING", "No VoltageSensor found in hardwareMap!");
            telemetry.update();
        }
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
        // --- D. Mecanum motor initialization
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

        motorLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorEx.Direction.FORWARD);

        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        initAprilTag();

        // --- Driver Hub Pre-Match Selection ---
        while (!isStarted() && !isStopRequested()) {
            // Alliance Selection
            if (gamepad1.b) {
                alliance = Alliance.BLUE;
            } else if (gamepad1.x) {
                alliance = Alliance.RED;
            }

            // Position Selection
            if (gamepad1.dpad_up) {
                position = Position.POS1;
            } else if (gamepad1.dpad_right) {
                position = Position.POS2;
            } else if (gamepad1.dpad_down) {
                position = Position.POS3;
            }

            telemetry.addData("Alliance", "Press B|O for Blue, X|□ for Red");
            telemetry.addData("Position", "Press D-pad Up/Right/Down for POS1/POS2/POS3");
            telemetry.addData("Current Selection", "Alliance: %s, Position: %s", alliance.toString(), position.toString());
            telemetry.update();
        }

        waitForStart();
        imu.resetYaw(); // safe practice if robot orientation is orthogonal to field, i.e. heading = 0

        // --- Autonomous Execution ---

        if (opModeIsActive()) {

            telemetry.addData("Status", "Executing Autonomous Routine");
            telemetry.addData("Routine", "Alliance: %s, Position: %s", alliance.toString(), position.toString());
            telemetry.update();

            // Select the correct waypoint based on user input
            Waypoint selectedWaypoint;
            switch (alliance) {
                case RED:
                    switch (position) {
                        case POS1:
                            selectedWaypoint = redPos1Waypoint;
                            break;
                        case POS2:
                            selectedWaypoint = redPos2Waypoint;
                            break;
                        case POS3:
                            selectedWaypoint = redPos3Waypoint;
                            break;
                        default:
                            selectedWaypoint = redPos1Waypoint; // Fallback
                            break;
                    }
                    break;
                case BLUE:
                    switch (position) {
                        case POS1:
                            selectedWaypoint = bluePos1Waypoint;
                            break;
                        case POS2:
                            selectedWaypoint = bluePos2Waypoint;
                            break;
                        case POS3:
                            selectedWaypoint = bluePos3Waypoint;
                            break;
                        default:
                            selectedWaypoint = bluePos1Waypoint; // Fallback
                            break;
                    }
                    break;
                default:
                    selectedWaypoint = redPos1Waypoint; // Fallback
                    break;
            }

            // Call the main autonomous routine with the selected waypoint
            runAutonomousRoutine(selectedWaypoint);
            sleep(1000);
        }
    }

    // --- Main Autonomous Routine with FSM ---
    private void runAutonomousRoutine(Waypoint startPoint) {

        double leaveDistance = 24.0;

        telemetry.addData("Executing Path", "Starting from (%.1f, %.1f) with heading %.1f", startPoint.x, startPoint.y, startPoint.heading);
        telemetry.update();

        // Main FSM loop. The opModeIsActive() check allows the driver to stop the OpMode at any time.
        while (opModeIsActive() && currentState != RobotState.COMPLETE) {
            opModeStatus = currentState.toString();
            telemetry.addData("Current State", opModeStatus);
            telemetry.update();

            switch (currentState) {
                case SCAN_OBELISK:
                    // Placeholder: Code to move to a position to scan the obelisk
                    // and use computer vision to determine its location.
                    // sleep(2000); // Wait for vision processing

                    // After scanning, transition to the next state
                    currentState = RobotState.SCAN_GOAL;
                    break;

                case SCAN_GOAL:
                    // Placeholder: Code to move to the determined goal position
                    // and verify its location before launch.
                    // sleep(1500); // Wait for position verification

                    // After scanning the goal, transition to launch
                    currentState = RobotState.LAUNCH;
                    break;

                case LAUNCH:
                    // launch(1.0); // launch power = 1.0

                    // After the launch action is initiated, wait for it to finish
                    currentState = RobotState.TRAVEL;
                    break;

                case TRAVEL:
                    /*
                    if(drive(DRIVE_SPEED, -4, DistanceUnit.INCH, 1)){
                        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        currentState = RobotState.ROTATING;
                    }
                     */
                    currentState = RobotState.LEAVE;
                    break;

                case LEAVE:
                    // speed = 0.5, distance = 12.0, heading = 0.0 -- for test purposes only
                    if (position == Position.POS2) {
                        leaveDistance = - 24.0;
                    }
                    // driveMecanum(DRIVE_SPEED, leaveDistance, 0.0);
                    driveVectorMecanum(DRIVE_SPEED, 0.0, leaveDistance, 0.0);
                    // strafeMecanum(DRIVE_SPEED, leaveDistance, 0.0);

                    currentState = RobotState.COMPLETE;
                    break;

                case COMPLETE:
                    // This state has no action. The `while` loop condition will handle termination.
                    break;
                default:
                    throw new IllegalStateException("FSM: unexpected value: " + currentState);
            }
            opModeStatus = currentState.toString();
            telemetry.addData("Auto", opModeStatus);
            telemetry.update();
        }

        // 5. Stop and Reset
        stopAllMotors();
        logData(); // Log final status

        // 3. CRITICAL: Close the datalogger to flush the buffer and save the file
        datalog.close();
    }

    /**
     * Drives the robot straight forward or backward using encoders for distance
     * and IMU for heading correction.
     * @param speed Base power (0.0 to 1.0, direction determined by distance)
     * @param distanceInches Target distance (+ for forward, - for backward)
     * @param desiredHeading The constant heading (Yaw) to maintain (e.g., 0.0 degrees)
     */
    public void driveMecanum(double speed, double distanceInches, double desiredHeading) {
        if (!opModeIsActive()) return;

        // 1. Calculate Target Ticks
        int moveCounts = (int) (distanceInches * TICKS_PER_INCH);

        int frontLeftTarget = motorLeftFront.getCurrentPosition() + moveCounts;
        int frontRightTarget = motorRightFront.getCurrentPosition() + moveCounts;
        int backLeftTarget = motorLeftBack.getCurrentPosition() + moveCounts;
        int backRightTarget = motorRightBack.getCurrentPosition() + moveCounts;

        // 2. Set Targets and Run Mode
        motorLeftFront.setTargetPosition(frontLeftTarget);
        motorRightFront.setTargetPosition(frontRightTarget);
        motorLeftBack.setTargetPosition(backLeftTarget);
        motorRightBack.setTargetPosition(backRightTarget);

        // Use RUN_TO_POSITION
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the movement
        double absSpeed = Math.abs(speed);

        // 3. Loop until all motors reach position (Encoder Logic)
        while (opModeIsActive() &&
                (motorLeftFront.isBusy() || motorRightFront.isBusy() ||
                        motorLeftBack.isBusy() || motorRightBack.isBusy())) {

            // 4. IMU Correction Logic
            // Get current heading (Yaw)
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate error
            double error = desiredHeading - currentHeading;
            while (error > 180)  error -= 360; // Normalize error to +/- 180
            while (error <= -180) error += 360;

            // Calculate turn correction
            double turnCorrection = error * HEADING_GAIN;

            // Determine if moving forward or backward to apply correction correctly
            double direction = Math.signum(distanceInches);

            // Apply correction
            // When moving forward (direction = +1): TurnCorrection is subtracted from Left and added to Right to correct yaw
            double leftPower = absSpeed * direction - turnCorrection * direction;
            double rightPower = absSpeed * direction + turnCorrection * direction;

            // Set Power (using RUN_TO_POSITION ignores power setting except to determine velocity/torque)
            // Note: For simple movements like this, simply setting the power based on the correction often works fine.
            motorLeftFront.setPower(leftPower);
            motorLeftBack.setPower(leftPower);
            motorRightFront.setPower(rightPower);
            motorRightBack.setPower(rightPower);
            logData();
        }


    }

    /**
     * Helper function to stop and reset motor modes.
     */
    private void stopAllMotors() {
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        // Switch back to RUN_USING_ENCODER for TeleOp or next movement
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * Initializes the AprilTag processor and Vision Portal.
     */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Helper method to start the launching mechanism.
     * Replace the placeholder code with your actual launcher control logic.
     */
    private void launch(double power) {
        // Example: Set the launcher motor power
        // launcherMotor.setPower(1.0);
    }

    /**
     * Helper method to check if the launch is complete.
     * Replace the placeholder code with your sensor or timer-based logic.
     * @return true if the launch action is finished, false otherwise.
     */
    private boolean isLaunchComplete() {
        // Example: Check if a sensor detects the game element has been launched
        // or if a timer has elapsed.
        // return (launcherMotor.getPower() == 0);
        return true; // Placeholder for now
    }
// NOTE: This code assumes constants (TICKS_PER_INCH, HEADING_GAIN)
// and hardware objects (motorLeftFront, imu, etc.) are declared
// as class members in your OpMode.

    /**
     * Executes a straight strafing movement (left or right) using encoders for distance control
     * and the IMU for maintaining a consistent heading.
     * * @param speed The maximum motor power (e.g., 0.5 for 50%).
     * @param distanceInches The distance to strafe. Positive for right, negative for left.
     * @param desiredHeading The field-centric heading (yaw) to maintain during the strafe (e.g., 0.0).
     */
    public void strafeMecanum(double speed, double distanceInches, double desiredHeading) {
        if (!opModeIsActive()) return;

        // The logic below assumes: +StrafeRight / -StrafeLeft

        // 1. Calculate Target Ticks (CRITICAL DIFFERENCE from driveMecanum)
        int moveCounts = (int) (distanceInches * TICKS_PER_INCH);

        // Strafe Right (moveCounts > 0): LF+, RF-, LB-, RB+
        // Strafe Left (moveCounts < 0): LF-, RF+, LB+, RB-
        int frontLeftTarget = motorLeftFront.getCurrentPosition() + moveCounts;
        int frontRightTarget = motorRightFront.getCurrentPosition() - moveCounts;
        int backLeftTarget = motorLeftBack.getCurrentPosition() - moveCounts;
        int backRightTarget = motorRightBack.getCurrentPosition() + moveCounts;

        // 2. Set Targets and Run Mode
        motorLeftFront.setTargetPosition(frontLeftTarget);
        motorRightFront.setTargetPosition(frontRightTarget);
        motorLeftBack.setTargetPosition(backLeftTarget);
        motorRightBack.setTargetPosition(backRightTarget);

        // Use RUN_TO_POSITION
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the movement
        double absSpeed = Math.abs(speed);

        // Determine the general strafe direction sign (1.0 for right, -1.0 for left)
        double strafeDirection = Math.signum(distanceInches);

        // 3. Loop until all motors reach position (Encoder Logic)
        while (opModeIsActive() &&
                (motorLeftFront.isBusy() || motorRightFront.isBusy() ||
                        motorLeftBack.isBusy() || motorRightBack.isBusy())) {

            // 4. IMU Correction Logic
            // Get current heading (Yaw)
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate error
            double error = desiredHeading - currentHeading;
            error = normalizeAngle(error); // Helper to keep error in +/- 180 range

            // Calculate turn correction
            double turnCorrection = error * HEADING_GAIN;

            // Apply correction for strafing (CRITICAL DIFFERENCE from driveMecanum)
            // Base Power is absSpeed * strafeDirection (+ for right, - for left)

            // Mecanum Strafe Power Application:
            // LF: Forward/Reverse + Turn Correction
            // RF: Reverse/Forward - Turn Correction
            // LB: Reverse/Forward + Turn Correction
            // RB: Forward/Reverse - Turn Correction
            double baseStrafePower = absSpeed * strafeDirection;

            double leftFrontPower  = baseStrafePower + turnCorrection;
            double rightFrontPower = -baseStrafePower - turnCorrection;
            double leftBackPower   = -baseStrafePower + turnCorrection;
            double rightBackPower  = baseStrafePower - turnCorrection;

            // Limit the powers to ensure they don't exceed the max speed
            double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            maxPower = Math.max(maxPower, Math.abs(leftBackPower));
            maxPower = Math.max(maxPower, Math.abs(rightBackPower));

            if (maxPower > absSpeed) {
                leftFrontPower  *= absSpeed / maxPower;
                rightFrontPower *= absSpeed / maxPower;
                leftBackPower   *= absSpeed / maxPower;
                rightBackPower  *= absSpeed / maxPower;
            }

            // Set Power
            motorLeftFront.setPower(leftFrontPower);
            motorRightFront.setPower(rightFrontPower);
            motorLeftBack.setPower(leftBackPower);
            motorRightBack.setPower(rightBackPower);
            logData();
        }

        // 5. Stop and Reset
        stopAllMotors(); // Use a helper function for cleanup
    }

    /**
     * Helper function to normalize angles to the range [-180, 180].
     */
    private double normalizeAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }


    /**
     * Drives the robot a specified distance at a constant heading for ANY angle.
     * This function combines forward/backward, strafe, and diagonal movement.
     *                            * @param speed The base magnitude of the drive power.
     *      * @param angle The direction of movement in degrees (0 = forward, 90 = strafe right).
     *      * @param distanceInches The total distance in inches to travel.
     *      * @param desiredHeading The target IMU heading (e.g., 0.0 to drive straight).
     */
    public void driveVectorMecanum(double speed, double angle, double distanceInches, double desiredHeading) {
        if (!opModeIsActive()) return;

        double startTime = getRuntime();
        integralSum = 0.0; // Reset integral sum for a new movement command

        // Convert angle to radians for trig functions
        double angleRad = Math.toRadians(angle);

        // 1. Calculate Target Ticks for ALL MOTORS
        // Since we're driving a vector, we calculate the total net ticks to move,
        // and then use the directional power split inside the loop to ensure the vector is followed.
        int moveCounts = (int) (distanceInches * TICKS_PER_INCH);

        // For a vector move, a simplified approach is to set the Target Position
        // to a value that *all* motors will reach together when the distance is achieved.
        // We'll use the average of the two front motors for the target.
        int currentAvgPos = (motorLeftFront.getCurrentPosition() + motorRightFront.getCurrentPosition()) / 2;
        int targetPos = currentAvgPos + moveCounts;


        // Set the target position to be the same for all motors.
        // The power adjustments inside the loop will force the robot into the correct vector path.
        motorLeftFront.setTargetPosition(targetPos);
        motorRightFront.setTargetPosition(targetPos);
        motorLeftBack.setTargetPosition(targetPos);
        motorRightBack.setTargetPosition(targetPos);

        // 2. Set Run Mode
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 3. Loop until all motors reach position
        while (opModeIsActive() &&
                (motorLeftFront.isBusy() || motorRightFront.isBusy() ||
                        motorLeftBack.isBusy() || motorRightBack.isBusy())) {

            // --- TIME & DELTA TIME CALCULATION (ESSENTIAL for I-Control) ---
            // Calculate the time step since the last loop iteration
            double currentTime = getRuntime();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime; // Update lastTime for the next iteration

            // --- Calculate Power Vector ---
            double drivePower  = Math.cos(angleRad); // Forward/Backward component of the vector
            double strafePower = Math.sin(angleRad); // Strafe component of the vector

            // --- Calculate Heading Correction (Same as before) ---
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = normalizeAngle(desiredHeading - currentHeading);
            double proportionalCorrection = error * HEADING_GAIN; // P-term, proportional correction
            // Accumulate the error scaled by time (dt)
            integralSum += (error * deltaTime); // I-Term (Integral Correction)
            double integralCorrection = integralSum * INTEGRAL_GAIN; // Total I-Correction
            double turnCorrection = proportionalCorrection + integralCorrection; // Final Turn Correction (P + I)

            // --- Apply Power Mix ---
            double leftFrontPower  = drivePower + strafePower + turnCorrection;
            double rightFrontPower = drivePower - strafePower - turnCorrection;
            double leftBackPower   = drivePower - strafePower + turnCorrection;
            double rightBackPower  = drivePower + strafePower - turnCorrection;

            // --- Scale and Limit Power ---
            // Find the largest power magnitude (Max Power)
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            // Scale all powers down if they exceed the set speed
            if (max > speed) {
                double scale = speed / max;
                leftFrontPower *= scale;
                rightFrontPower *= scale;
                leftBackPower *= scale;
                rightBackPower *= scale;
            }

            // 4. Set Power
            motorLeftFront.setPower(leftFrontPower);
            motorRightFront.setPower(rightFrontPower);
            motorLeftBack.setPower(leftBackPower);
            motorRightBack.setPower(rightBackPower);
            logData();
        }

        // 5. Stop and Reset
        // Use the stopAndResetMotors() helper from the previous response
        stopAllMotors();
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    // Placeholder methods for drive functionality
    private void driveStraight(double power, int runTimeMs) {
        // Logic to drive straight using power and encoder or time
    }

    private void turnToHeading(double power, double desiredHeading) {
        if (!opModeIsActive()) return;

        // Reset PI state for the new turn
        integralSum = 0.0;
        lastTime = getRuntime();

        // 1. Ensure motors are in RUN_USING_ENCODER mode for direct power control
        // This is necessary because RUN_TO_POSITION mode would interfere with the PI turn control.
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Loop until the robot is close enough to the target heading
        while (opModeIsActive()) {

            // --- TIME & DELTA TIME CALCULATION ---
            double currentTime = getRuntime();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            // --- Calculate Heading Correction (PI Control - uses TURN_GAIN/TURN_INTEGRAL_GAIN) ---
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = normalizeAngle(desiredHeading - currentHeading);

            // Check for loop termination: If error is within tolerance, break
            if (Math.abs(error) <= HEADING_TOLERANCE) {
                break;
            }

            // P-Term (Proportional Correction)
            double proportionalCorrection = error * TURN_GAIN;

            // I-Term (Integral Correction)
            // Accumulate the error scaled by time (dt)
            integralSum += (error * deltaTime);
            // Anti-Windup: Limit the integral sum to prevent massive overshoot
            integralSum = Range.clip(integralSum, -MAX_INTEGRAL_SUM, MAX_INTEGRAL_SUM);

            // Total I-Correction
            double integralCorrection = integralSum * TURN_INTEGRAL_GAIN;

            // Final Turn Power (P + I)
            double turnCorrection = proportionalCorrection + integralCorrection;

            // Limit the turn correction power to the maximum allowed speed
            double turnPower = Range.clip(turnCorrection, -power, power);

            // --- Apply Power ---
            // For turning in place, the left motors are set to the turn power, and the right motors
            // are set to the negative of the turn power.
            motorLeftFront.setPower(turnPower);
            motorRightFront.setPower(-turnPower);
            motorLeftBack.setPower(turnPower);
            motorRightBack.setPower(-turnPower);

            logData(); // Log data inside the control loop
            telemetry.addData("PI-Turn: P/I/Sum", "%.4f / %.4f / %.4f", proportionalCorrection, integralCorrection, integralSum);
            telemetry.addData("PI-Turn: Target/Actual/Error", "%.1f / %.1f / %.1f", desiredHeading, currentHeading, error);
            telemetry.update();
        }
        stopAllMotors(); // Stop motors after loop exit
    }

    // =========================================================================================
    // Logging Method
    // =========================================================================================

    /**
     * Collects and writes a single line of data to the CSV file.
     */
    private void logData() {
        double time = getRuntime();
        double currentHeading = getHeading();

        // Write the data row. The order MUST match the HEADERS in the Datalog class.
        datalog.log(
                time,
                opModeStatus,
                currentHeading,
                motorLeftFront.getCurrentPosition(),
                motorRightFront.getCurrentPosition(),
                motorLeftBack.getCurrentPosition(),
                motorRightBack.getCurrentPosition(),
                targetHeading,
                batterySensor.getVoltage(),
                HEADING_GAIN,
                DRIVE_SPEED
        );

        // Update telemetry for on-screen monitoring
        telemetry.addData("LOG: Status/Time", "%s / %.2f", opModeStatus, time);
        telemetry.addData("LOG: Heading/Target", "%.1f / %.1f", currentHeading, targetHeading);
    }

    // =========================================================================================
    // DATALOG Class Definition (Wraps the Datalogger.java utility)
    // =========================================================================================
    /**
     * This wrapper class defines the fields for the log file and manages the Datalogger instance.
     */
    public static class Datalog implements AutoCloseable {
        private final Datalogger datalogger;

        // CRITICAL: Define your column headers in the exact order you will log the data.
        private static final String[] HEADERS = new String[]{
                "Time(s)",
                "OpModeStatus",
                "Heading",
                "LFPos", "RFPos", "LBPos", "RBPos",
                "TargetHeading",
                "BatteryVoltage",
                "Kp",
                "Speed"
        };

        /**
         * @param name filename for output log
         */
        public Datalog(String name)
        {
            // The Datalogger constructor handles file creation and header writing.
            this.datalogger = new Datalogger(name, HEADERS);
        }

        /**
         * Logs a single line of data.
         * The order of arguments MUST match the order of HEADERS.
         */
        public void log(
                double time,
                String opModeStatus,
                double currentHeading,
                int lfPos, int rfPos, int lbPos, int rbPos,
                double targetHeading,
                double voltage, double headingGain, double driveSpeed) {

            // Manually construct the string array for the Datalogger.log() method, applying formatting.
            datalogger.log(
                    String.format("%.3f", time),
                    opModeStatus,
                    String.format("%.1f", currentHeading),
                    String.valueOf(lfPos),
                    String.valueOf(rfPos),
                    String.valueOf(lbPos),
                    String.valueOf(rbPos),
                    String.format("%.1f", targetHeading),
                    String.format("%.2f", voltage),
                    String.format("%.3f", HEADING_GAIN),
                    String.format("%.2f", DRIVE_SPEED)
            );
        }
        /**
         * Closes the datalogger. This is the fix for empty files!
         */
        @Override
        public void close() {
            datalogger.close();
        }
    }
}