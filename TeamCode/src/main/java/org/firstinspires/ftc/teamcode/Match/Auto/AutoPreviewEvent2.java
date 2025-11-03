/*
 * Combined Autonomous OpMode for AprilTag Localization, Drive-to-Tag, and Goal Action.
 *
 * Combines AprilTag Localization setup with continuous P-Control driving logic.
 * Corrected: Ensures VisionPortal.Builder() is always initialized with a camera.
 * 33.4308289114498; // SWYFT Drive v2; goBILDA 5203 series, 12.7:1, 86 mm
 */

package org.firstinspires.ftc.teamcode.Match.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Preview Event2 Auto", group = "Match", preselectTeleOp="TeleOpPreviewEvent")
@Disabled
public class AutoPreviewEvent2 extends LinearOpMode
{
    // --- VISION & DRIVE CONSTANTS (Adjust these for your robot and game) ---
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 20;        // Target AprilTag ID for the GOAL
    private static final double DESIRED_DISTANCE = 12.0; // Desired distance from the robot's front to the target (inches)

    // *** IMPORTANT: You must measure these values and set them here. ***
    private final double CAMERA_OFFSET_X = -3.5;         // Horizontal offset (in) from robot center to camera (+ right, - left)
    private final double CAMERA_OFFSET_Y = 0.0;          // Forward offset (in) from robot center to camera (+ front, - rear)

    // P-Control Gains
    private final double SPEED_GAIN  =  0.02  ;   // Forward Speed Control "Gain"
    private final double STRAFE_GAIN =  0.015 ;   // Strafe Speed Control "Gain"
    private final double TURN_GAIN   =  0.01  ;   // Turn Control "Gain"

    private final double MAX_AUTO_SPEED = 0.5;   // Max approach speed
    private final double MAX_AUTO_STRAFE= 0.5;   // Max strafing speed
    private final double MAX_AUTO_TURN  = 0.3;   // Max turn speed

    // --- HARDWARE ---
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightBack = null;

    // --- VISION COMPONENTS ---
    private VisionPortal visionPortal = null; // Initialize to null for safety
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private AprilTagMovementController movementController;

    // Enumerations for Alliance and Position
    enum Alliance { RED, BLUE }
    enum Position { POS1, POS2, POS3 }
    // Variables for Alliance and Position
    private Alliance alliance = Alliance.RED;
    private Position position = Position.POS1;

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
    @Override
    public void runOpMode()
    {
        // --- 1. INITIALIZE HARDWARE ---
        try {
            motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
            motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
            motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
            motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

            // Assuming motor configuration for a standard Mecanum setup
            motorLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
            motorLeftBack.setDirection(DcMotorEx.Direction.REVERSE);
            motorRightFront.setDirection(DcMotorEx.Direction.FORWARD);
            motorRightBack.setDirection(DcMotorEx.Direction.FORWARD);
        } catch (Exception e) {
            telemetry.addData("FATAL ERROR", "Drive motor initialization failed: " + e.getMessage());
            telemetry.update();
            // Movement functions will check for null motors and be non-functional
            sleep(5000);
            return;
        }

        // --- 2. INITIALIZE VISION ---

        // --- Vision Initialization (Protected) ---
        initAprilTag();
        movementController = new AprilTagMovementController();

        // --- Driver Hub Pre-Match Selection ---
        telemetry.addData("Status", "Ready for Selection");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) {
                alliance = Alliance.BLUE;
            } else if (gamepad1.x) {
                alliance = Alliance.RED;
            }
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

        // Optional: Set manual exposure for better tag detection under competition lights (Webcams only)
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        telemetry.addData("Status", "Initialized - Target Tag ID: " + DESIRED_TAG_ID);
        telemetry.update();

        // --- 3. WAIT FOR START ---
        waitForStart();


        // --- AUTONOMOUS SEQUENCE ---
        if (opModeIsActive()) {

            // If vision failed to initialize, skip drive-to-goal and move straight to launch/strafe
            if (visionPortal == null) {
                telemetry.addData("WARNING", "Vision Portal failed. Skipping AprilTag drive.");
                telemetry.update();
            } else {
                // --- PHASE 2: DRIVE TO GOAL (AprilTag Alignment) ---
                telemetry.addData("Status", "PHASE 2: Driving to Goal Tag ID %d...", DESIRED_TAG_ID);
                telemetry.update();

                double drive, strafe, turn;
                boolean targetReached = false;

                while (opModeIsActive() && !targetReached)
                {
                    desiredTag = null; // Clear previous detection
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    // Search for the desired tag
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null && detection.id == DESIRED_TAG_ID) {
                            desiredTag = detection;
                            break;
                        }
                    }

                    if (desiredTag != null) {
                        telemetryAprilTag(desiredTag);

                        // Calculate robot center position relative to the tag based on camera offsets
                        // ftcPose.x is the left-right distance (Strafe)
                        // ftcPose.y is the forward-backward distance (Range/Drive)
                        double robotCenterX = desiredTag.ftcPose.x - CAMERA_OFFSET_X;
                        double robotCenterY = desiredTag.ftcPose.y - CAMERA_OFFSET_Y;

                        // Calculate the errors we need to correct.
                        double  rangeError = (robotCenterY - DESIRED_DISTANCE);
                        double  strafeError = robotCenterX;
                        double  headingError = desiredTag.ftcPose.bearing;

                        // Use the speed and turn "gains" to calculate the motor powers.
                        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        strafe = Range.clip(strafeError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                        // Apply powers
                        moveRobot(drive, strafe, turn);

                        telemetry.addData("Robot","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                        telemetry.addData("Range Error", "%5.2f", rangeError);

                        // Check if target has been reached
                        if (Math.abs(rangeError) < 1.0) { // Check for range tolerance
                            moveRobot(0, 0, 0);
                            targetReached = true;
                            telemetry.addData("Status", "Target Reached!");
                            telemetry.update();
                            sleep(200);
                        }

                    } else {
                        // Tag not found in sight, stop or initiate slow search pattern (currently stops)
                        drive = 0; strafe = 0; turn = 0;
                        moveRobot(drive, strafe, turn);
                        telemetry.addData("Status", "Searching for Tag ID %d...", DESIRED_TAG_ID);
                    }
                    telemetry.update();
                    sleep(10); // Control loop delay
                }
            }


            // --- PHASE 3: LAUNCH ARTIFACTS ---
            telemetry.addData("Status", "PHASE 3: Launching Artifacts...");
            telemetry.update();
            launch(1.0); // Placeholder for actual launch sequence
            sleep(1000); // Wait for launch action to finish

            // --- PHASE 4: STRAFE LEFT 12 INCHES ---
            telemetry.addData("Status", "PHASE 4: Strafing Left 12 inches...");
            telemetry.update();
            strafeLeft(12.0, 0.4); // Strafe 12 inches at 40% power
            sleep(500); // Small pause

            // --- PHASE 5: COMPLETE ---
            moveRobot(0, 0, 0);
            telemetry.addData("Status", "Autonomous Complete.");
            telemetry.update();
        }

        // --- FINAL CLEANUP ---
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // --------------------------------------------------------------------------
    //  HELPER FUNCTIONS (ADAPTED FROM SOURCE OPMODES)
    // --------------------------------------------------------------------------

    /**
     * Initializes the AprilTag processor and Vision Portal.
     * **FIXED:** Ensures a camera is set on the builder, handling webcam configuration errors gracefully.
     */
    private void initAprilTag() {
        // Build the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDecimation(2)
                .build();

        // Build the Vision Portal.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        boolean cameraSet = false;

        if (USE_WEBCAM) {
            try {
                // Attempt to get the Webcam name from the configuration
                WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
                builder.setCamera(webcam);
                cameraSet = true;
            } catch (Exception e) {
                // If the webcam is not found or configured incorrectly, log and fall back
                telemetry.addData("Vision Error", "Webcam 'Webcam 1' not found/configured. Using built-in camera.");
            }
        }

        // If webcam failed or USE_WEBCAM is false, default to the built-in camera
        if (!cameraSet) {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        try {
            builder.addProcessor(aprilTag);
            visionPortal = builder.build();
        } catch (Exception e) {
            telemetry.addData("Vision Fatal", "Vision Portal could not be built: " + e.getMessage());
            visionPortal = null; // Set to null to skip vision logic later
        }
    }


    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise (turn left)
     */
    public void moveRobot(double x, double y, double yaw) {
        // Only proceed if drive motors are initialized
        if (motorLeftFront == null || motorRightFront == null || motorLeftBack == null || motorRightBack == null) return;

        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        motorLeftFront.setPower(frontLeftPower);
        motorRightFront.setPower(frontRightPower);
        motorLeftBack.setPower(backLeftPower);
        motorRightBack.setPower(backRightPower);
    }


    /**
     * Manually set the camera gain and exposure.
     * This can only be called AFTER calling initAprilTag(), and only works for Webcams;
     */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    /**
     * Placeholder function to launch artifacts.
     */
    private void launch(double power) {
        // Implement your actual launcher motor control here
        telemetry.addData("ACTION", "LAUNCHING ARTIFACTS @ %.2f power", power);
        telemetry.update();
        // Placeholder delay
        sleep(500);
    }

    /**
     * Strafe the robot to the left by a specified distance.
     * NOTE: This is a **simplified, time-based movement** and will **not** be accurate.
     */
    private void strafeLeft(double distanceInches, double power) {
        // Estimated Time Constant: (Time/Inch) / (Power/MaxPower)
        final double TIME_PER_INCH_AT_MAX_POWER = 0.0833;
        double durationSeconds = distanceInches * TIME_PER_INCH_AT_MAX_POWER / (power / MAX_AUTO_STRAFE);
        long durationMs = (long) (durationSeconds * 1000);

        telemetry.addData("ACTION", "Strafing Left for %.2f seconds", durationSeconds);
        telemetry.update();

        // Move robot: X=0 (Axial), Y=+power (Strafe Left), Yaw=0 (Turn)
        moveRobot(0, power, 0);
        sleep(durationMs);
        moveRobot(0, 0, 0); // Stop
    }

    /**
     * Display key telemetry for the detected AprilTag.
     */
    private void telemetryAprilTag(AprilTagDetection detection) {
        if (detection.metadata != null) {
            telemetry.addLine(String.format(Locale.US,"\n==== (ID %d) %s", detection.id, detection.metadata.name));

            // Show robot position relative to field origin (X: Right, Y: Forward, Z: Up)
            if (detection.robotPose != null) {
                telemetry.addLine(String.format(Locale.US,"Field XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format(Locale.US,"Field PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            }
            // Show camera position relative to the Tag
            telemetry.addLine(String.format(Locale.US,"Camera Range %6.1f, Bearing %6.1f, Yaw %6.1f",
                    detection.ftcPose.range,
                    detection.ftcPose.bearing,
                    detection.ftcPose.yaw));
        } else {
            telemetry.addLine(String.format(Locale.US,"\n==== (ID %d) Unknown", detection.id));
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }

    // --- AprilTag Alignment Controller Class (for close-range alignment) ---
    private class AprilTagMovementController {
        public double getRobotCenterY(AprilTagDetection detection) {
            return detection.ftcPose.y - CAMERA_OFFSET_Y;
        }

        public double[] calculatePowers(AprilTagDetection detection) {
            double robotCenterX = detection.ftcPose.x - CAMERA_OFFSET_X;
            double robotCenterY = getRobotCenterY(detection);

            double rangeError = (robotCenterY - DESIRED_DISTANCE);
            double strafeError = robotCenterX;
            double headingError = detection.ftcPose.bearing;

            double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double strafe = Range.clip(strafeError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            return new double[]{drive, strafe, turn};
        }
    }
}