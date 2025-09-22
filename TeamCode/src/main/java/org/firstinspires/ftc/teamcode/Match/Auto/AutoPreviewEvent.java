package org.firstinspires.ftc.teamcode.Match.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous(name="Preview Event", group="Match", preselectTeleOp="PreviewEventTeleOp")
// @Disabled
public class AutoPreviewEvent extends LinearOpMode {

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
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE
    }

    // Hardware for Mecanum Drivetrain
    private DcMotorEx motorLeftFront = null;
    private DcMotorEx motorLeftBack = null;
    private DcMotorEx motorRightFront = null;
    private DcMotorEx motorRightBack = null;

    // Variables for Alliance and Position
    private Alliance alliance = Alliance.RED; // Default to Red Alliance
    private Position position = Position.POS1; // Default to Position 1

    // Initializing the FSM state
    private RobotState currentState = RobotState.SCAN_OBELISK;

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

        // --- Driver Hub Pre-Match Selection ---

        telemetry.addData("Status", "Ready for Selection");
        telemetry.update();

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

            telemetry.addData("Alliance", "Press B for Blue, X for Red");
            telemetry.addData("Position", "Press D-pad Up/Right/Down for POS1/POS2/POS3");
            telemetry.addData("Current Selection", "Alliance: %s, Position: %s", alliance.toString(), position.toString());
            telemetry.update();
        }

        waitForStart();

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

        telemetry.addData("Executing Path", "Starting from (%.1f, %.1f) with heading %.1f", startPoint.x, startPoint.y, startPoint.heading);
        telemetry.update();

        // Main FSM loop. The opModeIsActive() check allows the driver to stop the OpMode at any time.
        while (opModeIsActive() && currentState != RobotState.COMPLETE) {

            telemetry.addData("Current State", currentState.toString());
            telemetry.update();

            switch (currentState) {
                case SCAN_OBELISK:
                    // Placeholder: Code to move to a position to scan the obelisk
                    // and use computer vision to determine its location.
                    sleep(2000); // Wait for vision processing

                    // After scanning, transition to the next state
                    currentState = RobotState.SCAN_GOAL;
                    break;

                case SCAN_GOAL:
                    // Placeholder: Code to move to the determined goal position
                    // and verify its location before launch.
                    sleep(1500); // Wait for position verification

                    // After scanning the goal, transition to launch
                    currentState = RobotState.LAUNCH;
                    break;

                case LAUNCH:
                    launch(1.0); // launch power = 1.0

                    // After the launch action is initiated, wait for it to finish
                    currentState = RobotState.WAIT_FOR_LAUNCH;
                    break;

                case WAIT_FOR_LAUNCH:
                    /*
                    if(launch(1.0)) { // launch power = 1.0
                        shotsToFire -= 1;
                        if(shotsToFire > 0) {
                            currentState = RobotState.LAUNCH;
                        } else {
                            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            launcher.setVelocity(0);
                            currentState = RobotState.DRIVING_AWAY_FROM_GOAL;
                        }
                    }
                     */
                    break;

                case DRIVING_AWAY_FROM_GOAL:
                    /*
                    if(drive(DRIVE_SPEED, -4, DistanceUnit.INCH, 1)){
                        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        currentState = RobotState.ROTATING;
                    }
                     */
                    break;

                case ROTATING:
                    // Placeholder: Rotate the robot to a specific heading for the next task.
                    // Use a gyro or IMU for a precise turn.
                    driveMecanum(0, 0, 0.5, 90, 2.0); // Example of a turn

                    // After rotating, drive off the line
                    currentState = RobotState.DRIVING_OFF_LINE;
                    break;

                case DRIVING_OFF_LINE:
                    // Placeholder: Drive the robot forward a specific distance to get off the line
                    // and into a safe position.
                    driveMecanum(0.5, 0, 0, 6, 2.0);

                    // The autonomous routine is now complete
                    currentState = RobotState.COMPLETE;
                    break;

                case COMPLETE:
                    // This state has no action. The `while` loop condition will handle termination.
                    break;
            }
        }
    }

    // --- Drivetrain Helper Method ---
    // This is a basic Mecanum drive function. For precise movement, you should
    // enhance this with encoder logic to calculate target ticks based on distance.
    private void driveMecanum(double axial, double lateral, double yaw, double distance, double timeoutS) {
        if (!opModeIsActive()) return;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        motorLeftFront.setPower(leftFrontPower);
        motorRightFront.setPower(rightFrontPower);
        motorLeftBack.setPower(leftBackPower);
        motorRightBack.setPower(rightBackPower);

        sleep((long) (timeoutS * 1000));

        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
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

}

