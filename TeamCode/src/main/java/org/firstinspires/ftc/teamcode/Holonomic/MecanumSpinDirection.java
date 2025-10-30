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
 * @version 1.1
 * @param none
 * @return none
 * @exception none
 * @see https://stemrobotics.cs.pdx.edu/node/7266
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
 *//*
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
 * @version 1.1
 * @param none
 * @return none
 * @exception none
 * @see https://stemrobotics.cs.pdx.edu/node/7266
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

package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;

@TeleOp(name = "Mecanum: Spin Direction", group = "Test")
@Disabled

public class MecanumSpinDirection extends LinearOpMode {

    private static final  String TAG = MecanumSpinDirection.class.getSimpleName();
    // Use the Datalogger directly and define the headers
    private final Datalogger datalogger = new Datalogger(
            TAG, // Filename
            "Loop Counter", "Yaw", "Pitch", "Roll", "Global Angle", "Delta Angle", "Battery" // Headers
    );

    static final double TICKS_PER_INCH = 56.9887969189608; // REV Robotics motor and wheel specific!

    // IMU and Angle tracking variables
    private IMU imu = null;
    YawPitchRollAngles lastAngles = null; // Initialized to null for safety check
    double globalAngle, power = .40, correction;
    boolean aButton, bButton, touched;
    private final ElapsedTime runtime = new ElapsedTime();
    private BatteryVoltageSensor batterySensor = null; // Initialize in runOpMode

    // motor entities for drivetrain
    // Array declaration moved here, initialization moved to runOpMode for safety
    private DcMotorEx[] motor = new DcMotorEx[4];
    private static final String[] MOTOR_NAMES = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    private boolean motorsInitialized = true; // Flag to track initialization success

    int[] motorTicks = {0, 0, 0, 0};    // current tick count from encoder for the respective motors
    double[] motorPower = {0.0, 0.0, 0.0, 0.0};
    int drivetrainSteps = 0;
    double deltaAngle; // Declared here for easy access in logging

    /**
     * Function to travel a linear distance. This function will require further tuning.
     */
    private void LinearTravel(double travelLength) {
        if (!motorsInitialized) return; // Exit if drive hardware failed initialization

        boolean travelCompleted = false;// = true when length has been traveled by robot
        double travelTicks = TICKS_PER_INCH * travelLength;

        while (opModeIsActive() && !travelCompleted)            // drive until end of period.
        {
            correction = checkDirection();  // Use gyro to drive in a straight line.

            telemetry.addData("1 IMU heading", imu != null ? lastAngles.getYaw(AngleUnit.DEGREES) : "N/A");
            telemetry.addData("2 Global heading", globalAngle);
            telemetry.addData("3 Correction", correction);
            telemetry.update();

            // Null check for motor before getting position/setting power
            if (motor[0] != null) {
                motorTicks[0] = motor[0].getCurrentPosition();
            } else {
                travelCompleted = true; // Cannot track position if a motor is missing
            }

            if (motorTicks[0] > travelTicks) {
                travelCompleted = true;
            } else {
                // Apply power only if motor exists
                if (motor[0] != null) motor[0].setPower(power - correction);
                if (motor[1] != null) motor[1].setPower(power + correction);
                if (motor[2] != null) motor[2].setPower(power + correction);
                if (motor[3] != null) motor[3].setPower(power - correction);

                aButton = gamepad1.a;       // allow teleop to change direction by 90 degrees
                bButton = gamepad1.b;
                //touched = touch.isPressed(); // Assume touch sensor is not critical

                if (/*touched || */aButton || bButton) {
                    backup();               // for a safer turn in the subsequent operation

                    if (/*touched || */aButton) rotate(-90, power); // turn 90 degrees right

                    if (bButton) rotate(90, power);             // turn 90 degrees left
                }
            }
        }
        fullStop();                     // turn the motors off
    }

    /**
     * Function to backup the robot after a stop to ensure it does not
     * collide with any object.
     */
    private void backup() {
        if (!motorsInitialized) return;

        for (DcMotorEx dcMotor : motor) {
            if (dcMotor != null) {
                dcMotor.setPower(-0.4);
            }
        }
        sleep(500);
        fullStop();
    }

    /**
     * Function to stop power to all defined motors.
     */
    private void fullStop() {
        for (DcMotorEx dcMotor : motor) {
            if (dcMotor != null) {
                dcMotor.setPower(0.0);
            }
        }
        sleep(500);
    }

    private void statusMessage(String caption, int number) {
        // Only run if the specific motor exists
        if (number >= 0 && number < motor.length && motor[number] != null) {
            telemetry.addData(caption, "forward");
            telemetry.update();
            sleep(5000);
            motor[number].setPower(0.5);
            sleep(5000);
            motor[number].setPower(0.0);
            telemetry.addData(caption, "reverse");
            telemetry.update();
            motor[number].setPower(-0.5);
            sleep(5000);
            motor[number].setPower(0.0);
        } else {
            telemetry.addData(caption, "Motor not initialized or out of bounds.");
            telemetry.update();
            sleep(1000);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException   // called when init button is  pressed.
    {
        telemetry.addData("Debug", "1");
        telemetry.update();

        // --- SAFE MOTOR INITIALIZATION ---
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            try {
                motor[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
                motor[i].setPower(0.0); // setting a power level of zero will brake the motor
                motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                telemetry.addData("ERROR", "Motor %s not found. Check configuration!", MOTOR_NAMES[i]);
                motorsInitialized = false;
            }
        }

        // --- SAFE IMU INITIALIZATION ---
        try {
            // 1. Get the IMU from the hardware map
            imu = hardwareMap.get(IMU.class, "imu"); // "imu" is the default device name

            // 2. Define the Hub's orientation on the robot
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            // 3. Set the parameters and initialize the IMU
            IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
            imu.initialize(parameters);

            // 4. Reset the Yaw angle to 0 for a consistent start
            imu.resetYaw();
            telemetry.addData("Status", "IMU Initialized and Yaw Reset");
        } catch (Exception e) {
            telemetry.addData("ERROR", "IMU (name 'imu') not found. Check configuration!");
        }

        // --- BATTERY SENSOR INITIALIZATION ---
        try {
            batterySensor = new BatteryVoltageSensor(hardwareMap);
            telemetry.addData("Status", "Battery Sensor Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not initialize Battery Voltage Sensor.");
        }
        telemetry.update();

        // Critical check: stop if drive motors are missing
        if (!motorsInitialized) {
            telemetry.addData("FATAL ERROR", "Drive motors missing. OpMode cannot run.");
            telemetry.update();
            waitForStart();
            // Critical: Close the datalogger even on early exit
            datalogger.close();
            return; // Exit if critical hardware is missing
        }

        waitForStart();                     // wait for Start button to be pressed for Linear OpMode

        telemetry.addData("Motor", "individual spin");
        telemetry.update();
        sleep(1000);

        // Use statusMessage which contains null checks
        statusMessage("leftFront", 0);
        statusMessage("leftBack", 1);
        statusMessage("rightFront", 2);
        statusMessage("rightBack", 3);

        telemetry.addData("Motor", "spin test complete");
        telemetry.update();
        sleep(1000); // Reduce sleep time from 60000 to 1000 for practicality

        if (isStopRequested()) {         // has stopping opMode been requested
            datalogger.close(); // Critical: Close the datalogger on stop request
            return;
        }

        // --- MAIN TELEOP LOOP ---
        while (opModeIsActive()) {
            // X axis for pivot turning
            // Y axis for forward/backward movement
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Normalize the power setting in the range {-1, 1}
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorPower[0] = (y + x + rx) / denominator;
            motorPower[1] = (y - x + rx) / denominator;
            motorPower[2] = (y - x - rx) / denominator;
            motorPower[3] = (y + x - rx) / denominator;

            for (int i = 0; i < motor.length; i++) {
                if (motor[i] != null) { // Null check before setting power
                    motor[i].setPower(motorPower[i]);
                }
            }

            // --- IMU Telemetry Updates (Protected by Null Check) ---
            if (imu != null) {
                // Get the robot's orientation
                YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

                if (robotOrientation != null) {
                    // Extract the angles (most teams only care about Yaw/Heading)
                    double yawAngle = robotOrientation.getYaw(AngleUnit.DEGREES);
                    double pitchAngle = robotOrientation.getPitch(AngleUnit.DEGREES);
                    double rollAngle = robotOrientation.getRoll(AngleUnit.DEGREES);

                    // Report the angles to the Driver Station
                    telemetry.addData("Yaw (Heading)", "%.2f Deg", yawAngle);
                    telemetry.addData("Pitch", "%.2f Deg", pitchAngle);
                    telemetry.addData("Roll", "%.2f Deg", rollAngle);

                    // Datalogging (Using new Datalogger API)
                    String batteryVoltage = (batterySensor != null) ? batterySensor.getFormattedVoltage() : "N/A";
                    datalogger.log(
                            String.valueOf(drivetrainSteps++), // Loop Counter
                            String.format("%.4f", yawAngle),
                            String.format("%.4f", pitchAngle),
                            String.format("%.4f", rollAngle),
                            String.format("%.4f", globalAngle),
                            String.format("%.4f", deltaAngle),
                            batteryVoltage
                    );
                }
            } else {
                telemetry.addData("IMU Status", "IMU Not Available");
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        // CRITICAL: Close the datalogger when the OpMode ends
        datalogger.close();
    }

    /**
     * Resets the cumulative angle tracking to zero and initializes the lastAngles entity
     */
    private void resetAngle() {
        if (imu != null) {
            lastAngles = imu.getRobotYawPitchRollAngles();
        } else {
            // IMU is not initialized, lastAngles remains null.
            telemetry.addData("ERROR", "IMU not initialized. Cannot reset angle.");
        }
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // Return 0.0 if IMU or angle data is not available
        if (imu == null || lastAngles == null) return 0.0;

        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        // Safety check if the new angle reading is null
        if (angles == null) return globalAngle;

        deltaAngle = angles.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        // deltaAngle is now set and can be logged in the main loop

        return globalAngle;
    }

    /**
     * If the current cumulative heading angle is not zero then the robot is
     * not travel in a straight line. Use a simple proportional control to
     * correct the deviation.
     * <p>
     * The gain value is essentially the proportional control correction factor.
     * This value may require some tuning based on field experiments.
     * </p>
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        if (imu == null) return 0.0; // Cannot check direction without IMU

        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        if (imu == null || !motorsInitialized) return; // Cannot rotate without IMU or motors

        double leftPower, rightPower;

        resetAngle();                   // reset heading for tracking with IMU data


        if (degrees < 0) {              // turn right
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {       // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // Apply power only if motors exist
        if (motor[0] != null) motor[0].setPower(leftPower);
        if (motor[1] != null) motor[1].setPower(rightPower);
        if (motor[2] != null) motor[2].setPower(rightPower);
        if (motor[3] != null) motor[3].setPower(leftPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && getAngle() == 0) {
            }

            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else { // left turn.
            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && getAngle() < degrees) {
            }
        }

        fullStop();

        resetAngle();                          // reset angle tracking on new heading
    }
}