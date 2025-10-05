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

package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 *  This particular OpMode illustrates driving a holonomic robot
 *  Notes:
 *  - A Mecanum drive must display an X roller-pattern in plan view (top/above)
 *  - Set the correct rotation direction for each motor
 *
 *  Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously:
 *  1 - axial
 *  2 - lateral
 *  3 - yaw
 *
 *  Each motion is controlled by one gamepad joystick axis
 *  Movement     Expected Response                        Operation
 *  1 - axial:   driving forward and backward             left joystick forward and backward
 *  2 - lateral: strafing right and left                  left joystick right and left
 *  3 - yaw:     rotating clockwise and counter clockwise right joystick right and left
 *
 *  Sanity check:
 *  When the left stick is pushed forward and the motor moves backward then the direction of the motor should be reversed
 *
 *  Robot Z points upwards to the ceiling.
 *  Robot Y points forward – whatever you decide is “forward” on your robot (which could be round!).
 *  Robot X points to the right side of the robot. Robot rotations follow the right-hand rule.
 *
 * For all axes, IMU angles are provided in the range of -180 to +180 degrees (or from -Pi to +Pi radians).
 * If you are working with values that might cross the +/- 180-degree transition,
 * handle this with your programming. That’s beyond the scope of this IMU tutorial.
 *
 * @link https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/bosch/BNO055IMU.Parameters.html
 * @link https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
 * @link https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html#sample-opmodes
 */

@TeleOp(name = "Strafe: Driver Controlled test", group = "Test")
@Disabled

public class TestDriveStrafe extends LinearOpMode {

    // Declare OpMode members for each of the 4 motor
    private static final  String TAG = TestDriveStrafe.class.getSimpleName();
    private final ElapsedTime runtime = new ElapsedTime();
    Datalog datalog = new Datalog(TAG);
    long logCounter = 0;                  // to index the records written to logger output file
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    boolean orientationIsValid = true;
    YawPitchRollAngles robotOrientation;  // Create an object to receive the IMU angles
    YawPitchRollAngles lastAngles;        // robot orientation at previous iteration step (or time check)

    VoltageSensor battery;
    // variables for motors
    String[] motorLabels = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    private final DcMotorEx[] motor = {null, null, null, null}; // reservations only, please
    private final double[] motorPower = {0.0, 0.0, 0.0, 0.0};
    int[] motorTicks = {0, 0, 0, 0};    // current tick count from encoder for the respective motors
    double TICKS_PER_INCH = 45.283963; // goBILDA 5203 19.2
    // variables for IMU
    Position position = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0); // instantaneous position (once integration is started in IMU)
    Velocity velocity = new Velocity(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0); // instantaneous velocity (once integration is started in IMU)
    Acceleration accel = new Acceleration(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0); // 1/s^2


    int pollInterval = 100;               // polling interval, ms, between successive calls to getLinearAcceleration

    double globalAngle, power = 0.60, correction, rotation, pitchInitial, rollInitial;
    boolean buttonA, buttonB, touched;
    PIDController pidDrive, pidRotate;

    // apply any requested orientation changes.
    void updateOrientation() {
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(                     // initialize IMU directly
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.UP,    // Moonpatrol orientation
                                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD // Moonpatrol orientation
                            )
                    )
            );
            // imu.initialize(myIMUparameters); // initialize IMU using parameters - future use
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
            telemetry.addData("IMU", "invalid data: logo: %d, usb: %d", logoDirection, usbDirection);
            telemetry.update();
        }
    }

    /**
     * perform lateral holonomic movement for a specified distance
     *
     * rotational changes to keep heading constant are:
     * clockwise:
     * +ve left front (0) and left back (1)
     * -ve right front (2) and right back (3)
     * anti-clockwise:
     * -ve left front (0) and left back (1)
     * +ve right front (2) and right back (3)
     *
     * @param right true for movement to right, false for movement to left
     * @param distance the specified distance for holonomic travel
     * @param startPower the initial power to the motors for the movement
     *
     * Invoked by
     *  runOpMode
     */
    public void DriveStrafe(boolean right, double distance, double startPower)
    {
        /*
         * Kp
         * 0.1 is BAD, explore lower values by two orders of magnitude
         * 0.001 has slight improvement, go with lower values
         * 0.0005 need to assess on proper surface
         * 0.00025 ned to go lower
         * 0.00015 need to higher
         * 0.0001 opposite end of spectrum, need to go higher
         */
        telemetry.addData("Strafing", "%.2f", distance);
        telemetry.update();
        double yawKp = 0.01;    // Proportional coefficient as in PID algorithm; will tweak with tests
        double[] motorPower = new double[]{startPower, startPower, startPower, startPower};
        double strafeCoeficient = 1.0; // empirical number that may need tweaking for greater accuracy
        boolean travelCompleted = false;// = true when length has been traveled by robot
        double pitch = 0;               // for pure strafe movement there is no pitch
        double lateral = startPower;
        double lateralError;
        long travelTicks = (long) (TICKS_PER_INCH * distance);
        double ticksError;
        double ticksKp = 0.00005;      // to correct the horizontal drift
        double yaw = 0, yawError;

        for (DcMotorEx dcMotor : motor) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // stops and then brakes, actively resisting any external force which attempts to turn the motor
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // apply a particular power level to the motor run at any velocity with specified power level
        }
        while (opModeIsActive() && !travelCompleted) {
            for (int i = 0; i < motor.length; i++)              // for each motor
            {
                motorTicks[i] = motor[i].getCurrentPosition();  // have we travelled far enough?
            }
            ticksError = ((double) (travelTicks - ((motorTicks[0] + motorTicks[3]) - (motorTicks[1] + motorTicks[2])) / 2)) * ticksKp;
            //lateral = lateral * ticksError / travelTicks;
            //lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //yawError = (globalAngle - lastAngles.firstAngle) * yawKp;
            robotOrientation = imu.getRobotYawPitchRollAngles();
            yawError = (globalAngle - robotOrientation.getYaw(AngleUnit.DEGREES)) * yawKp;
            correction = ticksError;// + yawError;
            double denominator = Math.max(Math.abs(lateral) /*+ Math.abs(pitch)*/ + Math.abs(correction), 1);
            if (right) {
                motorPower[0] = lateral - yawError;//(-lateral + correction) / denominator;
                motorPower[1] = -lateral + yawError;//( lateral - correction) / denominator;
                motorPower[2] = -lateral + yawError;//( lateral + correction) / denominator;
                motorPower[3] = lateral - yawError;//(-lateral + correction) / denominator;
            } else {
                motorPower[0] = -lateral + yawError;//( lateral - correction) / denominator;
                motorPower[1] = lateral - yawError;//(-lateral + correction) / denominator;
                motorPower[2] = lateral - yawError;//(-lateral - correction) / denominator;
                motorPower[3] = -lateral + yawError;//( lateral - correction) / denominator;
            }

            if (Math.abs(motorTicks[0]) < Math.abs(travelTicks))                    // are we there yet?
            {
                /*
                 * steering power applied along X axis principles
                 * motors 0,3 pair
                 * motors 1,2 pair
                 */
                for (int i = 0; i < motor.length; i++)              // for each motor
                {
                    motor[i].setPower(motorPower[i]);
                }

                /*
                 * Note that the order in which we set datalog fields
                 * does *not* matter! The order is configured inside
                 * the Datalog class constructor.
                 */

                //datalog.battery.set(battery.getVoltage());

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                datalog.yaw.set(orientation.getYaw(AngleUnit.DEGREES));
                datalog.pitch.set(orientation.getPitch(AngleUnit.DEGREES));
                datalog.roll.set(orientation.getRoll(AngleUnit.DEGREES));
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                //position = imu.getPosition();
                datalog.acquisitionTime.set(position.acquisitionTime); // time unit is always seconds
                //datalog.positionx.set(position.x);
                //datalog.positiony.set(position.y);
                //datalog.positionz.set(position.z);

                datalog.velocityZ.set(angularVelocity.zRotationRate);
                datalog.velocityY.set(angularVelocity.yRotationRate);
                datalog.velocityX.set(angularVelocity.xRotationRate);
                //accel = imu.getAcceleration();  // derivative of Velocity over time
                datalog.accelx.set(accel.xAccel);
                datalog.accely.set(accel.yAccel);
                datalog.accelz.set(accel.zAccel);
                datalog.ticks0.set(motor[0].getCurrentPosition());
                datalog.ticks1.set(motor[1].getCurrentPosition());
                datalog.ticks2.set(motor[2].getCurrentPosition());
                datalog.ticks3.set(motor[3].getCurrentPosition());
                datalog.writeLine();// The logged timestamp is taken when writeLine() is called

                telemetry.addData("Status", "Run Time: " + runtime); // Show the elapsed game time and wheel power
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", motor[0].getPower(), motor[2].getPower());
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", motor[1].getPower(), motor[3].getPower());
                telemetry.update();
                sleep(10L);
            }
            else
            {
                travelCompleted = true; // Alhamdolillah! Travelled the specified linear length.
            }
        }
        DriveStop();                     // all done!
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        double denom, axial, lateral, heading, yaw, rotX, rotY;
        telemetry.addData("IMU", "settings...");
        telemetry.update();
        // Set up IMU parameters
        // The integration algorithm here reports accelerations to the logcat log
        // It does not provide positional information.


        int slidePosition = 0, nextPosition = 0;
        telemetry.addData("IMU", "Initializing...");
        telemetry.update();
        imu = hardwareMap.get(IMU.class, "imu");
        updateOrientation();
        robotOrientation = imu.getRobotYawPitchRollAngles();
        lastAngles = robotOrientation;

        while (!isStopRequested() && !orientationIsValid)        // make sure the imu gyro is calibrated before continuing.
        {
            sleep(50);                  // calibration time does not exceed ~2 secs in practice
            idle();
        }
        telemetry.addData("Orientation status", orientationIsValid);
        telemetry.addData("Motors", "setting ...");
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
            motor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            // run at any velocity with specified power level
            motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        /*
         * https://docs.google.com/document/d/1E524I9tAHLgT_GOuBei2lbqhpzPuhkCtpAu6DB4v03M/
         *  Gamepad 1   Stick   Movement
         *              Left    Y         axial - forward or reverse
         *              Left    X         lateral - right or left
         *              Right   X         yaw - rotate clockwise or counter-clockwise
         */
        motor[0].setDirection(DcMotorEx.Direction.FORWARD); // redundant but retained for instructional purposes
        motor[1].setDirection(DcMotorEx.Direction.FORWARD); // redundant but retained for instructional purposes
        motor[2].setDirection(DcMotorEx.Direction.REVERSE);
        motor[3].setDirection(DcMotorEx.Direction.REVERSE);

        /*
         * Moonpatrol use
         */

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(0.003, 0.00003, 0);
        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(0.05, 0, 0);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Status: initialized!");
        telemetry.addLine("Start: Press PLAY");
        telemetry.update();

        waitForStart();
        // a thread that polls linear acceleration and integrates for velocity and position
        //imu.startAccelerationIntegration(position, velocity, pollInterval);
        // parameters for tank travel
        pidDrive.setSetpoint(0);            // ???
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        runtime.reset();
        DriveStop();
        // run until the end of the match (driver presses STOP)
        runtime.reset();
        while (opModeIsActive()) {
            try {
                // Store the gamepad values from the previous loop iteration in
                // previousGamepad1/2 to be used in this loop iteration.
                // This is equivalent to doing this at the end of the previous
                // loop iteration, as it will run in the same order except for
                // the first/last iteration of the loop.
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                // Store the gamepad values from this loop iteration in
                // currentGamepad1/2 to be used for the entirety of this loop iteration.
                // This prevents the gamepad values from changing between being
                // used and stored in previousGamepad1/2.
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (Exception e) {
            }

            // gamepad1 operations
            if (currentGamepad1.b && !previousGamepad1.b) {           // strafe right
                DriveStrafe(true, 24.0, 0.6);
                telemetry.addLine("Right 24\"");
                //} else if (gamepad1.x) {           // rotate CCW 45°
            } else if (currentGamepad1.x && !previousGamepad1.x) {           // strafe left
                DriveStrafe(false, 24.0, 0.6);
                telemetry.addLine("Left 24\"");
            } else {
                sleep(10L);
            }

        }
    }


    /**
     * Function to stop power to all defined motors in the arraylist motor[]
     * and to wait for 1 second.
     * The stopping action is controlled by the setting chosen for the motor
     * based on vendor specification by the program during the initialization.
     */
    private void DriveStop() {

        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);
        }

    }

    /**
     * Resets the cumulative angle tracking to zero and initializes the lastAngles entity
     * AxesReference - the axes reference in which the result will be expressed
     * EXTRINSIC | INTRINSIC
     * AxesOrder - the axes order in which the result will be expressed
     * AngleUnit - the angle units in which the result will be expressed
     * DEGREES | RADIANS
     */
    private void ResetAngle() {
        // the absolute orientation of the sensor as a set three angles
        lastAngles = imu.getRobotYawPitchRollAngles();
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double GetAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        // the absolute orientation of the sensor as a set three angles
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double deltaAngle = robotOrientation.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = robotOrientation;
        return globalAngle;
    }

    private void RotateIMU(int degrees, double power)
    {
        double  leftPower, rightPower;

        ResetAngle();                   // reset heading for tracking with IMU data

        if (degrees < 0)
        {                               // turn right
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {                               // turn left
            leftPower = -power;
            rightPower = power;
        }
        else return;

        motor[0].setPower(leftPower);   // set power to rotate
        motor[1].setPower(leftPower);
        motor[2].setPower(rightPower);
        motor[3].setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {                               // On right turn we have to get off zero first.
            // noinspection StatementWithEmptyBody
            while (opModeIsActive() && GetAngle() == 0) {}

            //noinspection StatementWithEmptyBody
            while (opModeIsActive() && GetAngle() > degrees) {}
        }
        else {                          // left turn
            // noinspection StatementWithEmptyBody
            while (opModeIsActive() && GetAngle() < degrees) {
            }
        }
        DriveStop();

        ResetAngle();                   // reset angle tracking on new heading
    }

    /**
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus   = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField acquisitionTime = new Datalogger.GenericField("Time");
        public Datalogger.GenericField yaw            = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField pitch          = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField roll           = new Datalogger.GenericField("Roll");
        public Datalogger.GenericField globalAngle    = new Datalogger.GenericField("Global Angle");
        public Datalogger.GenericField deltaAngle     = new Datalogger.GenericField("Delta Angle");
        public Datalogger.GenericField correction     = new Datalogger.GenericField("Correction");
        public Datalogger.GenericField positionX      = new Datalogger.GenericField("X");
        public Datalogger.GenericField positionY      = new Datalogger.GenericField("Y");
        public Datalogger.GenericField positionZ      = new Datalogger.GenericField("Z");
        public Datalogger.GenericField velocityX      = new Datalogger.GenericField("vx");
        public Datalogger.GenericField velocityY      = new Datalogger.GenericField("vy");
        public Datalogger.GenericField velocityZ      = new Datalogger.GenericField("vz");
        public Datalogger.GenericField accelx         = new Datalogger.GenericField("vx");
        public Datalogger.GenericField accely         = new Datalogger.GenericField("vy");
        public Datalogger.GenericField accelz         = new Datalogger.GenericField("vz");
        public Datalogger.GenericField ticks0         = new Datalogger.GenericField("tickslf");
        public Datalogger.GenericField ticks1         = new Datalogger.GenericField("tickslb");
        public Datalogger.GenericField ticks2         = new Datalogger.GenericField("ticksrf");
        public Datalogger.GenericField ticks3         = new Datalogger.GenericField("ticksrb");
        public Datalogger.GenericField powerLeftFront = new Datalogger.GenericField("PowerLeftFront");
        public Datalogger.GenericField powerLeftBack  = new Datalogger.GenericField("PowerLeftBack");
        public Datalogger.GenericField powerRightFront= new Datalogger.GenericField("PowerRightFront");
        public Datalogger.GenericField powerRightBack = new Datalogger.GenericField("PowerRightBack");
        public Datalogger.GenericField motor0         = new Datalogger.GenericField("Motor0");
        public Datalogger.GenericField motor1         = new Datalogger.GenericField("Motor1");
        public Datalogger.GenericField motor2         = new Datalogger.GenericField("Motor2");
        public Datalogger.GenericField motor3         = new Datalogger.GenericField("Motor3");
        //public Datalogger.GenericField battery        = new Datalogger.GenericField("Battery");

        public Datalog(String name)
        {
            datalogger = new Datalogger.Builder()// Build the underlying datalog object
                    .setFilename(name)// Pass through the filename
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)// Request an automatic timestamp field
                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            acquisitionTime,
                            yaw,
                            pitch,
                            roll,
                            globalAngle,
                            deltaAngle,
                            correction,
                            positionX,
                            positionY,
                            positionZ,
                            velocityX,
                            velocityY,
                            velocityZ,
                            accelx,
                            accely,
                            accelz,
                            ticks0,
                            ticks1,
                            ticks2,
                            ticks3,
                            powerLeftFront,
                            powerLeftBack,
                            powerRightFront,
                            powerRightBack,
                            motor0,
                            motor1,
                            motor2,
                            motor3
                            //battery
                    )
                    .build();
        }
        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}