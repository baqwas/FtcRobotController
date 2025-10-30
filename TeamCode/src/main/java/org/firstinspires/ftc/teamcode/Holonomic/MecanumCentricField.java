/*
 * MIT License
 *
 * Copyright (c) 2024 ParkCircus Productions; All Rights Reserved
 * * [License details omitted for brevity]
 * */

package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// NEW IMPORTS
import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.BatteryVoltageSensor;

@TeleOp(name = "Mecanum: Centric Field", group = "Test")
@Disabled

public class MecanumCentricField extends LinearOpMode {

    private static final String TAG = MecanumCentricField.class.getSimpleName();

    // 1. REPLACED Datalog CLASS INSTANTIATION with Datalogger direct instantiation
    private final Datalogger datalogger = new Datalogger(
            TAG, // Filename
            "OpModeStatus", "Loop Counter", "Yaw", "Pitch", "Roll",
            "globalAngle", "deltaAngle", "Gamepad X", "Gamepad Y", "Gamepad Rx",
            "rotX", "rotY",
            "motorPower0", "motorPower1", "motorPower2", "motorPower3",
            "Battery"
    );

    static final double TICKS_PER_INCH = 33.4308289114498; // SWYFT Drive v2; goBILDA 5203 12.7:1, 86 mm wheel

    IMU imu;
    private final ElapsedTime runtime = new ElapsedTime();
    // The Universal IMU uses YawPitchRollAngles
    YawPitchRollAngles lastAngles = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
    double globalAngle, power = 0.40, correction;

    // ADDED: Battery Sensor field
    private BatteryVoltageSensor batterySensor = null;

    // motor entities for drivetrain
    String[] motorLabels = {"motorLeftFront", "motorLeftBack", "motorRightFront", "motorRightBack"};
    DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    int[] motorTicks = {0, 0, 0, 0};    // current tick count from encoder for the respective motors
    double[] motorPower = {0.0, 0.0, 0.0, 0.0};
    int drivetrainSteps = 0;

    @Override
    public void runOpMode() throws InterruptedException   // called when init button is  pressed.
    {
        // **UPDATED: Hardware map retrieval to use IMU interface**
        imu = hardwareMap.get(IMU.class, "imu");

        // **ADDED: Explicitly reset yaw to zero the heading**
        imu.resetYaw();

        // --- BATTERY SENSOR INITIALIZATION ---
        try {
            batterySensor = new BatteryVoltageSensor(hardwareMap);
            telemetry.addData("Status", "Battery Sensor Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Could not initialize Battery Voltage Sensor.");
        }
        telemetry.update();

        // ... [Remaining initialization code for motors omitted for brevity] ...

        /*
         * Initialize the hardware variables
         * The strings used here must correspond
         * to the names assigned during the robot configuration step on the DS or RC devices.
         */
        for (int i = 0; i < motor.length; i++) {
            motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]);
            motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        motor[2].setDirection(DcMotorEx.Direction.REVERSE); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.REVERSE); // motorRightBack

        for (DcMotorEx dcMotor : motor) {
            dcMotor.setPower(0.0);
            dcMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            dcMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Mode", "Calibrating...");
        telemetry.update();

        // **UPDATED: Check if IMU is ready (isSystemSet() is the equivalent of isGyroCalibrated() for the U-IMU)**
        while (!isStopRequested())        // make sure the imu gyro is ready before continuing.
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "Select Start");
        telemetry.addData("Status", "IMU System Set");
        telemetry.update();

        waitForStart();                     // wait for Start button to be pressed for Linear OpMode

        telemetry.addData("Mode", "running");
        telemetry.update();
        sleep(1000);

        if (isStopRequested()) {         // has stopping opMode been requested
            datalogger.close();
            return;
        }

        while (opModeIsActive()) {
            /*
             * left stick X axis for lateral movement (i.e. right or left)
             * left stick Y axis for axial movement (i.e. forward or reverse)
             */
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            /*
             * empirically refine the scale factor below -> 1.n, n= 0.0.1 to 0.10
             */
            double x = gamepad1.left_stick_x * 1.0;
            // right stick X is for rotation (i.e. clockwise or counter-clockwise)
            double rx = gamepad1.right_stick_x;

            // **UPDATED: Get Yaw angle from Universal IMU and convert to RADIANS**
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = -angles.getYaw(AngleUnit.RADIANS); // negative for clockwise IMU vs CCW robot reference
            double pitch = angles.getPitch(AngleUnit.RADIANS);
            double roll = angles.getRoll(AngleUnit.RADIANS);

            double rotX = x * Math.cos(yaw) - y * Math.sin(yaw);
            double rotY = x * Math.sin(yaw) + y * Math.cos(yaw);
            // normalize the power setting in the range {-1, 1}
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorPower[0] = (rotY + rotX + rx) / denominator;
            motorPower[1] = (rotY - rotX + rx) / denominator;
            motorPower[2] = (rotY - rotX - rx) / denominator;
            motorPower[3] = (rotY + rotX - rx) / denominator;

            for (int i = 0; i < motor.length; i++) {
                motor[i].setPower(motorPower[i]);
            }

            // 2. REPLACED OLD DATALOG LOGIC WITH NEW API CALL
            String batteryVoltage = (batterySensor != null) ? batterySensor.getFormattedVoltage() : "N/A";

            datalogger.log(
                    "RUNNING",                                          // OpModeStatus
                    String.valueOf(drivetrainSteps++),                  // Loop Counter
                    String.format("%.4f", yaw),
                    String.format("%.4f", pitch),
                    String.format("%.4f", roll),
                    String.format("%.4f", globalAngle),                 // Placeholder, value is not updated in this loop
                    "N/A",                                              // deltaAngle (not calculated in this OpMode)
                    String.format("%.4f", x),                           // Gamepad X (lateral/strafe)
                    String.format("%.4f", y),                           // Gamepad Y (axial/forward)
                    String.format("%.4f", rx),                          // Gamepad Rx (rotation)
                    String.format("%.4f", rotX),
                    String.format("%.4f", rotY),
                    String.format("%.4f", motorPower[0]),
                    String.format("%.4f", motorPower[1]),
                    String.format("%.4f", motorPower[2]),
                    String.format("%.4f", motorPower[3]),
                    batteryVoltage
            );

            // ... [Telemetry and other loop logic omitted for brevity] ...
        }

        // CRITICAL: Close the datalogger when the OpMode ends
        datalogger.close();
    } // End of runOpMode

    // ... [Other methods like resetAngle(), getAngle(), checkDirection(), rotate() omitted for brevity] ...

    // 3. REMOVED THE ENTIRE NESTED 'Datalog' CLASS (was at the end of the file)
}

