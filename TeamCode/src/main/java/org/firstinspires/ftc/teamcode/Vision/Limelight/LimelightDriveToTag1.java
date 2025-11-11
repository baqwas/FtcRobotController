/**
Copyright (c) 2025 ParkCircus Productions
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  Performs the movement of a robot with a mecanum drivetrain towards an AprilTag using Limelight data

  <p></p>This OpMode is evaluation code to explore practical options for use Autonomous and perhaps in TeleOp
  to drive the robot to a preferred field position for more efficient actions after completing the move.</p>

  <p>The sensor to detect one or more AprilTags in this OpMode is Limelight 3A.
  This sensor can be configured to detect only a limited number of AprilTags that may imperceptibly improve
  response time.
  </p>

  </p>This OpMode assumes that the Limelight camera is mounted on a stationary frame. If the camera has
  to be mounted on a moving object, such as launcher turret, then a different version of the OpMode would
  serve the end-user needs in a more efficient manner.</p>

  @author ParkCircus Productions
 * @version 1.0
 * @since 1.0
 * @see https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
 * @license MIT
 * <p>The basic processing scheme is:
 * <ul><li>Use 2D targeting by driving until ty=0</li>
 * <li>Camera is mounted at a fixed angle either below or above the height of the target center</li>
 * <li>Use telemetry to ensure that ty changes proportionally with robot movement to or from target</li>
 * <li>Set Limelight crosshair for optimal ty by manually iterating robot preposition for launch tests</li>
 * <li>Aim robot (left-right) and calibrate tx crosshair</li>
 * <li>Process robot movement in runOpMode towards tx and ty to zero</li>
 * <li>Launch ARTIFACT if optimal launch position is reached</li></ul>
 * </p>
 * @see https://javadoc.io/static/org.firstinspires.ftc/Hardware/10.2.0/com/qualcomm/hardware/limelightvision/Limelight3A.html
 * Pose3D: https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/org/firstinspires/ftc/robotcore/external/navigation/Pose3D.html
 *      Position
 *      YawPitchRollAngles
 */
package org.firstinspires.ftc.teamcode.Vision.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;
import org.firstinspires.ftc.teamcode.Utility.PIDController;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

@Autonomous(name = "Limelight Drive to AprilTag v1", group = "Autonomous")
//@Disabled
public class LimelightDriveToTag1 extends LinearOpMode {

    private final String TAG = this.getClass().getSimpleName();

    private Datalog datalog = null;    // --- Logging Declaration ---
    // === HARDWARE DECLARATION (Refactored Motor Names) ===
    private DcMotorEx motorLeftFront, motorRightFront, motorLeftBack, motorRightBack;
    private IMU imu;
    // === LIMELIGHT CONFIGURATION (CALIBRATION IS CRUCIAL!) ===
    // Limelight data // === LIMELIGHT SDK OBJECTS ===
    private Limelight3A limelight; // support for the Limelight3A Vision sensor
    private LLResult llResult; // Custom subsystem object now reads directly from FTC telemetry, no NetworkTable object needed
    // === TARGET PARAMETERS (Calibrate Limelight Crosshairs such that these are 0.0) ===
// === TARGET PARAMETERS (Calibrate Limelight Crosshairs such that these are 0.0) ===
    private final double TARGET_TY_DEG = 0.0;
    private final double TARGET_TX_DEG = 0.0;

    // --- ACCEPTABLE TAG IDS ---
    private final int TAG_ID_1 = 20; // Acceptable Tag ID 1
    private final int TAG_ID_2 = 24; // Acceptable Tag ID 2

    // Target velocity is 0.0 for a stationary target (used for Feedforward)
    private final double TARGET_VELOCITY = 0.0;

    // === PID CONTROLLERS (Uses the uploaded PIDController class) ===
    // Kp, Ki, Kd, Kf, initialSetpoint
    // PID for Forward/Backward (using ty error)
    double forwardKp = 0.04;
    double forwardKi = 0.005;
    double forwardKd = 0.001;
    double forwardKf = 0.0;
    double forwardSp = 0.0;
    private PIDController forwardPID = new PIDController(forwardKp, forwardKi, forwardKd, forwardKf, forwardSp);
    // PID for Turning (using tx error)
    double turnKp = 0.02;
    double turnKi = 0.001;
    double turnKd = 0.0005;
    double turnKf = 0.0;
    double turnSp = 0.0;
    private PIDController turnPID = new PIDController(turnKp, turnKi,turnKd, turnKf, turnSp);

    // === CONTROL THRESHOLDS ===
    private final double TY_TOLERANCE_DEG = 0.5; // Tolerance for TY setpoint
    private final double TX_TOLERANCE_DEG = 1.0; // Tolerance for TX setpoint
    private final double MAX_POWER = 0.5;
    private final double MIN_POWER_FEEDFORWARD = 0.1; // Manual power floor
    private double forwardPower = 0.0;
    private double turnPower = 0.0;

    // === LIMELIGHT AND NETWORK TABLES ===
    private final String LIMELIGHT_NAME = "limelight";
    private double tx = 0.0;
    private double ty = 0.0;
    private double tz = 204.0;  // 144.0 * sqrt(2)
    private int tid = 20; // default, BLUE GOAL
    private double CAMERA_PITCH_DEG = 0.0;
    private double CAMERA_HEIGHT_IN = 4.0;

    // === CONTROL THRESHOLDS ===
    private final double RANGE_TOLERANCE_M = 0.02;
    private final double ROTATION_TOLERANCE_DEG = 1.0;
    private final double TARGET_HEIGHT_IN = 6.0; //29.5; // https://ftc-resources.firstinspires.org/ftc/game/manual, pages 65,
    private final double TARGET_DISTANCE_IN = 36.0;
    // === TIMER ===
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    /**
     * The main execution method for the OpMode. This function handles the entire
     * lifecycle of the robot's operation, from initialization and waiting to the
     * continuous execution loop for target tracking and control.
     *
     * <p>The loop implements a complete PID-controlled navigation routine designed
     * for locating and autonomously approaching a specific target (e.g., an AprilTag
     * or visual landmark) using the Limelight vision sensor and IMU.</p>
     *
     * <h3>Execution Flow:</h3>
     * <ol>
     * <li>**Datalogging Setup:** Initializes a datalogger with a unique timestamped filename.</li>
     * <li>**Hardware Initialization:** Calls helper methods to configure motors, directions, and the IMU.</li>
     * <li>**Vision Setup:** Initializes the Limelight 3A camera and its default pipeline.</li>
     * <li>**Wait and Start:** Waits for the OpMode to be started and resets the runtime timer.</li>
     * <li>**Control Loop:** Executes continuously while the OpMode is active, performing the following steps:
     * <ul>
     * <li>**Read Data:** Retrieves the latest pose and targeting data from the Limelight and IMU.</li>
     * <li>**Calculate PID:** Computes power outputs for range, strafe, and turn axes using dedicated PID controllers.</li>
     * <li>**Apply Limits:** Uses {@code limitAndSign} to cap power and apply minimum feedforward for stiction.</li>
     * <li>**Check Completion:** Evaluates tolerances to determine if the robot has successfully arrived at the target.</li>
     * <li>**Drive Mixing:** Sends the final, scaled power values to the Mecanum drive motors.</li>
     * <li>**Telemetry:** Updates the Driver Station with current status and error metrics.</li>
     * </ul>
     * </li>
     * </ol>
     *
     * @see #initializeHardware()
     * @see #initializeLimelight()
     * @see #readLimelightData()
     * @see #setMotorPower(double, double, double)
     * @see #limitAndSign(double, double, double)
     */
    public void runOpMode() {

        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss"); // Initialize the Datalogger with a filename and headers
        String timestamp = dateFormat.format(new Date());  // Get the current system date and time.
        String datalogFilename = TAG + "_" + timestamp; // Define the base filename and append the timestamp
        datalog = new Datalog(datalogFilename);

        initializeHardware();
        initializeLimelight();  // handles pipeline setting via telemetry
        configurePID(); // Sets TY_SETPOINT and PID configurations
        telemetry.addData("Status", "Hardware and Limelight Initialized (SDK 2D TY/TX Mode)");
        telemetry.addData("Target TY", String.format("%.2f deg", TARGET_TY_DEG));
        telemetry.addData("Target TX", String.format("%.2f deg", TARGET_TX_DEG));
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // 1. GET SENSOR DATA
            readLimelightData();

            // 2. CALCULATE PID OUTPUTS
            // Check if target is visible AND if the Tag ID is one of the desired ones
            if (!limelight.isRunning()) {
                telemetry.addData("Status", "Searching for Tag %d or %d. Current Tag: %.0f", TAG_ID_1, TAG_ID_2, tid);
                setMotorPower(0, 0, 0);
                telemetry.update();
                continue;
            }

            // 3. APPLY LIMITS AND FEEDFORWARD
            double forwardError = ty - TARGET_TY_DEG;
            double turnError = tx - TARGET_TX_DEG;

            // Note: Since PID error is calculated as (setpoint - current), which is (TY_SETPOINT - ty),
            // we use the power as calculated, and only need the error for the sign/feedforward check.
            forwardPower = limitAndSign(forwardPower, MAX_POWER, forwardError);
            turnPower = limitAndSign(turnPower, MAX_POWER, turnError);

            // No strafe power in this basic 2D example
            double strafePower = 0.0;

            // 4. CHECK FOR COMPLETION
            boolean isFinished = (Math.abs(forwardError) < TY_TOLERANCE_DEG) &&
                    (Math.abs(turnError) < TX_TOLERANCE_DEG);

            if (isFinished) {
                setMotorPower(0, 0, 0);
                telemetry.addData("Status", "TARGET REACHED and Locked (36 in)");
                telemetry.update();
                break;
            }

            // 5. MECANUM DRIVE MIXING
            setMotorPower(forwardPower, strafePower, turnPower);

            // 6. TELEMETRY
            telemetry.addData("Status", "Driving 2D to Tag");
            telemetry.addData("Target (in)", String.format("%.1f", TARGET_DISTANCE_IN));
            telemetry.addData("Current Range (in)", String.format("%.1f", tz));
            telemetry.addData("TY Error (deg)", String.format("%.2f / %.2f", ty, ROTATION_TOLERANCE_DEG));
            telemetry.addData("TX Error (deg)", String.format("%.2f", tx));
            telemetry.addData("Drive Power", String.format("%.2f", forwardPower));
            // Always ensure telemetry is updated at the end of the loop
            telemetry.update();
        }
    }
    // === LIMELIGHT 2D GEOMETRY FUNCTIONS ===
    /**
     * Calculates the ideal vertical angle setpoint (TY_SETPOINT_DEG) based on
     * the physical geometry of the camera and the target, and then configures
     * the setpoints and advanced features for the forward and turn PID controllers.
     *
     * <p>The vertical angle setpoint is calculated using trigonometry to determine
     * the angle from the horizontal to the target, and then subtracting the camera's
     * pitch angle. This results in the required Limelight 'ty' angle for the robot
     * to be at the ideal distance.
     *
     * <p>Formula for the Angle from Horizon:
     * $$\text{Angle from Horizon} = \text{atan}(\frac{h_2 - h_1}{D})$$
     * <p>Formula for TY Setpoint:
     * $$\text{TY\_SETPOINT\_DEG} = \text{Angle from Horizon} - \theta_{\text{pitch}}$$
     *
     * <ul>
     * <li>$h_2$: TARGET_HEIGHT_IN (Height of the target)</li>
     * <li>$h_1$: CAMERA_HEIGHT_IN (Height of the camera)</li>
     * <li>$D$: TARGET_DISTANCE_IN (Ideal distance from the camera to the target's base)</li>
     * <li>$\theta_{\text{pitch}}$: CAMERA_PITCH_DEG (The camera's angle relative to the ground)</li>
     * </ul>
     *
     * <p>This function also configures advanced features for the PID controllers:
     * <ul>
     * <li>{@code forwardPID}: Sets integral limit and integral error threshold.</li>
     * <li>{@code turnPID}: Sets a derivative filter and integral limit to reduce jitter.</li>
     * </ul>
     *
     * @implNote This function assumes that the following constants/variables are
     * properly defined and initialized:
     * {@code TARGET_HEIGHT_IN}, {@code CAMERA_HEIGHT_IN},
     * {@code TARGET_DISTANCE_IN}, {@code CAMERA_PITCH_DEG},
     * {@code TY_SETPOINT_DEG}, {@code TX_SETPOINT_DEG},
     * {@code forwardPID}, and {@code turnPID}.
     */
    private void configurePID() {
// TY PID (Forward/Backward)
        forwardPID.setIntegralLimit(0.5);
        forwardPID.setIntegralErrorThreshold(2.0); // Only integrate when ty is within 2 degrees of target

        // TX PID (Turning)
        turnPID.setDerivativeFilter(0.03); // 30ms filter to reduce turn jitter
        turnPID.setIntegralLimit(0.2);
    }

    /**
     * Calculates the horizontal distance (range) to a target using vertical angle measurements
     * and known mounting geometry.
     *
     * <p>This method implements a basic **trigonometric distance formula** common in
     * robotics vision. It requires the vertical offset angle from the Limelight
     * ({@code ty_angle_deg}) and compensates for the camera's fixed pitch and height
     * relative to the target's height. The formula used is:</p>
     * <p style="text-align:center;">$ D = \frac{H_{Target} - H_{Camera}}{\tan(\theta_{Pitch} + \theta_{TY})} $</p>
     * <ul>
     * <li>$D$: The calculated horizontal range.</li>
     * <li>$H_{Target}$: The known height of the target (e.g., {@code TARGET_HEIGHT_IN}).</li>
     * <li>$H_{Camera}$: The height of the camera lens (e.g., {@code CAMERA_HEIGHT_IN}).</li>
     * <li>$\theta_{Pitch}$: The fixed vertical angle (pitch) of the camera mount.</li>
     * <li>$\theta_{TY}$: The vertical offset angle measured by the Limelight.</li>
     * </ul>
     *
     * <p>The result is returned in the same units as the height constants (e.g., inches).</p>
     *
     * @param ty_angle_deg The vertical offset angle (TY) of the target, in degrees, as reported by the Limelight.
     * @return The estimated horizontal distance (range) to the target in inches.
     * @see #TARGET_HEIGHT_IN
     * @see #CAMERA_HEIGHT_IN
     * @see #CAMERA_PITCH_DEG
     */
    private double calculateRangeFromTY(double ty_angle_deg) {
        // Formula: D = (h2 - h1) / tan(theta + ty)

        // Angle in camera frame (vertical angle from horizon)
        double angleToTarget = CAMERA_PITCH_DEG + ty_angle_deg;
        double angleToTargetRad = Math.toRadians(angleToTarget);

        // This is the core formula for 2D distance calculation
        return (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleToTargetRad);
    }

    // === HELPER METHODS ===
    /**
     * Initializes all physical hardware components, including drive motors, directions,
     * power behavior, and the IMU (Inertial Measurement Unit).
     *
     * <p>This method performs the required setup tasks once at the start of the OpMode:</p>
     * <ol>
     * <li>**Mapping:** Retrieves all four drive motors and the IMU from the {@code hardwareMap}.</li>
     * <li>**Direction:** Sets the appropriate **reverse** and **forward** directions for the
     * motors to adhere to the standard Mecanum drive configuration (X-drive).</li>
     * <li>**Behavior:** Configures motors to run without encoders for pure velocity control
     * and sets their zero power behavior to {@code BRAKE} to immediately stop movement.</li>
     * <li>**IMU:** Initializes the IMU with a specified Rev Hub orientation on the robot
     * and performs an immediate {@code resetYaw()} to define the current heading as zero.</li>
     * </ol>
     *
     * <h3>IMU Initialization Details:</h3>
     * <p>The IMU is initialized based on the physical mounting of the Control Hub:</p>
     * <ul>
     * <li>**Logo Facing:** {@code FORWARD}</li>
     * <li>**USB Facing:** {@code UP}</li>
     * </ul>
     * This configuration ensures that yaw, pitch, and roll axes are correctly interpreted.
     *
     * @see #setMotorMode(DcMotorEx.RunMode)
     * @see #setMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior)
     * @see com.qualcomm.robotcore.hardware.IMU#resetYaw()
     */
    private void initializeHardware() {
                                                                        // --- HARDWARE MAP REFLECTS proposed nomenclature ---
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

                                                                        // Set direction based on standard Mecanum configuration
        motorLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorEx.Direction.FORWARD);

        setMotorMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");            // prudent assumption?
        // --- B. Define Hub Orientation on Robot ---
        // 1. Create the orientation object using the specified directions
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,   // Logo; Direction: The side of the hub with the REV logo
                RevHubOrientationOnRobot.UsbFacingDirection.UP          // USB Direction: The side of the hub with the USB-C port
        );
        // --- C. Create and Apply IMU Parameters ---
        // 2. Create the IMU Parameters object and pass the orientation
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        // 3. Initialize the IMU with the parameters
        imu.initialize(parameters);                                     // This process automatically calibrates and configures the sensor
        imu.resetYaw();
    }

    /**
     * Sets the run mode for all four drive motors.
     *
     * <p>This is a convenience method that applies the specified {@code DcMotorEx.RunMode}
     * to the left-front, right-front, left-back, and right-back motors simultaneously.
     * The {@code DcMotorEx} type allows for specific modes like {@code RUN_TO_POSITION},
     * {@code RUN_USING_ENCODER}, and {@code STOP_AND_RESET_ENCODER}.</p>
     *
     * @param mode The desired run mode to be set for the motors. Must not be null.
     * @see org.firstinspires.ftc.robotcore.external.Const
     */
    private void setMotorMode(DcMotorEx.RunMode mode) {
        motorLeftFront.setMode(mode);
        motorRightFront.setMode(mode);
        motorLeftBack.setMode(mode);
        motorRightBack.setMode(mode);
    }

    /**
     * Sets the zero power behavior for all four drive motors.
     *
     * <p>This is a convenience method that applies the specified {@code DcMotorEx.ZeroPowerBehavior}
     * to the left-front, right-front, left-back, and right-back motors simultaneously.
     * This determines what the motors do when a zero power is commanded (i.e., when
     * they are commanded to stop).</p>
     *
     * <p>Common behaviors include {@code BRAKE} (motor resists movement) and
     * {@code FLOAT} (motor coasts freely).</p>
     *
     * @param behavior The desired zero power behavior to be set for the motors. Must not be null.
     */
    private void setMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        motorLeftFront.setZeroPowerBehavior(behavior);
        motorRightFront.setZeroPowerBehavior(behavior);
        motorLeftBack.setZeroPowerBehavior(behavior);
        motorRightBack.setZeroPowerBehavior(behavior);
    }

    /**
     * Initializes the Limelight 3A vision sensor for use in the robot's pipeline.
     *
     * <p>This method retrieves the Limelight 3A object from the hardware map, sets the
     * telemetry transmission interval, and activates the initial vision pipeline. The
     * core purpose is to ready the camera for immediate data polling.</p>
     *
     * <h3>Limelight 3A Feature Scope (DECODE 2025):</h3>
     * <p>The Limelight 3A offers robust vision processing. For the current DECODE 2025
     * application, the following features are in scope:</p>
     * <ul>
     * <li>✓ **AprilTag Tracking and Robot Localization:** Applicable and in use.</li>
     * <li>✓ **Color Blob Tracking:** Applicable and in use.</li>
     * </ul>
     * <p>The following features are currently **not** in scope (X):</p>
     * <ul>
     * <li>X Neural Network Object Detection (CPU only)</li>
     * <li>X Neural Network Classification</li>
     * <li>X Barcode Tracking</li>
     * <li>X Custom Python Pipelines</li>
     * </ul>
     *
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a">Limelight 3A Official Documentation</a>
     * @see # pipelineSwitch(int)
     * Switches the active vision processing pipeline running on the Limelight 3A.
     *
     * <p>Pipelines are essentially pre-configured vision programs (e.g., AprilTag tracking,
     * color blob detection, or neural network inference) that determine how the
     * Limelight processes its camera feed. The device supports up to 10 user-configurable
     * pipelines, indexed from 0 to 9.</p>
     *
     * <p>This method sends a command over the network to the Limelight and returns immediately.
     * The pipeline switch operation is typically completed in a matter of milliseconds.
     * You can configure and tune these pipelines via the Limelight's web interface.</p>
     *
     * @param
     * */
    private void initializeLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); // check Driver Station
        telemetry.setMsTransmissionInterval(11);  // default for Limelight
        limelight.pipelineSwitch(0);        // switches to AprilTag (using index=0)
        limelight.start();                        // start periodic polling for data
    }

    /**
     * Reads the latest Limelight vision data by querying the FTC Telemetry map.
     *
     * <p>This method accesses Limelight data indirectly by inspecting the internal
     * {@code telemetry.getDeltas()} map, assuming the Limelight is configured to stream
     * its standard NetworkTables keys directly to the Driver Station telemetry service.
     * This approach bypasses dedicated Limelight SDK classes but relies entirely on
     * the Limelight's NetworkTables stream settings remaining at their default names.</p>
     *
     * <p>The function specifically attempts to retrieve the following key vision outputs:</p>
     * <ul>
     * <li>**tv (Target Valid):** Sets the {@code tv} variable (0.0 or 1.0).</li>
     * <li>**tx (Horizontal Angle Error):** Sets the {@code tx} variable (bearing).</li>
     * <li>**ty (Vertical Angle Error):** Sets the {@code ty} variable (elevation).</li>
     * </ul>
     *
     * <p>If a key is not found in the telemetry map, the corresponding variable is safely
     * set to {@code 0.0}. **Note:** For modern FTC development, using the official
     * {@code Limelight3A} class and its {@code getLatestResult()} method is the recommended practice.</p>
     */
    private void readLimelightData() {
        // Send a query to the Limelight over telemetry to request the values.
        llResult = limelight.getLatestResult(); // get the latest result from the Limelight camera
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            tid = fiducial.getFiducialId(); // The ID number of the fiducial
            tx = fiducial.getTargetXDegrees(); // Where it is (left-right)
            ty = fiducial.getTargetYDegrees(); // Where it is (up-down)
            tz = calculateRangeFromTY(ty);
            telemetry.addData("Fiducial " + tid, "is " + tz + " meters away");
        }
    }

    /**
     * Constrains a calculated motor power value and applies a minimum feedforward
     * before setting the sign based on the error.
     *
     * <p>This method performs three primary operations on the raw {@code power} input:</p>
     * <ol>
     * <li>The absolute value of {@code power} is capped at {@code maxPower}.</li>
     * <li>If the absolute {@code error} is greater than {@code 0.05}, the power
     * is guaranteed to be at least {@code MIN_POWER_FEEDFORWARD} to overcome
     * motor stiction (a common feedforward technique).</li>
     * <li>The final sign (direction) of the power is determined by the sign of the {@code error}.</li>
     * </ol>
     *
     * @param power The calculated raw motor power value (usually from a PID controller).
     * @param maxPower The maximum absolute power value allowed (e.g., 1.0 or 0.8).
     * @param error The current error in the system (e.g., target minus current position).
     * @return The final, limited, and signed motor power value ready for output to the motor.
     * @see #MIN_POWER_FEEDFORWARD
     */
    private double limitAndSign(double power, double maxPower, double error) {
        power = Math.min(Math.abs(power), maxPower);

        if (Math.abs(error) > 0.05) {
            power = Math.max(power, MIN_POWER_FEEDFORWARD);
        }

        return (error > 0) ? power : -power;
    }

    /**
     * Safely attempts to parse an arbitrary {@code Object} into a {@code double},
     * returning a specified default value upon failure.
     *
     * <p>This utility method is designed to handle common data retrieval scenarios,
     * such as reading raw values from a map or network table, where the data type
     * might be inconsistent or missing.</p>
     *
     * <p>The parsing logic follows these steps:</p>
     * <ol>
     * <li>Checks if the input {@code value} is an instance of {@code String}.</li>
     * <li>If it is a {@code String}, attempts to convert it to a {@code double} using {@code Double.parseDouble()}.</li>
     * <li>If the conversion fails (i.e., a {@code NumberFormatException} occurs), the specified {@code defaultValue} is returned.</li>
     * <li>If the input {@code value} is not a {@code String} (e.g., it is {@code null} or another type), the {@code defaultValue} is returned immediately.</li>
     * </ol>
     *
     * @param value The object to be parsed, typically expected to be a {@code String} representation of a number.
     * @param defaultValue The fallback value to return if the input is null, not a String, or cannot be successfully parsed.
     * @return The parsed {@code double} value if successful, otherwise the {@code defaultValue}.
     */
    private double tryParseDouble(Object value, double defaultValue) {
        if (value instanceof String) {
            try {
                return Double.parseDouble((String) value);
            } catch (NumberFormatException e) {
                // Return default if parsing fails
                return defaultValue;
            }
        }
        // If the value is null or not a string, return the default.
        return defaultValue;
    }
    /**
     * Calculates and sets the individual power levels for all four drive motors
     * based on Mecanum wheel kinematics.
     *
     * <p>This method combines the three primary directional inputs (drive, strafe, turn)
     * using the standard Mecanum wheel formulas to determine the raw power required
     * by each of the four motors (Left Front, Right Front, Left Back, Right Back).</p>
     *
     * <p>Crucially, the method then scales all motor powers down proportionally if
     * any single motor power exceeds the maximum limit of 1.0. This ensures the robot
     * retains its directional control without saturation when commanded to move
     * faster than physically possible (i.e., vector scaling). The variables
     * {@code motorLeftFront}, etc., are only ever directly accessed here.</p>
     *
     * @param drive The forward/backward translation input (Y-axis), typically ranging from -1.0 (backward) to 1.0 (forward).
     * @param strafe The left/right translation input (X-axis), typically ranging from -1.0 (right) to 1.0 (left).
     * @param turn The rotational input (Z-axis), typically ranging from -1.0 (clockwise) to 1.0 (counter-clockwise).
     * @see <a href="http://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib">Mecanum Wheel Kinematics</a>
     */
    private void setMotorPower(double drive, double strafe, double turn) {
                                                                        // Mecanum Kinematics: The only location where motor variables are directly used.
        double powerLeftFront = drive + strafe + turn;
        double powerRightFront = drive - strafe - turn;
        double powerLeftBack = drive - strafe + turn;
        double powerRightBack = drive + strafe - turn;

                                                                        // Scale powers
        double max = Math.max(Math.abs(powerLeftFront), Math.max(Math.abs(powerRightFront),
                Math.max(Math.abs(powerLeftBack), Math.abs(powerRightBack))));

        if (max > 1.0) {
            powerLeftFront /= max;
            powerRightFront /= max;
            powerLeftBack /= max;
            powerRightBack /= max;
        }

        motorLeftFront.setPower(powerLeftFront);
        motorRightFront.setPower(powerRightFront);
        motorLeftBack.setPower(powerLeftBack);
        motorRightBack.setPower(powerRightBack);
    }


    // =========================================================================================
    // DATALOG Class Definition (Wraps the Datalogger.java utility)
    // =========================================================================================
    /**
     * This wrapper class defines the fields for the log file and manages the Datalogger instance
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
                "Ki",
                "Speed"
        };

        /**
         * @param name filename for output log
         */
        public Datalog(String name)
        {
            // The Datalogger constructor handles file creation and header writing
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
                double voltage, double rKp, double rKi, double rKd) {

                                                                        // Manually construct the string array for the Datalogger.log() method, applying formatting
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
                    String.format("%.3f", rKp),
                    String.format("%.3f", rKi),
                    String.format("%.2f", rKd)
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

