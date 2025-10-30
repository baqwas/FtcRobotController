/*
Copyright (c) 2024 Limelight Vision

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
*/
package org.firstinspires.ftc.teamcode.Vision.Limelight;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * LimelightOdometryOpMode
 * Demonstrates how to use the FTC AprilTagProcessor with a Limelight 3A (configured as a Webcam)
 * to serve as a reliable source for odometry location data based on field AprilTags.
 */
@TeleOp(name = "Limelight Odometry", group = "Vision")
@Disabled // Remove the @Disabled annotation to enable this OpMode
public class LimelightOdometry extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private WebcamName camera; // Assume Limelight is configured as a standard Webcam in the config

    @Override
    public void runOpMode() {
        // --- 1. Initialization and Processor Setup ---
        // Get the Webcam/Limelight from the Hardware Map (must be configured as 'limelight')
        camera = hardwareMap.get(WebcamName.class, "limelight");

        // Create the AprilTag Processor. This handles detection and pose estimation.
        aprilTag = new AprilTagProcessor.Builder()
                // Set the desired camera viewing distance and field geometry units
                // NOTE: These parameters must be calibrated correctly for accurate odometry!
                // The Limelight's internal processing relies on correct camera location/rotation relative to the robot.
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(578.272, 578.272, 402.145, 441.405) // Placeholder: Use YOUR camera's calibration data
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Create the Vision Portal, which manages the camera and processors.
        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Good for network efficiency
                .setCameraResolution(new android.util.Size(640, 480)) // Match Limelight's preferred resolution
                .build();

        // --- 2. Startup Loop ---
        telemetry.addData("Status", "Limelight Ready. Press START.");
        telemetry.addData("VisionPortal Status", visionPortal.getCameraState());
        telemetry.update();

        waitForStart();

        // --- 3. Run Loop (Odometry Data Retrieval) ---
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Display the current robot pose (position and orientation)
                logAprilTagOdometry();

                // Add any other standard OpMode telemetry (motor power, etc.)
                // telemetry.addData("Drivetrain Power", motorFrontLeft.getPower());

                telemetry.update();
                sleep(50); // Loop quickly for near real-time updates
            }
        }

        // Deactivate the vision system on OpMode stop
        visionPortal.close();
    }

    /**
     * Extracts robot pose data from the AprilTag processor and logs it to telemetry.
     */
    private void logAprilTagOdometry() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# Tags Detected", currentDetections.size());

        // Check if we have at least one detection to base our odometry on
        if (currentDetections.size() > 0) {
            // We'll use the first detected tag for odometry visualization.
            // In a real autonomous routine, you would average detections or use the most reliable one.
            AprilTagDetection detection = currentDetections.get(0);

            // The Pose field contains the translation (X, Y, Z) and rotation (Yaw, Pitch, Roll)
            // relative to the tag. Since the AprilTagProcessor knows the field layout,
            // this data, when properly calibrated, can be treated as field-relative odometry.

            // Translation (X, Y, Z) - Location
            telemetry.addLine()
                    .addData("Tag ID", detection.id)
                    .addData("X (Inches)", "%.1f", detection.ftcPose.x)
                    .addData("Y (Inches)", "%.1f", detection.ftcPose.y)
                    .addData("Z (Inches)", "%.1f", detection.ftcPose.z);

            // Rotation (Yaw, Pitch, Roll) - Orientation
            telemetry.addLine()
                    .addData("Yaw (Deg)", "%.1f", detection.ftcPose.yaw)
                    .addData("Pitch (Deg)", "%.1f", detection.ftcPose.pitch)
                    .addData("Roll (Deg)", "%.1f", detection.ftcPose.roll);

            // The X, Y, Yaw values serve as your odometry coordinates.
            // A more advanced system would feed these values into a robot state estimator.

        } else {
            telemetry.addData("Odometry Status", "No Tags Detected - Relocating...");
        }

    }
}
