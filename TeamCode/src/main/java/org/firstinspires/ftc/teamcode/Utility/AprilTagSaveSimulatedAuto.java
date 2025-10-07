/* Copyright (c) 2023 FIRST. All rights reserved.
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

 This SharedData class is provided for FTC programmers.

 Most users will not need to edit this class
 */

package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Note: In a real OpMode, you would import the AprilTag classes:
// import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
// import org.firstinspires.ftc.vision.VisionPortal;
// import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
// import java.util.List;

@Autonomous(name="AprilTag Save Simulated Autonomous", group="Test", preselectTeleOp="AprilTag Read Simulated TeleOp")
@Disabled // Uncomment this line to temporarily disable the OpMode
public class AprilTagSaveSimulatedAuto extends LinearOpMode {

  // Placeholder for robot hardware and vision
  // private AprilTagProcessor aprilTag;
  // private VisionPortal visionPortal;

  // We'll use a constant to simulate the ID detected
  private final int SIMULATED_DETECTED_ID = 20;

  @Override
  public void runOpMode() {

    // --- Initialization and Setup ---
    telemetry.addData("Status", "Auto Initialized. (AprilTag ID is 0)");
    telemetry.update();

    // **(Real OpMode Step: Initialize VisionPortal and AprilTagProcessor here)**

    waitForStart();

    if (opModeIsActive()) {

      // --- Autonomous Actions and Detection ---

      // 1. Simulate movement to the vision target
      telemetry.addData("Status", "Moving to target...");
      telemetry.update();
      sleep(1500); // Simulate driving for 1.5 seconds

      // 2. Simulate AprilTag Detection
      int detectedId = SIMULATED_DETECTED_ID;

      // **(Real OpMode Step: Get detections and assign the relevant ID)**
            /*
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (!currentDetections.isEmpty()) {
                detectedId = currentDetections.get(0).id;
            }
            */

      // 3. --- **DATA TRANSFER (WRITE)** ---
      // Save the detected ID to the static variable.
      SharedData.obeliskID = detectedId;

      telemetry.addData("AprilTag Detection", "ID: " + detectedId + " SAVED.");
      telemetry.addData("Status", "Finishing Autonomous...");
      telemetry.update();

      // 4. Simulate end-game actions
      sleep(2000); // Simulate parking or final movements
    }

    // **(Real OpMode Step: Close visionPortal here to save resources)**
    // visionPortal.close();

    // OpMode ends. The static variable 'SharedData.aprilTagID' retains the value '42'.
  }
}