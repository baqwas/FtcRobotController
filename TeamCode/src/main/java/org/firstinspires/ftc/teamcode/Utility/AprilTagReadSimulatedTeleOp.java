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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="AprilTag Read Simulated TeleOp", group="Test")
@Disabled // Uncomment this line to temporarily disable the OpMode
public class AprilTagReadSimulatedTeleOp extends OpMode {

  private int autonomousObeliskID = 0;

  // Placeholder for robot hardware:
  // private DcMotor motorArm;

  // --- Initialization (Runs once when INIT is pressed) ---
  @Override
  public void init() {

    // --- **DATA TRANSFER (READ)** ---
    // Read the value of the static variable saved by the autonomous opmode.
    autonomousObeliskID = SharedData.obeliskID;

    // Reset the static variable for the next match (good practice)
    SharedData.obeliskID = 0;

    // --- Hardware Initialization (Placeholder) ---
    // motorArm = hardwareMap.get(DcMotor.class, "arm_motor");

    telemetry.addData("Status", "TeleOp Initialized.");
    telemetry.addData("Transferred AprilTag ID", autonomousObeliskID);
    telemetry.update();
  }

  // --- Loop (Runs repeatedly after START is pressed) ---
  @Override
  public void loop() {

    String actionMessage;

    // Use the Transferred Data to alter TeleOp behavior
    if (autonomousObeliskID != 0) {
      actionMessage = "Auto Target ID: " + autonomousObeliskID + ". Adjusting TeleOp strategy!";

      // **Example Calculation using the ID:**
      // Use the ID to set an initial target position, a multiplier, or a flag.
      // Example: If ID is 42, set a mechanism to a height of 4200.
      // if (gamepad1.a) {
      //     motorArm.setTargetPosition(autonomousAprilTagID * 100);
      // }

    } else {
      actionMessage = "No AprilTag ID transferred. Running default mode.";
    }

    // --- Driver Control Code (Placeholder) ---
    // double drive = -gamepad1.left_stick_y;
    // double turn  =  gamepad1.right_stick_x;
    // ... (your drive code)

    telemetry.addData("Target Info", actionMessage);
    telemetry.addData("Game Pad Left Y", gamepad1.left_stick_y);
    telemetry.update();
  }

  // --- Stop (Runs once when STOP is pressed) ---
  @Override
  public void stop() {
    // Stop all motors
  }
}