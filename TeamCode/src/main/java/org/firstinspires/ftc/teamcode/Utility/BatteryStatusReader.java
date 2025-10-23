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

package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * BatteryStatusReader OpMode
 * Demonstrates how to read and display the robot battery voltage using the HardwareMap's
 * VoltageSensor collection. This information is automatically sent to the Driver Hub
 * status pane via telemetry.
 */
@TeleOp(name = "Battery Status Reader", group = "Utility")
//@Disabled // Remove the @Disabled annotation to enable this OpMode
public class BatteryStatusReader extends LinearOpMode {

  private static final double MIN_VOLTAGE = 12.0; // Recommended minimum voltage for competition

  @Override
  public void runOpMode() {
    // Initialization (runs once after INIT is pressed)
    telemetry.addData("Status", "Initialized. Press START to monitor voltage.");
    telemetry.update();

    // Wait for the game to start (Driver presses PLAY)
    waitForStart();

    // Main execution loop (runs repeatedly after START is pressed)
    while (opModeIsActive()) {
      // Step 1: Initialize variables for total voltage and count
      double totalVoltage = 0.0;
      int voltageSensorCount = 0;

      // Step 2: Iterate through all devices that implement VoltageSensor (e.g., Control Hub, Expansion Hubs)
      for (VoltageSensor sensor : hardwareMap.voltageSensor) {
        totalVoltage += sensor.getVoltage();
        voltageSensorCount++;
      }

      // Step 3: Calculate the average voltage
      double averageVoltage = (voltageSensorCount > 0) ? (totalVoltage / voltageSensorCount) : 0.0;

      // Step 4: Display the voltage reading using telemetry
      // This information appears on the Driver Hub's status pane.
      telemetry.addData("Robot Voltage", "%.2f V", averageVoltage);

      // Add a simple status check for visual feedback
      if (averageVoltage < MIN_VOLTAGE) {
        telemetry.addData("Battery Status", "CRITICAL - CHANGE BATTERY!", averageVoltage);
      } else {
        telemetry.addData("Battery Status", "OK");
      }

      // Step 5: Update the Driver Hub display
      telemetry.update();

      // Pause the thread to conserve CPU resources
      sleep(100);
    }
  }
}
