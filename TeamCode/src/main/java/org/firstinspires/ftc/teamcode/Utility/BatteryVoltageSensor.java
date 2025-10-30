package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Utility class to read and format the robot's main battery voltage.
 * It searches the hardware map for all voltage sensors and returns the
 * maximum voltage, which typically corresponds to the main battery.
 */
public final class BatteryVoltageSensor {
  private final HardwareMap hardwareMap;

  /**
   * Constructs the utility with access to the robot's hardware.
   * @param hardwareMap The OpMode's hardware map.
   */
  public BatteryVoltageSensor(HardwareMap hardwareMap) {
    this.hardwareMap = hardwareMap;
  }

  /**
   * Reads the voltage from all connected sensors and returns the maximum reading.
   * This is generally the most reliable way to get the main battery voltage.
   * @return The maximum voltage reading in Volts.
   */
  private double getVoltage() {
    double maxVoltage = 0.0;
    try {
      // Iterate over all voltage sensors in the hardware map
      for (VoltageSensor sensor : hardwareMap.voltageSensor) {
        double voltage = sensor.getVoltage();
        if (voltage > maxVoltage) {
          maxVoltage = voltage;
        }
      }
    } catch (Exception e) {
      // Handle cases where no voltage sensors are found (unlikely in FTC)
      System.err.println("BATTERY VOLTAGE EXCEPTION: " + e.getMessage());
    }
    return maxVoltage;
  }

  /**
   * Gets the voltage and formats it as a String for Datalogging.
   * @return The formatted voltage string (e.g., "12.87").
   */
  public String getFormattedVoltage() {
    return String.format("%.2f", getVoltage());
  }
}