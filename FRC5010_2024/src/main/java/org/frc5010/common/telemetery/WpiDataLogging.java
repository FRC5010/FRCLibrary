// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.telemetery;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Add your docs here. */
public class WpiDataLogging {
  private static boolean logging = false;

  private WpiDataLogging() {}

  public static void start(boolean log) {
    logging = log;
    if (log) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog(), false);
      DataLogManager.logNetworkTables(true);

      Shuffleboard.getTab("Robot").addBoolean("Logging", () -> logging);
    }
  }

  public static void log(String message) {
    if (logging) {
      DataLogManager.log(message);
    }
  }
}
