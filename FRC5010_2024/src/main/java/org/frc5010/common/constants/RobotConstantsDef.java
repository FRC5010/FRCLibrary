// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

import org.frc5010.common.arch.Persisted;

/** For Constants that don't change from year to year. */
public class RobotConstantsDef {
  public static final String DRIVE_VISUAL_H = "driveVisualH";
  public static final String DRIVE_VISUAL_V = "driveVisualV";
  public static final String ROBOT_VISUAL_H = "robotVisualH";
  public static final String ROBOT_VISUAL_V = "robotVisualV";

  // --------------------------------------------------------
  // Persisted values can be handled in a Constants file like this as an example
  // Recommend using other files for this
  public static Persisted<Integer> robotVisualV =
      new Persisted<>(RobotConstantsDef.ROBOT_VISUAL_V, 60);
}
