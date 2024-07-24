// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.frc5010.common.arch.Persisted;

/** Add your docs here. */
public class VisionConstants {
  public static final double CAMERA_CAL_DISTANCE = 120;
  public static String SBTabVisionDisplay = "Vision";
  public static final double cameraHeight = Units.inchesToMeters(3);
  public static final double cameraAngle = Units.degreesToRadians(-20);
  public static final Transform3d kCameraToRobot =
      new Transform3d(
          new Translation3d(-0.06, 0.0, 0.09),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));
  public Persisted<String> laptopCamera;

  public VisionConstants() {
    laptopCamera = new Persisted<>(VisionConstantDefs.LAPTOP_CAMERA, "");
  }
}
