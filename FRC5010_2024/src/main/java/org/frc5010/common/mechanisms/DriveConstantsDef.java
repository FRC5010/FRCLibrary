// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.mechanisms;

/** Drive Train Constants that will be re-used year to year */
public class DriveConstantsDef {
  // ****** Persisted value name strings - use ALL_CAPS_UNDERSCORE_SEPARATED style
  // For consistency, make the String value the same as the Persisted variable below
  // This defines how it will display in the dashboards, so inconsistent naming will be confusing
  public static final String KV_DRIVE_LINEAR = "kvVoltSecondsPerMeter";
  public static final String KA_DRIVE_LINEAR = "kaVoltSecondsSquaredPerMeter";
  public static final String MOTOR_ROT_PER_WHEEL_ROT = "motorRotationsPerWheelRotation";
  public static final String TRACK_WIDTH = "trackWidth";
  public static final String MAX_CHASSIS_VELOCITY = "maxChassisVelocity";
  public static final String MAX_CHASSIS_ROTATION = "maxChassisRotation";

  public static final String KV_DRIVE_ANGULAR = "kVAngular";
  public static final String KA_DRIVE_ANGULAR = "kAAngular";

  public static final String SWERVE_TURN_P = "SwerveTurnP";
  public static final String SWERVE_TURN_I = "SwerveTurnI";
  public static final String SWERVE_TURN_D = "SwerveTurnD";
}
