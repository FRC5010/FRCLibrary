// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.chargedup.TargetConstants;

/** Add your docs here. */
public class TranslationConstants {

    public static Transform2d tagToLeftConeTransfrom = new Transform2d(
            new Translation2d(TargetConstants.tagToRobotXOffset, TargetConstants.tagToConeYOffset),
            Rotation2d.fromDegrees(180));

    public static Transform2d tagToCubeTransfrom = new Transform2d(
            new Translation2d(TargetConstants.tagToRobotXOffset, 0),
            Rotation2d.fromDegrees(180));

    public static Transform2d tagToRightConeTransfrom = new Transform2d(
            new Translation2d(TargetConstants.tagToRobotXOffset, -TargetConstants.tagToConeYOffset),
            Rotation2d.fromDegrees(180));

    public static Transform2d noteTransform = new Transform2d(
            new Translation2d(Units.inchesToMeters(-13.0), 0.0), new Rotation2d());
}
