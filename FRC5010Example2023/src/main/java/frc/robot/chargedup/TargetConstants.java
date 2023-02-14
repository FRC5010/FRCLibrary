// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FRC5010.Vision.AprilTags;

/** Add your docs here. */
public class TargetConstants {

    public static double tagToConeXOffset = Units.feetToMeters(1); 
    public static double tagToRobotYOffset = Units.feetToMeters(4); 

    public static Pose2d tag1LeftCone;
    public static Pose2d tag1RightCone;

    public static Pose2d tag2LeftCone;
    public static Pose2d tag2RightCone;

    public static Pose2d tag3LeftCone;
    public static Pose2d tag3RightCone;

    public static Pose2d tag4LeftStation;
    public static Pose2d tag4RightStation;
    
    public static Pose2d tag5LeftStation;
    public static Pose2d tag5RightStation;

    public static Pose2d tag6LeftCone;
    public static Pose2d tag6RightCone;

    public static Pose2d tag7LeftCone;
    public static Pose2d tag7RightCone;

    public static Pose2d tag8LeftCone;
    public static Pose2d tag8RightCone;


    public TargetConstants() {
        Transform2d tagToLeftConeTransfrom = new Transform2d(new Translation2d(0, tagToConeXOffset), Rotation2d.fromDegrees(0));
        Transform2d tagToRightConeTransfrom = new Transform2d(new Translation2d(0, -tagToConeXOffset)
        , Rotation2d.fromDegrees(0));

        tag1LeftCone = AprilTags.aprilTagFieldLayout.getTagPose(1).orElse(new Pose3d()).toPose2d().transformBy(tagToLeftConeTransfrom);
        tag1RightCone = AprilTags.aprilTagFieldLayout.getTagPose(1).orElse(new Pose3d()).toPose2d().transformBy(tagToRightConeTransfrom);

    }



    
}
