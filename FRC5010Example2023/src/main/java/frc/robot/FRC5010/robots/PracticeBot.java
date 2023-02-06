// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionPhotonMultiCam;
import frc.robot.FRC5010.constants.GenericSwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.robots.RobotFactory.Parts;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;

/** Add your docs here. */
public class PracticeBot extends RobotConfig {

    GenericSwerveConstants swerveConstants;

    public PracticeBot() {

        swerveConstants = new GenericSwerveConstants(Units.inchesToMeters(24.25), Units.inchesToMeters(20.5));
        swerveConstants.setkFrontLeftAbsoluteOffsetRad(-2.357 + Math.PI);
        swerveConstants.setkFrontRightAbsoluteOffsetRad(-2.792);
        swerveConstants.setkBackLeftAbsoluteOffsetRad(0.845 + Math.PI);
        swerveConstants.setkBackRightAbsoluteOffsetRad(-0.171);
        swerveConstants.setkPhysicalMaxSpeedMetersPerSecond(Units.feetToMeters(12));
        swerveConstants.setkPhysicalMaxAngularSpeedRadiansPerSecond(4 * Math.PI);
        swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(1);
        swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);
        swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.4);
        swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);

        VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1, AprilTags.aprilTagRoomLayout,PoseStrategy.CLOSEST_TO_LAST_POSE);
        multiVision.addPhotonCamera("FrontCamera", 
          new Transform3d( // This describes the vector between the camera lens to the robot center on the ground
            new Translation3d(Units.inchesToMeters(-2), Units.inchesToMeters(0.0), Units.inchesToMeters(3.5)), 
            new Rotation3d(0, Units.degreesToRadians(-20), 0)
          )
        );
        multiVision.addPhotonCamera("BackCamera", 
        new Transform3d( // This describes the vector between the camera lens to the robot center on the ground
          new Translation3d(Units.inchesToMeters(-5.5), 0, Units.inchesToMeters(3.5)), 
          new Rotation3d(0, Units.degreesToRadians(180), 0)
        )
      );
        // multiVision.createRobotPoseEstimator();
        List<SwervePorts> swervePorts = new ArrayList<>();
        swervePorts.add(new SwervePorts(1, 3, 0));
        swervePorts.add(new SwervePorts(10, 8, 1));
        swervePorts.add(new SwervePorts(5, 4, 2));
        swervePorts.add(new SwervePorts(2, 7, 3));

        GenericGyro gyro = new PigeonGyro(11);

        GenericMechanism drive = new Drive(multiVision, gyro, Drive.Type.MK4_SWERVE_DRIVE, swervePorts, swerveConstants);
        robotParts.put(Parts.VISION, multiVision);
        robotParts.put(Parts.DRIVE, drive);
    }
}
