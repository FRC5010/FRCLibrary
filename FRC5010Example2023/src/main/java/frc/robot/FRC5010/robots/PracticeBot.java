// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import frc.robot.FRC5010.Vision.VisionLimeLightSim;
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
        swerveConstants.setkFrontLeftAbsoluteOffsetRad(-2.416);
        swerveConstants.setkFrontRightAbsoluteOffsetRad(-2.789);
        swerveConstants.setkBackLeftAbsoluteOffsetRad(0.772);
        swerveConstants.setkBackRightAbsoluteOffsetRad(3.11);
        swerveConstants.setkPhysicalMaxSpeedMetersPerSecond(Units.feetToMeters(12));
        swerveConstants.setkPhysicalMaxAngularSpeedRadiansPerSecond(2 * Math.PI);
        swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(5);
        swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);
        swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.4);
        swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);

        VisionLimeLightSim multiVision = new VisionLimeLightSim("simulation", 0);
        // VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1, AprilTags.aprilTagRoomLayout,PoseStrategy.CLOSEST_TO_LAST_POSE);
        // multiVision.addPhotonCamera("Arducam_OV9281_USB_Camera", 
        //   new Transform3d( // This describes the vector between the camera lens to the robot center on the ground
        //     new Translation3d(Units.inchesToMeters(7), 0, Units.inchesToMeters(16.75)), 
        //     new Rotation3d(0, Units.degreesToRadians(-20), 0)
        //   )
        // );
        // multiVision.createRobotPoseEstimator();
        List<SwervePorts> swervePorts = new ArrayList<>();
        // TODO: Remap encoderPorts once PracticeBot has absolute encoders.
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
