// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionPhotonMultiCam;
import frc.robot.FRC5010.constants.GenericSwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.robots.RobotConfig;
import frc.robot.FRC5010.robots.RobotFactory.Parts;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.NavXGyro;

/** Add your docs here. */
public class CompBot extends RobotConfig {
    private GenericSwerveConstants swerveConstants; 
    private Drive drive; 
    // private List<List<PathPlannerTrajectory>> paths = new ArrayList<>(); 
    private HashMap<String, List<PathPlannerTrajectory>> eventMap = new HashMap<>();   
    public CompBot(){
        // Needs to be set
        swerveConstants = new GenericSwerveConstants(0, 0);

        // Baby Swerve values need to be changed
        swerveConstants.setkFrontLeftAbsoluteOffsetRad(0.26);
        swerveConstants.setkFrontRightAbsoluteOffsetRad(-3.14);
        swerveConstants.setkBackLeftAbsoluteOffsetRad(1.0+Math.PI);
        swerveConstants.setkBackRightAbsoluteOffsetRad(0.21+Math.PI);
        swerveConstants.setkPhysicalMaxSpeedMetersPerSecond(15);
        swerveConstants.setkPhysicalMaxAngularSpeedRadiansPerSecond(2 * Math.PI);
        swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(5);
        swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);
        swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.4);
        swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);

        // Will need to be changed for 2023 field
        VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1, AprilTags.aprilTagRoomLayout, PoseStrategy.AVERAGE_BEST_TARGETS);
        multiVision.addPhotonCamera("Arducam_OV9281_USB_Camera", 
          new Transform3d( // This describes the vector between the camera lens to the robot center on the ground
            new Translation3d(Units.inchesToMeters(7), 0, Units.inchesToMeters(16.75)), 
            new Rotation3d(0, Units.degreesToRadians(-20), 0)
          )
        );

        // Ports need to be changed when comp bot is ready
        List<SwervePorts> swervePorts = new ArrayList<>();
        swervePorts.add(new SwervePorts(1, 2, 0));
        swervePorts.add(new SwervePorts(7, 8, 1));
        swervePorts.add(new SwervePorts(3, 4, 2));
        swervePorts.add(new SwervePorts(5, 6, 3));

        GenericGyro gyro = new NavXGyro(SPI.Port.kMXP);

        drive = new Drive(multiVision, gyro, Drive.Type.THRIFTY_SWERVE_DRIVE, swervePorts, swerveConstants);
        robotParts.put(Parts.VISION, multiVision);
        robotParts.put(Parts.DRIVE, drive);
        robotParts.put(Parts.AUTO, setAutoCommands()); 
    } 

    public Map<String, Command> setAutoCommands(){
        return drive.setAutoCommands(eventMap); 
    }

}
