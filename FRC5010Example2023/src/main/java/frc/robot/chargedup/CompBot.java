// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionPhotonMultiCam;
import frc.robot.FRC5010.constants.AutoMaps;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.MK4SwerveModule;
import frc.robot.FRC5010.drive.MK4iSwerveModule;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.mechanisms.DriveConstantsDef;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.robots.RobotConfig;
import frc.robot.FRC5010.robots.RobotFactory.Parts;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;

/** Add your docs here. */
public class CompBot extends RobotConfig {
    private SwerveConstants swerveConstants; 
    private GenericMechanism drive; 
    //private Map<String,List<PathPlannerTrajectory>> paths = new HashMap<>(); 
    //private HashMap<String, Command> eventMap = new HashMap<>();   
    private AutoMaps autoMaps;

    private Persisted<Double> maxChassisVelocity;
    private Persisted<Double> maxChassisRotation;
    public CompBot(){
        // Needs to be set
        swerveConstants = new SwerveConstants(0, 0);

        // Baby Swerve values need to be changed
        swerveConstants.setkFrontLeftAbsoluteOffsetRad(0.26);
        swerveConstants.setkFrontRightAbsoluteOffsetRad(-3.14);
        swerveConstants.setkBackLeftAbsoluteOffsetRad(1.0+Math.PI);
        swerveConstants.setkBackRightAbsoluteOffsetRad(0.21+Math.PI);
        swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(5);
        swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);
        swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.4);
        swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
        swerveConstants.setSwerveModuleConstants(MK4iSwerveModule.MK4I_L1);
        swerveConstants.configureSwerve(NEO.MAXRPM, NEO.MAXRPM);
        maxChassisVelocity = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_VELOCITY, swerveConstants.getkTeleDriveMaxSpeedMetersPerSecond());
        maxChassisRotation = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_ROTATION, swerveConstants.getkTeleDriveMaxAngularSpeedRadiansPerSecond());
        
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


        GenericGyro gyro = new PigeonGyro(11);

        autoMaps = new AutoMaps();
        autoMaps.loadAutoPaths();

        GenericMechanism drive = new Drive(multiVision, gyro, Drive.Type.MK4I_SWERVE_DRIVE, swervePorts, swerveConstants);
        robotParts.put(Parts.VISION, multiVision);
        robotParts.put(Parts.DRIVE, drive);
        robotParts.put(Parts.AUTO, setAutoCommands()); 
    } 

    public Map<String,Command> setAutoCommands(){
      return drive.setAutoCommands(autoMaps.getPaths(), autoMaps.getEventMap()); 
    }

}
